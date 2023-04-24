#![no_std]
#![no_main]

mod panic_handler;
mod ring_buffer;

use core::{
	iter::once,
	ops::{Add, Div},
};

use cortex_m::{delay::Delay, prelude::_embedded_hal_timer_CountDown};
use cortex_m_rt::entry;
use embedded_hal::{
	adc::{Channel, OneShot},
	timer::Cancel,
};
use fugit::ExtU64;
use panic_handler::set_panic_pin;
use ring_buffer::RingBuffer;
use rp2040_hal::{
	clocks::init_clocks_and_plls,
	gpio::{self, Floating, Input, Pin, PinId},
	pac,
	prelude::_rphal_pio_PIOExt,
	Adc, Clock, Sio, Timer, Watchdog,
};
use smart_leds::{
	hsv::{hsv2rgb, Hsv},
	SmartLedsWrite, RGB8,
};
use ws2812_pio::Ws2812;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[entry]
fn entrypoint() -> ! {
	main()
}

const BUF_LEN: usize = 400;
const INPUT_BIAS: u16 = 90;
const NUM_LEDS: usize = 110;
const OUTPUT_SMOOTHING_LENGTH: usize = 3;

fn main() -> ! {
	let mut pac = pac::Peripherals::take().unwrap();
	let core = pac::CorePeripherals::take().unwrap();
	let mut watchdog = Watchdog::new(pac.WATCHDOG);
	let sio = Sio::new(pac.SIO);

	let external_xtal_freq_hz = 12_000_000u32;
	let clocks = init_clocks_and_plls(
		external_xtal_freq_hz,
		pac.XOSC,
		pac.CLOCKS,
		pac.PLL_SYS,
		pac.PLL_USB,
		&mut pac.RESETS,
		&mut watchdog,
	)
	.ok()
	.unwrap();

	let pins = gpio::Pins::new(
		pac.IO_BANK0,
		pac.PADS_BANK0,
		sio.gpio_bank0,
		&mut pac.RESETS,
	);

	let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

	set_panic_pin(pins.gpio25.into_mode(), &mut delay);

	let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

	let mut left_input = pins.gpio27.into_floating_input();
	let mut right_input = pins.gpio26.into_floating_input();

	let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

	let (mut pio, sm0, ..) = pac.PIO0.split(&mut pac.RESETS);
	let mut ws = Ws2812::new(
		pins.gpio4.into_mode(),
		&mut pio,
		sm0,
		clocks.peripheral_clock.freq(),
		timer.count_down(),
	);

	let mut left_sample_buffer = [0; BUF_LEN];
	let mut right_sample_buffer = [0; BUF_LEN];

	let mut left_output_buffer =
		RingBuffer::new([Hsv16::default(); NUM_LEDS / 2]);
	let mut right_output_buffer =
		RingBuffer::new([Hsv16::default(); NUM_LEDS / 2]);

	let mut hue = 0;

	let mut target_hue = hue;

	let mut hue_cycle = timer.count_down();
	hue_cycle.start(5.secs());

	let mut hue_fade = timer.count_down();

	loop {
		for (l, r) in left_sample_buffer
			.iter_mut()
			.zip(right_sample_buffer.iter_mut())
		{
			*l = read_channel(&mut adc, &mut left_input);
			*r = read_channel(&mut adc, &mut right_input);

			delay.delay_us(50);
		}

		channel_values(&mut left_sample_buffer, &mut left_output_buffer, hue);
		channel_values(&mut right_sample_buffer, &mut right_output_buffer, hue);

		ws.write(
			left_output_buffer
				.in_order_iter()
				.rev()
				.chain(right_output_buffer.in_order_iter())
				.copied()
				.map(|Hsv16 { hue, sat, val }| {
					hsv2rgb(Hsv {
						hue,
						sat,
						val: (val / 4).min(255) as u8,
					})
				}),
		)
		.unwrap();

		if let Ok(()) = hue_cycle.wait() {
			target_hue = next_hue(&left_sample_buffer);
			let dist = hue_dist(hue, target_hue);
			hue_fade.start((dist as u64).millis());
		}

		if hue == target_hue {
			let _ = hue_fade.cancel();
		} else if let Ok(()) = hue_fade.wait() {
			transition_hue(&mut hue, target_hue);
		}
	}
}

fn read_channel<'p, P>(
	adc: &mut Adc,
	pin: &'p mut Pin<P, Input<Floating>>,
) -> u16
where
	P: PinId + 'static,
	Pin<P, Input<Floating>>: Channel<Adc, ID = u8>,
{
	let value: u16 = adc.read(pin).unwrap();
	value.saturating_sub(INPUT_BIAS).min(i16::MAX as u16)
}

fn channel_values<const N: usize>(
	samples: &mut [u16],
	outputs: &mut RingBuffer<Hsv16, N>,
	hue: u8,
) {
	let now = samples.iter().map(|&x| x as u32).avg() as u16;

	let weighted_avg = once(now)
		.chain(outputs.in_order_iter().map(|hsv| hsv.val))
		.take(OUTPUT_SMOOTHING_LENGTH)
		.enumerate()
		.map(|(i, x)| x / (2 * (i as u16 + 1)))
		.sum::<u16>();

	outputs.push(Hsv16 {
		hue,
		sat: 250,
		val: weighted_avg,
	});
}

#[derive(Default, Clone, Copy)]
struct Hsv16 {
	hue: u8,
	sat: u8,
	val: u16,
}

fn next_hue(samples: &[u16]) -> u8 {
	samples
		.iter()
		.copied()
		.reduce(|acc, x| xorshift(acc ^ x))
		.unwrap() as u8
}

fn xorshift(mut x: u16) -> u16 {
	x ^= x << 7;
	x ^= x >> 8;
	x ^= x << 9;
	x
}

fn transition_hue(current: &mut u8, target: u8) {
	if *current < target {
		if target - *current < 128 {
			*current += 1;
		} else {
			*current = current.wrapping_sub(1);
		}
	} else {
		if *current - target < 128 {
			*current -= 1;
		} else {
			*current = current.wrapping_add(1);
		}
	}
}

fn hue_dist(current: u8, target: u8) -> u8 {
	let greater = current.max(target);
	let lesser = current.min(target);
	let direct_dist = greater - lesser;
	if direct_dist > 128 {
		255 - direct_dist
	} else {
		direct_dist
	}
}

trait Average: Iterator {
	fn avg(self) -> Self::Item;
}

impl<I: Iterator> Average for I
where
	I::Item: Add<Output = I::Item> + Div<Output = I::Item> + From<bool> + Copy,
{
	fn avg(self) -> Self::Item {
		let zero = I::Item::from(false);
		let one = I::Item::from(true);

		let (count, sum) =
			self.fold((zero, zero), |(count, sum), x| (count + one, sum + x));

		sum / count
	}
}
