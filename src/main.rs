#![no_std]
#![no_main]

mod panic_handler;
mod ring_buffer;

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_hal::adc::OneShot;
use panic_handler::set_panic_pin;
use ring_buffer::RingBuffer;
use rp2040_hal as hal;

use hal::{
	clocks::init_clocks_and_plls, pac, prelude::_rphal_pio_PIOExt, Adc, Clock,
	Sio, Timer, Watchdog,
};
use smart_leds::{
	hsv::{hsv2rgb, Hsv},
	SmartLedsWrite,
};
use ws2812_pio::Ws2812;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[entry]
fn entrypoint() -> ! {
	main()
}

const BUF_LEN: usize = 100;
const INPUT_BIAS: u16 = 90;

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

	let pins = hal::gpio::Pins::new(
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

	let mut left_buffer = RingBuffer::new([0; BUF_LEN]);
	let mut right_buffer = RingBuffer::new([0; BUF_LEN]);

	loop {
		let left: u16 = adc.read(&mut left_input).unwrap();
		let right: u16 = adc.read(&mut right_input).unwrap();
		left_buffer.push(left.saturating_sub(INPUT_BIAS));
		right_buffer.push(right.saturating_sub(INPUT_BIAS));
		// let left_avg = left_buffer.iter().map(|&x| x as u32).sum::<u32>() / BUF_LEN as u32;
		// let right_avg = right_buffer.iter().map(|&x| x as u32).sum::<u32>() / BUF_LEN as u32;
		ws.write(left_buffer.iter().map(|&amplitude| {
			hsv2rgb(Hsv {
				hue: 0,
				sat: 100,
				val: (amplitude / 256) as u8,
			})
		}))
		.unwrap();
	}
}
