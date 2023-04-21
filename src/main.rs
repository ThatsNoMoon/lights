#![no_std]
#![no_main]

mod panic_handler;
mod ring_buffer;

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_hal::{adc::OneShot, PwmPin};
use panic_handler::set_panic_pin;
use ring_buffer::RingBuffer;
use rp2040_hal as hal;

use hal::{
    pac,
    pwm::{FreeRunning, Slices},
    Adc, Sio, Watchdog, clocks::init_clocks_and_plls, Clock,
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[entry]
fn main() -> ! {
    real_main()
}

const BUF_LEN: usize = 100;
const INPUT_BIAS: u16 = 90;

fn real_main() -> ! {
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

    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    let mut pwm = pwm_slices.pwm0;
    pwm.set_ph_correct();
    pwm.enable();

    let pwm = pwm.into_mode::<FreeRunning>();

    let mut left_led = pwm.channel_b;
    left_led.output_to(pins.gpio17);

    let mut right_led = pwm.channel_a;
    right_led.output_to(pins.gpio16);

    let mut left_buffer = RingBuffer::new([0; BUF_LEN]);
    let mut right_buffer = RingBuffer::new([0; BUF_LEN]);

    loop {
        let left: u16 = adc.read(&mut left_input).unwrap();
        let right: u16 = adc.read(&mut right_input).unwrap();
        left_buffer.push(left.saturating_sub(INPUT_BIAS));
        right_buffer.push(right.saturating_sub(INPUT_BIAS));
        let left_avg = left_buffer.iter().map(|&x| x as u32).sum::<u32>() / BUF_LEN as u32;
        let right_avg = right_buffer.iter().map(|&x| x as u32).sum::<u32>() / BUF_LEN as u32;
        left_led.set_duty((left_avg as u16) << 4);
        right_led.set_duty((right_avg as u16) << 4);
    }
}
