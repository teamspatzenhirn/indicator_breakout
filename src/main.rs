#![no_std]
#![no_main]

use core::cmp::min;

// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate i2c_dev won't
// be linked)
use panic_halt as _;

use rp_pico::hal::clocks::init_clocks_and_plls;
use rp_pico::hal::gpio::{Function, Pin, I2C};
use rp_pico::hal::i2c::peripheral::I2CEvent;

use rp_pico::hal::{i2c, prelude::*, Timer, Watchdog};

use rp_pico::hal::pac;

use rp_pico::hal;

use smart_leds::{brightness, colors, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

const STRIP_LEN: usize = 16 * 3;

#[derive(Clone, Copy)]
enum Mode {
    Left,
    Right,
    Both,
    Off,
}

const FRAME_DURATION: u32 = 500;
const HOLD_DURATION: u32 = 250;
const FADE_DURATION: u32 = FRAME_DURATION - HOLD_DURATION;

fn calculate_frame(buf: &mut [RGB8; STRIP_LEN], t: u32, mode: Mode, brake: bool) {
    buf.fill(colors::BLACK);

    match mode {
        Mode::Left => {
            let progress = min(16, 16 * t / FADE_DURATION) as usize;
            buf[32..32 + progress].fill(colors::YELLOW);
        }
        Mode::Right => {
            let progress = min(16, 16 * t / FADE_DURATION) as usize;
            buf[16 - progress..16].fill(colors::YELLOW);
        }
        Mode::Both => {
            buf[0..16].fill(if t < FRAME_DURATION / 2 {
                colors::YELLOW
            } else {
                colors::BLACK
            });

            buf[STRIP_LEN - 16..STRIP_LEN].fill(if t < FRAME_DURATION / 2 {
                colors::YELLOW
            } else {
                colors::BLACK
            });
        }
        Mode::Off => {}
    }

    if brake {
        buf[0..3].fill(colors::RED);
        buf[STRIP_LEN - 3..STRIP_LEN].fill(colors::RED);
        buf[16..32].fill(colors::RED);
    }
}

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut frame_delay =
        cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Create a count down timer for the Ws2812 instance:
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    // Split the PIO state machine 0 into individual objects, so that
    // Ws2812 can use it:
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Instanciate a Ws2812 LED strip:
    let mut ws = Ws2812::new(
        // Use pin 6 on the Raspberry Pi Pico (which is GPIO4 of the rp2040 chip)
        // for the LED data output:
        pins.gpio4.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    led_pin.set_low().unwrap();

    let mut sda: Pin<_, Function<I2C>> = pins.gpio18.into_mode();
    let mut scl: Pin<_, Function<I2C>> = pins.gpio19.into_mode();
    scl.set_drive_strength(hal::gpio::OutputDriveStrength::TwoMilliAmps);
    sda.set_drive_strength(hal::gpio::OutputDriveStrength::TwoMilliAmps);

    let mut i2c_dev =
        hal::I2C::new_peripheral_event_iterator(pac.I2C1, sda, scl, &mut pac.RESETS, 0x003);

    let mut t = 0u32;
    let mut last_start = 0u32;
    let mut mode = Mode::Off;
    let mut brake = false;

    let mut leds: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];

    let strip_brightness: u8 = 20;

    loop {
        let event = i2c_dev.next();

        match event {
            Some(I2CEvent::Start | I2CEvent::Restart) => {}
            Some(hal::i2c::peripheral::I2CEvent::TransferRead) => {
                led_pin.set_high().unwrap();
                let reply = [42u8];
                i2c_dev.write(&reply);
            }
            Some(I2CEvent::TransferWrite) => {
                led_pin.set_low().unwrap();
                let mut read_buf = [0u8];
                i2c_dev.read(&mut read_buf);

                brake = (read_buf[0] & 0b00001000) != 0;
                let mode_nr: u8 = read_buf[0] & 0b111;

                mode = match mode_nr {
                    0 => Mode::Off,
                    1 => Mode::Both,
                    2 => Mode::Left,
                    3 => Mode::Right,
                    _ => mode,
                }
            }
            Some(I2CEvent::Stop) => {}
            None => {}
        }

        calculate_frame(&mut leds, t, mode, brake);
        ws.write(brightness(leds.iter().copied(), strip_brightness))
            .unwrap();

        frame_delay.delay_ms(5);
        t = (t + 5) % FRAME_DURATION;
    }
}
