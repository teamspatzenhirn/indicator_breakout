#![no_std]
#![no_main]

mod animation;

use indicator_interface::{IndicatorDuration, IndicatorState, RustIndicatorCommand};

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate i2c_dev won't
// be linked)
use panic_halt as _;

use rp_pico::hal::gpio::{Function, Pin, I2C};
use rp_pico::hal::i2c::peripheral::I2CEvent;

use rp_pico::hal::{prelude::*, Timer, Watchdog};

use rp_pico::hal::pac;

use rp_pico::hal;

use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

use rtic::app;

#[app(device=rp_pico::hal::pac, dispatchers=[SW0_IRQ, SW1_IRQ, SW2_IRQ])]
mod app {
    use crate::animation::{self};
    use embedded_hal::digital::v2::ToggleableOutputPin;
    use fugit::ExtU64;
    use indicator_interface::{IndicatorState, RustIndicatorCommand};
    use rp2040_hal::pio::PIOExt;
    use rp2040_hal::Clock;
    use rp_pico::{
        hal::{
            self,
            gpio::{pin::bank0::*, Pin, PushPullOutput},
            pio::SM0,
            timer::{monotonic::*, Alarm0, CountDown},
            Watchdog,
        },
        XOSC_CRYSTAL_FREQ,
    };

    use smart_leds::SmartLedsWrite;
    use ws2812_pio::Ws2812;

    #[monotonic(binds = TIMER_IRQ_0, default=true)]
    type MyMono = Monotonic<Alarm0>;

    #[shared]
    struct Shared {
        next_command: RustIndicatorCommand,
        brakelight: bool,
    }

    #[local]
    struct Local<'a> {
        led: Pin<Gpio25, PushPullOutput>,
        led_strip: Ws2812<rp2040_hal::pac::PIO0, SM0, rp2040_hal::timer::CountDown<'a>, Gpio4>,
        frame_delay: cortex_m::delay::Delay,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        let alarm = timer.alarm_0().unwrap();

        let sio = hal::Sio::new(c.device.SIO);
        // Set the pins up according to their function on this particular board
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        let led_pin = pins.led.into_push_pull_output();

        // Split the PIO state machine 0 into individual objects, so that
        // Ws2812 can use it:
        let (mut pio, sm0, _, _, _) = c.device.PIO0.split(&mut resets);

        let led_strip = Ws2812::new(
            // Use pin 6 on the Raspberry Pi Pico (which is GPIO4 of the rp2040 chip)
            // for the LED data output:
            pins.gpio4.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
           timer.count_down(),
        );

        let frame_delay =
            cortex_m::delay::Delay::new(c.core.SYST, clocks.system_clock.freq().to_Hz());

        blink_task::spawn().unwrap();

        (
            Shared {
                next_command: RustIndicatorCommand::OFF,
                brakelight: false,
            },
            Local {
                led: led_pin,
                led_strip,
                frame_delay,
            },
            init::Monotonics(Monotonic::new(timer, alarm)),
        )
    }

    #[task(shared=[next_command, brakelight],local=[led_strip, frame_delay], priority=2)]
    fn indicate_task(mut c: indicate_task::Context, mode: IndicatorState) {
        // TODO: play animation

        let mut leds: [smart_leds::RGB8; animation::STRIP_LEN] =
            [(0, 0, 0).into(); animation::STRIP_LEN];

        let end_t = monotonics::now() + animation::Duration::millis(animation::ANIMATION_DURATION_ms);

        let mut i = 0;

        while monotonics::now() < end_t {
            let mut brake = false;
            c.shared.brakelight.lock(|bl| brake = *bl);

            animation::calculate_frame(&mut leds, i, &mode, brake);
            c.local
                .led_strip
                .write(smart_leds::brightness(leds.iter().copied(), 20))
                .unwrap();

            c.local.frame_delay.delay_ms(5);
        }

        // TODO: start task for next command, decrement command
    }

    #[task(binds=I2C1_IRQ, priority=4, shared=[next_command,  brakelight])]
    fn i2c_receive_task(_: i2c_receive_task::Context) {
        // TODO: update next task
        // TODO: update brakelight
    }

    #[task(local=[led], priority=1)]
    fn blink_task(c: blink_task::Context) {
        blink_task::spawn_after(100.millis()).unwrap();

        let led = c.local.led;
        led.toggle().unwrap();
    }
}

#[allow(unused)]
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
    let mut mode = IndicatorState::OFF;
    let mut brake = false;

    let mut leds: [RGB8; animation::STRIP_LEN] = [(0, 0, 0).into(); animation::STRIP_LEN];

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
                    0 => IndicatorState::OFF,
                    1 => IndicatorState::BOTH,
                    2 => IndicatorState::LEFT,
                    3 => IndicatorState::RIGHT,
                    _ => mode,
                }
            }
            Some(I2CEvent::Stop) => {}
            None => {}
        }

        animation::calculate_frame(&mut leds, t, &mode, brake);
        ws.write(brightness(leds.iter().copied(), strip_brightness))
            .unwrap();

        frame_delay.delay_ms(5);
       // t = (t + 5) % animation::FRAME_DURATION;
    }
}
