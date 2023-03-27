#![no_std]
#![no_main]

mod animation;

use indicator_interface::{IndicatorDuration, RustIndicatorCommand};

// Ensure we halt the program on panic
use panic_halt as _;

use rtic::app;

pub enum IndicatorState {
    OFF,
    LEFT,
    RIGHT,
    BOTH,
}

fn state_from_command(c: &mut RustIndicatorCommand) -> IndicatorState {
    let (new_command, state): (RustIndicatorCommand, IndicatorState) = match c {
        RustIndicatorCommand::OFF => (RustIndicatorCommand::OFF, IndicatorState::OFF),
        RustIndicatorCommand::LEFT(c) => (
            match c {
                IndicatorDuration::INFINITE => {
                    RustIndicatorCommand::LEFT(IndicatorDuration::INFINITE)
                }
                IndicatorDuration::FINITE(c) if *c <= 1 => RustIndicatorCommand::OFF,
                IndicatorDuration::FINITE(c) => {
                    RustIndicatorCommand::LEFT(IndicatorDuration::FINITE(*c - 1))
                }
            },
            IndicatorState::LEFT,
        ),
        RustIndicatorCommand::RIGHT(c) => (
            match c {
                IndicatorDuration::INFINITE => {
                    RustIndicatorCommand::RIGHT(IndicatorDuration::INFINITE)
                }
                IndicatorDuration::FINITE(c) if *c <= 1 => RustIndicatorCommand::OFF,
                IndicatorDuration::FINITE(c) => {
                    RustIndicatorCommand::RIGHT(IndicatorDuration::FINITE(*c - 1))
                }
            },
            IndicatorState::LEFT,
        ),
        RustIndicatorCommand::BOTH(c) => (
            match c {
                IndicatorDuration::INFINITE => {
                    RustIndicatorCommand::BOTH(IndicatorDuration::INFINITE)
                }
                IndicatorDuration::FINITE(c) if *c <= 1 => RustIndicatorCommand::OFF,
                IndicatorDuration::FINITE(c) => {
                    RustIndicatorCommand::BOTH(IndicatorDuration::FINITE(*c - 1))
                }
            },
            IndicatorState::BOTH,
        ),
    };

    *c = new_command;
    state
}

#[app(device=rp_pico::hal::pac, dispatchers=[SW0_IRQ, SW1_IRQ, SW2_IRQ])]
mod app {
    use crate::animation::{self};
    use core::ops::Deref;
    use hal::i2c::{SclPin, SdaPin};
    use indicator_interface::{BrakeLightState, RustIndicatorCommand};
    use rp2040_hal::pio::PIOExt;
    use rp2040_hal::Clock;
    use rp2040_hal::{gpio::FunctionI2C, pac};
    use rp_pico::{
        hal::{
            self,
            gpio::{pin::bank0::*, Pin},
            pio::SM0,
            timer::{monotonic::*, Alarm0},
            Watchdog,
        },
        XOSC_CRYSTAL_FREQ,
    };

    use heapless;
    use smart_leds::SmartLedsWrite;
    use ws2812_pio::Ws2812Direct;

    #[monotonic(binds = TIMER_IRQ_0, default=true)]
    type MyMono = Monotonic<Alarm0>;

    #[shared]
    struct Shared {
        next_command: Option<RustIndicatorCommand>,
        brakelight: bool,
        leds: [smart_leds::RGB8; animation::STRIP_LEN],
    }

    #[local]
    struct Local {
        led_strip: Ws2812Direct<rp2040_hal::pac::PIO0, SM0, Gpio4>,
        frame_delay: cortex_m::delay::Delay,
        i2c: rp2040_hal::pac::I2C1,
        decode_buffer: heapless::Vec<u8, 128>,
    }

    fn configure_i2c<
        I2C: Deref<Target = pac::i2c0::RegisterBlock>,
        Sda: hal::gpio::PinId + BankPinId,
        Scl: hal::gpio::PinId + BankPinId,
    >(
        i2c: I2C,
        resets: &pac::RESETS,
        interrupt: pac::Interrupt,
        slave_address: u8,
        _sda_pin: Pin<Sda, FunctionI2C>,
        _scl_pin: Pin<Scl, FunctionI2C>,
    ) -> I2C
    where
        Sda: SdaPin<I2C>,
        Scl: SclPin<I2C>,
    {
        // Reset i2c1 module
        resets.reset.modify(|_, w| w.i2c1().set_bit());
        resets.reset.modify(|_, w| w.i2c1().clear_bit());
        while resets.reset_done.read().i2c1().bit_is_clear() {}

        // Disable i2c1
        i2c.ic_enable.write(|w| w.enable().disabled());

        // Set slave address
        i2c.ic_sar
            .modify(|_, w| unsafe { w.ic_sar().bits(slave_address as u16) });

        // Select 7-bit addressing, enable slave-only mode
        i2c.ic_con.modify(|_, w| {
            w.ic_10bitaddr_slave()
                .addr_7bits()
                .ic_slave_disable()
                .slave_enabled()
        });

        // Setup fifo

        // hold bus when rx fifo is full
        i2c.ic_con
            .modify(|_, w| w.rx_fifo_full_hld_ctrl().enabled());

        // "enabled" means "masking enabled" here, which means the interrupt will *not* fire
        i2c.ic_intr_mask.modify(|_, w| {
            // We dont use generall call
            w.m_gen_call()
                .enabled()
                // We dont transmit
                .m_tx_over()
                .enabled()
                .m_tx_abrt()
                .enabled()
                .m_tx_empty()
                .enabled()
        });

        // Setup interrupt
        unsafe { pac::NVIC::unmask(interrupt) }

        // Enable i2c1
        i2c.ic_enable.modify(|_, w| w.enable().enabled());

        i2c
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

        // Split the PIO state machine 0 into individual objects, so that
        // Ws2812 can use it:
        let (mut pio, sm0, _, _, _) = c.device.PIO0.split(&mut resets);

        let led_strip = Ws2812Direct::new(
            // Use pin 6 on the Raspberry Pi Pico (which is GPIO4 of the rp2040 chip)
            // for the LED data output:
            pins.gpio4.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        let frame_delay =
            cortex_m::delay::Delay::new(c.core.SYST, clocks.system_clock.freq().to_Hz());

        // Configure pinmuxing
        let mut sda = pins.gpio18.into_mode::<hal::gpio::FunctionI2C>();
        let mut scl = pins.gpio19.into_mode::<hal::gpio::FunctionI2C>();
        scl.set_drive_strength(hal::gpio::OutputDriveStrength::TwoMilliAmps);
        sda.set_drive_strength(hal::gpio::OutputDriveStrength::TwoMilliAmps);

        let i2c = configure_i2c(
            c.device.I2C1,
            &resets,
            pac::interrupt::I2C1_IRQ,
            0x9,
            sda,
            scl,
        );

        indicate_task::spawn().unwrap();

        (
            Shared {
                next_command: Some(RustIndicatorCommand::LEFT(
                    indicator_interface::IndicatorDuration::INFINITE,
                )),
                brakelight: false,
                leds: [smart_leds::colors::BLACK; animation::STRIP_LEN],
            },
            Local {
                led_strip,
                frame_delay,
                i2c,
                decode_buffer: heapless::Vec::new(),
            },
            init::Monotonics(Monotonic::new(timer, alarm)),
        )
    }

    #[task(shared=[next_command, brakelight, leds], local=[frame_delay], priority=2)]
    fn indicate_task(mut c: indicate_task::Context) {
        let mut i = 0;

        let current_action = c
            .shared
            .next_command
            .lock(|c| c.as_mut().map(crate::state_from_command));

        let mode = match current_action {
            Some(a) => a,
            None => return,
        };

        loop {
            // Update latest brakelight state
            let mut brake = false;
            c.shared.brakelight.lock(|bl| brake = *bl);

            // Calculate LED pattern (this may be safely interrupted by I2C)
            let next = c
                .shared
                .leds
                .lock(|leds| animation::calculate_frame(leds, i, &mode, brake));

            // Write out this LED pattern in high-prio task (which will be interrupted by I2C)
            led_write_task::spawn().unwrap();

            // Wait frame delay using systick timer.
            // This looks like busy-waiting, but higher priority tasks such as writing out the LED pattern
            // or receiving I2C data interrupts this.
            match next {
                animation::AnimationResult::NextFrameIn(delay) => {
                    c.local
                        .frame_delay
                        .delay_us(u32::try_from(delay.to_micros()).unwrap_or(u32::MAX));
                }
                animation::AnimationResult::Done(delay) => {
                    c.local
                        .frame_delay
                        .delay_us(u32::try_from(delay.to_micros()).unwrap_or(u32::MAX));
                    break;
                }
            }

            i += 1;
        }

        indicate_task::spawn().unwrap();
    }

    #[task(priority=4, shared=[leds], local=[led_strip])]
    fn led_write_task(mut c: led_write_task::Context) {
        c.shared.leds.lock(|leds| {
            c.local
                .led_strip
                .write(smart_leds::brightness(
                    smart_leds::gamma(leds.iter().copied()),
                    10,
                ))
                .unwrap()
        });
    }

    #[task(binds=I2C1_IRQ, priority=4, shared=[next_command, brakelight], local=[i2c, decode_buffer])]
    fn i2c_receive_task(mut c: i2c_receive_task::Context) {
        let i2c = c.local.i2c;

        let mut interrupt_handled = false;

        /*
        R_GEN_CALL: Interrupt masked, see config function
        R_TX_OVER: Masked, we don't transmit
        R_TX_ABRT: Masked, we don't transmit
        R_TX_EMPTY: Masked, we don't transmit
        */

        if i2c.ic_intr_stat.read().r_rx_under().is_active() {
            // Occurs if we try to read from fifo when it's empty. We shouldn't do this i guess...
            panic!();
        }

        if i2c.ic_intr_stat.read().r_rx_over().is_active() {
            // We configure RX_FIFO_FULL_HLD_CTRL, so this interrupt never occurs
            panic!();
        }

        if i2c.ic_intr_stat.read().r_rd_req().is_active() {
            // We don't have any data for master, but not clearing this IRQ holds bus in wait state...
            // Let's hope this sends a NAK or something? Maybe releases the bus at least?
            i2c.ic_clr_rd_req.read().clr_rd_req().bit();
            interrupt_handled = true;
        }

        if i2c.ic_intr_stat.read().r_rx_full().is_active() {
            let data_reg = i2c.ic_data_cmd.read();
            let is_first = data_reg.first_data_byte().is_active();
            if is_first {
                c.local.decode_buffer.clear();
            }

            let data = data_reg.dat().bits();
            c.local
                .decode_buffer
                .push(data)
                .expect("Decode buffer too small!");

            // Interrupt is automatically cleared when buffer level goes below threshold.
            interrupt_handled = true;
        }

        if i2c.ic_intr_stat.read().r_rx_done().is_active() {
            let data_reg = i2c.ic_data_cmd.read();
            let is_first = data_reg.first_data_byte().is_active();
            if is_first {
                c.local.decode_buffer.clear();
            }

            let data = data_reg.dat().bits();
            c.local
                .decode_buffer
                .push(data)
                .expect("Decode buffer too small!");

            let decoding_result = indicator_interface::postcard::from_bytes::<
                indicator_interface::I2CMessage,
            >(c.local.decode_buffer);
            if let Ok(message) = decoding_result {
                match message {
                    indicator_interface::I2CMessage::BrakeLight(bl) => {
                        c.shared.brakelight.lock(|b| {
                            *b = match bl {
                                BrakeLightState::ON => true,
                                BrakeLightState::OFF => false,
                            }
                        })
                    }
                    indicator_interface::I2CMessage::Indicator(command) => {
                        c.shared.next_command.lock(|nc| *nc = Some(command));
                        indicate_task::spawn().unwrap();
                    }
                }
            }

            i2c.ic_clr_rx_done.read().clr_rx_done().bit();
            interrupt_handled = true;
        }

        if !interrupt_handled {
            // TODO: Add some error handling here, so we can tell if some unexpected interrupt occurred...
            panic!();
        }
    }
}
