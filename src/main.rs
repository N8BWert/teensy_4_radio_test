//! The starter code slowly blinks the LED, sets up
//! USB logging, and creates a UART driver using pins
//! 14 and 15. The UART baud rate is [`UART_BAUD`].
//!
//! Despite targeting the Teensy 4.0, this starter code
//! also works on the Teensy 4.1.

#![no_std]
#![no_main]

use teensy4_panic as _;

mod motion_control;

#[rtic::app(device = teensy4_bsp, peripherals = true)]
mod app {
    use imxrt_hal as hal;
    use hal::lpspi::{Lpspi, Pins};
    use hal::gpio::{Trigger, Output};
    use hal::iomuxc::pads::{gpio_b0::{GPIO_B0_11, GPIO_B0_02, GPIO_B0_01, GPIO_B0_03, GPIO_B0_00, GPIO_B0_10}, gpio_b1::GPIO_B1_00};

    use imxrt_ral as ral;
    use ral::lpspi::LPSPI4;

    use teensy4_bsp as bsp;
    use bsp::board;

    use crate::motion_control::MotionControlCommand;

    use nrf24_rs::Nrf24l01;
    use nrf24_rs::config::NrfConfig;

    #[local]
    struct Local {
        radio: Nrf24l01<Lpspi<Pins<GPIO_B0_02, GPIO_B0_01, GPIO_B0_03, GPIO_B0_00>, 4>, Output<GPIO_B0_11>, Output<GPIO_B1_00>>,
        // radio: NRF24L01<core::convert::Infallible, Output<GPIO_B0_11>, Output<GPIO_B1_00>, Lpspi<Pins<GPIO_B0_02, GPIO_B0_01, GPIO_B0_03, GPIO_B0_00>, 4>>,
    }

    #[shared]
    struct Shared {
        latest_motion_control_command: MotionControlCommand,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board_resources = board::t41(board::instances());
        let cortex_m_peripherals = cortex_m::Peripherals::take().unwrap();

        // Setup USB Logging
        let usb = board_resources.usb;
        teensy4_bsp::LoggingFrontend::default_log().register_usb(usb);

        let pins = board_resources.pins;
        let mut gpio2 = board_resources.gpio2;

        // Initialize Radio Spi
        let radio_spi_pins = Pins {
            sdo: pins.p11,
            sdi: pins.p12,
            sck: pins.p13,
            pcs0: pins.p10,
        };

        let radio_spi = unsafe { LPSPI4::instance() };
        let radio_spi = Lpspi::new(radio_spi, radio_spi_pins);

        let radio_cs = gpio2.output(pins.p8);
        let radio_ce = gpio2.output(pins.p9);
        let interrupt_pin = gpio2.input(pins.p6);
        let radio_interrupt = gpio2.set_interrupt(&interrupt_pin, Some(Trigger::Low));

        let mut delay = cortex_m::delay::Delay::new(cortex_m_peripherals.SYST, 10u32);
        let radio = Nrf24l01::new(radio_spi, radio_ce, radio_cs, &mut delay, NrfConfig::default()).unwrap();

        (
            Shared {
                latest_motion_control_command: MotionControlCommand::new(),
            },
            Local {
                radio,
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = GPIO2_COMBINED_0_15, local = [radio])]
    fn radio_interrupt(cx: radio_interrupt::Context) {

    }
}
