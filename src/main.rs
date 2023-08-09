//! This code is a PoC for using the Teensy 4.1 with the nRF24L01

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use teensy4_panic as _;

mod motion_control;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [DCP])]
mod app {
    use core::convert::Infallible;

    use teensy4_bsp as bsp;
    use bsp::board;

    use bsp::hal as hal;
    use hal::lpspi::Lpspi;
    use hal::gpio::{Trigger, Output};
    use hal::iomuxc::pads::{gpio_b0::GPIO_B0_00, gpio_b1::GPIO_B1_00};

    use bsp::ral as ral;
    use ral::lpspi::LPSPI4;

    use crate::motion_control::MotionControlCommand;

    use embedded_nrf24l01::{Rx, NRF24L01, ChangeModes};
    use embedded_nrf24l01::config::{NRF24L01Config, InterruptMask, RetransmitConfig, CrcMode, DataRate, PALevel};

    use rtic_monotonics::systick::{Systick, ExtU32};

    #[local]
    struct Local {
        radio: NRF24L01<'static, Infallible, Output<GPIO_B1_00>, Output<GPIO_B0_00>, Lpspi<(), 4>>,
        counter: u32,
    }

    #[shared]
    struct Shared {
        _latest_motion_control_command: MotionControlCommand,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board_resources = board::t41(board::instances());
        let usb = board_resources.usb;
        let spi4 = unsafe { LPSPI4::instance() };
        let mut gpio2 = board_resources.gpio2;
        let pins = board_resources.pins;

        // Configure USB Logger
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // Configure Radio
        let radio_spi = Lpspi::without_pins(spi4);

        let radio_config = NRF24L01Config {
            data_rate: DataRate::R1Mbps,
            crc_mode: CrcMode::Disabled,
            rf_channel: 0u8,
            pa_level: PALevel::PA0dBm,
            interrupt_mask: InterruptMask { data_ready_rx: true, data_sent_tx: false, max_retramsits_tx: false },
            read_enabled_pipes: [true, false, false, false, false, false],
            rx_addrs: [b"aaa", b"aab", b"aac", b"aad", b"aae", b"aaf"],
            tx_addr: b"aag",
            retransmit_config: RetransmitConfig { delay: 0u8, count: 0u8 },
            auto_ack_pipes: [false; 6],
            address_width: 3u8,
            pipe_payload_lengths: [None; 6],
        };

        let radio_ce = gpio2.output(pins.p8);
        let radio_cs = gpio2.output(pins.p10);

        let mut nrf_radio = match NRF24L01::new_with_config(radio_ce, radio_cs, radio_spi, radio_config) {
            Err(err) => panic!("Error Initializing NRF24L01: {:?}", err),
            Ok(radio) => radio,
        };
        nrf_radio.to_rx().unwrap();

        let radio_interrupt_pin = gpio2.input(pins.p6);
        gpio2.set_interrupt(&radio_interrupt_pin, Some(Trigger::RisingEdge));

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 12_000_000, systick_token);

        log_info::spawn().ok();

        (
            Shared {
                _latest_motion_control_command: MotionControlCommand::new(),
            },
            Local {
                radio: nrf_radio,
                counter: 0u32,
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1, local = [counter])]
    async fn log_info(ctx: log_info::Context) {
        loop {
            Systick::delay(ExtU32::millis(1_000u32)).await;

            log::info!("Logging... Current Number: {}", ctx.local.counter);
            
            *ctx.local.counter = ctx.local.counter.wrapping_add(1);
        }
    }

    #[task(binds = GPIO2_COMBINED_0_15, local = [radio])]
    fn receive_radio(ctx: receive_radio::Context) {
        log::info!("Radio Interrupt Tripped");
        // I know the radio will be reading from the first pipe
        while let Ok(Some(_)) = ctx.local.radio.can_read() {
            match ctx.local.radio.read() {
                Ok(packet) => log::info!("Received Packet: {:?}", packet.as_ref()),
                Err(err) => log::info!("Unable to Receive Packet: {:?}", err),
            }
        }
        log::info!("Radio Interrupt Done");
    }
}
