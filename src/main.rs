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
    use hal::iomuxc::pads::{gpio_b0::{GPIO_B0_11, GPIO_B0_02, GPIO_B0_01, GPIO_B0_03, GPIO_B0_00}, gpio_b1::GPIO_B1_00};
    use hal::gpt;

    use imxrt_ral as ral;
    use ral::lpspi::LPSPI4;

    use teensy4_bsp as bsp;
    use bsp::board;

    use crate::motion_control::MotionControlCommand;

    use embedded_nrf24l01::{Rx, NRF24L01, ChangeModes};
    use embedded_nrf24l01::config::{NRF24L01Config, InterruptMask, RetransmitConfig, CrcMode, DataRate, PALevel, NRF24L01Configuration};

    const GPT_CLOCK_SOURCE: gpt::ClockSource = gpt::ClockSource::PeripheralClock;
    const GPT_DIVIDER: u32 = 8;
    const GPT_FREQUENCY: u32 = board::PERCLK_FREQUENCY / GPT_DIVIDER;

    const GPT_DELAY_MS: u32 = GPT_FREQUENCY / 1_000 * 250;
    const OCR: hal::gpt::OutputCompareRegister = hal::gpt::OutputCompareRegister::OCR3;

    #[local]
    struct Local {
        radio: NRF24L01<'static, core::convert::Infallible, Output<GPIO_B0_11>, Output<GPIO_B1_00>, Lpspi<Pins<GPIO_B0_02, GPIO_B0_01, GPIO_B0_03, GPIO_B0_00>, 4>>,
        gpt1: hal::gpt::Gpt1,
        counter: u32,
    }

    #[shared]
    struct Shared {
        _latest_motion_control_command: MotionControlCommand,
    }

    fn init_gpt<const N: u8>(gpt: &mut hal::gpt::Gpt<N>) {
        gpt.set_clock_source(GPT_CLOCK_SOURCE);
        gpt.set_divider(GPT_DIVIDER);
        gpt.set_output_compare_count(OCR, GPT_DELAY_MS);
        gpt.set_mode(hal::gpt::Mode::Restart);
        gpt.set_reset_on_enable(true);
        gpt.set_output_interrupt_on_compare(OCR, true);
    }

    #[init]
    fn init(_ctx: init::Context) -> (Shared, Local) {
        let board_resources = board::t41(board::instances());

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
        gpio2.set_interrupt(&interrupt_pin, Some(Trigger::Low));

        let nrf_config = NRF24L01Config {
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
        let mut radio = NRF24L01::new_with_config(radio_ce, radio_cs, radio_spi, nrf_config).unwrap();
        match radio.is_connected() {
            Err(err) => panic!("Error Received for Radio Connection: {:?}", err),
            Ok(false) => panic!("Radio is not connected"),
            _ => log::info!("Radio Connected"),
        }
        if let Err(err) = radio.to_rx() {
            log::info!("Unknown Error Occured Switching to Rx: {:?}", err);
        }

        let mut gpt1 = board_resources.gpt1;
        init_gpt(&mut gpt1);
        gpt1.enable();

        (
            Shared {
                _latest_motion_control_command: MotionControlCommand::new(),
            },
            Local {
                radio,
                gpt1,
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

    #[task(binds = GPIO2_COMBINED_0_15, local = [radio])]
    fn radio_interrupt(ctx: radio_interrupt::Context) {
        if let Ok(Some(pipe)) = ctx.local.radio.can_read() {
            let mut read_enabled_pipes = [false; 6];
            read_enabled_pipes[pipe as usize] = true;
            if let Ok(_) = ctx.local.radio.set_read_enabled_pipes(&read_enabled_pipes) {
                match ctx.local.radio.read() {
                    Ok(payload) => {
                        log::info!("Obtained Data: {:?}", payload.as_ref());
                    },
                    Err(err) => {
                        log::info!("Failed to obtain radio data: {:?}", err);
                    }
                }
            }
        }
    }

    #[task(binds = GPT1, local = [gpt1, counter])]
    fn gpt_interrupt(ctx: gpt_interrupt::Context) {
        *ctx.local.counter = ctx.local.counter.wrapping_add(1);

        log::info!("Counting Up!!! Now At: {}", *ctx.local.counter);
    }
}
