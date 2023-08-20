//! This code is a PoC for using the Teensy 4.1 with the nRF24L01

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use teensy4_panic as _;

mod motion_control;

mod rgb_led;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [DCP])]
mod app {
    use core::convert::Infallible;

    use teensy4_bsp as bsp;
    use bsp::board;

    use bsp::hal as hal;
    use hal::lpspi::{Lpspi, LpspiError, Pins};
    use hal::gpio::{Trigger, Output, Input};
    use hal::iomuxc::pads::{gpio_b0::{GPIO_B0_00, GPIO_B0_10, GPIO_B0_02, GPIO_B0_01, GPIO_B0_03, GPIO_B0_11}, gpio_b1::GPIO_B1_00, gpio_ad_b0::{GPIO_AD_B0_03, GPIO_AD_B0_02}, gpio_emc::GPIO_EMC_04};
    use hal::timer::Blocking;
    use hal::gpt::{ClockSource, Gpt};
    use hal::ccm::analog::pll2;

    use bsp::ral as ral;
    use ral::lpspi::LPSPI4;
    use teensy4_bsp::hal::dma::channel;

    use crate::motion_control::MotionControlCommand;

    // use embedded_nrf24l01::{Rx, NRF24L01, ChangeModes};
    // use embedded_nrf24l01::config::{NRF24L01Config, InterruptMask, RetransmitConfig, CrcMode, DataRate, PALevel};

    use rtic_monotonics::systick::{Systick, ExtU32};

    use crate::rgb_led::RGBLed;

    const GPT1_FREQUENCY: u32 = 1_000;
    const GPT1_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
    const GPT1_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT1_FREQUENCY;

    #[local]
    struct Local {
        // radio: NRF24L01<'static, Infallible, Output<GPIO_B1_00>, Output<GPIO_B0_00>, Lpspi<(), 4>>,
        radio_interrupt_pin: Input<GPIO_B0_10>,
    }

    #[shared]
    struct Shared {
        _latest_motion_control_command: MotionControlCommand,
        counter: u32,
        rgb_led: RGBLed<Infallible, Output<GPIO_AD_B0_03>, Output<GPIO_AD_B0_02>, Output<GPIO_EMC_04>>,
        // radio: Result<nrf24l01::NRF24L01<Lpspi<(), 4>, Output<GPIO_B0_11>, Output<GPIO_B1_00>>, nrf24l01::Error<LpspiError>>,
        err: Result<(), nrf24l01::Error<LpspiError>>,
        delay: Blocking::<Gpt<1>, GPT1_FREQUENCY>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let board_resources: board::Resources<you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::pins::t41::Pins> = board::t41(ctx.device);
        let usb = board_resources.usb;
        let pins = board_resources.pins;
        let mut gpio1 = board_resources.gpio1;
        let mut gpio2 = board_resources.gpio2;
        let mut gpio4 = board_resources.gpio4;
        let mut gpt1 = board_resources.gpt1;
        let spi4 = board_resources.lpspi4;

        // Configure USB Logger
        bsp::LoggingFrontend::default_log().register_usb(usb);

        // Configure RGB LED
        let red = gpio1.output(pins.p0);
        let green = gpio1.output(pins.p1);
        let blue = gpio4.output(pins.p2);
        let rgb_led = RGBLed::new(red, green, blue);

        // Configure gpt1 for blocking delays
        gpt1.disable();
        gpt1.set_divider(GPT1_DIVIDER);
        gpt1.set_clock_source(GPT1_CLOCK_SOURCE);
        let mut delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);

        let mut radio_spi = hal::lpspi::Lpspi::without_pins(spi4);
        radio_spi.disabled(|spi| {
            spi.set_clock_hz(board::LPSPI_FREQUENCY, 1_000_000);
            spi.set_mode(nrf24l01::MODE);
        });

        let ce = gpio2.output(pins.p8);
        let csn = gpio2.output(pins.p9);

        let mut radio = match nrf24l01::NRF24L01::new(radio_spi, csn, ce, 0u8, 5u8) {
            Ok(radio) => radio,
            Err(err) => panic!("Error Occurred: {:?}", err),
        };
        let res = radio.set_raddr(b"aaa");
        // radio.config().unwrap();

        let radio_interrupt_pin = gpio2.input(pins.p6);
        gpio2.set_interrupt(&radio_interrupt_pin, Some(Trigger::RisingEdge));

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 36_000_000, systick_token);

        red_led_task::spawn().unwrap();

        (
            Shared {
                _latest_motion_control_command: MotionControlCommand::new(),
                counter: 0u32,
                rgb_led,
                delay,
                err: res,
            },
            Local {
                radio_interrupt_pin,
            }
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1, shared = [counter, rgb_led, err])]
    async fn red_led_task(mut ctx: red_led_task::Context) {
        loop {
            ctx.shared.rgb_led.lock(|rgb_led| {
                rgb_led.red().unwrap();
            });

            ctx.shared.counter.lock(|counter| {
                log::info!("Current Number: {}", counter);

                *counter = counter.wrapping_add(1);
            });

            Systick::delay(ExtU32::millis(10_000u32)).await;

            ctx.shared.rgb_led.lock(|rgb_led| {
                rgb_led.off().unwrap();
            });

            ctx.shared.counter.lock(|counter| {
                log::info!("Current Number: {}", counter);

                *counter = counter.wrapping_add(1);
            });

            Systick::delay(ExtU32::millis(10_000u32)).await;

            ctx.shared.err.lock(|err| {
                if let Err(err) = err {
                    log::info!("Error Occurred: {:?}", err);
                }
            });
        }
    }

    // #[task(binds = GPIO2_COMBINED_0_15, local = [radio, radio_interrupt_pin], shared = [rgb_led])]
    // fn receive_radio(mut ctx: receive_radio::Context) {
    //     ctx.shared.rgb_led.lock(|rgb_led| {
    //         rgb_led.green().unwrap();
    //     });
    //     if ctx.local.radio_interrupt_pin.is_triggered() {
    //         log::info!("Radio Interrupt Tripped");
    //         let mut buffer = [0; b"Hello".len()];
    //         ctx.local.radio.read(&mut buffer).unwrap();
    //         log::info!("Received Data: {:?}", buffer);
    //         log::info!("Radio Interrupt Done");
    //         ctx.local.radio_interrupt_pin.clear_triggered();
    //     } else {
    //         log::info!("Unknown Interrupt Tripped");
    //     }
    // }
}
