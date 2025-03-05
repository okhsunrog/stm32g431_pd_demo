#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    rcc::{
        Hse, HseMode, LsConfig, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllRDiv, PllSource,
        Sysclk,
    },
    time::mhz,
};
use embassy_time::{Duration, Timer};
use fmt::unwrap;
use stm32g431_pd_demo::power::{self, UcpdResources};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut stm32_config = embassy_stm32::Config::default();
    // HSI must be enabled for UCPD
    stm32_config.rcc.hsi = true;
    stm32_config.rcc.hse = Some(Hse {
        freq: mhz(8),
        mode: HseMode::Oscillator,
    });
    stm32_config.rcc.pll = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV2,
        mul: PllMul::MUL85, // 170 MHz
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV2),
        divr: Some(PllRDiv::DIV2),
    });
    stm32_config.rcc.boost = true;
    stm32_config.rcc.sys = Sysclk::PLL1_R;
    // connect DBx pins to CCx for dead battery feature and enable this:
    // stm32_config.enable_ucpd1_dead_battery = true;

    let p = embassy_stm32::init(stm32_config);
    let led = Output::new(p.PC6, Level::High, Speed::Low);
    spawner.spawn(blink_led(led)).unwrap();

    let ucpd_resources = UcpdResources {
        pin_cc1: p.PB6,
        pin_cc2: p.PB4,
        ucpd: p.UCPD1,
        rx_dma: p.DMA1_CH1,
        tx_dma: p.DMA1_CH2,
    };
    unwrap!(spawner.spawn(power::ucpd_task(ucpd_resources)))
}

#[embassy_executor::task]
async fn blink_led(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
