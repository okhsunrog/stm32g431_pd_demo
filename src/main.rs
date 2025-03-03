#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut stm32_config = embassy_stm32::Config::default();
    {
        // HSE + PLL fot 80 MHz
        use embassy_stm32::rcc::{
            Hse, HseMode, LsConfig, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk,
        };
        use embassy_stm32::time::mhz;
        stm32_config.rcc.hsi = false;
        stm32_config.rcc.hse = Some(Hse {
            freq: mhz(8),
            mode: HseMode::Oscillator,
        });
        stm32_config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL20,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
        stm32_config.rcc.ls = LsConfig::default_lse();
        stm32_config.rcc.sys = Sysclk::PLL1_R;
    }
    let p = embassy_stm32::init(stm32_config);
    let mut led = Output::new(p.PC6, Level::High, Speed::Low);

    loop {
        info!("Hello, World!");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
