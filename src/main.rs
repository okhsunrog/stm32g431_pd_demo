#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use defmt::Format;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    peripherals,
    ucpd::{self, CcPhy, CcPull, CcSel, CcVState, Ucpd},
};
use embassy_time::{with_timeout, Duration, Timer};
use fmt::{error, info};

bind_interrupts!(struct Irqs {
    UCPD1 => ucpd::InterruptHandler<peripherals::UCPD1>;
});

#[derive(Debug, Format)]
enum CableOrientation {
    Normal,
    Flipped,
    DebugAccessoryMode,
}

// Returns true when the cable
async fn wait_attached<T: ucpd::Instance>(cc_phy: &mut CcPhy<'_, T>) -> CableOrientation {
    loop {
        let (cc1, cc2) = cc_phy.vstate();
        if cc1 == CcVState::LOWEST && cc2 == CcVState::LOWEST {
            // Detached, wait until attached by monitoring the CC lines.
            cc_phy.wait_for_vstate_change().await;
            continue;
        }

        // Attached, wait for CC lines to be stable for tCCDebounce (100..200ms).
        if with_timeout(Duration::from_millis(100), cc_phy.wait_for_vstate_change())
            .await
            .is_ok()
        {
            // State has changed, restart detection procedure.
            continue;
        };

        // State was stable for the complete debounce period, check orientation.
        return match (cc1, cc2) {
            (_, CcVState::LOWEST) => CableOrientation::Normal, // CC1 connected
            (CcVState::LOWEST, _) => CableOrientation::Flipped, // CC2 connected
            _ => CableOrientation::DebugAccessoryMode,         // Both connected (special cable)
        };
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut stm32_config = embassy_stm32::Config::default();
    {
        // HSE + PLL fot 170 MHz
        use embassy_stm32::rcc::{
            Hse, HseMode, LsConfig, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk, PllPDiv, PllQDiv
        };
        use embassy_stm32::time::mhz;
        stm32_config.rcc.hsi = true;
        stm32_config.rcc.hse = Some(Hse {
            freq: mhz(8),
            mode: HseMode::Oscillator,
        });
        stm32_config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL85,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV2),
            divr: Some(PllRDiv::DIV2),
        });
        stm32_config.rcc.boost = true;
        stm32_config.rcc.ls = LsConfig::default_lse();
        stm32_config.rcc.sys = Sysclk::PLL1_R;
        // stm32_config.enable_ucpd1_dead_battery = true;
    }
    let p = embassy_stm32::init(stm32_config);
    let led = Output::new(p.PC6, Level::High, Speed::Low);
    let mut ucpd = Ucpd::new(p.UCPD1, Irqs {}, p.PB6, p.PB4, Default::default());
    ucpd.cc_phy().set_pull(CcPull::Sink);
    spawner.spawn(blink_led(led)).unwrap();

    info!("Waiting for USB connection...");
    let cable_orientation = wait_attached(ucpd.cc_phy()).await;
    info!("USB cable connected, orientation: {}", cable_orientation);

    let cc_sel = match cable_orientation {
        CableOrientation::Normal => {
            info!("Starting PD communication on CC1 pin");
            CcSel::CC1
        }
        CableOrientation::Flipped => {
            info!("Starting PD communication on CC2 pin");
            CcSel::CC2
        }
        CableOrientation::DebugAccessoryMode => panic!("No PD communication in DAM"),
    };
    let (_cc_phy, mut pd_phy) = ucpd.split_pd_phy(p.DMA1_CH1, p.DMA1_CH2, cc_sel);

    loop {
        // Enough space for the longest non-extended data message.
        let mut buf = [0_u8; 30];
        match pd_phy.receive(buf.as_mut()).await {
            Ok(n) => info!("USB PD RX: {=[u8]:?}", &buf[..n]),
            Err(e) => error!("USB PD RX: {}", e),
        }
    }
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
