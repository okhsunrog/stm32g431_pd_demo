//! Handles USB PD negotiation.
use defmt::{Format, info, warn};
use embassy_futures::select::{Either, select};
use embassy_stm32::ucpd::{self, CcPhy, CcPull, CcSel, CcVState, PdPhy, Ucpd};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::{Duration, Timer, with_timeout};
use usbpd::Driver as SinkDriver;
use usbpd::sink::device_policy_manager::DevicePolicyManager;
use usbpd::sink::policy_engine::Sink;
use usbpd::timers::Timer as SinkTimer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    UCPD1 => ucpd::InterruptHandler<peripherals::UCPD1>;
});

pub struct UcpdResources {
    pub ucpd: peripherals::UCPD1,
    pub pin_cc1: peripherals::PB6,
    pub pin_cc2: peripherals::PB4,
    pub rx_dma: peripherals::DMA1_CH1,
    pub tx_dma: peripherals::DMA1_CH2,
}

#[derive(Debug, Format)]
enum CableOrientation {
    Normal,
    Flipped,
    DebugAccessoryMode,
}

struct UcpdSinkDriver<'d> {
    /// The UCPD PD phy instance.
    pd_phy: PdPhy<'d, peripherals::UCPD1>,
}

impl<'d> UcpdSinkDriver<'d> {
    fn new(pd_phy: PdPhy<'d, peripherals::UCPD1>) -> Self {
        Self { pd_phy }
    }
}

impl SinkDriver for UcpdSinkDriver<'_> {
    async fn wait_for_vbus(&self) {
        // The sink policy engine is only running when attached. Therefore VBus is present.
    }

    async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, usbpd::DriverRxError> {
        self.pd_phy.receive(buffer).await.map_err(|err| match err {
            ucpd::RxError::Crc | ucpd::RxError::Overrun => usbpd::DriverRxError::Discarded,
            ucpd::RxError::HardReset => usbpd::DriverRxError::HardReset,
        })
    }

    async fn transmit(&mut self, data: &[u8]) -> Result<(), usbpd::DriverTxError> {
        self.pd_phy.transmit(data).await.map_err(|err| match err {
            ucpd::TxError::Discarded => usbpd::DriverTxError::Discarded,
            ucpd::TxError::HardReset => usbpd::DriverTxError::HardReset,
        })
    }

    async fn transmit_hard_reset(&mut self) -> Result<(), usbpd::DriverTxError> {
        self.pd_phy
            .transmit_hardreset()
            .await
            .map_err(|err| match err {
                ucpd::TxError::Discarded => usbpd::DriverTxError::Discarded,
                ucpd::TxError::HardReset => usbpd::DriverTxError::HardReset,
            })
    }
}

async fn wait_detached<T: ucpd::Instance>(cc_phy: &mut CcPhy<'_, T>) {
    loop {
        let (cc1, cc2) = cc_phy.vstate();
        if cc1 == CcVState::LOWEST && cc2 == CcVState::LOWEST {
            return;
        }
        cc_phy.wait_for_vstate_change().await;
    }
}

// Returns true when the cable was attached.
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

struct EmbassySinkTimer {}

impl SinkTimer for EmbassySinkTimer {
    async fn after_millis(milliseconds: u64) {
        Timer::after_millis(milliseconds).await
    }
}

struct Device {}

impl DevicePolicyManager for Device {}

/// Handle USB PD negotiation.
#[embassy_executor::task]
pub async fn ucpd_task(mut ucpd_resources: UcpdResources) {
    loop {
        let mut ucpd = Ucpd::new(
            &mut ucpd_resources.ucpd,
            Irqs {},
            &mut ucpd_resources.pin_cc1,
            &mut ucpd_resources.pin_cc2,
            Default::default(),
        );

        ucpd.cc_phy().set_pull(CcPull::Sink);

        info!("Waiting for USB connection");
        let cable_orientation = wait_attached(ucpd.cc_phy()).await;
        info!("USB cable attached, orientation: {}", cable_orientation);

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
        let (mut cc_phy, pd_phy) = ucpd.split_pd_phy(
            &mut ucpd_resources.rx_dma,
            &mut ucpd_resources.tx_dma,
            cc_sel,
        );

        let driver = UcpdSinkDriver::new(pd_phy);
        let mut sink: Sink<UcpdSinkDriver<'_>, EmbassySinkTimer, _> = Sink::new(driver, Device {});
        info!("Run sink");

        match select(sink.run(), wait_detached(&mut cc_phy)).await {
            Either::First(result) => warn!("Sink loop broken with result: {}", result),
            Either::Second(_) => {
                info!("Detached");
                continue;
            }
        }
    }
}
