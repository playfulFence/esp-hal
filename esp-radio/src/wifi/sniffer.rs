//! Wi-Fi sniffer.

use core::marker::PhantomData;

use esp_sync::NonReentrantMutex;
use procmacros::BuilderLite;

use super::{RxControlInfo, SNIFFER_BIT, SecondaryChannel, release, try_acquire};
use crate::{
    WifiError,
    sys::include::{
        esp_wifi_80211_tx,
        esp_wifi_get_channel,
        esp_wifi_get_promiscuous,
        esp_wifi_get_promiscuous_filter,
        esp_wifi_set_channel,
        esp_wifi_set_promiscuous,
        esp_wifi_set_promiscuous_filter,
        esp_wifi_set_promiscuous_rx_cb,
        wifi_interface_t,
        wifi_interface_t_WIFI_IF_AP,
        wifi_interface_t_WIFI_IF_STA,
        wifi_pkt_rx_ctrl_t,
        wifi_promiscuous_filter_t,
        wifi_promiscuous_pkt_t,
        wifi_promiscuous_pkt_type_t,
    },
    wifi::esp_wifi_result,
};

/// Bitmask-based filter for promiscuous mode packet types.
///
/// Combine predefined constants or custom bitmasks to select which frames to
/// capture.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct PromiscuousFilter {
    /// Raw filter bitmask as accepted by `esp_wifi_set_promiscuous_filter`.
    pub mask: u32,
}

impl PromiscuousFilter {
    /// Capture all frame types.
    pub const ALL: Self = Self { mask: 0xFF };
    /// Capture management frames only.
    pub const MGMT: Self = Self { mask: 0x01 };
    /// Capture data frames only.
    pub const DATA: Self = Self { mask: 0x04 };
}

/// Configuration applied when creating a [`Sniffer`] instance.
///
/// All options are applied atomically at construction time to ensure the
/// correct ordering required by the underlying blob API.
#[derive(Clone, Debug, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct SnifferConfig {
    /// Packet type filter. `None` leaves the default.
    filter: Option<PromiscuousFilter>,

    /// Override the Wi-Fi channel. `None` keeps the current channel.
    channel: Option<u8>,

    /// Override the secondary channel. `None` keeps the current setting.
    secondary_channel: Option<SecondaryChannel>,

    /// Whether to enable promiscuous mode immediately. `None` leaves it off.
    promiscuous: Option<bool>,
}

impl Default for SnifferConfig {
    fn default() -> Self {
        Self {
            filter: None,
            channel: None,
            secondary_channel: None,
            promiscuous: None,
        }
    }
}

impl SnifferConfig {
    fn apply(&self) -> Result<(), WifiError> {
        if let Some(filter) = self.filter {
            esp_wifi_result!(unsafe {
                esp_wifi_set_promiscuous_filter(&wifi_promiscuous_filter_t { filter_mask: filter.mask })
            })?;
        }
        if let Some(channel) = self.channel {
            let secondary = self
                .secondary_channel
                .unwrap_or(SecondaryChannel::None)
                .to_raw();
            esp_wifi_result!(unsafe { esp_wifi_set_channel(channel, secondary) })?;
        }
        if let Some(enabled) = self.promiscuous {
            esp_wifi_result!(unsafe { esp_wifi_set_promiscuous(enabled) })?;
        }
        Ok(())
    }
}

/// Represents a Wi-Fi packet in promiscuous mode.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct PromiscuousPkt<'a> {
    /// Control information related to packet reception.
    pub rx_cntl: RxControlInfo,
    /// Frame type of the received packet.
    pub frame_type: wifi_promiscuous_pkt_type_t,
    /// Length of the received packet.
    pub len: usize,
    /// Data contained in the received packet.
    pub data: &'a [u8],
}

#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl PromiscuousPkt<'_> {
    /// # Safety
    ///
    /// When calling this, you have to ensure, that `buf` points to a valid
    /// [wifi_promiscuous_pkt_t].
    pub(crate) unsafe fn from_raw(
        buf: *const wifi_promiscuous_pkt_t,
        frame_type: wifi_promiscuous_pkt_type_t,
    ) -> Self {
        let rx_cntl = unsafe { RxControlInfo::from_raw(&(*buf).rx_ctrl) };
        let len = rx_cntl.sig_len as usize;
        PromiscuousPkt {
            rx_cntl,
            frame_type,
            len,
            data: unsafe {
                core::slice::from_raw_parts(
                    (buf as *const u8).add(core::mem::size_of::<wifi_pkt_rx_ctrl_t>()),
                    len,
                )
            },
        }
    }
}

static SNIFFER_CB: NonReentrantMutex<Option<fn(PromiscuousPkt<'_>)>> = NonReentrantMutex::new(None);

unsafe extern "C" fn promiscuous_rx_cb(buf: *mut core::ffi::c_void, frame_type: u32) {
    unsafe {
        if let Some(sniffer_callback) = SNIFFER_CB.with(|callback| *callback) {
            let promiscuous_pkt = PromiscuousPkt::from_raw(buf as *const _, frame_type);
            sniffer_callback(promiscuous_pkt);
        }
    }
}

/// A Wi-Fi sniffer.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub struct Sniffer<'d> {
    _phantom: PhantomData<&'d ()>,
}

impl Sniffer<'_> {
    pub(crate) fn new(config: SnifferConfig) -> Self {
        assert!(try_acquire(SNIFFER_BIT), "sniffer already in use");

        // If registering the callback fails we panic, so release the singleton
        // bit first so the panic doesn't leave the slot permanently occupied.
        let res =
            esp_wifi_result!(unsafe { esp_wifi_set_promiscuous_rx_cb(Some(promiscuous_rx_cb)) });
        if res.is_err() {
            release(SNIFFER_BIT);
            unwrap!(res);
        }

        config.apply().expect("sniffer config apply failed");

        Self {
            _phantom: PhantomData,
        }
    }

    /// Set promiscuous mode enabled or disabled.
    #[instability::unstable]
    pub fn set_promiscuous_mode(&self, enabled: bool) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_promiscuous(enabled) })?;
        Ok(())
    }

    /// Returns whether promiscuous mode is currently enabled.
    #[instability::unstable]
    pub fn promiscuous_mode(&self) -> Result<bool, WifiError> {
        let mut enabled = false;
        esp_wifi_result!(unsafe { esp_wifi_get_promiscuous(&mut enabled) })?;
        Ok(enabled)
    }

    /// Returns the currently configured promiscuous filter.
    #[instability::unstable]
    pub fn filter(&self) -> Result<PromiscuousFilter, WifiError> {
        let mut raw = wifi_promiscuous_filter_t { filter_mask: 0 };
        esp_wifi_result!(unsafe { esp_wifi_get_promiscuous_filter(&mut raw) })?;
        Ok(PromiscuousFilter { mask: raw.filter_mask })
    }

    /// Set the current Wi-Fi channel.
    #[instability::unstable]
    pub fn set_channel(
        &self,
        channel: u8,
        secondary: SecondaryChannel,
    ) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_channel(channel, secondary.to_raw()) })
    }

    /// Transmit a raw frame.
    #[instability::unstable]
    pub fn send_raw_frame(
        &mut self,
        use_sta_interface: bool,
        buffer: &[u8],
        use_internal_seq_num: bool,
    ) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe {
            esp_wifi_80211_tx(
                if use_sta_interface {
                    wifi_interface_t_WIFI_IF_STA
                } else {
                    wifi_interface_t_WIFI_IF_AP
                } as wifi_interface_t,
                buffer.as_ptr() as *const _,
                buffer.len() as i32,
                use_internal_seq_num,
            )
        })
    }

    /// Set the callback for receiving a packet.
    #[instability::unstable]
    pub fn set_receive_cb(&mut self, cb: fn(PromiscuousPkt<'_>)) {
        SNIFFER_CB.with(|callback| *callback = Some(cb));
    }
}

impl Drop for Sniffer<'_> {
    fn drop(&mut self) {
        // Clear the user callback first so the C trampoline becomes a no-op even
        // if it fires during the teardown window below.
        SNIFFER_CB.with(|callback| *callback = None);
        // Best-effort cleanup: log on failure but keep going so we still release
        // the singleton bit, otherwise a future Sniffer could never be created.
        if let Err(e) = esp_wifi_result!(unsafe { esp_wifi_set_promiscuous(false) }) {
            warn!(
                "Failed to disable promiscuous mode on sniffer drop: {:?}",
                e
            );
        }
        if let Err(e) = esp_wifi_result!(unsafe { esp_wifi_set_promiscuous_rx_cb(None) }) {
            warn!(
                "Failed to unregister promiscuous rx cb on sniffer drop: {:?}",
                e
            );
        }
        release(SNIFFER_BIT);
    }
}
