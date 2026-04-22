//! Circular DMA buffer types and traits.

use super::{
    super::{DescriptorSet, DmaDescriptor, RxCircularState, TransferDirection, TxCircularState},
    BufView,
    BurstConfig,
    DmaBufError,
    DmaRxBuffer,
    DmaTxBuffer,
    Preparation,
};
#[cfg(dma_can_access_psram)]
use crate::soc::is_valid_ram_address;

/// DMA circular transmit buffer.
///
/// Build with [`crate::dma_circular_buffers`] / [`crate::dma_circular_buffers_chunk_size`].
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaTxCircularBuf {
    descriptors: DescriptorSet<'static>,
    buffer: &'static mut [u8],
    burst: BurstConfig,
}

impl DmaTxCircularBuf {
    /// Creates a new [`DmaTxCircularBuf`] from descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer (same rules as
    /// [`crate::dma::DescriptorChain`] circular mode). Both must live in
    /// DMA-capable memory.
    pub fn new(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        Self::new_with_config(descriptors, buffer, BurstConfig::default())
    }

    /// Same as [`Self::new`] with an explicit [`BurstConfig`].
    pub fn new_with_config(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
        config: impl Into<BurstConfig>,
    ) -> Result<Self, DmaBufError> {
        let burst = config.into();
        let mut buf = Self {
            descriptors: DescriptorSet::new(descriptors)?,
            buffer,
            burst,
        };
        buf.descriptors
            .configure_tx_circular(buf.buffer, buf.burst)?;
        Ok(buf)
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'static mut [DmaDescriptor], &'static mut [u8]) {
        (self.descriptors.into_inner(), self.buffer)
    }

    /// Returns the size of the underlying buffer
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Returns the buf as a mutable slice than can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.buffer
    }

    /// Returns the buf as a slice than can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }
}

unsafe impl DmaTxBuffer for DmaTxCircularBuf {
    type View = BufView<DmaTxCircularBuf>;
    type Final = DmaTxCircularBuf;

    fn prepare(&mut self) -> Preparation {
        for desc in self.descriptors.linked_iter_mut() {
            desc.reset_for_tx(true);
        }

        cfg_if::cfg_if! {
            if #[cfg(dma_can_access_psram)] {
                let is_data_in_psram = !is_valid_ram_address(self.buffer.as_ptr() as usize);
                if is_data_in_psram {
                    unsafe {
                        crate::soc::cache_writeback_addr(
                            self.buffer.as_ptr() as u32,
                            self.buffer.len() as u32,
                        )
                    };
                }
            }
        }

        Preparation {
            start: self.descriptors.head(),
            direction: TransferDirection::Out,
            #[cfg(dma_can_access_psram)]
            accesses_psram: is_data_in_psram,
            burst_transfer: self.burst,
            check_owner: Some(false),
            auto_write_back: true,
        }
    }

    fn into_view(self) -> BufView<DmaTxCircularBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }

    fn tx_stream_state(&self) -> Option<TxCircularState> {
        let head = self.descriptors.head_ptr();
        let buffer_start = unsafe { (*head).buffer as *const u8 };
        let buffer_len: usize = self.descriptors.linked_iter().map(|d| d.len()).sum();
        Some(TxCircularState::new_from_ring(
            head,
            buffer_start,
            buffer_len,
        ))
    }
}

/// DMA circular receive buffer.
///
/// Build with [`crate::dma_circular_buffers`] / [`crate::dma_circular_buffers_chunk_size`].
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaRxCircularBuf {
    descriptors: DescriptorSet<'static>,
    buffer: &'static mut [u8],
    burst: BurstConfig,
}

impl DmaRxCircularBuf {
    /// Creates a new [`DmaRxCircularBuf`] from descriptors and a buffer.
    ///
    /// The buffer length must be a multiple of four. Descriptor count must match
    /// circular layout (see [`crate::dma_circular_descriptors`]).
    pub fn new(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        Self::new_with_config(descriptors, buffer, BurstConfig::default())
    }

    /// Same as [`Self::new`] with an explicit [`BurstConfig`].
    pub fn new_with_config(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
        config: impl Into<BurstConfig>,
    ) -> Result<Self, DmaBufError> {
        let burst = config.into();
        let mut buf = Self {
            descriptors: DescriptorSet::new(descriptors)?,
            buffer,
            burst,
        };
        buf.descriptors
            .configure_rx_circular(buf.buffer, buf.burst)?;
        Ok(buf)
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'static mut [DmaDescriptor], &'static mut [u8]) {
        (self.descriptors.into_inner(), self.buffer)
    }

    /// Returns the size of the underlying buffer
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Returns the buf as a mutable slice than can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.buffer
    }

    /// Returns the buf as a slice than can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }
}

unsafe impl DmaRxBuffer for DmaRxCircularBuf {
    type View = BufView<DmaRxCircularBuf>;
    type Final = DmaRxCircularBuf;

    fn prepare(&mut self) -> Preparation {
        for desc in self.descriptors.linked_iter_mut() {
            desc.reset_for_rx();
        }

        cfg_if::cfg_if! {
            if #[cfg(dma_can_access_psram)] {
                let is_data_in_psram = !is_valid_ram_address(self.buffer.as_ptr() as usize);
                if is_data_in_psram {
                    unsafe {
                        crate::soc::cache_invalidate_addr(
                            self.buffer.as_ptr() as u32,
                            self.buffer.len() as u32,
                        )
                    };
                }
            }
        }

        Preparation {
            start: self.descriptors.head(),
            direction: TransferDirection::In,
            #[cfg(dma_can_access_psram)]
            accesses_psram: is_data_in_psram,
            burst_transfer: self.burst,
            check_owner: Some(false),
            auto_write_back: true,
        }
    }

    fn into_view(self) -> BufView<DmaRxCircularBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }

    fn rx_stream_state(&self) -> Option<RxCircularState> {
        Some(RxCircularState::new_from_ring(
            self.descriptors.head_ptr(),
            self.descriptors.tail_ptr(),
        ))
    }

    fn peripheral_rx_dma_length(&self) -> usize {
        self.buffer.len()
    }
}
