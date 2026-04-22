//! Circular DMA buffer types and traits.

use super::{
    super::{DescriptorSet, DmaDescriptor, RxCircularState, TransferDirection, TxCircularState},
    BufView,
    BurstConfig,
    DmaBufError,
    Preparation,
};
#[cfg(dma_can_access_psram)]
use crate::soc::is_valid_ram_address;
/// [DmaTxCircularBuffer] is a DMA descriptor ring and memory for continuous
/// transmit (circular) transfers.
///
/// # Safety
///
/// The implementing type must keep all its descriptors and the buffers they
/// point to valid while the buffer is being transferred.
pub unsafe trait DmaTxCircularBuffer {
    /// A type providing operations that are safe to perform on the buffer
    /// whilst the DMA is actively using it.
    type View;

    /// The type returned to the user when a transfer finishes.
    type Final;

    /// Prepares the buffer for an imminent transfer and returns
    /// information required to use this buffer.
    fn prepare(&mut self) -> Preparation;

    /// This is called before the DMA starts using the buffer.
    fn into_view(self) -> Self::View;

    /// This is called after the DMA is done using the buffer.
    fn from_view(view: Self::View) -> Self::Final;

    /// Snapshot for [`TxCircularState`] after descriptors are configured.
    fn tx_circular_state(&self) -> TxCircularState;
}

/// [DmaRxCircularBuffer] is a DMA descriptor ring and memory for continuous
/// receive (circular) transfers.
///
/// # Safety
///
/// The implementing type must keep all its descriptors and the buffers they
/// point to valid while the buffer is being transferred.
pub unsafe trait DmaRxCircularBuffer {
    /// A type providing operations that are safe to perform on the buffer
    /// whilst the DMA is actively using it.
    type View;

    /// The type returned to the user when a transfer finishes.
    type Final;

    /// Prepares the buffer for an imminent transfer and returns
    /// information required to use this buffer.
    fn prepare(&mut self) -> Preparation;

    /// This is called before the DMA starts using the buffer.
    fn into_view(self) -> Self::View;

    /// This is called after the DMA is done using the buffer.
    fn from_view(view: Self::View) -> Self::Final;

    /// Snapshot for [`RxCircularState`] after descriptors are configured.
    fn rx_circular_state(&self) -> RxCircularState;

    /// Total ring size in bytes (passed to the peripheral as the RX DMA length).
    fn ring_byte_len(&self) -> usize;
}

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

unsafe impl DmaTxCircularBuffer for DmaTxCircularBuf {
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

    fn tx_circular_state(&self) -> TxCircularState {
        let head = self.descriptors.head_ptr();
        let buffer_start = unsafe { (*head).buffer as *const u8 };
        let buffer_len: usize = self.descriptors.linked_iter().map(|d| d.len()).sum();
        TxCircularState::new_from_ring(head, buffer_start, buffer_len)
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

unsafe impl DmaRxCircularBuffer for DmaRxCircularBuf {
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

    fn rx_circular_state(&self) -> RxCircularState {
        RxCircularState::new_from_ring(self.descriptors.head_ptr(), self.descriptors.tail_ptr())
    }

    fn ring_byte_len(&self) -> usize {
        self.buffer.len()
    }
}
