use crate::{
    otg_fs::USB,
    peripherals,
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

use core::cell::RefCell;
use core::marker::PhantomData;
use critical_section::Mutex;

use embassy_usb_driver::{
    Bus, ControlPipe, Driver as embassy_driver, Endpoint, EndpointAddress, EndpointAllocError,
    EndpointError, EndpointIn, EndpointInfo, EndpointOut, EndpointType, Event, Unsupported,
};
use esp_synopsys_usb_otg::{
    endpoint::{
        set_stalled, Endpoint as SynopsysEndpoint, EndpointIn as SynopsysEndpointIn,
        EndpointOut as SynopsysEndpointOut,
    },
    endpoint_memory::EndpointBuffer,
    ral::{endpoint_in, endpoint_out, modify_reg, read_reg},
    target::{fifo_write, UsbRegisters},
    transition::EndpointDescriptor,
    UsbBus, UsbPeripheral,
};
use usb_device::{
    endpoint::{EndpointAddress as SynopsysEndpointAddr, EndpointType as SynopsysEndpointType},
    UsbDirection,
};

const NEW_AW: AtomicWaker = AtomicWaker::new();

use core::future::poll_fn;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;
use embassy_sync::waitqueue::AtomicWaker;
static BUS_WAKER: AtomicWaker = NEW_AW;
static IRQ_RESUME: AtomicBool = AtomicBool::new(false);
static IRQ_RESET: AtomicBool = AtomicBool::new(false);
static IRQ_SUSPEND: AtomicBool = AtomicBool::new(false);

const EP_COUNT: usize = 5;
static EP0_SETUP: AtomicBool = AtomicBool::new(false);
static EP_IN_WAKERS: [AtomicWaker; EP_COUNT] = [NEW_AW; EP_COUNT];
static EP_OUT_WAKERS: [AtomicWaker; EP_COUNT] = [NEW_AW; EP_COUNT];

pub struct UsbBusWrapper<USB> {
    pub bus: UsbBus<USB>,
    pub inited: bool,
}

pub struct EndpointOutWrapper {
    ep_out: SynopsysEndpointOut,
    info: EndpointInfo,
}

pub struct EndpointInWrapper {
    ep_in: SynopsysEndpointIn,
    info: EndpointInfo,
}

impl<USB: UsbPeripheral> Bus for UsbBusWrapper<USB> {
    async fn enable(&mut self) {
        self.bus.enable();
    }

    async fn disable(&mut self) {}

    async fn poll(&mut self) -> Event {
        poll_fn(move |cx| {
            BUS_WAKER.register(cx.waker());
            // TODO: implement VBUS detection.
            if !self.inited {
                self.inited = true;
                return Poll::Ready(Event::PowerDetected);
            }

            let regs = unsafe { &*crate::peripherals::USB0::PTR };

            if IRQ_RESUME.load(Ordering::Acquire) {
                IRQ_RESUME.store(false, Ordering::Relaxed);
                return Poll::Ready(Event::Resume);
            }

            if IRQ_RESET.load(Ordering::Acquire) {
                IRQ_RESET.store(false, Ordering::Relaxed);

                // regs.dcfg.devaddr

                self.bus.reset();

                // trace!("RESET");

                // TODO: EDIT EPS

                //     regs.epr(0).write(|w| {
                //         w.set_ep_type(EpType::CONTROL);
                //         w.set_stat_rx(Stat::NAK);
                //         w.set_stat_tx(Stat::NAK);
                //     });

                //     for i in 1..USB::ENDPOINT_COUNT {
                //         regs.epr(i).write(|w| {
                //             w.set_ea(i as _);
                //             w.set_ep_type(self.ep_types[i - 1]);
                //         })
                //     }
                for w in &EP_IN_WAKERS {
                    w.wake()
                }
                for w in &EP_OUT_WAKERS {
                    w.wake()
                }

                return Poll::Ready(Event::Reset);
            }

            if IRQ_SUSPEND.load(Ordering::Acquire) {
                IRQ_SUSPEND.store(false, Ordering::Relaxed);
                return Poll::Ready(Event::Suspend);
            }

            Poll::Pending
        })
        .await
    }

    /// Enable or disable an endpoint.
    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {}

    /// Set or clear the STALL condition for an endpoint.
    ///
    /// If the endpoint is an OUT endpoint, it should be prepared to receive data again.
    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        self.bus
            .set_stalled(SynopsysEndpointAddr::from(u8::from(ep_addr)), stalled);
    }

    /// Get whether the STALL condition is set for an endpoint.
    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        self.bus
            .is_stalled(SynopsysEndpointAddr::from(u8::from(ep_addr)))
    }

    /// Simulate a disconnect from the USB bus, causing the host to reset and re-enumerate the
    /// device.
    ///
    /// The default implementation just returns `Unsupported`.
    ///
    /// # Errors
    ///
    /// * [`Unsupported`](crate::Unsupported) - This UsbBus implementation doesn't support
    ///   simulating a disconnect or it has not been enabled at creation time.
    fn force_reset(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }

    /// Initiate a remote wakeup of the host by the device.
    ///
    /// # Errors
    ///
    /// * [`Unsupported`](crate::Unsupported) - This UsbBus implementation doesn't support
    ///   remote wakeup or it has not been enabled at creation time.
    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

impl Endpoint for EndpointOutWrapper {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let index = self.info().addr.index();
        let regs = self.ep_out.usb.endpoint_out(index as usize);
        poll_fn(|cx| {
            EP_OUT_WAKERS[index].register(cx.waker());
            if read_reg!(endpoint_out, regs, DOEPCTL, EPENA) == 0 {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    }
}

impl EndpointOut for EndpointOutWrapper {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let index = self.info().addr.index();
        poll_fn(|cx| {
            // Check if there is data available to read
            if !critical_section::with(|cs| self.ep_out.buffer.borrow(cs).borrow_mut().has_data) {
                // If there is no data, we need to wait, so we'll register the waker
                // and return Poll::Pending.
                EP_OUT_WAKERS[index].register(cx.waker());
                return Poll::Pending;
            }
            let data_size =
                critical_section::with(|cs| self.ep_out.buffer.borrow(cs).borrow_mut().data_size)
                    as usize;

            if buf.len() < data_size {
                return Poll::Ready(Err(EndpointError::BufferOverflow));
            }

            let mut read = 0;
            while read < data_size {
                let word = critical_section::with(|cs| {
                    self.ep_out.buffer.borrow(cs).borrow().buffer[read / 4].get()
                });
                let bytes = word.to_ne_bytes();
                let end = (read + 4).min(data_size);
                buf[read..end].copy_from_slice(&bytes[..end - read]);
                read += 4;
            }

            critical_section::with(|cs| {
                self.ep_out.buffer.borrow(cs).borrow_mut().has_data = false
            });
            // Return Poll::Ready with the result of the read operation.
            Poll::Ready(Ok(data_size))
        })
        .await
    }
}

impl Endpoint for EndpointInWrapper {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let index = self.info().addr.index();
        let regs = self.ep_in.usb.endpoint_in(index as usize);
        poll_fn(|cx| {
            EP_OUT_WAKERS[index].register(cx.waker());
            if read_reg!(endpoint_in, regs, DIEPCTL, EPENA) == 0 {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    }
}

impl EndpointIn for EndpointInWrapper {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        let index = self.info().addr.index();
        poll_fn(|cx| {
            let ep = critical_section::with(|_cs| {
                self.ep_in.usb.endpoint_in(self.ep_in.index() as usize)
            });

            if self.ep_in.index() != 0 && read_reg!(endpoint_in, ep, DIEPCTL, EPENA) != 0 {
                EP_OUT_WAKERS[index].register(cx.waker());
                return Poll::Pending;
            }

            if buf.len() > self.ep_in.descriptor.max_packet_size as usize {
                return Poll::Ready(Err(EndpointError::BufferOverflow));
            }

            if !buf.is_empty() {
                // Check for FIFO free space
                let size_words = (buf.len() + 3) / 4;
                if size_words > read_reg!(endpoint_in, ep, DTXFSTS, INEPTFSAV) as usize {
                    EP_OUT_WAKERS[index].register(cx.waker());
                    return Poll::Pending;
                }
            }

            #[cfg(feature = "fs")]
            write_reg!(endpoint_in, ep, DIEPTSIZ, PKTCNT: 1, XFRSIZ: buf.len() as u32);
            #[cfg(feature = "hs")]
            write_reg!(endpoint_in, ep, DIEPTSIZ, MCNT: 1, PKTCNT: 1, XFRSIZ: buf.len() as u32);

            modify_reg!(endpoint_in, ep, DIEPCTL, CNAK: 1, EPENA: 1);

            fifo_write(self.ep_in.usb, self.ep_in.index(), buf);
            Poll::Ready(Ok(()))
        })
        .await
    }
}

pub struct ControlPipeWrapper<USB> {
    max_packet_size: u16,
    ep_in: EndpointInWrapper,
    ep_out: EndpointOutWrapper,
    bus: UsbBusWrapper<USB>,
}

impl<USB: UsbPeripheral> ControlPipe for ControlPipeWrapper<USB> {
    fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    async fn setup(&mut self) -> [u8; 8] {
        loop {
            poll_fn(|cx| {
                EP_OUT_WAKERS[0].register(cx.waker());
                if EP0_SETUP.load(Ordering::Relaxed) {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;

            let word = critical_section::with(|cs| {
                self.ep_out.ep_out.buffer.borrow(cs).borrow().buffer[0].get()
            });
            let bytes = word.to_ne_bytes();

            let buf: [u8; 8] = [
                bytes[0], bytes[1], bytes[2], bytes[3], 0, 0, 0,
                0, // Zero-extend the value to 8 bytes
            ];

            critical_section::with(|cs| {
                self.ep_out.ep_out.buffer.borrow(cs).borrow_mut().has_data = false
            });

            EP0_SETUP.store(false, Ordering::Relaxed);

            return buf;
        }
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        _first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        Ok(self.ep_out.read(buf).await?)
    }

    /// Send a DATA IN packet with `data` in response to a control read request.
    ///
    /// If `last_packet` is true, the STATUS packet will be ACKed following the transfer of `data`.
    async fn data_in(
        &mut self,
        data: &[u8],
        _first: bool,
        last: bool,
    ) -> Result<(), EndpointError> {
        self.ep_in.write(data).await?;

        if last {
            self.ep_out.read(&mut []).await?;
        }

        Ok(())
    }

    /// Accept a control request.
    ///
    /// Causes the STATUS packet for the current request to be ACKed.
    async fn accept(&mut self) {
        self.ep_in.write(&[]).await.ok();
    }

    /// Reject a control request.
    ///
    /// Sets a STALL condition on the pipe to indicate an error.
    async fn reject(&mut self) {
        set_stalled(self.ep_in.ep_in.usb, self.ep_in.ep_in.address(), true);
        set_stalled(self.ep_out.ep_out.usb, self.ep_out.ep_out.address(), true);
    }

    /// Accept SET_ADDRESS control and change bus address.
    ///
    /// For most drivers this function should firstly call `accept()` and then change the bus address.
    /// However, there are peripherals (Synopsys USB OTG) that have reverse order.
    async fn accept_set_address(&mut self, addr: u8) {
        self.bus.bus.set_device_address(addr);

        self.accept().await;
    }
}

pub struct Driver<USB> {
    // phantom: PhantomData<USB>,
    alloc: [EndpointData; EP_COUNT],
    bus: UsbBusWrapper<USB>,
    // ep_mem_free: u16,
}

impl<'a, USB: UsbPeripheral + 'a> embassy_driver<'a> for Driver<USB> {
    type EndpointOut = EndpointOutWrapper;
    type EndpointIn = EndpointInWrapper;
    type ControlPipe = ControlPipeWrapper<USB>;
    type Bus = UsbBusWrapper<USB>;

    ///pub struct EndpointOutWrapper {
    // ep_out: SynopsysEndpointOut,
    // info: EndpointInfo,
    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        let addr = self
            .bus
            .bus
            .alloc_ep(
                UsbDirection::Out,
                None,
                match ep_type {
                    EndpointType::Control => SynopsysEndpointType::Control,
                    EndpointType::Isochronous => SynopsysEndpointType::Isochronous,
                    EndpointType::Bulk => SynopsysEndpointType::Bulk,
                    EndpointType::Interrupt => SynopsysEndpointType::Interrupt,
                },
                max_packet_size,
                interval_ms,
            )
            .map_err(|_e| EndpointAllocError)?;

        Ok(EndpointOutWrapper {
            ep_out: SynopsysEndpointOut {
                common: SynopsysEndpoint {
                    descriptor: EndpointDescriptor {
                        address: addr,
                        ep_type: match ep_type {
                            EndpointType::Control => SynopsysEndpointType::Control,
                            EndpointType::Isochronous => SynopsysEndpointType::Isochronous,
                            EndpointType::Bulk => SynopsysEndpointType::Bulk,
                            EndpointType::Interrupt => SynopsysEndpointType::Interrupt,
                        },
                        max_packet_size: max_packet_size,
                        interval: interval_ms,
                    },
                    usb: UsbRegisters::new::<USB>(),
                },
                buffer: Mutex::new(RefCell::new(EndpointBuffer::new(&mut []))),
            },
            info: EndpointInfo {
                addr: EndpointAddress::from(u8::from(addr)),
                ep_type: ep_type,
                max_packet_size,
                interval_ms,
            },
        })
    }

    /// Allocates an IN endpoint.
    ///
    /// This method is called by the USB stack to allocate endpoints.
    /// It can only be called before [`start`](Self::start) is called.
    ///
    /// # Arguments
    /// * `ep_type` - the endpoint's type.
    /// * `max_packet_size` - Maximum packet size in bytes.
    /// * `interval_ms` - Polling interval parameter for interrupt endpoints.
    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        let addr = self
            .bus
            .bus
            .alloc_ep(
                UsbDirection::In,
                None,
                match ep_type {
                    EndpointType::Control => SynopsysEndpointType::Control,
                    EndpointType::Isochronous => SynopsysEndpointType::Isochronous,
                    EndpointType::Bulk => SynopsysEndpointType::Bulk,
                    EndpointType::Interrupt => SynopsysEndpointType::Interrupt,
                },
                max_packet_size,
                interval_ms,
            )
            .map_err(|_e| EndpointAllocError)?;

        Ok(EndpointInWrapper {
            ep_in: SynopsysEndpointIn {
                common: SynopsysEndpoint {
                    descriptor: EndpointDescriptor {
                        address: addr,
                        ep_type: match ep_type {
                            EndpointType::Control => SynopsysEndpointType::Control,
                            EndpointType::Isochronous => SynopsysEndpointType::Isochronous,
                            EndpointType::Bulk => SynopsysEndpointType::Bulk,
                            EndpointType::Interrupt => SynopsysEndpointType::Interrupt,
                        },
                        max_packet_size: max_packet_size,
                        interval: interval_ms,
                    },
                    usb: UsbRegisters::new::<USB>(),
                },
            },
            info: EndpointInfo {
                addr: EndpointAddress::from(u8::from(addr)),
                ep_type: ep_type,
                max_packet_size,
                interval_ms,
            },
        })
    }

    /// Start operation of the USB device.
    ///
    /// This returns the `Bus` and `ControlPipe` instances that are used to operate
    /// the USB device. Additionally, this makes all the previously allocated endpoints
    /// start operating.
    ///
    /// This consumes the `Driver` instance, so it's no longer possible to allocate more
    /// endpoints.
    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let ep_out_addr = self.bus.bus.alloc_ep(
            UsbDirection::Out,
            None,
            EndpointType::Control,
            max_packet_size,
            interval_ms,
        );

        let ep_in_addr = self.bus.bus.alloc_ep(
            UsbDirection::In,
            None,
            EndpointType::Control,
            max_packet_size,
            interval_ms,
        );

        let mut ep_types = [EpType::BULK; EP_COUNT - 1];
        for i in 1..EP_COUNT {
            ep_types[i - 1] = convert_type(self.alloc[i].ep_type);
        }

        (
            Bus {
                phantom: PhantomData,
                inited: false,
            },
            ControlPipe {
                _phantom: PhantomData,
                max_packet_size: control_max_packet_size,
                ep_out,
                ep_in,
            },
        )
    }
}
