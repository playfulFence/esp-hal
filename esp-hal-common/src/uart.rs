//! UART driver

use core::marker::PhantomData;

use fugit::{HertzU32, MicrosDurationU32};
use paste::paste;

use self::config::Config;
#[cfg(any(esp32, esp32s3))]
use crate::pac::UART2;
use crate::{
    clock::Clocks,
    gpio::types::{InputSignal, OutputSignal},
    pac::uart0::{fifo::FIFO_SPEC, RegisterBlock},
    peripheral::{Peripheral, PeripheralRef},
    InputPin,
    OutputPin,
};

const UART_FIFO_SIZE: u16 = 128;

/// Custom serial error type
#[derive(Debug)]
pub enum Error {}

#[cfg(feature = "eh1")]
impl embedded_hal_1::serial::Error for Error {
    fn kind(&self) -> embedded_hal_1::serial::ErrorKind {
        embedded_hal_1::serial::ErrorKind::Other
    }
}

/// UART configuration
pub mod config {
    /// Number of data bits
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum DataBits {
        DataBits5 = 0,
        DataBits6 = 1,
        DataBits7 = 2,
        DataBits8 = 3,
    }

    impl From<u8> for DataBits {
        fn from(value: u8) -> DataBits {
            // This field is only 2 bits wide
            match value & 0x3 {
                0 => DataBits::DataBits5,
                1 => DataBits::DataBits6,
                2 => DataBits::DataBits7,
                3 => DataBits::DataBits8,
                _ => unreachable!(),
            }
        }
    }

    /// Parity check
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum Parity {
        ParityNone,
        ParityEven,
        ParityOdd,
    }

    impl From<u8> for Parity {
        fn from(value: u8) -> Parity {
            // We check two bits:
            //   [1] UART_PARITY_EN
            //   [0] UART_PARITY
            let enable = value & 0x2;
            let parity = value & 0x1;

            match (enable, parity) {
                (1, 0) => Parity::ParityEven,
                (1, 1) => Parity::ParityOdd,
                _ => Parity::ParityNone,
            }
        }
    }

    /// Number of stop bits
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum StopBits {
        /// 1 stop bit
        STOP1   = 1,
        /// 1.5 stop bits
        STOP1P5 = 2,
        /// 2 stop bits
        STOP2   = 3,
    }

    impl From<u8> for StopBits {
        fn from (value: u8) -> StopBits {
            // This field is only 2 bits wide
            match value & 0x3 {
                1 => StopBits::STOP1,
                2 => StopBits::STOP1P5,
                3 => StopBits::STOP2,
                _ => unreachable!(),
            }
        }
    }

    /// UART configuration
    #[derive(Debug, Copy, Clone)]
    pub struct Config {
        pub baudrate: u32,
        pub data_bits: DataBits,
        pub parity: Parity,
        pub stop_bits: StopBits,
    }

    impl Config {
        pub fn baudrate(mut self, baudrate: u32) -> Self {
            self.baudrate = baudrate;
            self
        }

        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }

        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }

        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }

        pub fn data_bits(mut self, data_bits: DataBits) -> Self {
            self.data_bits = data_bits;
            self
        }

        pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
            self.stop_bits = stop_bits;
            self
        }
    }

    impl Default for Config {
        fn default() -> Config {
            Config {
                baudrate: 115_200,
                data_bits: DataBits::DataBits8,
                parity: Parity::ParityNone,
                stop_bits: StopBits::STOP1,
            }
        }
    }

    /// Configuration for the AT-CMD detection functionality
    pub struct AtCmdConfig {
        pub pre_idle_count: Option<u16>,
        pub post_idle_count: Option<u16>,
        pub gap_timeout: Option<u16>,
        pub cmd_char: u8,
        pub char_num: Option<u8>,
    }

    impl AtCmdConfig {
        pub fn new(
            pre_idle_count: Option<u16>,
            post_idle_count: Option<u16>,
            gap_timeout: Option<u16>,
            cmd_char: u8,
            char_num: Option<u8>,
        ) -> AtCmdConfig {
            Self {
                pre_idle_count,
                post_idle_count,
                gap_timeout,
                cmd_char,
                char_num,
            }
        }
    }
}

pub trait Uart {
    fn port() -> u8;

    fn register_block(&self) -> &RegisterBlock;

    fn disable_tx_interrupts(&mut self) {
        self.register_block().int_clr.write(|w| {
            w.txfifo_empty_int_clr()
                .set_bit()
                .tx_brk_done_int_clr()
                .set_bit()
                .tx_brk_idle_done_int_clr()
                .set_bit()
                .tx_done_int_clr()
                .set_bit()
        });

        self.register_block().int_ena.write(|w| {
            w.txfifo_empty_int_ena()
                .clear_bit()
                .tx_brk_done_int_ena()
                .clear_bit()
                .tx_brk_idle_done_int_ena()
                .clear_bit()
                .tx_done_int_ena()
                .clear_bit()
        });
    }

    fn disable_rx_interrupts(&mut self) {
        self.register_block().int_clr.write(|w| {
            w.rxfifo_full_int_clr()
                .set_bit()
                .rxfifo_ovf_int_clr()
                .set_bit()
                .rxfifo_tout_int_clr()
                .set_bit()
        });

        self.register_block().int_ena.write(|w| {
            w.rxfifo_full_int_ena()
                .clear_bit()
                .rxfifo_ovf_int_ena()
                .clear_bit()
                .rxfifo_tout_int_ena()
                .clear_bit()
        });
    }

    fn get_tx_fifo_count(&mut self) -> u16 {
        self.register_block()
            .status
            .read()
            .txfifo_cnt()
            .bits()
            .into()
    }

    fn get_rx_fifo_count(&mut self) -> u16 {
        self.register_block()
            .status
            .read()
            .rxfifo_cnt()
            .bits()
            .into()
    }

    fn is_tx_idle(&self) -> bool {
        #[cfg(esp32)]
        let idle = self.register_block().status.read().st_utx_out().bits() == 0x0u8;
        #[cfg(not(esp32))]
        let idle = self.register_block().fsm_status.read().st_utx_out().bits() == 0x0u8;

        idle
    }

    fn is_rx_idle(&self) -> bool {
        #[cfg(esp32)]
        let idle = self.register_block().status.read().st_urx_out().bits() == 0x0u8;
        #[cfg(not(esp32))]
        let idle = self.register_block().fsm_status.read().st_urx_out().bits() == 0x0u8;

        idle
    }

    fn tx_signal(&self) -> OutputSignal;

    fn rx_signal(&self) -> InputSignal;

    fn cts_signal(&self) -> InputSignal;

    fn rts_signal(&self) -> OutputSignal;
}

/// Serial abstraction
pub struct UartDriver<'d, UART>
where
    UART: Uart,
{
    uart: PeripheralRef<'d, UART>,
    rx: UartRxDriver<'d, UART>,
    tx: UartTxDriver<'d, UART>,
}

/// Serial receiver
pub struct UartRxDriver<'d, UART>
where
    UART: Uart,
{
    uart: PhantomData<&'d UART>,
}

/// Serial transmitter
pub struct UartTxDriver<'d, UART>
where
    UART: Uart,
{
    uart: PhantomData<&'d UART>,
}

impl<'d, UART> UartDriver<'d, UART>
where
    UART: Uart,
{
    /// Create a new serial driver
    pub fn new(
        uart: impl Peripheral<P = UART> + 'd,
        tx: impl Peripheral<P = impl OutputPin> + 'd,
        rx: impl Peripheral<P = impl InputPin> + 'd,
        cts: Option<impl Peripheral<P = impl InputPin> + 'd>,
        rts: Option<impl Peripheral<P = impl OutputPin> + 'd>,
    ) -> Result<Self, Error> {
        Self::new_with_config(uart, tx, rx, cts, rts, &Config::default())
    }

    /// Create a new serial driver with the specified configuration
    pub fn new_with_config(
        uart: impl Peripheral<P = UART> + 'd,
        tx: impl Peripheral<P = impl OutputPin> + 'd,
        rx: impl Peripheral<P = impl InputPin> + 'd,
        cts: Option<impl Peripheral<P = impl InputPin> + 'd>,
        rts: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        config: &Config,
    ) -> Result<Self, Error> {
        crate::into_ref!(uart, tx, rx);

        let cts = cts.map(|cts| cts.into_ref());
        let rts = rts.map(|rts| rts.into_ref());

        let mut uart = uart;
        uart.disable_rx_interrupts();
        uart.disable_tx_interrupts();

        // TODO: configure pins

        let mut me = Self {
            uart: uart,
            rx: UartRxDriver { uart: PhantomData },
            tx: UartTxDriver { uart: PhantomData },
        };

        me.change_data_bits(config.data_bits)?;
        me.change_parity(config.parity)?;
        me.change_stop_bits(config.stop_bits)?;
        // me.change_baudrate(config.baudrate); // FIXME

        Ok(me)
    }

    /// Change the number of stop bits
    pub fn change_stop_bits(&mut self, stop_bits: config::StopBits) -> Result<&mut Self, Error> {
        let reg_block = self.uart.register_block();

        // Workaround for hardware issue, when UART stop bit set as 2-bit mode
        #[cfg(esp32)]
        if stop_bits == config::StopBits::STOP2 {
            reg_block.rs485_conf.modify(|_, w| w.dl1_en().bit(true));
            reg_block
                .conf0
                .modify(|_, w| unsafe { w.stop_bit_num().bits(1) });
        } else {
            reg_block.rs485_conf.modify(|_, w| w.dl1_en().bit(false));
            reg_block
                .conf0
                .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });
        }

        #[cfg(not(esp32))]
        reg_block
            .conf0
            .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });

        Ok(self)
    }

    /// Retruns the current number of stop bits
    pub fn stop_bits(&self) -> Result<config::StopBits, Error> {
        let stop_bits = self.uart
            .register_block()
            .conf0
            .read()
            .stop_bit_num()
            .bits();
        let stop_bits = config::StopBits::from(stop_bits);

        Ok(stop_bits)
    }

    /// Change the number of data bits
    pub fn change_data_bits(&mut self, data_bits: config::DataBits) -> Result<&mut Self, Error> {
        self.uart
            .register_block()
            .conf0
            .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });

        Ok(self)
    }

    /// Return the current number of data bits
    pub fn data_bits(&self) -> Result<config::DataBits, Error> {
        let data_bits = self.uart.register_block().conf0.read().bit_num().bits();
        let data_bits = config::DataBits::from(data_bits);

        Ok(data_bits)
    }

    /// Change the type of parity checking
    pub fn change_parity(&mut self, parity: config::Parity) -> Result<&mut Self, Error> {
        use config::Parity::*;

        self.uart
            .register_block()
            .conf0
            .modify(|_, w| match parity {
                ParityNone => w.parity_en().clear_bit(),
                ParityEven => w.parity_en().set_bit().parity().clear_bit(),
                ParityOdd => w.parity_en().set_bit().parity().set_bit(),
            });

        Ok(self)
    }

    /// Returns the current type of parity checking
    pub fn parity(&self) -> Result<config::Parity, Error> {
        let enable = self.uart.register_block().conf0.read().parity_en().bit();
        let parity = self.uart.register_block().conf0.read().parity().bit();
        let parity = config::Parity::from(enable as u8 | parity as u8);

        Ok(parity)
    }

    #[cfg(not(any(esp32c3, esp32s3)))]
    /// Change the baudrate.
    ///
    /// Will automatically select the clock source. When possible the reference
    /// clock (1MHz) will be used, because this is constant when the clock
    /// source/frequency changes. However if one of the clock frequencies is
    /// below 10MHz or if the baudrate is above the reference clock or if
    /// the baudrate cannot be set within 1.5% then use the APB clock.
    pub fn change_baudrate<T>(&mut self, clocks: &Clocks, baudrate: T) -> Result<&mut Self, Error>
    where
        T: Into<HertzU32> + Copy,
    {
        let baudrate = baudrate.into();

        // We force the clock source to be APB and don't use the decimal part of the
        // divider
        let clk = clocks.apb_clock.to_Hz();

        self.uart
            .register_block()
            .conf0
            .modify(|_, w| w.tick_ref_always_on().bit(true));
        let divider = clk / baudrate.raw();

        self.uart
            .register_block()
            .clkdiv
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });

        Ok(self)
    }

<<<<<<< Updated upstream:esp-hal-common/src/serial.rs
    #[cfg(any(esp32c2, esp32c3, esp32s3))]
    fn change_baud(&self, baudrate: u32, clocks: &Clocks) {
        // we force the clock source to be APB and don't use the decimal part of the
=======
    #[cfg(any(esp32c3, esp32s3))]
    /// Change the baudrate.
    ///
    /// Will automatically select the clock source. When possible the reference
    /// clock (1MHz) will be used, because this is constant when the clock
    /// source/frequency changes. However if one of the clock frequencies is
    /// below 10MHz or if the baudrate is above the reference clock or if
    /// the baudrate cannot be set within 1.5% then use the APB clock.
    pub fn change_baudrate<T>(&mut self, clocks: &Clocks, baudrate: T) -> Result<&mut Self, Error>
    where
        T: Into<HertzU32> + Copy,
    {
        let baudrate = baudrate.into();

        // We force the clock source to be APB and don't use the decimal part of the
>>>>>>> Stashed changes:esp-hal-common/src/uart.rs
        // divider
        let clk = clocks.apb_clock.to_Hz();
        let max_div = 0b1111_1111_1111 - 1;
        let clk_div = ((clk) + (max_div * baudrate.raw()) - 1) / (max_div * baudrate.raw());

        self.uart.register_block().clk_conf.write(|w| unsafe {
            w.sclk_sel()
                .bits(1) // APB
                .sclk_div_a()
                .bits(0)
                .sclk_div_b()
                .bits(0)
                .sclk_div_num()
                .bits(clk_div as u8 - 1)
                .rx_sclk_en()
                .bit(true)
                .tx_sclk_en()
                .bit(true)
        });

        let clk = clk / clk_div;
        let divider = clk / baudrate.raw();
        let divider = divider as u16;

        self.uart
            .register_block()
            .clkdiv
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });

        Ok(self)
    }

    /// Returns the current baudrate
    pub fn baudrate(&self) -> Result<HertzU32, Error> {
        // let mut baudrate: u32 = 0;
        // esp_result!(
        //     unsafe { uart_get_baudrate(UART::port(), &mut baudrate) },
        //     baudrate.into()
        // )
        todo!()
    }

    /// Split the serial driver in separate TX and RX drivers
    pub fn split(&mut self) -> (&mut UartTxDriver<'d, UART>, &mut UartRxDriver<'d, UART>) {
        (&mut self.tx, &mut self.rx)
    }

    /// Read multiple bytes into a slice
    pub fn read<T>(&mut self, buf: &mut [u8], delay: T) -> Result<usize, Error>
    where
        T: Into<MicrosDurationU32>,
    {
        self.rx.read(buf, delay.into().ticks())
    }

    /// Write multiple bytes from a slice
    pub fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
        self.tx.write(buf)
    }

    pub fn flush_read(&mut self) -> Result<(), Error> {
        self.rx.flush()
    }

    pub fn flush_write(&mut self) -> Result<(), Error> {
        self.tx.flush()
    }
}

impl<'d, UART> Drop for UartDriver<'d, UART>
where
    UART: Uart,
{
    fn drop(&mut self) {
        // esp!(unsafe { uart_driver_delete(UART::port()) }).unwrap();
        todo!()
    }
}

unsafe impl<'d, UART> Send for UartDriver<'d, UART> where UART: Uart {}

#[cfg(feature = "eh1")]
impl<'d, UART> embedded_hal_1::serial::ErrorType for UartDriver<'d, UART>
where
    UART: Uart,
{
    type Error = Error;
}

impl<'d, UART> embedded_hal::serial::Read<u8> for UartDriver<'d, UART>
where
    UART: Uart,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        embedded_hal::serial::Read::read(&mut self.rx)
    }
}

#[cfg(feature = "eh1")]
impl<'d, UART> embedded_hal_nb::serial::Read<u8> for UartDriver<'d, UART>
where
    UART: Uart,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        embedded_hal_nb::serial::Read::read(&mut self.rx)
    }
}

impl<'d, UART> embedded_hal::serial::Write<u8> for UartDriver<'d, UART>
where
    UART: Uart,
{
    type Error = Error;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        embedded_hal::serial::Write::flush(&mut self.tx)
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        embedded_hal::serial::Write::write(&mut self.tx, byte)
    }
}

#[cfg(feature = "eh1")]
impl<'d, UART> embedded_hal_nb::serial::Write<u8> for UartDriver<'d, UART>
where
    UART: Uart,
{
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::flush(&mut self.tx)
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::write(&mut self.tx, byte)
    }
}

impl<'d, UART> core::fmt::Write for UartDriver<'d, UART>
where
    UART: Uart,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}

#[cfg(feature = "eh1")]
impl<'d, UART> embedded_hal_1::serial::ErrorType for UartRxDriver<'d, UART>
where
    UART: Uart,
{
    type Error = Error;
}

impl<'d, UART> UartRxDriver<'d, UART>
where
    UART: Uart,
{
    /// Get count of bytes in the receive FIFO
    pub fn count(&self) -> Result<u8, Error> {
        // let mut size = 0_u32;
        // esp_result!(
        //     unsafe { uart_get_buffered_data_len(UART::port(), &mut size) },
        //     size as u8
        // )
        todo!()
    }

    /// Read multiple bytes into a slice; block until specified timeout
    pub fn read(&mut self, buf: &mut [u8], delay: u32) -> Result<usize, Error> {
        // FIXME: what should `TickType_t` be?
        // // uart_read_bytes() returns error (-1) or how many bytes were read out
        // // 0 means timeout and nothing is yet read out
        // let len = unsafe {
        //     uart_read_bytes(
        //         UART::port(),
        //         buf.as_mut_ptr() as *mut _,
        //         buf.len() as u32,
        //         delay,
        //     )
        // };
        //
        // if len >= 0 {
        //     Ok(len as usize)
        // } else {
        //     Err(Error::from(ESP_ERR_INVALID_STATE).unwrap())
        // }
        todo!()
    }

    pub fn flush(&self) -> Result<(), Error> {
        // esp!(unsafe { uart_flush_input(UART::port()) })?;
        //
        // Ok(())
        todo!()
    }
}

impl<'d, UART> embedded_hal::serial::Read<u8> for UartRxDriver<'d, UART>
where
    UART: Uart,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        // let mut buf = [0_u8];

        // let result = self.read(&mut buf, NON_BLOCK);

        // check_nb(result, buf[0])
        todo!()
    }
}

#[cfg(feature = "eh1")]
impl<'d, UART> embedded_hal_nb::serial::Read<u8> for UartRxDriver<'d, UART>
where
    UART: Uart,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        // let mut buf = [0_u8];

        // let result = self.read(&mut buf, NON_BLOCK);

        // check_nb(result, buf[0])
        todo!()
    }
}

impl<'d, UART> UartTxDriver<'d, UART>
where
    UART: Uart,
{
    /// Write multiple bytes from a slice
    pub fn write(&mut self, bytes: &[u8]) -> Result<usize, Error> {
        // // `uart_write_bytes()` returns error (-1) or how many bytes were written
        // let len = unsafe {
        //     uart_write_bytes(UART::port(), bytes.as_ptr() as *const _, bytes.len() as
        // u32) };

        // if len >= 0 {
        //     Ok(len as usize)
        // } else {
        //     Err(Error::from(ESP_ERR_INVALID_STATE).unwrap())
        // }
        todo!()
    }

    pub fn flush(&mut self) -> Result<(), Error> {
        // esp!(unsafe { uart_wait_tx_done(UART::port(), 0) })?;

        // Ok(())
        todo!()
    }
}

#[cfg(feature = "eh1")]
impl<'d, UART> embedded_hal_1::serial::ErrorType for UartTxDriver<'d, UART>
where
    UART: Uart,
{
    type Error = Error;
}

impl<'d, UART> embedded_hal::serial::Write<u8> for UartTxDriver<'d, UART>
where
    UART: Uart,
{
    type Error = Error;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // UartTxDriver::flush(self).map_err(to_nb_err)
        todo!()
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        // check_nb(self.write(&[byte]), ())
        todo!()
    }
}

#[cfg(feature = "eh1")]
impl<'d, UART> embedded_hal_nb::serial::Write<u8> for UartTxDriver<'d, UART>
where
    UART: Uart,
{
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // UartTxDriver::flush(self).map_err(to_nb_err)
        todo!()
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        // check_nb(self.write(&[byte]), ())
        todo!()
    }
}

impl<'d, UART> core::fmt::Write for UartTxDriver<'d, UART>
where
    UART: Uart,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let buf = s.as_bytes();
        let mut offset = 0;

        while offset < buf.len() {
            offset += self.write(buf).map_err(|_| core::fmt::Error)?
        }

        Ok(())
    }
}

macro_rules! impl_uart {
    ($uart:ident, $peripheral:ty, $port:expr) => {
        crate::impl_peripheral!($uart);

        impl Uart for $uart {
            fn port() -> u8 {
                $port
            }

            #[inline(always)]
            fn register_block(&self) -> &RegisterBlock {
                unsafe { &*(<$peripheral>::PTR) }
            }

            fn tx_signal(&self) -> OutputSignal {
                paste! { OutputSignal::[<U $port TXD>] }
            }

            fn rx_signal(&self) -> InputSignal {
                paste! { InputSignal::[<U $port RXD>] }
            }

            fn cts_signal(&self) -> InputSignal {
                paste! { InputSignal::[<U $port CTS>] }
            }

            fn rts_signal(&self) -> OutputSignal {
                paste! { OutputSignal::[<U $port RTS>] }
            }
        }
    };
}

impl_uart!(Uart0, crate::pac::UART0, 0);
impl_uart!(Uart1, crate::pac::UART1, 1);
#[cfg(any(esp32, esp32s3))]
impl_uart!(Uart2, crate::pac::UART2, 2);
