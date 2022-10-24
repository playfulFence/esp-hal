use core::{convert::Infallible, marker::PhantomData};

pub use self::chip::*;
use crate::peripheral::{Peripheral, PeripheralRef};

mod ll;
pub mod signals;

/// A trait implemented by every pin instance
pub trait Pin: Peripheral<P = Self> + Sized + Send + 'static {
    fn pin(&self) -> u8;
}

/// A marker trait designating a pin which is capable of operating as an input
/// pin
pub trait InputPin: Pin + Into<AnyInputPin> {
    fn downgrade_input(self) -> AnyInputPin {
        self.into()
    }
}

/// A marker trait designating a pin which is capable of operating as an output
/// pin
pub trait OutputPin: Pin + Into<AnyOutputPin> {
    fn downgrade_output(self) -> AnyOutputPin {
        self.into()
    }
}

/// A marker trait designating a pin which is capable of operating as an input
/// and output pin
pub trait IOPin: InputPin + OutputPin + Into<AnyIOPin> {
    fn downgrade(self) -> AnyIOPin {
        self.into()
    }
}

/// A marker trait designating a pin which is capable of operating as an RTC pin
pub trait RTCPin: Pin {
    fn rtc_pin(&self) -> u8;
}

pub(crate) mod sealed {
    pub trait ADCPin {
        // NOTE: Will likely disappear in subsequent versions, once ADC support
        //       pops up in e-hal1. Hence sealed
        const CHANNEL: u8;
    }
}

/// A marker trait designating a pin which is capable of operating as an ADC pin
pub trait ADCPin: sealed::ADCPin + Pin {
    // type Adc: Adc; // FIXME

    fn adc_channel(&self) -> u8 {
        Self::CHANNEL
    }
}

/// A marker trait designating a pin which is capable of operating as a DAC pin
#[cfg(not(any(esp32c2, esp32c3, esp32s3)))]
pub trait DACPin: Pin {
    fn dac_channel(&self) -> u8;
}

/// A marker trait designating a pin which is capable of operating as a touch
/// pin
#[cfg(not(any(esp32c2, esp32c3)))]
pub trait TouchPin: Pin {
    fn touch_channel(&self) -> u8;
}

/// Generic Gpio input-output pin
pub struct AnyIOPin {
    pin: u8,
    _p: PhantomData<*const ()>,
}

impl AnyIOPin {
    /// # Safety
    ///
    /// Care should be taken not to instantiate this Pin, if it is already
    /// instantiated and used elsewhere, or if it is not set already in the mode
    /// of operation which is being instantiated
    pub unsafe fn new(pin: u8) -> Self {
        Self {
            pin,
            _p: PhantomData,
        }
    }
}

crate::impl_peripheral_trait!(AnyIOPin);

impl Pin for AnyIOPin {
    fn pin(&self) -> u8 {
        self.pin
    }
}

impl InputPin for AnyIOPin {}

impl OutputPin for AnyIOPin {}

impl IOPin for AnyIOPin {}

/// Generic Gpio input pin
pub struct AnyInputPin {
    pin: u8,
    _p: PhantomData<*const ()>,
}

impl AnyInputPin {
    /// # Safety
    ///
    /// Care should be taken not to instantiate this Pin, if it is already
    /// instantiated and used elsewhere, or if it is not set already in the mode
    /// of operation which is being instantiated
    pub unsafe fn new(pin: u8) -> Self {
        Self {
            pin,
            _p: PhantomData,
        }
    }
}

crate::impl_peripheral_trait!(AnyInputPin);

impl Pin for AnyInputPin {
    fn pin(&self) -> u8 {
        self.pin
    }
}

impl InputPin for AnyInputPin {}

impl From<AnyIOPin> for AnyInputPin {
    fn from(pin: AnyIOPin) -> Self {
        unsafe { Self::new(pin.pin()) }
    }
}

/// Generic Gpio output pin
pub struct AnyOutputPin {
    pin: u8,
    _p: PhantomData<*const ()>,
}

impl AnyOutputPin {
    /// # Safety
    ///
    /// Care should be taken not to instantiate this Pin, if it is already
    /// instantiated and used elsewhere, or if it is not set already in the mode
    /// of operation which is being instantiated
    pub unsafe fn new(pin: u8) -> Self {
        Self {
            pin,
            _p: PhantomData,
        }
    }
}

crate::impl_peripheral_trait!(AnyOutputPin);

impl Pin for AnyOutputPin {
    fn pin(&self) -> u8 {
        self.pin
    }
}

impl OutputPin for AnyOutputPin {}

impl From<AnyIOPin> for AnyOutputPin {
    fn from(pin: AnyIOPin) -> Self {
        unsafe { Self::new(pin.pin()) }
    }
}

/// Interrupt types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptType {
    PosEdge,
    NegEdge,
    AnyEdge,
    LowLevel,
    HighLevel,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum GpioMode {
    Disabled,
    Input,
    InputOutput,
    InputOutputOpenDrain,
    Output,
    OutputOpenDrain,
}

impl GpioMode {
    pub fn is_input(&self) -> bool {
        use GpioMode::*;
        matches!(self, Input | InputOutput | InputOutputOpenDrain)
    }

    pub fn is_output(&self) -> bool {
        use GpioMode::*;
        matches!(
            self,
            InputOutput | InputOutputOpenDrain | Output | OutputOpenDrain
        )
    }

    pub fn is_open_drain(&self) -> bool {
        use GpioMode::*;
        matches!(self, InputOutputOpenDrain | OutputOpenDrain)
    }
}

/// Drive strength (values are approximates)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveStrength {
    I5mA  = 0,
    I10mA = 1,
    I20mA = 2,
    I40mA = 3,
}

impl From<u8> for DriveStrength {
    fn from(value: u8) -> DriveStrength {
        match value & 0x3 {
            0 => DriveStrength::I5mA,
            1 => DriveStrength::I10mA,
            2 => DriveStrength::I20mA,
            3 => DriveStrength::I40mA,
            _ => unreachable!(),
        }
    }
}

// Pull setting for an input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pull {
    Floating,
    Up,
    Down,
    UpDown,
}

/// Digital input or output level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Low,
    High,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<Level> for bool {
    fn from(val: Level) -> bool {
        match val {
            Level::Low => false,
            Level::High => true,
        }
    }
}

impl core::ops::Not for Level {
    type Output = Level;

    fn not(self) -> Self::Output {
        match self {
            Level::Low => Level::High,
            Level::High => Level::Low,
        }
    }
}

pub trait InputMode {
    const RTC: bool;
}

pub trait OutputMode {
    const RTC: bool;
}

pub struct Disabled;

pub struct Input;

pub struct Output;

pub struct InputOutput;

#[cfg(not(any(esp32c2, esp32c3)))]
pub struct RtcDisabled;

#[cfg(not(any(esp32c2, esp32c3)))]
pub struct RtcInput;

#[cfg(not(any(esp32c2, esp32c3)))]
pub struct RtcOutput;

#[cfg(not(any(esp32c2, esp32c3)))]
pub struct RtcInputOutput;

impl InputMode for Input {
    const RTC: bool = false;
}

impl InputMode for InputOutput {
    const RTC: bool = false;
}

impl OutputMode for Output {
    const RTC: bool = false;
}

impl OutputMode for InputOutput {
    const RTC: bool = false;
}

#[cfg(not(any(esp32c2, esp32c3)))]
impl InputMode for RtcInput {
    const RTC: bool = true;
}

#[cfg(not(any(esp32c2, esp32c3)))]
impl InputMode for RtcInputOutput {
    const RTC: bool = true;
}

#[cfg(not(any(esp32c2, esp32c3)))]
impl OutputMode for RtcOutput {
    const RTC: bool = true;
}

#[cfg(not(any(esp32c2, esp32c3)))]
impl OutputMode for RtcInputOutput {
    const RTC: bool = true;
}

/// A driver for a GPIO pin.
///
/// The driver can set the pin as a disconnected/disabled one, input, or output
/// pin, or both or analog. On some chips (i.e. esp32 and esp32s*), the driver
/// can also set the pin in RTC IO mode. Depending on the current operating
/// mode, different sets of functions are available.
///
/// The mode-setting depends on the capabilities of the pin as well, i.e.
/// input-only pins cannot be set into output or input-output mode.
pub struct PinDriver<'d, T: Pin, MODE> {
    pin: PeripheralRef<'d, T>,
    _mode: PhantomData<MODE>,
}

impl<'d, T: Pin> PinDriver<'d, T, Disabled> {
    /// Creates the driver for a pin in disabled state.
    #[inline]
    pub fn disabled(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_disabled()
    }
}

impl<'d, T: InputPin> PinDriver<'d, T, Input> {
    /// Creates the driver for a pin in input state.
    #[inline]
    pub fn input(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_input()
    }
}

impl<'d, T: InputPin + OutputPin> PinDriver<'d, T, InputOutput> {
    /// Creates the driver for a pin in input-output state.
    #[inline]
    pub fn input_output(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_input_output()
    }
}

impl<'d, T: InputPin + OutputPin> PinDriver<'d, T, InputOutput> {
    /// Creates the driver for a pin in input-output open-drain state.
    #[inline]
    pub fn input_output_od(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_input_output_od()
    }
}

impl<'d, T: OutputPin> PinDriver<'d, T, Output> {
    /// Creates the driver for a pin in output state.
    #[inline]
    pub fn output(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_output()
    }
}

impl<'d, T: OutputPin> PinDriver<'d, T, Output> {
    /// Creates the driver for a pin in output open-drain state.
    #[inline]
    pub fn output_od(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_output_od()
    }
}

#[cfg(not(any(esp32c2, esp32c3)))]
impl<'d, T: Pin + RTCPin> PinDriver<'d, T, RtcDisabled> {
    /// Creates the driver for a pin in disabled state.
    #[inline]
    pub fn rtc_disabled(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_disabled()
    }
}

#[cfg(not(any(esp32c2, esp32c3)))]
impl<'d, T: InputPin + RTCPin> PinDriver<'d, T, RtcInput> {
    /// Creates the driver for a pin in RTC input state.
    #[inline]
    pub fn rtc_input(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_input()
    }
}

#[cfg(not(any(esp32c2, esp32c3)))]
impl<'d, T: InputPin + OutputPin + RTCPin> PinDriver<'d, T, RtcInputOutput> {
    /// Creates the driver for a pin in RTC input-output state.
    #[inline]
    pub fn rtc_input_output(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_input_output()
    }
}

#[cfg(not(any(esp32c2, esp32c3)))]
impl<'d, T: InputPin + OutputPin + RTCPin> PinDriver<'d, T, RtcInputOutput> {
    /// Creates the driver for a pin in RTC input-output open-drain state.
    #[inline]
    pub fn rtc_input_output_od(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_input_output_od()
    }
}

#[cfg(not(any(esp32c2, esp32c3)))]
impl<'d, T: OutputPin + RTCPin> PinDriver<'d, T, RtcOutput> {
    /// Creates the driver for a pin in RTC output state.
    #[inline]
    pub fn rtc_output(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_output()
    }
}

#[cfg(not(any(esp32c2, esp32c3)))]
impl<'d, T: OutputPin + RTCPin> PinDriver<'d, T, RtcOutput> {
    /// Creates the driver for a pin in RTC output open-drain state.
    #[inline]
    pub fn rtc_output_od(pin: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_output_od()
    }
}

impl<'d, T: Pin, MODE> PinDriver<'d, T, MODE> {
    /// Returns the pin number.
    pub fn pin(&self) -> u8 {
        self.pin.pin()
    }

    /// Put the pin into disabled mode.
    pub fn into_disabled(self) -> PinDriver<'d, T, Disabled> {
        self.into_mode(GpioMode::Disabled)
    }

    /// Put the pin into input mode.
    #[inline]
    pub fn into_input(self) -> PinDriver<'d, T, Input>
    where
        T: InputPin,
    {
        self.into_mode(GpioMode::Input)
    }

    /// Put the pin into input + output mode.
    ///
    /// This is commonly used for "open drain" mode.
    /// the hardware will drive the line low if you set it to low, and will
    /// leave it floating if you set it to high, in which case you can read
    /// the input to figure out whether another device is driving the line
    /// low.
    ///
    /// The pin level will be whatever was set before (or low by default). If
    /// you want it to begin at a specific level, call `set_high`/`set_low`
    /// on the pin first.
    #[inline]
    pub fn into_input_output(self) -> PinDriver<'d, T, InputOutput>
    where
        T: InputPin + OutputPin,
    {
        self.into_mode(GpioMode::InputOutput)
    }

    /// Put the pin into input + output Open Drain mode.
    ///
    /// This is commonly used for "open drain" mode.
    /// the hardware will drive the line low if you set it to low, and will
    /// leave it floating if you set it to high, in which case you can read
    /// the input to figure out whether another device is driving the line
    /// low.
    ///
    /// The pin level will be whatever was set before (or low by default). If
    /// you want it to begin at a specific level, call `set_high`/`set_low`
    /// on the pin first.
    #[inline]
    pub fn into_input_output_od(self) -> PinDriver<'d, T, InputOutput>
    where
        T: InputPin + OutputPin,
    {
        self.into_mode(GpioMode::InputOutputOpenDrain)
    }

    /// Put the pin into output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If
    /// you want it to begin at a specific level, call `set_high`/`set_low`
    /// on the pin first.
    #[inline]
    pub fn into_output(self) -> PinDriver<'d, T, Output>
    where
        T: OutputPin,
    {
        self.into_mode(GpioMode::Output)
    }

    /// Put the pin into output Open Drain mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If
    /// you want it to begin at a specific level, call `set_high`/`set_low`
    /// on the pin first.
    #[inline]
    pub fn into_output_od(self) -> PinDriver<'d, T, Output>
    where
        T: OutputPin,
    {
        self.into_mode(GpioMode::OutputOpenDrain)
    }

    /// Put the pin into RTC disabled mode.
    #[inline]
    #[cfg(not(any(esp32c2, esp32c3)))]
    pub fn into_rtc_disabled(self) -> PinDriver<'d, T, RtcDisabled>
    where
        T: RTCPin,
    {
        self.into_rtc_mode(GpioMode::Disabled)
    }

    /// Put the pin into RTC input mode.
    #[inline]
    #[cfg(not(any(esp32c2, esp32c3)))]
    pub fn into_rtc_input(self) -> PinDriver<'d, T, RtcInput>
    where
        T: InputPin + RTCPin,
    {
        self.into_rtc_mode(GpioMode::Input)
    }

    /// Put the pin into RTC input + output mode.
    ///
    /// This is commonly used for "open drain" mode.
    /// the hardware will drive the line low if you set it to low, and will
    /// leave it floating if you set it to high, in which case you can read
    /// the input to figure out whether another device is driving the line
    /// low.
    ///
    /// The pin level will be whatever was set before (or low by default). If
    /// you want it to begin at a specific level, call `set_high`/`set_low`
    /// on the pin first.
    #[inline]
    #[cfg(not(any(esp32c2, esp32c3)))]
    pub fn into_rtc_input_output(self) -> PinDriver<'d, T, RtcInputOutput>
    where
        T: InputPin + OutputPin + RTCPin,
    {
        self.into_rtc_mode(GpioMode::InputOutput)
    }

    /// Put the pin into RTC input + output Open Drain mode.
    ///
    /// This is commonly used for "open drain" mode.
    /// the hardware will drive the line low if you set it to low, and will
    /// leave it floating if you set it to high, in which case you can read
    /// the input to figure out whether another device is driving the line
    /// low.
    ///
    /// The pin level will be whatever was set before (or low by default). If
    /// you want it to begin at a specific level, call `set_high`/`set_low`
    /// on the pin first.
    #[inline]
    #[cfg(not(any(esp32c2, esp32c3)))]
    pub fn into_rtc_input_output_od(self) -> PinDriver<'d, T, RtcInputOutput>
    where
        T: InputPin + OutputPin + RTCPin,
    {
        self.into_rtc_mode(GpioMode::InputOutputOpenDrain)
    }

    /// Put the pin into RTC output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If
    /// you want it to begin at a specific level, call `set_high`/`set_low`
    /// on the pin first.
    #[inline]
    #[cfg(not(any(esp32c2, esp32c3)))]
    pub fn into_rtc_output(self) -> PinDriver<'d, T, RtcOutput>
    where
        T: OutputPin + RTCPin,
    {
        self.into_rtc_mode(GpioMode::Output)
    }

    /// Put the pin into RTC output Open Drain mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If
    /// you want it to begin at a specific level, call `set_high`/`set_low`
    /// on the pin first.
    #[inline]
    #[cfg(not(any(esp32c2, esp32c3)))]
    pub fn into_rtc_output_od(self) -> PinDriver<'d, T, RtcOutput>
    where
        T: OutputPin + RTCPin,
    {
        self.into_rtc_mode(GpioMode::OutputOpenDrain)
    }

    #[inline]
    fn into_mode<M>(mut self, mode: GpioMode) -> PinDriver<'d, T, M>
    where
        T: Pin,
    {
        let pin = unsafe { self.pin.clone_unchecked() };

        drop(self);

        if mode != GpioMode::Disabled {
            ll::gpio_set_direction(pin.pin(), mode);
        }

        PinDriver {
            pin,
            _mode: PhantomData,
        }
    }

    #[inline]
    #[cfg(not(any(esp32c2, esp32c3)))]
    fn into_rtc_mode<M>(mut self, mode: GpioMode) -> PinDriver<'d, T, M>
    where
        T: RTCPin,
    {
        let pin = unsafe { self.pin.clone_unchecked() };

        drop(self);

        ll::rtc_gpio_init(pin.pin());
        ll::rtc_gpio_set_direction(pin.pin(), mode);

        PinDriver {
            pin,
            _mode: PhantomData,
        }
    }

    #[inline]
    pub fn get_drive_strength(&self) -> DriveStrength
    where
        MODE: OutputMode,
    {
        if MODE::RTC {
            #[cfg(not(any(esp32c2, esp32c3)))]
            return ll::rtc_gpio_get_drive_capability(self.pin.pin());

            #[cfg(any(esp32c2, esp32c3))]
            unreachable!();
        } else {
            ll::gpio_get_drive_capability(self.pin.pin())
        }
    }

    #[inline]
    pub fn set_drive_strength(&mut self, strength: DriveStrength)
    where
        MODE: OutputMode,
    {
        if MODE::RTC {
            #[cfg(not(any(esp32c2, esp32c3)))]
            ll::rtc_gpio_set_drive_capability(self.pin.pin(), strength);

            #[cfg(any(esp32c2, esp32c3))]
            unreachable!()
        } else {
            ll::gpio_set_drive_capability(self.pin.pin(), strength);
        }
    }

    #[inline]
    pub fn is_high(&self) -> bool
    where
        MODE: InputMode,
    {
        self.get_level().into()
    }

    #[inline]
    pub fn is_low(&self) -> bool
    where
        MODE: InputMode,
    {
        !self.is_high()
    }

    #[inline]
    pub fn get_level(&self) -> Level
    where
        MODE: InputMode,
    {
        if MODE::RTC {
            #[cfg(not(any(esp32c2, esp32c3)))]
            return ll::rtc_gpio_get_level(self.pin.pin());

            #[cfg(any(esp32c2, esp32c3))]
            unreachable!();
        } else {
            ll::gpio_get_level(self.pin.pin())
        }
    }

    #[inline]
    pub fn is_set_high(&self) -> bool
    where
        MODE: OutputMode,
    {
        !self.is_set_low()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool
    where
        MODE: OutputMode,
    {
        self.get_output_level() == Level::Low
    }

    /// What level output is set to
    #[inline]
    fn get_output_level(&self) -> Level
    where
        MODE: OutputMode,
    {
        if MODE::RTC {
            #[cfg(not(any(esp32c2, esp32c3)))]
            return ll::rtc_gpio_get_output_level(self.pin.pin());

            #[cfg(any(esp32c2, esp32c3))]
            unreachable!();
        } else {
            ll::gpio_get_output_level(self.pin.pin())
        }
    }

    #[inline]
    pub fn set_high(&mut self)
    where
        MODE: OutputMode,
    {
        self.set_level(Level::High);
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self)
    where
        MODE: OutputMode,
    {
        self.set_level(Level::Low);
    }

    #[inline]
    pub fn set_level(&mut self, level: Level)
    where
        MODE: OutputMode,
    {
        if MODE::RTC {
            #[cfg(not(any(esp32c2, esp32c3)))]
            ll::rtc_gpio_set_level(self.pin.pin(), level);

            #[cfg(any(esp32c2, esp32c3))]
            unreachable!();
        } else {
            ll::gpio_set_level(self.pin.pin(), level);
        }
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self)
    where
        MODE: OutputMode,
    {
        if self.is_set_low() {
            self.set_high();
        } else {
            self.set_low();
        }
    }

    pub fn set_pull(&mut self, pull: Pull)
    where
        T: InputPin + OutputPin,
        MODE: InputMode,
    {
        if MODE::RTC {
            #[cfg(not(any(esp32c2, esp32c3)))]
            ll::rtc_gpio_set_pull_mode(self.pin.pin(), pull);

            #[cfg(any(esp32c2, esp32c3))]
            unreachable!();
        } else {
            ll::gpio_set_pull_mode(self.pin.pin(), pull);
        }
    }
}

impl<'d, T: Pin, MODE> Drop for PinDriver<'d, T, MODE> {
    fn drop(&mut self) {
        reset_pin(self.pin.pin(), GpioMode::Disabled);
    }
}

unsafe impl<'d, T: Pin, MODE> Send for PinDriver<'d, T, MODE> {}

pub(crate) fn rtc_reset_pin(pin: u8) {
    reset_pin(pin, GpioMode::Disabled);

    #[cfg(not(any(esp32c2, esp32c3)))]
    ll::rtc_gpio_init(pin);
}

fn reset_pin(pin: u8, mode: GpioMode) {
    ll::gpio_reset_pin(pin);
    ll::gpio_set_direction(pin, mode);
}

impl<'d, T: Pin, MODE> embedded_hal::digital::v2::InputPin for PinDriver<'d, T, MODE>
where
    MODE: InputMode,
{
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(PinDriver::is_high(self))
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(PinDriver::is_low(self))
    }
}

#[cfg(feature = "eh1")]
impl<'d, T: Pin, MODE> embedded_hal_1::digital::ErrorType for PinDriver<'d, T, MODE> {
    type Error = Infallible;
}

#[cfg(feature = "eh1")]
impl<'d, T: Pin, MODE> embedded_hal_1::digital::blocking::InputPin for PinDriver<'d, T, MODE>
where
    MODE: InputMode,
{
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(PinDriver::is_high(self))
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(PinDriver::is_low(self))
    }
}

impl<'d, T: Pin, MODE> embedded_hal::digital::v2::OutputPin for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    type Error = Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::High);
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::Low);
        Ok(())
    }
}

#[cfg(feature = "eh1")]
impl<'d, T: Pin, MODE> embedded_hal_1::digital::blocking::OutputPin for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::High);
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::Low);
        Ok(())
    }
}

#[cfg(feature = "eh1")]
impl<'d, T: Pin, MODE> embedded_hal_1::digital::blocking::StatefulOutputPin
    for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.get_output_level().into())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!bool::from(self.get_output_level()))
    }
}

impl<'d, T: Pin, MODE> embedded_hal::digital::v2::StatefulOutputPin for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.get_output_level().into())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!bool::from(self.get_output_level()))
    }
}

impl<'d, T: Pin, MODE> embedded_hal::digital::v2::ToggleableOutputPin for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    type Error = Infallible;

    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.set_level(!self.get_output_level());
        Ok(())
    }
}

#[cfg(feature = "eh1")]
impl<'d, T: Pin, MODE> embedded_hal_1::digital::blocking::ToggleableOutputPin
    for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.set_level(!self.get_output_level());
        Ok(())
    }
}

macro_rules! impl_input {
    ( $pxi:ident: $pin:expr ) => {
        crate::impl_peripheral!($pxi);

        impl Pin for $pxi {
            fn pin(&self) -> u8 {
                $pin
            }
        }

        impl InputPin for $pxi {}

        impl From<$pxi> for AnyInputPin {
            fn from(pin: $pxi) -> Self {
                unsafe { Self::new(pin.pin()) }
            }
        }
    };
}

macro_rules! impl_input_output {
    ( $pxi:ident: $pin:expr ) => {
        impl_input!($pxi: $pin);

        impl OutputPin for $pxi {}

        impl IOPin for $pxi {}

        impl From<$pxi> for AnyOutputPin {
            fn from(pin: $pxi) -> Self {
                unsafe { Self::new(pin.pin()) }
            }
        }

        impl From<$pxi> for AnyIOPin {
            fn from(pin: $pxi) -> Self {
                unsafe { Self::new(pin.pin()) }
            }
        }
    };
}

macro_rules! impl_rtc {
    ( $pxi:ident: $pin:expr, RTC: $rtc:expr ) => {
        impl RTCPin for $pxi {
            fn rtc_pin(&self) -> u8 {
                $rtc
            }
        }
    };

    ( $pxi:ident: $pin:expr, NORTC: $rtc:expr ) => {};
}

macro_rules! impl_adc {
    ( $pxi:ident: $pin:expr, ADC1: $adc:expr ) => {
        impl sealed::ADCPin for $pxi {
            const CHANNEL: u8 = $adc;
        }

        impl ADCPin for $pxi {
            // type Adc = ADC1; // FIXME

            fn adc_channel(&self) -> u8 {
                $adc
            }
        }
    };

    ( $pxi:ident: $pin:expr, ADC2: $adc:expr ) => {
        impl sealed::ADCPin for $pxi {
            const CHANNEL: u8 = $adc;
        }

        impl ADCPin for $pxi {
            // type Adc = ADC2; // FIXME

            fn adc_channel(&self) -> u8 {
                $adc
            }
        }
    };

    ( $pxi:ident: $pin:expr, NOADC: $adc:expr ) => {};
}

macro_rules! impl_dac {
    ( $pxi:ident: $pin:expr, DAC: $dac:expr ) => {
        #[cfg(not(any(esp32c2, esp32c3, esp32s3)))]
        impl DACPin for $pxi {
            fn dac_channel(&self) -> u8 {
                $dac
            }
        }
    };

    ( $pxi:ident: $pin:expr, NODAC: $dac:expr ) => {};
}

macro_rules! impl_touch {
    ( $pxi:ident: $pin:expr, TOUCH: $touch:expr ) => {
        #[cfg(not(any(esp32c2, esp32c3)))]
        impl TouchPin for $pxi {
            fn touch_channel(&self) -> u8 {
                $touch
            }
        }
    };

    ( $pxi:ident: $pin:expr, NOTOUCH: $touch:expr ) => {};
}

macro_rules! pin {
    ( $pxi:ident: $pin:expr, Input, $rtc:ident: $rtcno:expr, $adc:ident: $adcno:expr, $dac:ident: $dacno:expr, $touch:ident: $touchno:expr ) => {
        impl_input!($pxi: $pin);
        impl_rtc!($pxi: $pin, $rtc: $rtcno);
        impl_adc!($pxi: $pin, $adc: $adcno);
        impl_dac!($pxi: $pin, $dac: $dacno);
        impl_touch!($pxi: $pin, $touch: $touchno);
    };

    ( $pxi:ident: $pin:expr, IO, $rtc:ident: $rtcno:expr, $adc:ident: $adcno:expr, $dac:ident: $dacno:expr, $touch:ident: $touchno:expr ) => {
        impl_input_output!($pxi: $pin);
        impl_rtc!($pxi: $pin, $rtc: $rtcno);
        impl_adc!($pxi: $pin, $adc: $adcno);
        impl_dac!($pxi: $pin, $dac: $dacno);
        impl_touch!($pxi: $pin, $touch: $touchno);
    };
}

#[cfg(esp32)]
mod chip {
    use super::*;
    use crate::pac::GPIO;

    // NOTE: Gpio26 - Gpio32 are used by SPI0/SPI1 for external PSRAM/SPI Flash and
    //       are not recommended for other uses
    pin!(Gpio0:0,   IO,    RTC:11,  ADC2:1,  NODAC:0, TOUCH:1);
    pin!(Gpio1:1,   IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2,   IO,    RTC:12,  ADC2:2,  NODAC:0, TOUCH:2);
    pin!(Gpio3:3,   IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4,   IO,    RTC:10,  ADC2:0,  NODAC:0, TOUCH:0);
    pin!(Gpio5:5,   IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio6:6,   IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio7:7,   IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio8:8,   IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio9:9,   IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio10:10, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio11:11, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio12:12, IO,    RTC:15,  ADC2:5,  NODAC:0, TOUCH:5);
    pin!(Gpio13:13, IO,    RTC:14,  ADC2:4,  NODAC:0, TOUCH:4);
    pin!(Gpio14:14, IO,    RTC:16,  ADC2:6,  NODAC:0, TOUCH:6);
    pin!(Gpio15:15, IO,    RTC:13,  ADC2:3,  NODAC:0, TOUCH:3);
    pin!(Gpio16:16, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio17:17, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio18:18, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio21:21, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio22:22, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio23:23, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio25:25, IO,    RTC:6,   ADC2:8,  DAC:1,   NOTOUCH:0);
    pin!(Gpio26:26, IO,    RTC:7,   ADC2:9,  DAC:2,   NOTOUCH:0);
    pin!(Gpio27:27, IO,    RTC:17,  ADC2:7,  NODAC:0, TOUCH:7);
    pin!(Gpio32:32, IO,    RTC:9,   ADC1:4,  NODAC:0, TOUCH:9);
    pin!(Gpio33:33, IO,    RTC:8,   ADC1:5,  NODAC:0, TOUCH:8);
    pin!(Gpio34:34, Input, RTC:4,   ADC1:6,  NODAC:0, NOTOUCH:0);
    pin!(Gpio35:35, Input, RTC:5,   ADC1:7,  NODAC:0, NOTOUCH:0);
    pin!(Gpio36:36, Input, RTC:0,   ADC1:0,  NODAC:0, NOTOUCH:0);
    pin!(Gpio37:37, Input, RTC:1,   ADC1:1,  NODAC:0, NOTOUCH:0);
    pin!(Gpio38:38, Input, RTC:2,   ADC1:2,  NODAC:0, NOTOUCH:0);
    pin!(Gpio39:39, Input, RTC:3,   ADC1:3,  NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio6: Gpio6,
        pub gpio7: Gpio7,
        pub gpio8: Gpio8,
        pub gpio9: Gpio9,
        pub gpio10: Gpio10,
        pub gpio11: Gpio11,
        pub gpio12: Gpio12,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio15: Gpio15,
        pub gpio16: Gpio16,
        pub gpio17: Gpio17,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio21: Gpio21,
        pub gpio22: Gpio22,
        pub gpio23: Gpio23,
        pub gpio25: Gpio25,
        pub gpio26: Gpio26,
        pub gpio27: Gpio27,
        pub gpio32: Gpio32,
        pub gpio33: Gpio33,
        pub gpio34: Gpio34,
        pub gpio35: Gpio35,
        pub gpio36: Gpio36,
        pub gpio37: Gpio37,
        pub gpio38: Gpio38,
        pub gpio39: Gpio39,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::new(),
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                gpio5: Gpio5::new(),
                gpio6: Gpio6::new(),
                gpio7: Gpio7::new(),
                gpio8: Gpio8::new(),
                gpio9: Gpio9::new(),
                gpio10: Gpio10::new(),
                gpio11: Gpio11::new(),
                gpio12: Gpio12::new(),
                gpio13: Gpio13::new(),
                gpio14: Gpio14::new(),
                gpio15: Gpio15::new(),
                gpio16: Gpio16::new(),
                gpio17: Gpio17::new(),
                gpio18: Gpio18::new(),
                gpio19: Gpio19::new(),
                gpio21: Gpio21::new(),
                gpio22: Gpio22::new(),
                gpio23: Gpio23::new(),
                gpio25: Gpio25::new(),
                gpio26: Gpio26::new(),
                gpio27: Gpio27::new(),
                gpio32: Gpio32::new(),
                gpio33: Gpio33::new(),
                gpio34: Gpio34::new(),
                gpio35: Gpio35::new(),
                gpio36: Gpio36::new(),
                gpio37: Gpio37::new(),
                gpio38: Gpio38::new(),
                gpio39: Gpio39::new(),
            }
        }
    }

    pub trait GpioExt {
        fn split(self) -> Pins;
    }

    impl GpioExt for GPIO {
        fn split(self) -> Pins {
            unsafe { Pins::new() }
        }
    }
}

#[cfg(esp32c2)]
mod chip {
    use super::*;
    use crate::pac::GPIO;

    pin!(Gpio0:0,   IO, RTC:0,   ADC1:0,  NODAC:0, NOTOUCH:0);
    pin!(Gpio1:1,   IO, RTC:1,   ADC1:1,  NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2,   IO, RTC:2,   ADC1:2,  NODAC:0, NOTOUCH:0);
    pin!(Gpio3:3,   IO, RTC:3,   ADC1:3,  NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4,   IO, RTC:4,   ADC1:4,  NODAC:0, NOTOUCH:0);
    pin!(Gpio5:5,   IO, RTC:5,   NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio6:6,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio7:7,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio8:8,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio9:9,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio10:10, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio18:18, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio6: Gpio6,
        pub gpio7: Gpio7,
        pub gpio8: Gpio8,
        pub gpio9: Gpio9,
        pub gpio10: Gpio10,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio20: Gpio20,
    }

    impl Pins {
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::new(),
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                gpio5: Gpio5::new(),
                gpio6: Gpio6::new(),
                gpio7: Gpio7::new(),
                gpio8: Gpio8::new(),
                gpio9: Gpio9::new(),
                gpio10: Gpio10::new(),
                gpio18: Gpio18::new(),
                gpio19: Gpio19::new(),
                gpio20: Gpio20::new(),
            }
        }
    }

    pub trait GpioExt {
        fn split(self) -> Pins;
    }

    impl GpioExt for GPIO {
        fn split(self) -> Pins {
            unsafe { Pins::new() }
        }
    }
}

#[cfg(any(esp32c2, esp32c3))]
mod chip {
    use super::*;
    use crate::pac::GPIO;

    // NOTE: Gpio12 - Gpio17 are used by SPI0/SPI1 for external PSRAM/SPI Flash and
    //       are not recommended for other uses
    pin!(Gpio0:0,   IO,   RTC:0,  ADC1:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio1:1,   IO,   RTC:1,  ADC1:1, NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2,   IO,   RTC:2,  ADC1:2, NODAC:0, NOTOUCH:0);
    pin!(Gpio3:3,   IO,   RTC:3,  ADC1:3, NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4,   IO,   RTC:4,  ADC1:4, NODAC:0, NOTOUCH:0);
    pin!(Gpio5:5,   IO,   RTC:5,  ADC2:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio6:6,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio7:7,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio8:8,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio9:9,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio10:10, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio11:11, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio12:12, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio13:13, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio14:14, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio15:15, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio16:16, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio17:17, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio18:18, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio21:21, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio6: Gpio6,
        pub gpio7: Gpio7,
        pub gpio8: Gpio8,
        pub gpio9: Gpio9,
        pub gpio10: Gpio10,
        pub gpio11: Gpio11,
        pub gpio12: Gpio12,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio15: Gpio15,
        pub gpio16: Gpio16,
        pub gpio17: Gpio17,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio20: Gpio20,
        pub gpio21: Gpio21,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::new(),
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                gpio5: Gpio5::new(),
                gpio6: Gpio6::new(),
                gpio7: Gpio7::new(),
                gpio8: Gpio8::new(),
                gpio9: Gpio9::new(),
                gpio10: Gpio10::new(),
                gpio11: Gpio11::new(),
                gpio12: Gpio12::new(),
                gpio13: Gpio13::new(),
                gpio14: Gpio14::new(),
                gpio15: Gpio15::new(),
                gpio16: Gpio16::new(),
                gpio17: Gpio17::new(),
                gpio18: Gpio18::new(),
                gpio19: Gpio19::new(),
                gpio20: Gpio20::new(),
                gpio21: Gpio21::new(),
            }
        }
    }

    pub trait GpioExt {
        fn split(self) -> Pins;
    }

    impl GpioExt for GPIO {
        fn split(self) -> Pins {
            unsafe { Pins::new() }
        }
    }
}

#[cfg(esp32s2)]
mod chip {
    use super::*;
    use crate::pac::GPIO;

    // NOTE: Gpio26 - Gpio32 (and Gpio33 - Gpio37 if using Octal RAM/Flash) are used
    //       by SPI0/SPI1 for external PSRAM/SPI Flash and are not recommended for
    //       other uses
    pin!(Gpio0:0,   IO,    RTC:0,   NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio1:1,   IO,    RTC:1,   ADC1:0,  NODAC:0, TOUCH:1);
    pin!(Gpio2:2,   IO,    RTC:2,   ADC1:1,  NODAC:0, TOUCH:2);
    pin!(Gpio3:3,   IO,    RTC:3,   ADC1:2,  NODAC:0, TOUCH:3);
    pin!(Gpio4:4,   IO,    RTC:4,   ADC1:3,  NODAC:0, TOUCH:4);
    pin!(Gpio5:5,   IO,    RTC:5,   ADC1:4,  NODAC:0, TOUCH:5);
    pin!(Gpio6:6,   IO,    RTC:6,   ADC1:5,  NODAC:0, TOUCH:6);
    pin!(Gpio7:7,   IO,    RTC:7,   ADC1:6,  NODAC:0, TOUCH:7);
    pin!(Gpio8:8,   IO,    RTC:8,   ADC1:7,  NODAC:0, TOUCH:8);
    pin!(Gpio9:9,   IO,    RTC:9,   ADC1:8,  NODAC:0, TOUCH:9);
    pin!(Gpio10:10, IO,    RTC:10,  ADC1:9,  NODAC:0, TOUCH:10);
    pin!(Gpio11:11, IO,    RTC:11,  ADC2:0,  NODAC:0, TOUCH:11);
    pin!(Gpio12:12, IO,    RTC:12,  ADC2:1,  NODAC:0, TOUCH:12);
    pin!(Gpio13:13, IO,    RTC:13,  ADC2:2,  NODAC:0, TOUCH:13);
    pin!(Gpio14:14, IO,    RTC:14,  ADC2:3,  NODAC:0, TOUCH:14);
    pin!(Gpio15:15, IO,    RTC:15,  ADC2:4,  NODAC:0, NOTOUCH:0);
    pin!(Gpio16:16, IO,    RTC:16,  ADC2:5,  NODAC:0, NOTOUCH:0);
    pin!(Gpio17:17, IO,    RTC:17,  ADC2:6,  DAC:1,   NOTOUCH:0);
    pin!(Gpio18:18, IO,    RTC:18,  ADC2:7,  DAC:2,   NOTOUCH:0);
    pin!(Gpio19:19, IO,    RTC:19,  ADC2:8,  NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO,    RTC:20,  ADC2:9,  NODAC:0, NOTOUCH:0);
    pin!(Gpio21:21, IO,    RTC:21,  NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio26:26, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio27:27, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio28:28, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio29:29, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio30:30, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio31:31, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio32:32, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio33:33, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio34:34, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio35:35, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio36:36, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio37:37, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio38:38, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio39:39, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio40:40, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio41:41, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio42:42, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio43:43, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio44:44, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio45:45, IO,    NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio46:46, Input, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio6: Gpio6,
        pub gpio7: Gpio7,
        pub gpio8: Gpio8,
        pub gpio9: Gpio9,
        pub gpio10: Gpio10,
        pub gpio11: Gpio11,
        pub gpio12: Gpio12,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio15: Gpio15,
        pub gpio16: Gpio16,
        pub gpio17: Gpio17,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio20: Gpio20,
        pub gpio21: Gpio21,
        pub gpio26: Gpio26,
        pub gpio27: Gpio27,
        pub gpio28: Gpio28,
        pub gpio29: Gpio29,
        pub gpio30: Gpio30,
        pub gpio31: Gpio31,
        pub gpio32: Gpio32,
        pub gpio33: Gpio33,
        pub gpio34: Gpio34,
        pub gpio35: Gpio35,
        pub gpio36: Gpio36,
        pub gpio37: Gpio37,
        pub gpio38: Gpio38,
        pub gpio39: Gpio39,
        pub gpio40: Gpio40,
        pub gpio41: Gpio41,
        pub gpio42: Gpio42,
        pub gpio43: Gpio43,
        pub gpio44: Gpio44,
        pub gpio45: Gpio45,
        pub gpio46: Gpio46,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::new(),
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                gpio5: Gpio5::new(),
                gpio6: Gpio6::new(),
                gpio7: Gpio7::new(),
                gpio8: Gpio8::new(),
                gpio9: Gpio9::new(),
                gpio10: Gpio10::new(),
                gpio11: Gpio11::new(),
                gpio12: Gpio12::new(),
                gpio13: Gpio13::new(),
                gpio14: Gpio14::new(),
                gpio15: Gpio15::new(),
                gpio16: Gpio16::new(),
                gpio17: Gpio17::new(),
                gpio18: Gpio18::new(),
                gpio19: Gpio19::new(),
                gpio20: Gpio20::new(),
                gpio21: Gpio21::new(),
                gpio26: Gpio26::new(),
                gpio27: Gpio27::new(),
                gpio28: Gpio28::new(),
                gpio29: Gpio29::new(),
                gpio30: Gpio30::new(),
                gpio31: Gpio31::new(),
                gpio32: Gpio32::new(),
                gpio33: Gpio33::new(),
                gpio34: Gpio34::new(),
                gpio35: Gpio35::new(),
                gpio36: Gpio36::new(),
                gpio37: Gpio37::new(),
                gpio38: Gpio38::new(),
                gpio39: Gpio39::new(),
                gpio40: Gpio40::new(),
                gpio41: Gpio41::new(),
                gpio42: Gpio42::new(),
                gpio43: Gpio43::new(),
                gpio44: Gpio44::new(),
                gpio45: Gpio45::new(),
                gpio46: Gpio46::new(),
            }
        }
    }

    pub trait GpioExt {
        fn split(self) -> Pins;
    }

    impl GpioExt for GPIO {
        fn split(self) -> Pins {
            unsafe { Pins::new() }
        }
    }
}

#[cfg(esp32s3)]
mod chip {
    use super::*;
    use crate::pac::GPIO;

    // NOTE: Gpio26 - Gpio32 (and Gpio33 - Gpio37 if using Octal RAM/Flash) are used
    //       by SPI0/SPI1 for external PSRAM/SPI Flash and are not recommended for
    //       other uses
    pin!(Gpio0:0,   IO, RTC:0,   NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio1:1,   IO, RTC:1,   ADC1:0,  NODAC:0, TOUCH:1);
    pin!(Gpio2:2,   IO, RTC:2,   ADC1:1,  NODAC:0, TOUCH:2);
    pin!(Gpio3:3,   IO, RTC:3,   ADC1:2,  NODAC:0, TOUCH:3);
    pin!(Gpio4:4,   IO, RTC:4,   ADC1:3,  NODAC:0, TOUCH:4);
    pin!(Gpio5:5,   IO, RTC:5,   ADC1:4,  NODAC:0, TOUCH:5);
    pin!(Gpio6:6,   IO, RTC:6,   ADC1:5,  NODAC:0, TOUCH:6);
    pin!(Gpio7:7,   IO, RTC:7,   ADC1:6,  NODAC:0, TOUCH:7);
    pin!(Gpio8:8,   IO, RTC:8,   ADC1:7,  NODAC:0, TOUCH:8);
    pin!(Gpio9:9,   IO, RTC:9,   ADC1:8,  NODAC:0, TOUCH:9);
    pin!(Gpio10:10, IO, RTC:10,  ADC1:9,  NODAC:0, TOUCH:10);
    pin!(Gpio11:11, IO, RTC:11,  ADC2:0,  NODAC:0, TOUCH:11);
    pin!(Gpio12:12, IO, RTC:12,  ADC2:1,  NODAC:0, TOUCH:12);
    pin!(Gpio13:13, IO, RTC:13,  ADC2:2,  NODAC:0, TOUCH:13);
    pin!(Gpio14:14, IO, RTC:14,  ADC2:3,  NODAC:0, TOUCH:14);
    pin!(Gpio15:15, IO, RTC:15,  ADC2:4,  NODAC:0, NOTOUCH:0);
    pin!(Gpio16:16, IO, RTC:16,  ADC2:5,  NODAC:0, NOTOUCH:0);
    pin!(Gpio17:17, IO, RTC:17,  ADC2:6,  NODAC:0, NOTOUCH:0);
    pin!(Gpio18:18, IO, RTC:18,  ADC2:7,  NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO, RTC:19,  ADC2:8,  NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO, RTC:20,  ADC2:9,  NODAC:0, NOTOUCH:0);
    pin!(Gpio21:21, IO, RTC:21,  NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio26:26, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio27:27, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio28:28, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio29:29, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio30:30, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio31:31, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio32:32, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio33:33, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio34:34, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio35:35, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio36:36, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio37:37, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio38:38, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio39:39, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio40:40, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio41:41, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio42:42, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio43:43, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio44:44, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio45:45, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio46:46, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio47:47, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio48:48, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio6: Gpio6,
        pub gpio7: Gpio7,
        pub gpio8: Gpio8,
        pub gpio9: Gpio9,
        pub gpio10: Gpio10,
        pub gpio11: Gpio11,
        pub gpio12: Gpio12,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio15: Gpio15,
        pub gpio16: Gpio16,
        pub gpio17: Gpio17,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio20: Gpio20,
        pub gpio21: Gpio21,
        pub gpio26: Gpio26,
        pub gpio27: Gpio27,
        pub gpio28: Gpio28,
        pub gpio29: Gpio29,
        pub gpio30: Gpio30,
        pub gpio31: Gpio31,
        pub gpio32: Gpio32,
        pub gpio33: Gpio33,
        pub gpio34: Gpio34,
        pub gpio35: Gpio35,
        pub gpio36: Gpio36,
        pub gpio37: Gpio37,
        pub gpio38: Gpio38,
        pub gpio39: Gpio39,
        pub gpio40: Gpio40,
        pub gpio41: Gpio41,
        pub gpio42: Gpio42,
        pub gpio43: Gpio43,
        pub gpio44: Gpio44,
        pub gpio45: Gpio45,
        pub gpio46: Gpio46,
        pub gpio47: Gpio47,
        pub gpio48: Gpio48,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::new(),
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                gpio5: Gpio5::new(),
                gpio6: Gpio6::new(),
                gpio7: Gpio7::new(),
                gpio8: Gpio8::new(),
                gpio9: Gpio9::new(),
                gpio10: Gpio10::new(),
                gpio11: Gpio11::new(),
                gpio12: Gpio12::new(),
                gpio13: Gpio13::new(),
                gpio14: Gpio14::new(),
                gpio15: Gpio15::new(),
                gpio16: Gpio16::new(),
                gpio17: Gpio17::new(),
                gpio18: Gpio18::new(),
                gpio19: Gpio19::new(),
                gpio20: Gpio20::new(),
                gpio21: Gpio21::new(),
                gpio26: Gpio26::new(),
                gpio27: Gpio27::new(),
                gpio28: Gpio28::new(),
                gpio29: Gpio29::new(),
                gpio30: Gpio30::new(),
                gpio31: Gpio31::new(),
                gpio32: Gpio32::new(),
                gpio33: Gpio33::new(),
                gpio34: Gpio34::new(),
                gpio35: Gpio35::new(),
                gpio36: Gpio36::new(),
                gpio37: Gpio37::new(),
                gpio38: Gpio38::new(),
                gpio39: Gpio39::new(),
                gpio40: Gpio40::new(),
                gpio41: Gpio41::new(),
                gpio42: Gpio42::new(),
                gpio43: Gpio43::new(),
                gpio44: Gpio44::new(),
                gpio45: Gpio45::new(),
                gpio46: Gpio46::new(),
                gpio47: Gpio47::new(),
                gpio48: Gpio48::new(),
            }
        }
    }

    pub trait GpioExt {
        fn split(self) -> Pins;
    }

    impl GpioExt for GPIO {
        fn split(self) -> Pins {
            unsafe { Pins::new() }
        }
    }
}
