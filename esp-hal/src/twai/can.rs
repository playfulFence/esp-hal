#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct StandardId(u16);

impl StandardId {
    pub const ZERO: Self = StandardId(0);
    pub const MAX: Self = StandardId(0x7FF);

    pub fn new(raw: u16) -> Option<Self> {
        if raw <= 0x7FF {
            Some(StandardId(raw))
        } else {
            None
        }
    }

    pub const unsafe fn new_unchecked(raw: u16) -> Self {
        StandardId(raw)
    }

    pub fn as_raw(&self) -> u16 {
        self.0
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct ExtendedId(u32);

impl ExtendedId {
    pub const ZERO: Self = ExtendedId(0);
    pub const MAX: Self = ExtendedId(0x1FFF_FFFF);

    pub fn new(raw: u32) -> Option<Self> {
        if raw <= 0x1FFF_FFFF {
            Some(ExtendedId(raw))
        } else {
            None
        }
    }

    pub const unsafe fn new_unchecked(raw: u32) -> Self {
        ExtendedId(raw)
    }

    pub fn as_raw(&self) -> u32 {
        self.0
    }

    pub fn standard_id(&self) -> StandardId {
        StandardId((self.0 >> 18) as u16)
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Id {
    Standard(StandardId),
    Extended(ExtendedId),
}

impl From<StandardId> for Id {
    #[inline]
    fn from(id: StandardId) -> Self {
        Id::Standard(id)
    }
}

impl From<ExtendedId> for Id {
    #[inline]
    fn from(id: ExtendedId) -> Self {
        Id::Extended(id)
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum ErrorKind {
    /// The peripheral receive buffer was overrun.
    Overrun,

    // MAC sublayer errors
    /// A bit error is detected at that bit time when the bit value that is
    /// monitored differs from the bit value sent.
    Bit,

    /// A stuff error is detected at the bit time of the sixth consecutive
    /// equal bit level in a frame field that shall be coded by the method
    /// of bit stuffing.
    Stuff,

    /// Calculated CRC sequence does not equal the received one.
    Crc,

    /// A form error shall be detected when a fixed-form bit field contains
    /// one or more illegal bits.
    Form,

    /// An ACK  error shall be detected by a transmitter whenever it does not
    /// monitor a dominant bit during the ACK slot.
    Acknowledge,

    /// A different error occurred. The original error may contain more information.
    Other,
}

pub trait Error: core::fmt::Debug {
    fn kind(&self) -> ErrorKind;
}

impl core::fmt::Display for ErrorKind {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ErrorKind::Overrun => write!(f, "The peripheral receive buffer was overrun"),
            ErrorKind::Bit => write!(
                f,
                "Bit value that is monitored differs from the bit value sent"
            ),
            ErrorKind::Stuff => write!(f, "Sixth consecutive equal bits detected"),
            ErrorKind::Crc => write!(f, "Calculated CRC sequence does not equal the received one"),
            ErrorKind::Form => write!(
                f,
                "A fixed-form bit field contains one or more illegal bits"
            ),
            ErrorKind::Acknowledge => write!(f, "Transmitted frame was not acknowledged"),
            ErrorKind::Other => write!(
                f,
                "A different error occurred. The original error may contain more information"
            ),
        }
    }
}

pub trait Frame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self>
    where
        Self: Sized;

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self>
    where
        Self: Sized;

    fn is_extended(&self) -> bool;

    fn is_remote_frame(&self) -> bool;

    fn is_data_frame(&self) -> bool {
        !self.is_remote_frame()
    }

    fn id(&self) -> Id;

    fn dlc(&self) -> usize;

    fn data(&self) -> &[u8];
}

pub trait Can {
    /// Associated frame type.
    type Frame: Frame;

    /// Associated error type.
    type Error: Error;

    /// Puts a frame in the transmit buffer to be sent on the bus.
    ///
    /// If the transmit buffer is full, this function will try to replace a pending
    /// lower priority frame and return the frame that was replaced.
    /// Returns `Err(WouldBlock)` if the transmit buffer is full and no frame can be
    /// replaced.
    ///
    /// # Notes for implementers
    ///
    /// * Frames of equal identifier shall be transmited in FIFO fashion when more
    ///   than one transmit buffer is available.
    /// * When replacing pending frames make sure the frame is not in the process of
    ///   being send to the bus.
    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error>;

    /// Returns a received frame if available.
    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error>;
}
