/// The name of the chip as `&str`
///
/// # Example
///
/// ```rust, no_run
/// use esp_hal::chip;
/// let chip_name = chip!();
#[doc = concat!("assert_eq!(chip_name, ", chip!(), ")")]
/// ```
#[macro_export]
macro_rules! chip {
    () => {
        "esp32c61"
    };
}
/// The properties of this chip and its drivers.
#[macro_export]
macro_rules! property {
    ("chip") => {
        "esp32c61"
    };
    ("arch") => {
        "riscv"
    };
    ("cores") => {
        1
    };
    ("cores", str) => {
        stringify!(1)
    };
    ("trm") => {
        "https://www.espressif.com/sites/default/files/documentation/esp32-c61_technical_reference_manual_en.pdf"
    };
}
/// Macro to get the address range of the given memory region.
#[macro_export]
macro_rules! memory_range {
    ("DRAM") => {
        1082130432..1082458112
    };
}
#[macro_export]
macro_rules! for_each_peripheral {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((all));
    };
}
