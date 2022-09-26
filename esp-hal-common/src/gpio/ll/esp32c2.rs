//! Low-level GPIO functionality for the ESP32-C2

use crate::{
    gpio::{
        signals::{OutputSignal, OutputSignalType},
        DriveStrength,
        GpioMode,
        Level,
        Pull,
    },
    pac::{GPIO, IO_MUX},
};

/// Set the direction of a GPIO pin
pub(crate) fn gpio_set_direction(pin: u8, mode: GpioMode) {
    let gpio = unsafe { &*GPIO::PTR };
    let io_mux = unsafe { &*IO_MUX::PTR };

    // Input enable/disable
    if mode.is_input() || mode.is_open_drain() {
        io_mux.gpio[pin as usize].modify(|_, w| w.fun_ie().set_bit());
    } else {
        io_mux.gpio[pin as usize].modify(|_, w| w.fun_ie().clear_bit());
    }

    // Output enable/disable
    if mode.is_output() {
        gpio.enable_w1ts.write(|w| unsafe { w.bits(1 << pin) });
    } else {
        gpio.enable_w1tc.write(|w| unsafe { w.bits(1 << pin) });
    }

    // FIXME: should `func_in_sel_cfg` be set for inputs instead?
    gpio.func_out_sel_cfg[pin as usize]
        .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

    // FIXME: are there any additional fields we should initialize?
    io_mux.gpio[pin as usize].modify(|_, w| unsafe {
        w.mcu_sel()
            .bits(0x1) // Function 1
            .fun_wpu()
            .clear_bit()
            .fun_wpd()
            .clear_bit()
            .slp_sel()
            .clear_bit()
    });
}

/// Get the currently configured drive strength of a GPIO pin
pub(crate) fn gpio_get_drive_capability(pin: u8) -> DriveStrength {
    let bits = unsafe { &*IO_MUX::PTR }.gpio[pin as usize]
        .read()
        .fun_drv()
        .bits();

    DriveStrength::from(bits)
}

/// Read the input level of a GPIO pin
pub(crate) fn gpio_get_level(pin: u8) -> Level {
    let level = (unsafe { &*GPIO::PTR }.in_.read().bits() & (1 << pin)) != 0;

    Level::from(level)
}

/// Read the output level of a GPIO pin
pub(crate) fn gpio_get_output_level(pin: u8) -> Level {
    let level = (unsafe { &*GPIO::PTR }.out.read().bits() & (1 << pin)) != 0;

    Level::from(level)
}

/// Reset a GPIO pin to its default state
pub(crate) fn gpio_reset_pin(pin: u8) {
    // FIXME: which registers/fields *NEED* to be reset?
}

/// Set the drive strength of a GPIO pin
pub(crate) fn gpio_set_drive_capability(pin: u8, strength: DriveStrength) {
    unsafe { &*IO_MUX::PTR }.gpio[pin as usize]
        .modify(|_, w| unsafe { w.fun_drv().bits(strength as u8) });
}

/// Set the output level of a GPIO pin
pub(crate) fn gpio_set_level(pin: u8, level: Level) {
    if level == Level::High {
        unsafe { &*GPIO::PTR }
            .out_w1ts
            .write(|w| unsafe { w.bits(1 << pin) });
    } else {
        unsafe { &*GPIO::PTR }
            .out_w1tc
            .write(|w| unsafe { w.bits(1 << pin) });
    }
}

/// Set the pull mode of a GPIO pin
pub(crate) fn gpio_set_pull_mode(pin: u8, pull: Pull) {
    let pull_up = matches!(pull, Pull::Up | Pull::UpDown);
    let pull_down = matches!(pull, Pull::Down | Pull::UpDown);

    unsafe { &*IO_MUX::PTR }.gpio[pin as usize]
        .modify(|_, w| w.fun_wpu().bit(pull_up).fun_wpd().bit(pull_down));
}
