//! Low-level GPIO functionality for the ESP32

use crate::{
    gpio::{
        signals::{OutputSignal, OutputSignalType},
        DriveStrength,
        GpioMode,
        Level,
        Pull,
    },
    pac::GPIO,
};

const IO_MUX_GPIO_REG_RESET: u32 = 0x800;

// Unfortunately the GPIO registers in the IO_MUX are not sequential with even
// intervals. Because of this we cannot collect these registers into an array,
// and as such we have created a lookup table instead.
const IO_MUX_GPIO_REGS: [u32; 40] = [
    0x3ff4_9044, // [0]
    0x3ff4_9088, // [1]
    0x3ff4_9040, // [2]
    0x3ff4_9084, // [3]
    0x3ff4_9048, // [4]
    0x3ff4_906c, // [5]
    0x3ff4_9060, // [6]
    0x3ff4_9064, // [7]
    0x3ff4_9068, // [8]
    0x3ff4_9054, // [9]
    0x3ff4_9058, // [10]
    0x3ff4_905c, // [11]
    0x3ff4_9034, // [12]
    0x3ff4_9038, // [13]
    0x3ff4_9030, // [14]
    0x3ff4_903c, // [15]
    0x3ff4_904c, // [16]
    0x3ff4_9050, // [17]
    0x3ff4_9070, // [18]
    0x3ff4_9074, // [19]
    0x3ff4_9078, // [20]
    0x3ff4_907c, // [21]
    0x3ff4_9080, // [22]
    0x3ff4_908c, // [23]
    0x3ff4_9090, // [24]
    0x3ff4_9024, // [25]
    0x3ff4_9028, // [26]
    0x3ff4_902c, // [27]
    0x0,         // Padding
    0x0,         // Padding
    0x0,         // Padding
    0x0,         // Padding
    0x3ff4_901c, // [32]
    0x3ff4_9020, // [33]
    0x3ff4_9014, // [34]
    0x3ff4_9018, // [35]
    0x3ff4_9004, // [36]
    0x3ff4_9008, // [37]
    0x3ff4_900c, // [38]
    0x3ff4_9010, // [39]
];

/// Set the direction of a GPIO pin
pub(crate) fn gpio_set_direction(pin: u8, mode: GpioMode) {
    let gpio = unsafe { &*GPIO::PTR };
    let io_mux = IO_MUX_GPIO_REGS[pin as usize];

    let mut word = unsafe { core::ptr::read_volatile(io_mux as *const u32) };

    // Input enable/disable
    if mode.is_input() || mode.is_open_drain() {
        word |= 1 << 9; // fun_ie
    } else {
        word &= !(1 << 9); // fun_ie
    }

    // Output enable/disable
    if mode.is_output() {
        if pin < 32 {
            unsafe { &*GPIO::PTR }
                .enable_w1ts
                .write(|w| unsafe { w.bits(1 << pin) });
        } else {
            unsafe { &*GPIO::PTR }
                .enable1_w1ts
                .write(|w| unsafe { w.bits(1 << (pin % 32)) });
        }
    } else {
        if pin < 32 {
            unsafe { &*GPIO::PTR }
                .enable_w1tc
                .write(|w| unsafe { w.bits(1 << pin) });
        } else {
            unsafe { &*GPIO::PTR }
                .enable1_w1tc
                .write(|w| unsafe { w.bits(1 << (pin % 32)) });
        }
    }

    // FIXME: should `func_in_sel_cfg` be set for inputs instead?
    gpio.func_out_sel_cfg[pin as usize]
        .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as OutputSignalType) });

    // FIXME: are there any additional fields we should initialize?
    word = (word & !(3 << 12)) | (2 << 12); // mcu_sel
    word &= !(1 << 3); // mcu_wpu
    word &= !(1 << 2); // mcu_wpd
    word &= !(1 << 1); // slp_sel

    unsafe { core::ptr::write_volatile(io_mux as *mut _, word) };
}

/// Get the currently configured drive strength of a GPIO pin
pub(crate) fn gpio_get_drive_capability(pin: u8) -> DriveStrength {
    let word = unsafe { core::ptr::read_volatile(IO_MUX_GPIO_REGS[pin as usize] as *const u32) };
    let word = (word >> 10) & 0x3;

    DriveStrength::from(word as u8)
}

/// Read the input level of a GPIO pin
pub(crate) fn gpio_get_level(pin: u8) -> Level {
    let level = if pin < 32 {
        (unsafe { &*GPIO::PTR }.in_.read().bits() & (1 << pin)) != 0
    } else {
        (unsafe { &*GPIO::PTR }.in1.read().bits() & (1 << (pin % 32))) != 0
    };

    Level::from(level)
}

/// Read the output level of a GPIO pin
pub(crate) fn gpio_get_output_level(pin: u8) -> Level {
    let level = if pin < 32 {
        (unsafe { &*GPIO::PTR }.out.read().bits() & (1 << pin)) != 0
    } else {
        (unsafe { &*GPIO::PTR }.out1.read().bits() & (1 << (pin % 32))) != 0
    };

    Level::from(level)
}

/// Reset a GPIO pin to its default state
pub(crate) fn gpio_reset_pin(pin: u8) {
    // FIXME: which registers/fields *NEED* to be reset?
}

/// Set the drive strength of a GPIO pin
pub(crate) fn gpio_set_drive_capability(pin: u8, strength: DriveStrength) {
    let word = unsafe { core::ptr::read_volatile(IO_MUX_GPIO_REGS[pin as usize] as *const u32) };
    let word = (word & !(0x3 << 10)) | ((strength as u32) << 10);

    unsafe {
        core::ptr::write_volatile(IO_MUX_GPIO_REGS[pin as usize] as *mut _, word);
    }
}

/// Set the output level of a GPIO pin
pub(crate) fn gpio_set_level(pin: u8, level: Level) {
    if pin < 32 {
        gpio_set_level_bank0(pin, level);
    } else {
        gpio_set_level_bank1(pin % 32, level);
    }
}

fn gpio_set_level_bank0(pin: u8, level: Level) {
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

fn gpio_set_level_bank1(pin: u8, level: Level) {
    if level == Level::High {
        unsafe { &*GPIO::PTR }
            .out1_w1ts
            .write(|w| unsafe { w.bits(1 << pin) });
    } else {
        unsafe { &*GPIO::PTR }
            .out1_w1tc
            .write(|w| unsafe { w.bits(1 << pin) });
    }
}

/// Set the pull mode of a GPIO pin
pub(crate) fn gpio_set_pull_mode(pin: u8, pull: Pull) {
    let pull_up = matches!(pull, Pull::Up | Pull::UpDown);
    let pull_down = matches!(pull, Pull::Down | Pull::UpDown);

    let word = unsafe { core::ptr::read_volatile(IO_MUX_GPIO_REGS[pin as usize] as *const u32) };
    let word = (word & !(1 << 3)) | ((pull_up as u32) << 3);
    let word = (word & !(1 << 2)) | ((pull_down as u32) << 2);

    unsafe {
        core::ptr::write_volatile(IO_MUX_GPIO_REGS[pin as usize] as *mut _, word);
    }
}

pub(crate) fn rtc_gpio_init(pin: u8) {
    todo!()
}

pub(crate) fn rtc_gpio_set_direction(pin: u8, mode: GpioMode) {
    todo!()
}

pub(crate) fn rtc_gpio_get_drive_capability(pin: u8) -> DriveStrength {
    todo!()
}

pub(crate) fn rtc_gpio_get_level(pin: u8) -> Level {
    todo!()
}

pub(crate) fn rtc_gpio_get_output_level(pin: u8) -> Level {
    todo!()
}

pub(crate) fn rtc_gpio_set_drive_capability(pin: u8, strength: DriveStrength) {
    todo!()
}

pub(crate) fn rtc_gpio_set_level(pin: u8, level: Level) {
    todo!()
}

pub(crate) fn rtc_gpio_set_pull_mode(pin: u8, pull: Pull) {
    todo!()
}
