#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32c2, path = "esp32c2.rs")]
#[cfg_attr(esp32c3, path = "esp32c3.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
mod chip;

pub(crate) use chip::{
    gpio_get_drive_capability,
    gpio_get_level,
    gpio_get_output_level,
    gpio_reset_pin,
    gpio_set_direction,
    gpio_set_drive_capability,
    gpio_set_level,
    gpio_set_pull_mode,
};
#[cfg(not(esp32c3))]
pub(crate) use chip::{
    rtc_gpio_get_drive_capability,
    rtc_gpio_get_level,
    rtc_gpio_get_output_level,
    rtc_gpio_init,
    rtc_gpio_set_direction,
    rtc_gpio_set_drive_capability,
    rtc_gpio_set_level,
    rtc_gpio_set_pull_mode,
};
