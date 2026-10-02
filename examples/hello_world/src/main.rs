//! hello world
//!
//! This is a very basic example that shows how to print messages.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{main, time::Instant};

esp_bootloader_esp_idf::esp_app_desc!();

static BIG: [u8; 4096] = [0x5a; 4096];

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let _peripherals = esp_hal::init(esp_hal::Config::default());

    esp_println::println!("Init! {}", core::hint::black_box(&BIG)[100]);

    loop {
        esp_println::println!("Bing!");

        let now = Instant::now();
        while now.elapsed().as_millis() < 5000 {}
    }
}
