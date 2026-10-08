//! hello world
//!
//! This is a very basic example that shows how to print messages.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{main, time::Instant};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let _peripherals = esp_hal::init(esp_hal::Config::default());

    esp_println::println!("Init!");

    static BIG: [u8; 4096] = [1; 4096];
    static mut ZEROS: [u8; 1024] = [0; 1024];
    core::hint::black_box(&BIG);
    core::hint::black_box(&raw mut ZEROS);

    loop {
        esp_println::println!("Bing!");

        let now = Instant::now();
        while now.elapsed().as_millis() < 5000 {}
    }
}
