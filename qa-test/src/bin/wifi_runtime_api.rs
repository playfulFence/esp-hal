//! QA test for Wi-Fi runtime getters/setters, ESP-NOW init config, and sniffer
//! config.

//% CHIP_FILTER: wifi_driver_supported
//% FEATURES: esp-radio esp-radio/wifi esp-radio/esp-now esp-radio/sniffer esp-radio/unstable esp-hal/unstable

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    time::Duration,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::{
    esp_now::{EspNowConfig, PeerRateConfig, WifiPhyRate, BROADCAST_ADDRESS},
    wifi::{
        ap::AccessPointConfig,
        sniffer::{PromiscuousFilter, SnifferConfig},
        Bandwidth, Bandwidths, Config, ControllerConfig, PowerSaveMode, Protocol, Protocols,
        SecondaryChannel,
    },
};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 32 * 1024);
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let ap_config = Config::AccessPoint(
        AccessPointConfig::default()
            .with_ssid("qa-runtime-api")
            .with_channel(1),
    );

    let mut controller = esp_radio::wifi::WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(ap_config),
    )
    .unwrap();

    println!("Testing Wi-Fi runtime getters/setters");

    controller
        .set_power_saving(PowerSaveMode::Minimum)
        .unwrap();
    assert_eq!(controller.power_saving().unwrap(), PowerSaveMode::Minimum);

    controller.set_max_tx_power(24).unwrap();
    assert_eq!(controller.max_tx_power().unwrap(), 24);

    let protocols = Protocols::default().with_2_4(Protocol::B | Protocol::G);
    controller.set_protocols(protocols).unwrap();
    assert_eq!(controller.protocols().unwrap(), protocols);

    let initial_bandwidths = controller.bandwidths().unwrap();
    controller.set_bandwidths(initial_bandwidths).unwrap();
    assert_eq!(controller.bandwidths().unwrap(), initial_bandwidths);

    controller
        .set_channel(6, SecondaryChannel::None)
        .unwrap();
    let (channel, secondary) = controller.channel().unwrap();
    assert_eq!(channel, 6);
    assert_eq!(secondary, SecondaryChannel::None);

    let country = controller.country_info().unwrap();
    println!("Country: {:?}", country);

    println!("Testing ESP-NOW init config");

    let esp_now_config = EspNowConfig::default()
        .with_user_oui([0x12, 0x34, 0x56])
        .with_wake_window(Duration::from_millis(100))
        .with_wake_interval(Duration::from_millis(200))
        .with_channel(6)
        .with_default_rate(WifiPhyRate::Rate11mL);

    let esp_now = controller.esp_now(esp_now_config);
    assert_eq!(esp_now.user_oui().unwrap(), [0x12, 0x34, 0x56]);
    assert!(esp_now.version().unwrap() > 0);

    esp_now
        .set_peer_rate(
            &BROADCAST_ADDRESS,
            PeerRateConfig {
                rate: WifiPhyRate::Rate6m,
            },
        )
        .unwrap();

    drop(esp_now);

    println!("Testing sniffer config");

    let sniffer_config = SnifferConfig::default()
        .with_filter(PromiscuousFilter::DATA)
        .with_channel(6)
        .with_secondary_channel(SecondaryChannel::None)
        .with_promiscuous(true);

    let sniffer = controller.sniffer(sniffer_config);
    assert!(sniffer.promiscuous_mode().unwrap());
    assert_eq!(sniffer.filter().unwrap().mask, PromiscuousFilter::DATA.mask);

    sniffer
        .set_channel(11, SecondaryChannel::None)
        .unwrap();
    let (channel, _) = controller.channel().unwrap();
    assert_eq!(channel, 11);

    println!("wifi_runtime_api passed");
}
