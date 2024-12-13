//% FEATURES: esp-wifi esp-wifi/wifi esp-wifi/utils
//% CHIPS: esp32c6

#![no_std]
#![no_main]

use core::fmt::Write as coreWrite;
extern crate alloc;

use blocking_network_stack::{Socket, Stack};
use bme280::i2c::BME280;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    geometry::*,
    mono_font::{MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::*,
    prelude::*,
    primitives::*,
    text::*,
};
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_io::*;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Level, NoPin, Output},
    i2c::master::I2c,
    prelude::*,
    rng::Rng,
    spi,
    spi::{
        master::{Config, Spi},
        SpiMode,
    },
    time::{self, Duration},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_wifi::{
    init,
    wifi::{
        utils::create_network_interface,
        AccessPointInfo,
        ClientConfiguration,
        Configuration,
        WifiError,
        WifiStaDevice,
    },
};
use heapless::String;
use mipidsi::Builder;
use smoltcp::{
    iface::{SocketSet, SocketStorage},
    wire::{DhcpOption, IpAddress, Ipv4Address},
};

pub enum DisplaySegment {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
    Center,
}

use profont::{PROFONT_14_POINT, PROFONT_18_POINT, PROFONT_24_POINT};

// Definition of default styles using the ProFont monospace font at different
// sizes.
pub const DEFAULT_STYLE_SMALL: MonoTextStyle<Rgb565> = MonoTextStyleBuilder::new()
    .font(&PROFONT_14_POINT)
    .text_color(RgbColor::BLACK)
    .build();

pub const DEFAULT_STYLE_MID: MonoTextStyle<Rgb565> = MonoTextStyleBuilder::new()
    .font(&PROFONT_18_POINT)
    .text_color(RgbColor::BLACK)
    .build();

pub const DEFAULT_STYLE_LARGE: MonoTextStyle<Rgb565> = MonoTextStyleBuilder::new()
    .font(&PROFONT_24_POINT)
    .text_color(RgbColor::BLACK)
    .build();

const SSID: &str = "EspressifSystems";
const PASSWORD: &str = "Espressif32";

#[entry]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut delay = Delay::new();
    let mut rng = Rng::new(peripherals.RNG);

    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7);

    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(100.kHz())
            .with_mode(SpiMode::Mode0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO0)
    .with_miso(peripherals.GPIO4)
    .with_mosi(peripherals.GPIO2)
    .with_cs(peripherals.GPIO5);

    let di = SPIInterface::new(
        ExclusiveDevice::new(spi, NoPin, delay).unwrap(),
        Output::new(peripherals.GPIO9, Level::Low),
    );

    let mut display = Builder::new(mipidsi::models::ILI9341Rgb565, di)
        .reset_pin(Output::new(peripherals.GPIO10, Level::Low))
        .color_order(mipidsi::options::ColorOrder::Rgb)
        // .orientation()
        .init(&mut delay)
        .unwrap();

    display.clear(Rgb565::WHITE).unwrap();

    let mut sensor = BME280::new_primary(i2c);
    sensor.init(&mut delay).unwrap();

    let init = init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap();

    let mut wifi = peripherals.WIFI;

    let (iface, device, mut controller) =
        create_network_interface(&init, &mut wifi, WifiStaDevice).unwrap();

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let mut dhcp_socket = smoltcp::socket::dhcpv4::Socket::new();
    // we can set a hostname here (or add other DHCP options)
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12,
        data: b"esp-wifi",
    }]);
    socket_set.add(dhcp_socket);

    let now = || time::now().duration_since_epoch().to_millis();
    let stack = Stack::new(iface, device, socket_set, now, rng.random());

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        ..Default::default()
    });
    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    println!("Start Wifi Scan");
    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
    if let Ok((res, _count)) = res {
        for ap in res {
            println!("{:?}", ap);
        }
    }

    println!("{:?}", controller.capabilities());
    println!("wifi_connect {:?}", controller.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        match controller.is_connected() {
            Ok(true) => break,
            Ok(false) => {}
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", controller.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        stack.work();

        if stack.is_iface_up() {
            println!("got ip {:?}", stack.get_ip_info());
            break;
        }
    }

    println!("Start busy loop on main");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    socket
        .open(IpAddress::Ipv4(Ipv4Address::new(213, 188, 196, 246)), 80)
        .unwrap();

    write_segment_name(
        &mut display,
        DisplaySegment::TopLeft,
        "Temperature",
        DEFAULT_STYLE_MID,
    );

    write_segment_name(
        &mut display,
        DisplaySegment::TopRight,
        "Humidity",
        DEFAULT_STYLE_MID,
    );

    // We'll need it to convert numbers to strings, writable on display
    let mut data: String<32> = String::new();
    let mut timestamp = get_timestamp(socket).unwrap();

    let (mut h, mut m, mut s) = timestamp_to_hms(timestamp);

    let mut measurement = sensor.measure(&mut delay).unwrap();
    loop {
        write!(data, "{:2}°C", measurement.temperature).expect("write! failed...");
        write_to_segment(
            &mut display,
            DisplaySegment::TopLeft,
            data.as_str(),
            DEFAULT_STYLE_MID,
        );
        data.clear();

        write!(data, "{:2}%", measurement.humidity).expect("write! failed...");
        write_to_segment(
            &mut display,
            DisplaySegment::TopRight,
            data.as_str(),
            DEFAULT_STYLE_MID,
        );
        data.clear();

        write!(data, "{}:{}:{}", h, m, s).expect("write! failed...");

        write_to_segment(
            &mut display,
            DisplaySegment::Center,
            data.as_str(),
            DEFAULT_STYLE_MID,
        );

        write_segment_name(
            &mut display,
            DisplaySegment::Center,
            weekday_from_timestamp(&timestamp),
            DEFAULT_STYLE_SMALL,
        );

        // Adjust delay in case with this one time gets moved.
        // It is not exactly 1s due to some processes above are consuming some time
        delay.delay_millis(600);
        timestamp += 1;
        (h, m, s) = timestamp_to_hms(timestamp);
        data.clear();

        measurement = sensor.measure(&mut delay).unwrap();
    }
}

fn write_segment_name<
    M: esp_hal::spi::master::Instance + 'static,
    RST: OutputPin<Error = core::convert::Infallible>,
    DC: OutputPin<Error = core::convert::Infallible>,
>(
    display: &mut mipidsi::Display<
        SPIInterface<
            ExclusiveDevice<spi::master::Spi<'static, esp_hal::Blocking, M>, NoPin, Delay>,
            DC,
        >,
        mipidsi::models::ILI9341Rgb565,
        RST,
    >,
    segment: DisplaySegment,
    name: &str,
    font: MonoTextStyle<Rgb565>,
) {
    let size = display.size();
    let segment_size = Size::new(size.width / 2, size.height / 2);

    let (x, y, width, _) = match segment {
        DisplaySegment::TopLeft => (0, 0 + 15, segment_size.width, segment_size.height),
        DisplaySegment::TopRight => (
            segment_size.width as i32,
            0 + 15,
            segment_size.width,
            segment_size.height,
        ),
        DisplaySegment::BottomLeft => (
            0,
            segment_size.height as i32 + 15,
            segment_size.width,
            segment_size.height,
        ),
        DisplaySegment::BottomRight => (
            segment_size.width as i32,
            segment_size.height as i32 + 15,
            segment_size.width,
            segment_size.height,
        ),
        DisplaySegment::Center => (
            (size.width / 4) as i32,
            (size.height / 4) as i32 + 15,
            segment_size.width,
            segment_size.height,
        ),
    };

    let text_size = font.font.character_size;
    let text_length = name.len() as i32 * text_size.width as i32;

    let text_start = Point::new(x + (width as i32 - text_length) / 2, y);

    Text::with_text_style(
        name,
        text_start,
        font,
        TextStyleBuilder::new().baseline(Baseline::Top).build(),
    )
    .draw(display)
    .unwrap();
}

pub fn send_request<'a, 's, MODE>(socket: &mut Socket<'s, 'a, MODE>, request: &'a [u8])
where
    MODE: smoltcp::phy::Device,
{
    socket.write(request).unwrap();
    socket.flush().unwrap();
}

pub fn get_response<'a, 's, MODE>(
    mut socket: Socket<'s, 'a, MODE>,
) -> Result<([u8; 4096], usize), ()>
where
    MODE: smoltcp::phy::Device,
{
    let mut buffer = [0u8; 4096];
    let mut total_size = 0usize;

    loop {
        if total_size >= buffer.len() {
            // Buffer is full
            println!("Buffer is full, processed {} bytes", total_size);
            // Here you might want to process the buffer and then clear it
            total_size = 0;
            break;
        }

        let buffer_slice = &mut buffer[total_size..]; // Slice the buffer from the current total_size to the end
        match socket.read(buffer_slice) {
            Ok(0) => {
                // The connection has been closed by the peer
                println!("Connection closed, total read size: {}", total_size);
                break;
            }
            Ok(len) => {
                println!("Read {} bytes", len);
                total_size += len;
                // buffer[..total_size] now contains the data read in this
                // iteration
            }
            Err(e) => {
                println!("Failed to read from socket: {:?}", e);
                break;
            }
        }
    }

    socket.disconnect();
    println!("Socket disconnected, waiting...");
    let deadline = time::now() + Duration::secs(5);
    while time::now() < deadline {
        socket.work();
    }
    println!("Waiting finished");

    Ok((buffer, total_size))
}

pub fn get_time<'a, 's, MODE>(mut socket: Socket<'s, 'a, MODE>) -> Result<(u8, u8, u8), ()>
where
    MODE: smoltcp::phy::Device,
{
    let request =
        "GET /api/timezone/Europe/Prague HTTP/1.1\r\nHost: worldtimeapi.org\r\n\r\n".as_bytes();

    // Using classic "worldtime.api" to get time
    send_request(&mut socket, request);

    let (response, total_size) = get_response(socket).unwrap();

    if let Some(timestamp) = find_unixtime(&response[..total_size]) {
        let mut timestamp = timestamp;
        timestamp += 120 * 60 + 10; // align with CEST and compensate socket delay
        return Ok(timestamp_to_hms(timestamp));
    } else {
        println!("Failed to find or parse the 'unixtime' field.");
        return Err(());
    }
}

pub fn get_timestamp<'a, 's, MODE>(mut socket: Socket<'s, 'a, MODE>) -> Result<u64, ()>
where
    MODE: smoltcp::phy::Device,
{
    let request =
        "GET /api/timezone/Europe/Prague HTTP/1.1\r\nHost: worldtimeapi.org\r\n\r\n".as_bytes();

    // Using classic "worldtime.api" to get time
    send_request(&mut socket, request);

    let (response, total_size) = get_response(socket).unwrap();

    if let Some(timestamp) = find_unixtime(&response[..total_size]) {
        let mut timestamp = timestamp;
        timestamp += 120 * 60 + 10; // align with CEST and compensate socket delay
        return Ok(timestamp);
    } else {
        println!("Failed to find or parse the 'unixtime' field.");
        return Err(());
    }
}

pub fn timestamp_to_hms(timestamp: u64) -> (u8, u8, u8) {
    let seconds_per_minute = 60;
    let minutes_per_hour = 60;
    let hours_per_day = 24;
    let seconds_per_hour = seconds_per_minute * minutes_per_hour;
    let seconds_per_day = seconds_per_hour * hours_per_day;

    let hours = (timestamp % seconds_per_day) / seconds_per_hour;
    let minutes = (timestamp % seconds_per_hour) / seconds_per_minute;
    let seconds = timestamp % seconds_per_minute;

    (hours as u8, minutes as u8, seconds as u8)
}

pub fn weekday_from_timestamp(timestamp: &u64) -> &'static str {
    let days_since_1970 = timestamp / 86400; // seconds in a day
    let day_of_week = (days_since_1970 + 4) % 7; // Adjusting the offset since 1-1-1970 was a Thursday
    match day_of_week {
        0 => "Sunday",
        1 => "Monday",
        2 => "Tuesday",
        3 => "Wednesday",
        4 => "Thursday",
        5 => "Friday",
        6 => "Saturday",
        _ => "Error",
    }
}

pub fn find_unixtime(response: &[u8]) -> Option<u64> {
    // Convert the response to a string slice
    let response_str = core::str::from_utf8(response).ok()?;

    // Look for the "unixtime" key in the response
    let unixtime_key = b"\"unixtime\":";
    if let Some(start) = response_str.find(core::str::from_utf8(unixtime_key).ok()?) {
        // Find the start of the number (skipping the key and any potential spaces)
        let number_start = start + unixtime_key.len();
        let number_end = response_str[number_start..]
            .find(|c: char| !c.is_digit(10) && c != ' ')
            .map_or(response_str.len(), |end| number_start + end);

        // Parse the number
        response_str[number_start..number_end].parse().ok()
    } else {
        None
    }
}

fn write_to_segment<
    M: esp_hal::spi::master::Instance + 'static,
    RST: OutputPin<Error = core::convert::Infallible>,
    DC: OutputPin<Error = core::convert::Infallible>,
>(
    display: &mut mipidsi::Display<
        SPIInterface<
            ExclusiveDevice<spi::master::Spi<'static, esp_hal::Blocking, M>, NoPin, Delay>,
            DC,
        >,
        mipidsi::models::ILI9341Rgb565,
        RST,
    >,
    segment: DisplaySegment,
    text: &str,
    font: MonoTextStyle<Rgb565>,
) {
    let size = display.size();
    let segment_size = Size::new(size.width / 2, size.height / 2);

    let (x, y, width, height) = match segment {
        DisplaySegment::TopLeft => (0, 0, segment_size.width, segment_size.height),
        DisplaySegment::TopRight => (
            segment_size.width as i32,
            0,
            segment_size.width,
            segment_size.height,
        ),
        DisplaySegment::BottomLeft => (
            0,
            segment_size.height as i32,
            segment_size.width,
            segment_size.height,
        ),
        DisplaySegment::BottomRight => (
            segment_size.width as i32,
            segment_size.height as i32,
            segment_size.width,
            segment_size.height,
        ),
        DisplaySegment::Center => (
            (size.width / 4) as i32,
            (size.height / 4) as i32,
            segment_size.width,
            segment_size.height,
        ),
    };

    let char_size = font.font.character_size;
    let text_length = text.len() as i32 * char_size.width as i32;

    let section_name_height = char_size.height as i32 + 15;

    // Not forget to clean previous string (just white rect?)
    let clear_rect = Rectangle::new(
        Point::new(x, y + section_name_height),
        Size::new(width, height - section_name_height as u32),
    );
    let clear_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::WHITE)
        .build();

    clear_rect.into_styled(clear_style).draw(display).unwrap();

    let text_start = Point::new(
        x + (width as i32 - text_length) / 2,
        y + (height as i32 - char_size.height as i32) / 2,
    );

    Text::with_text_style(
        text,
        text_start,
        font,
        TextStyleBuilder::new().baseline(Baseline::Top).build(),
    )
    .draw(display)
    .unwrap();
}
