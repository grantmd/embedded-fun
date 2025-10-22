#![no_std]
#![no_main]

use core::cell::RefCell;
use core::str::from_utf8;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_storage::{ReadStorage, Storage};
use esp_hal::gpio::Output;
use esp_hal::i2c::master::I2c;
use esp_hal::uart::UartRx;
use esp_hal::Async;
use esp_hal::Blocking;
use esp_storage::FlashStorage;
use heapless::String;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    esp_println::println!("Panic: {}", info);
    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// Type aliases for our shared I2C buses (async and blocking)
type I2cBusAsync = Mutex<NoopRawMutex, I2c<'static, Async>>;
type I2cBusBlocking = BlockingMutex<CriticalSectionRawMutex, RefCell<I2c<'static, Blocking>>>;
type I2cProxy = I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>;

const WIFI_CREDS_OFFSET: u32 = 0x9000;
const MAX_SSID_LEN: usize = 32;
const MAX_PASSWORD_LEN: usize = 64;
const MAGIC_MARKER: u32 = 0xDEADBEEF;

#[derive(Debug, Clone)]
struct WifiCredentials {
    ssid: String<32>,
    password: String<64>,
}

impl WifiCredentials {
    fn new() -> Self {
        Self {
            ssid: String::new(),
            password: String::new(),
        }
    }

    fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 8 {
            return None;
        }

        let magic = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        if magic != MAGIC_MARKER {
            return None;
        }

        let ssid_len = data[4] as usize;
        let password_len = data[5] as usize;

        if ssid_len > MAX_SSID_LEN || password_len > MAX_PASSWORD_LEN {
            return None;
        }

        if data.len() < 8 + ssid_len + password_len {
            return None;
        }

        let ssid_bytes = &data[8..8 + ssid_len];
        let password_bytes = &data[8 + ssid_len..8 + ssid_len + password_len];

        let ssid = from_utf8(ssid_bytes).ok()?;
        let password = from_utf8(password_bytes).ok()?;

        let mut creds = Self::new();
        creds.ssid.push_str(ssid).ok()?;
        creds.password.push_str(password).ok()?;

        Some(creds)
    }

    fn to_bytes(&self) -> heapless::Vec<u8, 256> {
        let mut bytes = heapless::Vec::new();

        bytes.extend_from_slice(&MAGIC_MARKER.to_le_bytes()).ok();
        bytes.push(self.ssid.len() as u8).ok();
        bytes.push(self.password.len() as u8).ok();
        bytes.push(0).ok(); // Reserved
        bytes.push(0).ok(); // Reserved
        bytes.extend_from_slice(self.ssid.as_bytes()).ok();
        bytes.extend_from_slice(self.password.as_bytes()).ok();

        bytes
    }
}

fn load_credentials() -> Option<WifiCredentials> {
    let mut storage = FlashStorage::new();
    let mut buffer = [0u8; 256];

    match storage.read(WIFI_CREDS_OFFSET, &mut buffer) {
        Ok(_) => WifiCredentials::from_bytes(&buffer),
        Err(_) => None,
    }
}

fn save_credentials(creds: &WifiCredentials) -> Result<(), &'static str> {
    let mut storage = FlashStorage::new();
    let data = creds.to_bytes();

    storage
        .write(WIFI_CREDS_OFFSET, data.as_slice())
        .map_err(|_| "Failed to write credentials to flash")
}

fn read_line_from_uart(rx: &mut UartRx<'_, Blocking>, buffer: &mut [u8], prompt: &str) -> usize {
    esp_println::print!("{}", prompt);

    let mut pos = 0;
    loop {
        let mut byte = [0u8; 1];
        if rx.read(&mut byte).is_ok() {
            let ch = byte[0];

            if ch == b'\r' || ch == b'\n' {
                esp_println::println!();
                break;
            }

            if ch == 0x7F || ch == 0x08 {
                if pos > 0 {
                    pos -= 1;
                    esp_println::print!("\x08 \x08");
                }
                continue;
            }

            if pos < buffer.len() - 1 && (32..=126).contains(&ch) {
                buffer[pos] = ch;
                pos += 1;
                esp_println::print!("{}", ch as char);
            }
        }
    }

    pos
}

fn get_credentials_from_user(rx: &mut UartRx<'_, Blocking>) -> Option<WifiCredentials> {
    esp_println::println!("\n=== WiFi Configuration ===");
    esp_println::println!("Please enter WiFi credentials:");

    let mut ssid_buffer = [0u8; MAX_SSID_LEN];
    let ssid_len = read_line_from_uart(rx, &mut ssid_buffer, "SSID: ");

    if ssid_len == 0 {
        esp_println::println!("Error: SSID cannot be empty");
        return None;
    }

    let mut password_buffer = [0u8; MAX_PASSWORD_LEN];
    let password_len = read_line_from_uart(rx, &mut password_buffer, "Password: ");

    let ssid_str = from_utf8(&ssid_buffer[..ssid_len]).ok()?;
    let password_str = from_utf8(&password_buffer[..password_len]).ok()?;

    let mut creds = WifiCredentials::new();
    creds.ssid.push_str(ssid_str).ok()?;
    creds.password.push_str(password_str).ok()?;

    esp_println::println!("Save credentials for next boot? (y/n): ");
    let mut response = [0u8; 1];
    if rx.read(&mut response).is_ok() && (response[0] == b'y' || response[0] == b'Y') {
        match save_credentials(&creds) {
            Ok(_) => esp_println::println!("Credentials saved to flash"),
            Err(e) => esp_println::println!("Failed to save credentials: {}", e),
        }
    }

    Some(creds)
}

// Signals to notify tasks that WiFi network is ready
static WIFI_CONNECTED: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static NETWORK_CONNECTED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task]
async fn fuel_gauge_task(i2c_bus: &'static I2cBusBlocking) {
    use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
    use max170xx::Max17048;

    esp_println::println!("Initializing MAX17048 fuel gauge...");

    // Create I2C device proxy for this task
    let i2c = I2cDevice::new(i2c_bus);

    // Initialize the MAX17048
    let mut fuel_gauge = Max17048::new(i2c);

    // Try to read the version to verify communication
    match fuel_gauge.version() {
        Ok(version) => {
            esp_println::println!("✓ MAX17048 detected, version: 0x{:04X}", version);
        }
        Err(_) => {
            esp_println::println!("✗ Failed to communicate with MAX17048");
            return;
        }
    }

    // Startup can be noisy, reset calculations
    fuel_gauge.quickstart().unwrap();

    loop {
        // Read state of charge (SOC), voltage, and charge rate
        match fuel_gauge.soc() {
            Ok(soc) => match fuel_gauge.voltage() {
                Ok(voltage) => match fuel_gauge.charge_rate() {
                    Ok(rate) => {
                        esp_println::println!(
                            "Battery: {:.1}% | {:.3}V | {:.2}%/hr",
                            soc,
                            voltage,
                            rate
                        );
                    }
                    Err(_) => {
                        esp_println::println!(
                            "Battery: {:.1}% | {:.3}V | charge rate error",
                            soc,
                            voltage
                        );
                    }
                },
                Err(_) => {
                    esp_println::println!("Battery: {:.1}% | voltage read error", soc);
                }
            },
            Err(_) => {
                esp_println::println!("Battery: SOC read error");
            }
        }

        Timer::after(Duration::from_secs(60)).await;
    }
}

#[embassy_executor::task]
async fn gps_task(i2c_bus: &'static I2cBusBlocking) {
    use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
    use embedded_hal::i2c::I2c;
    use ublox::Parser;

    const GPS_ADDR: u8 = 0x42;
    const REG_DATA_STREAM: u8 = 0xFF;
    const REG_AVAIL_MSB: u8 = 0xFD;

    esp_println::println!("Initializing MAX-M10S GPS...");

    let mut i2c = I2cDevice::new(i2c_bus);
    let mut parser = Parser::<ublox::FixedBuffer<512>>::new_fixed();

    // Give GPS module time to boot up
    Timer::after(Duration::from_secs(2)).await;

    // Tickle the device to start I2C broadcasting by sending UBX-MON-VER command
    // This command requests the device to send its version information
    // Retry until successful
    esp_println::println!("GPS: Sending initialization command...");
    let init_cmd: [u8; 8] = [0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34];
    loop {
        match i2c.write(GPS_ADDR, &init_cmd) {
            Ok(_) => {
                esp_println::println!("GPS: Initialization command sent successfully");
                break;
            }
            Err(e) => {
                esp_println::println!("GPS: Failed to send init command: {:?}, retrying...", e);
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }

    Timer::after(Duration::from_millis(100)).await;

    esp_println::println!("GPS: Starting data polling...");

    loop {
        // Read the number of available bytes from REG_AVAIL_MSB (2 bytes, big-endian)
        let mut avail_bytes = [0u8; 2];
        match i2c.write_read(GPS_ADDR, &[REG_AVAIL_MSB], &mut avail_bytes) {
            Ok(_) => {
                let available = u16::from_be_bytes(avail_bytes) as usize;

                if available > 0 {
                    // Cap the read size to our buffer capacity
                    let read_size = available.min(256);
                    let mut buffer = [0u8; 256];

                    // Read from REG_DATA_STREAM
                    match i2c.write_read(GPS_ADDR, &[REG_DATA_STREAM], &mut buffer[..read_size]) {
                        Ok(_) => {
                            // Print raw bytes
                            esp_println::print!("GPS: Received {} bytes", read_size);
                            if available > read_size {
                                esp_println::print!(" ({} available, truncated)", available);
                            }
                            esp_println::print!(": ");
                            for i in 0..read_size {
                                esp_println::print!("{:02X} ", buffer[i]);
                            }
                            esp_println::println!();

                            // Parse UBX packets
                            let mut packets = parser.consume_ubx(&buffer[..read_size]);

                            // Print parsed packets
                            while let Some(packet_result) = packets.next() {
                                match packet_result {
                                    Ok(packet) => {
                                        esp_println::println!("GPS: Received packet: {:?}", packet);
                                    }
                                    Err(e) => {
                                        esp_println::println!("GPS: Parser error: {:?}", e);
                                    }
                                }
                            }
                        }
                        Err(_) => {}
                    }
                }
            }
            Err(_) => {}
        }

        // Poll at reasonable interval
        Timer::after(Duration::from_millis(250)).await;
    }
}

#[embassy_executor::task]
async fn imu_task(i2c_bus: &'static I2cBusBlocking) {
    use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
    use embedded_hal_02::blocking::delay::DelayMs;
    use embedded_hal_02::blocking::i2c::{Read, Write, WriteRead};
    use icm20948::{ICMI2C, ICM20948_CHIP_ADR};

    esp_println::println!("Initializing ICM-20948 IMU...");

    // Create I2C device proxy for this task
    let i2c_v1 = I2cDevice::new(i2c_bus);

    // Wrapper to adapt embedded-hal 1.0 I2C to embedded-hal 0.2
    struct I2cAdapter<I> {
        i2c: I,
    }

    impl<I> Write for I2cAdapter<I>
    where
        I: embedded_hal::i2c::I2c,
    {
        type Error = I::Error;

        fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            self.i2c.write(address, bytes)
        }
    }

    impl<I> Read for I2cAdapter<I>
    where
        I: embedded_hal::i2c::I2c,
    {
        type Error = I::Error;

        fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            self.i2c.read(address, buffer)
        }
    }

    impl<I> WriteRead for I2cAdapter<I>
    where
        I: embedded_hal::i2c::I2c,
    {
        type Error = I::Error;

        fn write_read(
            &mut self,
            address: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.i2c.write_read(address, bytes, buffer)
        }
    }

    let mut i2c_adapted = I2cAdapter { i2c: i2c_v1 };

    // Create a simple delay implementation
    struct SimpleDelay;
    impl DelayMs<u8> for SimpleDelay {
        fn delay_ms(&mut self, ms: u8) {
            // Use embassy timer for blocking delay
            embassy_time::block_for(Duration::from_millis(ms as u64));
        }
    }
    impl DelayMs<u16> for SimpleDelay {
        fn delay_ms(&mut self, ms: u16) {
            // Use embassy timer for blocking delay
            embassy_time::block_for(Duration::from_millis(ms as u64));
        }
    }
    let mut delay = SimpleDelay;

    // Initialize the ICM-20948 with address 0x69
    let mut imu = match ICMI2C::<_, _, ICM20948_CHIP_ADR>::new(&mut i2c_adapted) {
        Ok(imu) => imu,
        Err(e) => {
            esp_println::println!("✗ Failed to create ICM-20948: {:?}", e);
            return;
        }
    };

    match imu.init(&mut i2c_adapted, &mut delay) {
        Ok(_) => esp_println::println!("✓ ICM-20948 initialized"),
        Err(e) => {
            esp_println::println!("✗ Failed to initialize ICM-20948: {:?}", e);
            return;
        }
    }

    loop {
        // Read accelerometer and gyroscope data
        match imu.get_values_accel_gyro(&mut i2c_adapted) {
            Ok((ax, ay, az, gx, gy, gz)) => {
                // Scale the raw values
                let (ax_scaled, ay_scaled, az_scaled, gx_scaled, gy_scaled, gz_scaled) =
                    imu.scale_raw_accel_gyro((ax, ay, az, gx, gy, gz));

                esp_println::println!(
                    "IMU: Accel({:.3}, {:.3}, {:.3}) m/s² | Gyro({:.3}, {:.3}, {:.3}) rad/s",
                    ax_scaled, ay_scaled, az_scaled,
                    gx_scaled, gy_scaled, gz_scaled
                );
            }
            Err(e) => {
                esp_println::println!("IMU: Read error: {:?}", e);
            }
        }

        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn led_blink_task(mut led: Output<'static>) {
    // Blink while waiting for WiFi connection
    loop {
        led.toggle();

        // Use select to check both timeout and signal
        match embassy_futures::select::select(
            Timer::after(Duration::from_millis(250)),
            NETWORK_CONNECTED.wait(),
        )
        .await
        {
            embassy_futures::select::Either::First(_) => {
                // Timeout - continue blinking
                continue;
            }
            embassy_futures::select::Either::Second(_) => {
                // WiFi connected! Turn LED on solid
                led.set_high();
                break;
            }
        }
    }

    // Keep LED on solid
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[embassy_executor::task]
async fn wifi_task(stack: &'static embassy_net::Stack<'static>) {
    loop {
        if stack.is_link_up() {
            // Signal that WiFi is connected to all waiting tasks
            WIFI_CONNECTED.signal(());
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // Keep monitoring connection
    loop {
        Timer::after(Duration::from_secs(10)).await;
        if !stack.is_link_up() {
            esp_println::println!("WiFi connection lost!");
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, esp_wifi::wifi::WifiDevice<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn ping_task(stack: &'static embassy_net::Stack<'static>) {
    // Wait for WiFi to be connected
    esp_println::println!("Ping task waiting for WiFi connection...");
    WIFI_CONNECTED.wait().await;

    esp_println::println!("WiFi connected! Starting ping loop...");

    // Wait a bit more to ensure we have an IP address
    Timer::after(Duration::from_secs(2)).await;

    let mut seq = 0u16;

    loop {
        if !stack.is_link_up() {
            esp_println::println!("Network down, waiting for connection...");
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        // Check if we have an IP address
        if stack.config_v4().is_none() {
            esp_println::println!("No IP address yet, waiting...");
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        // This probably means we have a network connection
        NETWORK_CONNECTED.signal(());

        seq = seq.wrapping_add(1);

        match stack
            .dns_query("8.8.8.8", embassy_net::dns::DnsQueryType::A)
            .await
        {
            Ok(_) => {
                esp_println::println!("Ping #{}: 8.8.8.8 is reachable (via DNS query)", seq);
            }
            Err(e) => {
                esp_println::println!("Ping #{}: DNS query failed: {:?}", seq, e);
            }
        }

        Timer::after(Duration::from_secs(60)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::println!("🚀 Embedded Fun with Embassy!");
    esp_println::println!("==================================");

    let peripherals = esp_hal::init(Default::default());

    esp_alloc::heap_allocator!(size: 72 * 1024);

    // Initialize UART for user input
    let uart0 = esp_hal::uart::Uart::new(peripherals.UART0, Default::default()).unwrap();
    let (mut uart_rx, _uart_tx) = uart0.split();

    // Check for stored credentials
    esp_println::println!("Loading stored credentials...");
    let credentials = load_credentials();

    let credentials = match credentials {
        Some(creds) => {
            esp_println::println!("✓ Found stored credentials for SSID: {}", creds.ssid);
            creds
        }
        None => {
            esp_println::println!("No stored credentials found.");
            match get_credentials_from_user(&mut uart_rx) {
                Some(creds) => creds,
                None => {
                    esp_println::println!("Failed to get credentials from user");
                    esp_println::println!("System halted. Please reset to try again.");
                    #[allow(clippy::empty_loop)]
                    loop {
                        // Halt execution - intentionally empty loop
                        // We can't use panic!() here as we want a clean halt message
                    }
                }
            }
        }
    };

    // Initialize embassy time driver first (required before spawning tasks)
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Initialize status LED on GPIO13
    let led = Output::new(
        peripherals.GPIO13,
        esp_hal::gpio::Level::Low,
        Default::default(),
    );
    static LED_REF: static_cell::StaticCell<Output<'static>> = static_cell::StaticCell::new();
    let led_ref = LED_REF.init(led);

    // Spawn LED blink task early (will blink during WiFi connection, then stay solid)
    let led_owned = unsafe { core::ptr::read(led_ref as *const _) };
    spawner.spawn(led_blink_task(led_owned)).ok();

    // Initialize I2C bus (GPIO22=SCL, GPIO21=SDA) in blocking mode
    // Using default frequency (should be 100kHz - MAX17048 supports up to 400kHz, MAX-M10S up to 320kHz)
    esp_println::println!("Initializing I2C bus...");
    use esp_hal::i2c::master::Config;
    let i2c_config = Config::default();
    esp_println::println!("I2C timeout: {:?}", i2c_config.timeout());
    let i2c_blocking = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

    // Wrap in a BlockingMutex + RefCell for sharing across multiple tasks
    static I2C_BUS_BLOCKING: static_cell::StaticCell<I2cBusBlocking> =
        static_cell::StaticCell::new();
    let i2c_bus_blocking = I2C_BUS_BLOCKING.init(BlockingMutex::new(RefCell::new(i2c_blocking)));
    esp_println::println!(
        "✓ I2C bus initialized on GPIO21 (SDA) and GPIO22 (SCL) (blocking, shared)"
    );

    // Perform I2C bus scan
    esp_println::println!("\nScanning I2C bus...");
    {
        i2c_bus_blocking.lock(|bus| {
            let mut b = bus.borrow_mut();
            // Try to scan common addresses
            for addr in 0x08..=0x77 {
                let mut dummy = [0u8; 1];
                if b.read(addr, &mut dummy).is_ok() {
                    esp_println::println!("  Found device at 0x{:02X}", addr);
                }
            }
        });
    }
    esp_println::println!("I2C scan complete\n");

    // Spawn I2C sensor tasks
    spawner.spawn(fuel_gauge_task(i2c_bus_blocking)).ok();
    spawner.spawn(gps_task(i2c_bus_blocking)).ok();
    spawner.spawn(imu_task(i2c_bus_blocking)).ok();

    // Initialize WiFi
    esp_println::println!("Initializing WiFi hardware...");
    use alloc::string::String as AllocString;

    let wifi_config = esp_wifi::wifi::Configuration::Client(esp_wifi::wifi::ClientConfiguration {
        ssid: AllocString::from(credentials.ssid.as_str()),
        password: AllocString::from(credentials.password.as_str()),
        auth_method: esp_wifi::wifi::AuthMethod::WPA2Personal,
        bssid: None,
        channel: None,
    });

    let timg1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
    static WIFI_INIT: static_cell::StaticCell<esp_wifi::EspWifiController> =
        static_cell::StaticCell::new();
    let wifi_init = WIFI_INIT
        .init(esp_wifi::init(timg1.timer0, esp_hal::rng::Rng::new(peripherals.RNG)).unwrap());

    let (mut controller, wifi_interfaces) =
        esp_wifi::wifi::new(wifi_init, peripherals.WIFI).unwrap();

    // Set up embassy-net
    use embassy_net::DhcpConfig;

    let dhcp_config = DhcpConfig::default();
    let config = embassy_net::Config::dhcpv4(dhcp_config);

    // Allocate stack resources statically
    static RESOURCES: static_cell::StaticCell<embassy_net::StackResources<3>> =
        static_cell::StaticCell::new();
    let resources = RESOURCES.init(embassy_net::StackResources::<3>::new());

    let seed = 1234u64; // Simple seed, could be improved with RNG
    let (stack, runner) = embassy_net::new(wifi_interfaces.sta, config, resources, seed);

    static STACK_REF: static_cell::StaticCell<embassy_net::Stack<'static>> =
        static_cell::StaticCell::new();
    let stack_ref = STACK_REF.init(stack);

    static RUNNER_REF: static_cell::StaticCell<
        embassy_net::Runner<'static, esp_wifi::wifi::WifiDevice<'static>>,
    > = static_cell::StaticCell::new();
    let runner_ref = RUNNER_REF.init(runner);

    // Spawn network task
    // Take ownership from StaticCell by replacing with a dummy - this is safe because we only call this once
    let runner_owned = unsafe { core::ptr::read(runner_ref as *const _) };
    spawner.spawn(net_task(runner_owned)).ok();

    spawner.spawn(wifi_task(stack_ref)).ok();

    // Spawn ping task (will wait for WiFi connection)
    spawner.spawn(ping_task(stack_ref)).ok();

    // Configure and start WiFi
    controller.set_configuration(&wifi_config).unwrap();

    esp_println::print!("Starting WiFi... ");
    controller.start_async().await.unwrap();
    esp_println::println!("Started!");

    esp_println::print!("Connecting to WiFi... ");
    controller.connect_async().await.unwrap();
    esp_println::println!("Connected!");

    // Main loop - could add other tasks here
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
