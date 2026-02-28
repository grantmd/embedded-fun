#![no_std]
#![no_main]

use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_time::{Duration, Timer};
use esp_hal::gpio::Output;
use esp_hal::i2c::master::{BusTimeout, Config, I2c};
use esp_hal::time::Rate;
use esp_wifi::wifi::{AuthMethod, ClientConfiguration, Configuration};

use esp32_wifi::led::led_blink_task;
use esp32_wifi::network::credentials::{get_credentials_from_user, load_credentials};
use esp32_wifi::network::wifi::{net_task, ping_task, wifi_task};
use esp32_wifi::sensors::fuel_gauge::fuel_gauge_task;
use esp32_wifi::sensors::gps::gps_task;
use esp32_wifi::sensors::imu::imu_task;
use esp32_wifi::I2cBusBlocking;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    esp_println::println!("Panic: {}", info);
    loop {}
}

extern crate alloc;
use alloc::string::String as AllocString;

esp_bootloader_esp_idf::esp_app_desc!();

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
    let credentials = match load_credentials() {
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
                    loop {}
                }
            }
        }
    };

    // Initialize embassy time driver (required before spawning tasks)
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Initialize status LED on GPIO13 and spawn blink task
    let led = Output::new(
        peripherals.GPIO13,
        esp_hal::gpio::Level::Low,
        Default::default(),
    );
    spawner.spawn(led_blink_task(led)).ok();

    // Initialize I2C bus (GPIO22=SCL, GPIO21=SDA) in blocking mode at 100kHz
    // Reduced from 400kHz for GPS clock stretching compatibility
    esp_println::println!("Initializing I2C bus...");
    let i2c_config = Config::default()
        .with_frequency(Rate::from_khz(100))
        .with_timeout(BusTimeout::Maximum);
    esp_println::println!("I2C frequency: 100 kHz, timeout: {:?}", i2c_config.timeout());
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

    let wifi_config = Configuration::Client(ClientConfiguration {
        ssid: AllocString::from(credentials.ssid.as_str()),
        password: AllocString::from(credentials.password.as_str()),
        auth_method: AuthMethod::WPA2Personal,
        bssid: None,
        channel: None,
    });

    let timg1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    static WIFI_INIT: static_cell::StaticCell<esp_wifi::EspWifiController> =
        static_cell::StaticCell::new();
    let wifi_init = WIFI_INIT.init(esp_wifi::init(timg1.timer0, rng).unwrap());

    let (mut controller, wifi_interfaces) =
        esp_wifi::wifi::new(wifi_init, peripherals.WIFI).unwrap();

    // Set up embassy-net
    let config = embassy_net::Config::dhcpv4(embassy_net::DhcpConfig::default());

    static RESOURCES: static_cell::StaticCell<embassy_net::StackResources<3>> =
        static_cell::StaticCell::new();
    let resources = RESOURCES.init(embassy_net::StackResources::<3>::new());

    let (stack, runner) = embassy_net::new(wifi_interfaces.sta, config, resources, seed);

    static STACK_REF: static_cell::StaticCell<embassy_net::Stack<'static>> =
        static_cell::StaticCell::new();
    let stack_ref = STACK_REF.init(stack);

    // Spawn network tasks
    spawner.spawn(net_task(runner)).ok();
    spawner.spawn(wifi_task(stack_ref)).ok();
    spawner.spawn(ping_task(stack_ref)).ok();

    // Configure and start WiFi
    controller.set_configuration(&wifi_config).unwrap();

    esp_println::print!("Starting WiFi... ");
    controller.start_async().await.unwrap();
    esp_println::println!("Started!");

    esp_println::print!("Connecting to WiFi... ");
    controller.connect_async().await.unwrap();
    esp_println::println!("WiFi connected!");

    // Main loop
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
