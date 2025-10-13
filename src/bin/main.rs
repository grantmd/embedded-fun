#![no_std]
#![no_main]

use core::str::from_utf8;
use embedded_storage::{ReadStorage, Storage};
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::time::{Duration, Instant};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart, UartRx};
use esp_hal::Blocking;
use esp_storage::FlashStorage;
use heapless::String;
use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use smoltcp::phy::ChecksumCapabilities;
use smoltcp::socket::{dhcpv4, raw};
use smoltcp::time::Instant as SmoltcpInstant;
use smoltcp::wire::{
    HardwareAddress, Icmpv4Repr, IpAddress, IpCidr, IpProtocol, IpVersion, Ipv4Address,
};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    esp_println::println!("Panic: {}", info);
    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

const WIFI_CREDS_OFFSET: u32 = 0x9000;
const MAX_SSID_LEN: usize = 32;
const MAX_PASSWORD_LEN: usize = 64;
const MAGIC_MARKER: u32 = 0xDEADBEEF;

#[derive(Debug)]
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

            if pos < buffer.len() - 1 && ch >= 32 && ch <= 126 {
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

#[main]
fn main() -> ! {
    esp_println::println!("🚀 Embedded Fun!");
    esp_println::println!("==================================");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    // Initialize UART for user input
    let uart_config = UartConfig::default();
    let uart0 = Uart::new(peripherals.UART0, uart_config).unwrap();
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
                    loop {}
                }
            }
        }
    };

    // Initialize WiFi
    esp_println::println!("\nInitializing WiFi hardware...");

    // Convert heapless::String to alloc::String for WiFi configuration
    use alloc::string::String as AllocString;

    let wifi_config = esp_wifi::wifi::Configuration::Client(esp_wifi::wifi::ClientConfiguration {
        ssid: AllocString::from(credentials.ssid.as_str()),
        password: AllocString::from(credentials.password.as_str()),
        auth_method: esp_wifi::wifi::AuthMethod::WPA2Personal,
        bssid: None,
        channel: None,
    });

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init = esp_wifi::init(timg0.timer0, esp_hal::rng::Rng::new(peripherals.RNG)).unwrap();
    let (mut wifi_controller, mut wifi_ifaces) =
        esp_wifi::wifi::new(&wifi_init, peripherals.WIFI).unwrap();

    wifi_controller.set_configuration(&wifi_config).unwrap();

    esp_println::print!("Starting WiFi... ");
    match wifi_controller.start() {
        Ok(()) => {
            while !wifi_controller.is_started().unwrap() {}
            esp_println::println!("Started!");
        }
        Err(e) => {
            esp_println::println!("Failed!");
            esp_println::println!("Error starting WiFi: {:?}", e);
            loop {
                esp_println::println!(
                    "System halted due to WiFi start failure. Reset to try again."
                );
                let delay_start = Instant::now();
                while delay_start.elapsed() < Duration::from_millis(5000) {}
            }
        }
    }
    esp_println::print!("Connecting to WiFi... ");
    match wifi_controller.connect() {
        Ok(()) => {
            // Wait for connection with timeout
            let start = Instant::now();
            let timeout = Duration::from_millis(30000); // 30 second timeout
            let mut connected = false;
            let mut connection_error = false;

            loop {
                match wifi_controller.is_connected() {
                    Ok(true) => {
                        connected = true;
                        break;
                    }
                    Ok(false) => {
                        // Still trying to connect
                        if start.elapsed() > timeout {
                            esp_println::println!("Connection timeout after 30 seconds");
                            break;
                        }
                    }
                    Err(e) => {
                        // Connection failed with an error
                        esp_println::println!("Connection status check error: {:?}", e);
                        connection_error = true;
                        break;
                    }
                }

                // Small delay to avoid busy waiting
                let delay_start = Instant::now();
                while delay_start.elapsed() < Duration::from_millis(100) {}
            }

            if connected {
                esp_println::println!("✓ Successfully connected to WiFi!");
            } else if connection_error {
                esp_println::println!("✗ WiFi connection failed with error");
                esp_println::println!("The WiFi controller reported a disconnection");
                esp_println::println!("Possible reasons:");
                esp_println::println!("  - Wrong password");
                esp_println::println!("  - SSID not found");
                esp_println::println!("  - Network rejected the connection");
                esp_println::println!("  - Authentication method mismatch");
                esp_println::println!("\nCredentials used:");
                esp_println::println!("  - SSID: '{}'", credentials.ssid);
                esp_println::println!("  - Password length: {} chars", credentials.password.len());
            } else {
                esp_println::println!("✗ Failed to connect to WiFi network (timeout)");
                esp_println::println!("Please check:");
                esp_println::println!("  - SSID is correct: '{}'", credentials.ssid);
                esp_println::println!("  - Password is correct");
                esp_println::println!("  - WiFi network is in range");
                esp_println::println!("  - WiFi network supports WPA2/WPA3");
            }
        }
        Err(e) => {
            esp_println::println!(" Failed!");
            esp_println::println!("Error connecting to WiFi: {:?}", e);
            esp_println::println!("Connection error details:");
            esp_println::println!("  - SSID: '{}'", credentials.ssid);
            esp_println::println!("  - Password length: {} chars", credentials.password.len());
            esp_println::println!("  - Auth method: WPA2/WPA3 Personal");

            // Try to provide more specific error guidance
            esp_println::println!("\nPossible causes:");
            esp_println::println!("  1. Invalid SSID or password");
            esp_println::println!("  2. WiFi network out of range");
            esp_println::println!("  3. Incompatible security settings");
            esp_println::println!("  4. Network rejecting connection");

            esp_println::println!("\nPress reset + 'r' during boot to enter new credentials");
        }
    }

    if !wifi_controller.is_connected().unwrap() {
        panic!("WiFi connection failed");
    }

    // Set up networking with smoltcp directly
    esp_println::println!("\n🌐 Setting up network stack with DHCP...");

    // Create smoltcp interface
    let mut wifi_device = wifi_ifaces.sta;
    let ethernet_addr = HardwareAddress::Ethernet(smoltcp::wire::EthernetAddress::from_bytes(
        &wifi_device.mac_address(),
    ));
    let config = Config::new(ethernet_addr);
    let mut iface = Interface::new(config, &mut wifi_device, SmoltcpInstant::from_millis(0));

    // Set up sockets
    let mut socket_storage = [SocketStorage::EMPTY; 2];
    let mut sockets = SocketSet::new(&mut socket_storage[..]);

    // Create DHCP socket
    let dhcp_socket = dhcpv4::Socket::new();
    let dhcp_handle = sockets.add(dhcp_socket);

    // Create raw socket for ICMP ping
    let mut raw_rx_buffer = [0; 1500];
    let mut raw_tx_buffer = [0; 1500];
    let mut raw_rx_metadata = [raw::PacketMetadata::EMPTY; 4];
    let mut raw_tx_metadata = [raw::PacketMetadata::EMPTY; 4];

    let raw_socket = raw::Socket::new(
        IpVersion::Ipv4,
        IpProtocol::Icmp,
        raw::PacketBuffer::new(&mut raw_rx_metadata[..], &mut raw_rx_buffer[..]),
        raw::PacketBuffer::new(&mut raw_tx_metadata[..], &mut raw_tx_buffer[..]),
    );
    let raw_handle = sockets.add(raw_socket);

    esp_println::println!("📡 Starting DHCP client...");

    let mut dhcp_configured = false;
    let mut ping_sequence = 0u16;
    let mut last_ping_ms = 0i64;
    let ping_target = Ipv4Address::new(8, 8, 8, 8);
    let mut counter = 0u64;

    loop {
        counter += 1;
        let timestamp = SmoltcpInstant::from_millis(counter as i64);

        // Poll the interface
        let poll_result = iface.poll(timestamp, &mut wifi_device, &mut sockets);

        // Handle DHCP
        let dhcp_socket = sockets.get_mut::<dhcpv4::Socket>(dhcp_handle);
        match dhcp_socket.poll() {
            None => {}
            Some(dhcpv4::Event::Configured(config)) => {
                if !dhcp_configured {
                    esp_println::println!("✅ DHCP configuration obtained!");
                    esp_println::println!("  IP Address: {}", config.address);
                    esp_println::println!("  Gateway: {:?}", config.router);
                    esp_println::println!("  DNS: {:?}", config.dns_servers);

                    // Configure interface with obtained IP
                    iface.update_ip_addrs(|addrs| {
                        addrs
                            .push(IpCidr::new(IpAddress::Ipv4(config.address.address()), 24))
                            .ok();
                    });

                    if let Some(gateway) = config.router {
                        iface.routes_mut().add_default_ipv4_route(gateway).unwrap();
                    }

                    dhcp_configured = true;
                    esp_println::println!("\n🏓 Starting ping test to 8.8.8.8...");
                }
            }
            Some(dhcpv4::Event::Deconfigured) => {
                esp_println::println!("⚠️  DHCP lease expired, renewing...");
                dhcp_configured = false;
            }
        }

        // Send pings if we have IP configured
        if dhcp_configured && timestamp.millis() - last_ping_ms > 2000 {
            ping_sequence = ping_sequence.wrapping_add(1);

            let raw_socket = sockets.get_mut::<raw::Socket>(raw_handle);

            // Build ICMP echo request
            let icmp_repr = Icmpv4Repr::EchoRequest {
                ident: 0x1234,
                seq_no: ping_sequence,
                data: b"esp32-ping",
            };

            let mut packet_buffer = [0u8; 64];
            let packet_len = icmp_repr.buffer_len();

            if packet_len <= packet_buffer.len() {
                let mut packet =
                    smoltcp::wire::Icmpv4Packet::new_unchecked(&mut packet_buffer[..packet_len]);
                let checksum_caps = ChecksumCapabilities::default();
                icmp_repr.emit(&mut packet, &checksum_caps);

                match raw_socket.send_slice(&packet_buffer[..packet_len]) {
                    Ok(_) => {
                        esp_println::print!("Ping #{} sent to {}... ", ping_sequence, ping_target);
                        last_ping_ms = timestamp.millis();
                    }
                    Err(e) => {
                        esp_println::println!("Failed to send ping: {:?}", e);
                    }
                }
            }
        }

        // Check for ping responses
        if dhcp_configured {
            let raw_socket = sockets.get_mut::<raw::Socket>(raw_handle);
            if raw_socket.can_recv() {
                let mut buffer = [0u8; 256];
                match raw_socket.recv_slice(&mut buffer) {
                    Ok(size) => {
                        if size >= 8 {
                            let icmp_packet =
                                smoltcp::wire::Icmpv4Packet::new_unchecked(&buffer[..size]);
                            let checksum_caps = ChecksumCapabilities::default();
                            if let Ok(icmp_repr) = Icmpv4Repr::parse(&icmp_packet, &checksum_caps) {
                                match icmp_repr {
                                    Icmpv4Repr::EchoReply {
                                        ident,
                                        seq_no,
                                        data: _,
                                    } if ident == 0x1234 => {
                                        esp_println::println!(
                                            "Reply from {}: seq={} bytes={}",
                                            ping_target,
                                            seq_no,
                                            size
                                        );
                                    }
                                    _ => {}
                                }
                            }
                        }
                    }
                    Err(_) => {}
                }
            }
        }

        // Small delay
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(1) {}
    }
}
