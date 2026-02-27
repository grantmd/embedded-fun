use core::str::from_utf8;

use embedded_storage::{ReadStorage, Storage};
use esp_hal::uart::UartRx;
use esp_hal::Blocking;
use esp_storage::FlashStorage;
use heapless::String;

const WIFI_CREDS_OFFSET: u32 = 0x9000;
const MAX_SSID_LEN: usize = 32;
const MAX_PASSWORD_LEN: usize = 64;
const MAGIC_MARKER: u32 = 0xDEADBEEF;

#[derive(Debug, Clone)]
pub struct WifiCredentials {
    pub ssid: String<32>,
    pub password: String<64>,
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

pub fn load_credentials() -> Option<WifiCredentials> {
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

pub fn get_credentials_from_user(rx: &mut UartRx<'_, Blocking>) -> Option<WifiCredentials> {
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
