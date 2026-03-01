use core::str::from_utf8;

use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_futures::block_on;
use esp_hal::uart::UartRx;
use esp_hal::Blocking;
use esp_storage::FlashStorage;
use heapless::String;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage};

/// Flash range: two 4KB pages starting at 0x9000
const FLASH_RANGE: core::ops::Range<u32> = 0x9000..0xB000;

/// Key assignments
const KEY_WIFI_SSID: u8 = 0;
const KEY_WIFI_PASSWORD: u8 = 1;
const KEY_API_KEY: u8 = 2;

const MAX_SSID_LEN: usize = 32;
const MAX_PASSWORD_LEN: usize = 64;
const MAX_API_KEY_LEN: usize = 48;

fn flash_store(key: u8, value: &str) -> Result<(), &'static str> {
    let flash = BlockingAsync::new(FlashStorage::new());
    let mut storage = MapStorage::<u8, _, _>::new(
        flash,
        const { MapConfig::new(FLASH_RANGE) },
        NoCache::new(),
    );
    let mut data_buffer = [0u8; 128];
    let val: String<64> = String::try_from(value).map_err(|_| "Value too long")?;
    block_on(storage.store_item(&mut data_buffer, &key, &val))
        .map_err(|_| "Failed to write to flash")
}

fn flash_fetch<const N: usize>(key: u8) -> Option<String<N>> {
    let flash = BlockingAsync::new(FlashStorage::new());
    let mut storage = MapStorage::<u8, _, _>::new(
        flash,
        const { MapConfig::new(FLASH_RANGE) },
        NoCache::new(),
    );
    let mut data_buffer = [0u8; 128];
    block_on(storage.fetch_item::<String<N>>(&mut data_buffer, &key)).ok()?
}

#[derive(Debug, Clone)]
pub struct WifiCredentials {
    pub ssid: String<32>,
    pub password: String<64>,
}

pub fn load_credentials() -> Option<WifiCredentials> {
    let ssid: String<32> = flash_fetch(KEY_WIFI_SSID)?;
    let password: String<64> = flash_fetch(KEY_WIFI_PASSWORD)?;
    if ssid.is_empty() {
        return None;
    }
    Some(WifiCredentials { ssid, password })
}

fn save_credentials(creds: &WifiCredentials) -> Result<(), &'static str> {
    flash_store(KEY_WIFI_SSID, creds.ssid.as_str())?;
    flash_store(KEY_WIFI_PASSWORD, creds.password.as_str())?;
    Ok(())
}

pub fn load_api_key() -> Option<String<48>> {
    let key: String<48> = flash_fetch(KEY_API_KEY)?;
    if key.is_empty() {
        return None;
    }
    Some(key)
}

fn save_api_key(key: &str) -> Result<(), &'static str> {
    flash_store(KEY_API_KEY, key)
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

pub fn get_api_key_from_user(rx: &mut UartRx<'_, Blocking>) -> Option<String<48>> {
    esp_println::println!("\n=== ThingSpeak API Key ===");

    let mut key_buffer = [0u8; MAX_API_KEY_LEN];
    let key_len = read_line_from_uart(rx, &mut key_buffer, "API Key: ");

    if key_len == 0 {
        esp_println::println!("Error: API key cannot be empty");
        return None;
    }

    let key_str = from_utf8(&key_buffer[..key_len]).ok()?;
    let mut key = String::new();
    key.push_str(key_str).ok()?;

    esp_println::println!("Save API key for next boot? (y/n): ");
    let mut response = [0u8; 1];
    if rx.read(&mut response).is_ok() && (response[0] == b'y' || response[0] == b'Y') {
        match save_api_key(key_str) {
            Ok(_) => esp_println::println!("API key saved to flash"),
            Err(e) => esp_println::println!("Failed to save API key: {}", e),
        }
    }

    Some(key)
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

    let mut creds = WifiCredentials {
        ssid: String::new(),
        password: String::new(),
    };
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
