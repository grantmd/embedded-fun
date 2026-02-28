use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal::i2c::I2c;

use crate::I2cBusBlocking;

const GPS_ADDR: u8 = 0x42;

/// Parsed GPS data from the latest NAV-PVT message.
pub struct GpsData {
    pub has_fix: bool,
    pub fix_type: u8, // 0=none, 2=2D, 3=3D, 4=3D+DR
    pub satellites: u8,
    pub latitude: i32,  // degrees × 1e7
    pub longitude: i32, // degrees × 1e7
    pub altitude_msl_mm: i32,
    pub horizontal_acc_mm: u32,
    pub vertical_acc_mm: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

/// Shared GPS data, updated by the GPS task, readable by other tasks.
pub static GPS_DATA: BlockingMutex<CriticalSectionRawMutex, RefCell<GpsData>> =
    BlockingMutex::new(RefCell::new(GpsData {
        has_fix: false,
        fix_type: 0,
        satellites: 0,
        latitude: 0,
        longitude: 0,
        altitude_msl_mm: 0,
        horizontal_acc_mm: 0,
        vertical_acc_mm: 0,
        year: 0,
        month: 0,
        day: 0,
        hour: 0,
        minute: 0,
        second: 0,
    }));

/// Lightweight lock-free GPS fix flag. Used by LED task.
pub static GPS_HAS_FIX: AtomicBool = AtomicBool::new(false);

fn ubx_checksum(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &byte in data {
        ck_a = ck_a.wrapping_add(byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

fn ubx_poll_msg(class: u8, id: u8) -> [u8; 8] {
    let payload: &[u8] = &[class, id, 0x00, 0x00];
    let (ck_a, ck_b) = ubx_checksum(payload);
    [0xB5, 0x62, class, id, 0x00, 0x00, ck_a, ck_b]
}

/// Build a 17-byte UBX-CFG-VALSET message to set a single U1 key in RAM.
fn ubx_cfg_valset_u1(key: u32, value: u8) -> [u8; 17] {
    let kb = key.to_le_bytes();
    let body: [u8; 13] = [
        0x06, 0x8A, // class/id: CFG-VALSET
        0x09, 0x00, // payload length: 9
        0x00, // version
        0x01, // layers: RAM only
        0x00, 0x00, // reserved
        kb[0], kb[1], kb[2], kb[3],
        value,
    ];
    let (ck_a, ck_b) = ubx_checksum(&body);
    [
        0xB5, 0x62, body[0], body[1], body[2], body[3], body[4], body[5], body[6], body[7],
        body[8], body[9], body[10], body[11], body[12], ck_a, ck_b,
    ]
}

fn ms() -> u64 {
    Instant::now().as_millis()
}

/// Find a UBX message header (B5 62 + class + id) in burst data.
fn find_ubx_msg(data: &[u8], class: u8, id: u8) -> Option<usize> {
    if data.len() < 4 {
        return None;
    }
    data.windows(4)
        .position(|w| w[0] == 0xB5 && w[1] == 0x62 && w[2] == class && w[3] == id)
}

/// Write a UBX command to the GPS with retries on I2C error.
async fn write_with_retry(i2c: &mut impl I2c, data: &[u8], label: &str) {
    loop {
        match i2c.write(GPS_ADDR, data) {
            Ok(_) => {
                if !label.is_empty() {
                    esp_println::println!("[{}ms] GPS: {}", ms(), label);
                }
                break;
            }
            Err(e) => {
                esp_println::println!("[{}ms] GPS: {} failed: {:?}, retrying...", ms(), label, e);
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    }
}

/// Extract NAV-PVT fields, update GPS_DATA and GPS_HAS_FIX.
/// Only logs when the time second changes to avoid duplicate lines.
/// Returns true if data was successfully parsed.
fn parse_nav_pvt(data: &[u8], last_logged_second: &mut u8) -> bool {
    let payload = &data[6..];
    let plen = payload.len();

    if plen < 24 {
        return false;
    }

    let year = u16::from_le_bytes([payload[4], payload[5]]);
    let month = payload[6];
    let day = payload[7];
    let hour = payload[8];
    let min = payload[9];
    let sec = payload[10];
    let fix_type = payload[20];
    let flags = payload[21];
    let num_sv = payload[23];

    if year < 2020 || year > 2030 || month > 12 || day > 31 || hour > 23 || min > 59 || sec > 60 {
        return false;
    }

    let has_fix = fix_type >= 2 && fix_type <= 4;
    let gnss_fix_ok = flags & 0x01 != 0;

    let (lon, lat, hmsl_mm, hacc_mm, vacc_mm) = if plen >= 48 {
        (
            i32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]),
            i32::from_le_bytes([payload[28], payload[29], payload[30], payload[31]]),
            i32::from_le_bytes([payload[36], payload[37], payload[38], payload[39]]),
            u32::from_le_bytes([payload[40], payload[41], payload[42], payload[43]]),
            u32::from_le_bytes([payload[44], payload[45], payload[46], payload[47]]),
        )
    } else {
        (0, 0, 0, 0, 0)
    };

    // Update shared state
    GPS_DATA.lock(|cell| {
        let mut d = cell.borrow_mut();
        d.has_fix = has_fix;
        d.fix_type = fix_type;
        d.satellites = num_sv;
        d.latitude = lat;
        d.longitude = lon;
        d.altitude_msl_mm = hmsl_mm;
        d.horizontal_acc_mm = hacc_mm;
        d.vertical_acc_mm = vacc_mm;
        d.year = year;
        d.month = month;
        d.day = day;
        d.hour = hour;
        d.minute = min;
        d.second = sec;
    });
    GPS_HAS_FIX.store(has_fix, Ordering::Relaxed);

    // Only log when the second changes
    if sec != *last_logged_second {
        *last_logged_second = sec;

        let fix_str = match fix_type {
            0 => "none",
            1 => "DR",
            2 => "2D",
            3 => "3D",
            4 => "3D+DR",
            5 => "time",
            _ => "?",
        };

        if plen >= 48 {
            let lat_deg = lat / 10_000_000;
            let lat_frac = (lat % 10_000_000).unsigned_abs();
            let lon_deg = lon / 10_000_000;
            let lon_frac = (lon % 10_000_000).unsigned_abs();

            esp_println::println!(
                "[{}ms] GPS: {}-{:02}-{:02} {:02}:{:02}:{:02} fix={}{} sats={} pos={}.{:07},{}.{:07} alt={}m hacc={}m vacc={}m",
                ms(),
                year, month, day, hour, min, sec,
                fix_str,
                if gnss_fix_ok { "*" } else { "" },
                num_sv,
                lat_deg, lat_frac,
                lon_deg, lon_frac,
                hmsl_mm / 1000,
                hacc_mm / 1000,
                vacc_mm / 1000,
            );
        } else {
            esp_println::println!(
                "[{}ms] GPS: {}-{:02}-{:02} {:02}:{:02}:{:02} fix={}{} sats={}",
                ms(),
                year, month, day, hour, min, sec,
                fix_str,
                if gnss_fix_ok { "*" } else { "" },
                num_sv,
            );
        }
    }

    true
}

/// Parse NAV-SAT response — log satellite visibility summary only.
fn parse_nav_sat(data: &[u8]) {
    if data.len() < 12 {
        return;
    }
    let payload = &data[6..];
    if payload.len() < 8 {
        return;
    }
    let num_svs = payload[5];
    let mut tracked = 0u8;
    let mut strong = 0u8;

    let sat_data = &payload[8..];
    let max_sats = sat_data.len() / 12;
    for i in 0..max_sats.min(num_svs as usize) {
        let cno = sat_data[i * 12 + 2];
        if cno > 0 {
            tracked += 1;
            if cno >= 20 {
                strong += 1;
            }
        }
    }

    esp_println::println!(
        "[{}ms] GPS: SAT: {} visible, {} tracked, {} strong (>=20dBHz)",
        ms(),
        num_svs,
        tracked,
        strong,
    );
}

#[embassy_executor::task]
pub async fn gps_task(i2c_bus: &'static I2cBusBlocking) {
    esp_println::println!("[{}ms] GPS: Initializing MAX-M10S...", ms());

    let mut i2c = I2cDevice::new(i2c_bus);
    let mut buffer = [0u8; 32];

    // Give GPS module time to boot
    Timer::after(Duration::from_secs(2)).await;

    // Kick-start I2C output
    let mon_ver = ubx_poll_msg(0x0A, 0x04);
    write_with_retry(&mut i2c, &mon_ver, "MON-VER poll sent").await;
    Timer::after(Duration::from_millis(100)).await;

    // Enable automatic NAV-PVT at 1Hz
    let enable_pvt = ubx_cfg_valset_u1(0x20910006, 1);
    write_with_retry(&mut i2c, &enable_pvt, "Periodic NAV-PVT enabled (1Hz)").await;
    Timer::after(Duration::from_millis(100)).await;

    // Disable NMEA sentences on I2C to reduce buffer clutter
    for key in [
        0x209100BAu32,
        0x209100BF,
        0x209100C4,
        0x209100AB,
        0x209100B0,
    ] {
        let msg = ubx_cfg_valset_u1(key, 0);
        write_with_retry(&mut i2c, &msg, "").await;
        Timer::after(Duration::from_millis(50)).await;
    }
    esp_println::println!("[{}ms] GPS: NMEA sentences disabled on I2C", ms());

    Timer::after(Duration::from_millis(100)).await;

    // Enable all GNSS constellations in RAM
    for (key, name) in [
        (0x1031001Fu32, "GPS"),
        (0x10310021u32, "Galileo"),
        (0x10310022u32, "BeiDou"),
        (0x10310025u32, "GLONASS"),
    ] {
        let msg = ubx_cfg_valset_u1(key, 1);
        write_with_retry(&mut i2c, &msg, name).await;
        Timer::after(Duration::from_millis(50)).await;
    }
    esp_println::println!("[{}ms] GPS: All constellations enabled", ms());

    // Main read loop
    let nav_pvt_poll = ubx_poll_msg(0x01, 0x07);
    let nav_sat_poll = ubx_poll_msg(0x01, 0x35);
    let mut last_pvt_poll = Instant::now();
    let mut last_sat_poll = Instant::now();
    let mut last_logged_second: u8 = 0xFF;
    let mut draining = false;
    let mut burst = [0u8; 128];
    let mut burst_len: usize = 0;

    loop {
        // Backup NAV-PVT poll every 5s
        if last_pvt_poll.elapsed() > Duration::from_secs(5) {
            let _ = i2c.write(GPS_ADDR, &nav_pvt_poll);
            last_pvt_poll = Instant::now();
            Timer::after(Duration::from_millis(50)).await;
        }

        // NAV-SAT poll every 30s
        if last_sat_poll.elapsed() > Duration::from_secs(30) {
            let _ = i2c.write(GPS_ADDR, &nav_sat_poll);
            last_sat_poll = Instant::now();
            Timer::after(Duration::from_millis(50)).await;
        }

        let read_result = if draining {
            i2c.write_read(GPS_ADDR, &[0xFF], &mut buffer)
        } else {
            if i2c.write(GPS_ADDR, &[0xFF]).is_err() {
                Timer::after(Duration::from_millis(250)).await;
                continue;
            }
            Timer::after(Duration::from_millis(50)).await;
            i2c.read(GPS_ADDR, &mut buffer)
        };

        match read_result {
            Ok(_) => {
                let actual = buffer
                    .iter()
                    .rposition(|&b| b != 0xFF)
                    .map_or(0, |p| p + 1);

                if actual > 0 {
                    let space = burst.len() - burst_len;
                    let copy = actual.min(space);
                    burst[burst_len..burst_len + copy].copy_from_slice(&buffer[..copy]);
                    burst_len += copy;

                    // Try parsing UBX messages from accumulated burst
                    if let Some(pos) = find_ubx_msg(&burst[..burst_len], 0x01, 0x07) {
                        if burst_len - pos >= 54 {
                            parse_nav_pvt(&burst[pos..burst_len], &mut last_logged_second);
                            burst_len = 0;
                        }
                    } else if let Some(pos) = find_ubx_msg(&burst[..burst_len], 0x01, 0x35) {
                        if burst_len - pos >= 14 {
                            parse_nav_sat(&burst[pos..burst_len]);
                            burst_len = 0;
                        }
                    }

                    draining = true;
                    continue;
                }
                // All 0xFF — buffer empty, burst over
                draining = false;
                burst_len = 0;
            }
            Err(_) => {
                draining = false;
                burst_len = 0;
                let _ = i2c.write(GPS_ADDR, &[0xFF]);
            }
        }

        Timer::after(Duration::from_millis(250)).await;
    }
}
