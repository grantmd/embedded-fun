use core::fmt::Write as FmtWrite;
use core::sync::atomic::Ordering;

use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{Mode, RawFile, RawDirectory, SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use esp_hal::gpio::Output;
use esp_hal::spi::master::Spi;
use esp_hal::Blocking;
use heapless::String;

use crate::network::ntp::ntp_datetime;
use crate::sensors::fuel_gauge::{BATTERY_DATA, BATTERY_READY};
use crate::sensors::gps::GPS_DATA;
use crate::sensors::imu::{IMU_DATA, IMU_READY};

type SdSpiDevice = ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, Delay>;

pub type SdVolumeManager = VolumeManager<SdCard<SdSpiDevice, Delay>, GpsTimeSource>;

pub struct GpsTimeSource;

impl TimeSource for GpsTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        let zero = Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        };

        // Try GPS first
        let gps = GPS_DATA.lock(|cell| {
            let d = cell.borrow();
            (d.year, d.month, d.day, d.hour, d.minute, d.second)
        });

        if gps.0 >= 2020 {
            return Timestamp::from_calendar(gps.0, gps.1, gps.2, gps.3, gps.4, gps.5)
                .unwrap_or(zero);
        }

        // Fall back to NTP
        if let Some((y, mo, d, h, mi, s)) = ntp_datetime() {
            return Timestamp::from_calendar(y, mo, d, h, mi, s).unwrap_or(zero);
        }

        zero
    }
}

fn ms() -> u64 {
    Instant::now().as_millis()
}

const CSV_HEADER: &[u8] = b"timestamp,fix_type,satellites,latitude,longitude,altitude_msl_mm,hacc_mm,vacc_mm,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,bat_soc,bat_voltage,bat_rate\n";

#[embassy_executor::task]
pub async fn sd_logger_task(volume_mgr: SdVolumeManager) {
    esp_println::println!("[{}ms] SD: Mounting filesystem...", ms());

    let raw_volume = match volume_mgr.open_raw_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            esp_println::println!("[{}ms] SD: Failed to open volume: {:?}", ms(), e);
            return;
        }
    };

    let root_dir = match volume_mgr.open_root_dir(raw_volume) {
        Ok(d) => d,
        Err(e) => {
            esp_println::println!("[{}ms] SD: Failed to open root dir: {:?}", ms(), e);
            return;
        }
    };

    esp_println::println!("[{}ms] SD: Logger started, waiting for data...", ms());

    let mut current_file: Option<RawFile> = None;
    let mut current_date: (u16, u8, u8) = (0, 0, 0);
    let mut row_count: u32 = 0;

    loop {
        Timer::after(Duration::from_secs(10)).await;

        // Read GPS data
        let (year, month, day, hour, minute, second, fix_type, satellites, lat, lon, alt_mm, hacc_mm, vacc_mm) =
            GPS_DATA.lock(|cell| {
                let d = cell.borrow();
                (
                    d.year, d.month, d.day, d.hour, d.minute, d.second,
                    d.fix_type, d.satellites, d.latitude, d.longitude,
                    d.altitude_msl_mm, d.horizontal_acc_mm, d.vertical_acc_mm,
                )
            });

        // Determine today's date for file rotation (GPS first, then NTP)
        let today = if year >= 2020 {
            (year, month, day)
        } else if let Some((ny, nmo, nd, _, _, _)) = ntp_datetime() {
            (ny, nmo, nd)
        } else {
            (0, 0, 0)
        };

        // Rotate file if date changed
        if today != current_date {
            // Close previous file
            if let Some(old_file) = current_file.take() {
                let _ = volume_mgr.close_file(old_file);
            }

            let filename = format_filename(today);
            match open_log_file(&volume_mgr, root_dir, filename.as_str()) {
                Some(f) => {
                    esp_println::println!("[{}ms] SD: Opened {}", ms(), filename);
                    current_file = Some(f);
                    current_date = today;
                }
                None => {
                    // Will retry next iteration
                    continue;
                }
            }
        }

        let Some(file) = current_file else {
            continue;
        };

        // Read IMU data
        let (ax, ay, az, gx, gy, gz) = if IMU_READY.load(Ordering::Relaxed) {
            IMU_DATA.lock(|cell| {
                let d = cell.borrow();
                (d.accel_x, d.accel_y, d.accel_z, d.gyro_x, d.gyro_y, d.gyro_z)
            })
        } else {
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        };

        // Read battery data
        let (bat_soc, bat_voltage, bat_rate) = if BATTERY_READY.load(Ordering::Relaxed) {
            BATTERY_DATA.lock(|cell| {
                let d = cell.borrow();
                (d.soc, d.voltage, d.rate)
            })
        } else {
            (0.0, 0.0, 0.0)
        };

        // Format CSV row
        let mut row: String<256> = String::new();
        let _ = write!(
            row,
            "{}-{:02}-{:02}T{:02}:{:02}:{:02},{},{},{},{},{},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.1},{:.3},{:.2}\n",
            year, month, day, hour, minute, second,
            fix_type, satellites, lat, lon, alt_mm, hacc_mm, vacc_mm,
            ax, ay, az, gx, gy, gz,
            bat_soc, bat_voltage, bat_rate,
        );

        match volume_mgr.write(file, row.as_bytes()) {
            Ok(_) => {
                let _ = volume_mgr.flush_file(file);
                row_count += 1;
                if row_count % 10 == 0 {
                    esp_println::println!("[{}ms] SD: {} rows written", ms(), row_count);
                }
            }
            Err(e) => {
                esp_println::println!("[{}ms] SD: Write error: {:?}", ms(), e);
            }
        }
    }
}

fn format_filename(date: (u16, u8, u8)) -> String<12> {
    let mut name: String<12> = String::new();
    if date.0 >= 2020 {
        let _ = write!(name, "{}{:02}{:02}.CSV", date.0, date.1, date.2);
    } else {
        let _ = write!(name, "LOG.CSV");
    }
    name
}

fn open_log_file(
    volume_mgr: &SdVolumeManager,
    root_dir: RawDirectory,
    filename: &str,
) -> Option<RawFile> {
    match volume_mgr.open_file_in_dir(root_dir, filename, Mode::ReadWriteCreateOrAppend) {
        Ok(f) => {
            // Write header if file is empty (newly created)
            if volume_mgr.file_length(f).unwrap_or(1) == 0 {
                if let Err(e) = volume_mgr.write(f, CSV_HEADER) {
                    esp_println::println!("[{}ms] SD: Failed to write header: {:?}", ms(), e);
                }
            }
            Some(f)
        }
        Err(e) => {
            esp_println::println!("[{}ms] SD: Failed to open {}: {:?}", ms(), filename, e);
            None
        }
    }
}
