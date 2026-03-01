use core::fmt::Write as FmtWrite;
use core::sync::atomic::Ordering;

use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{Mode, SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use esp_hal::gpio::Output;
use esp_hal::spi::master::Spi;
use esp_hal::Blocking;
use heapless::String;

use crate::sensors::fuel_gauge::{BATTERY_DATA, BATTERY_READY};
use crate::sensors::gps::GPS_DATA;
use crate::sensors::imu::{IMU_DATA, IMU_READY};

type SdSpiDevice = ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, Delay>;

pub type SdVolumeManager = VolumeManager<SdCard<SdSpiDevice, Delay>, GpsTimeSource>;

pub struct GpsTimeSource;

impl TimeSource for GpsTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        GPS_DATA.lock(|cell| {
            let d = cell.borrow();
            if d.year >= 2020 {
                Timestamp::from_calendar(d.year, d.month, d.day, d.hour, d.minute, d.second)
                    .unwrap_or(Timestamp {
                        year_since_1970: 0,
                        zero_indexed_month: 0,
                        zero_indexed_day: 0,
                        hours: 0,
                        minutes: 0,
                        seconds: 0,
                    })
            } else {
                Timestamp {
                    year_since_1970: 0,
                    zero_indexed_month: 0,
                    zero_indexed_day: 0,
                    hours: 0,
                    minutes: 0,
                    seconds: 0,
                }
            }
        })
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

    let file = match volume_mgr.open_file_in_dir(root_dir, "LOG.CSV", Mode::ReadWriteCreateOrAppend)
    {
        Ok(f) => f,
        Err(e) => {
            esp_println::println!("[{}ms] SD: Failed to open LOG.CSV: {:?}", ms(), e);
            return;
        }
    };

    // Write header if file is empty (newly created)
    if volume_mgr.file_length(file).unwrap_or(1) == 0 {
        if let Err(e) = volume_mgr.write(file, CSV_HEADER) {
            esp_println::println!("[{}ms] SD: Failed to write header: {:?}", ms(), e);
        }
    }

    esp_println::println!("[{}ms] SD: Logging to LOG.CSV every 10s", ms());

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
