use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_time::{Duration, Timer};
use embedded_hal_02::blocking::delay::DelayMs;
use embedded_hal_02::blocking::i2c::{Read, Write, WriteRead};
use icm20948::{ICMI2C, ICM20948_CHIP_ADR};

use crate::I2cBusBlocking;

/// Adapter to bridge embedded-hal 1.0 I2C to embedded-hal 0.2.
/// Needed because the icm20948 crate uses the legacy HAL traits.
pub struct I2cAdapter<I> {
    i2c: I,
}

impl<I> I2cAdapter<I> {
    pub fn new(i2c: I) -> Self {
        Self { i2c }
    }
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

/// Simple blocking delay using Embassy timer.
pub struct SimpleDelay;

impl DelayMs<u8> for SimpleDelay {
    fn delay_ms(&mut self, ms: u8) {
        embassy_time::block_for(Duration::from_millis(ms as u64));
    }
}

impl DelayMs<u16> for SimpleDelay {
    fn delay_ms(&mut self, ms: u16) {
        embassy_time::block_for(Duration::from_millis(ms as u64));
    }
}

#[embassy_executor::task]
pub async fn imu_task(i2c_bus: &'static I2cBusBlocking) {
    esp_println::println!("Initializing ICM-20948 IMU...");

    let i2c_v1 = I2cDevice::new(i2c_bus);
    let mut i2c_adapted = I2cAdapter::new(i2c_v1);
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
        match imu.get_values_accel_gyro(&mut i2c_adapted) {
            Ok((ax, ay, az, gx, gy, gz)) => {
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
