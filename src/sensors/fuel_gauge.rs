use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_time::{Duration, Instant, Timer};
use max170xx::Max17048;

use crate::I2cBusBlocking;

#[embassy_executor::task]
pub async fn fuel_gauge_task(i2c_bus: &'static I2cBusBlocking) {
    esp_println::println!("[{}ms] Battery: Initializing MAX17048...", Instant::now().as_millis());

    let i2c = I2cDevice::new(i2c_bus);
    let mut fuel_gauge = Max17048::new(i2c);

    match fuel_gauge.version() {
        Ok(version) => {
            esp_println::println!("[{}ms] Battery: MAX17048 detected, version: 0x{:04X}", Instant::now().as_millis(), version);
        }
        Err(_) => {
            esp_println::println!("[{}ms] Battery: Failed to communicate with MAX17048", Instant::now().as_millis());
            return;
        }
    }

    // Startup can be noisy, reset calculations
    fuel_gauge.quickstart().unwrap();

    loop {
        let mut reading = || -> Result<_, max170xx::Error<_>> {
            Ok((fuel_gauge.soc()?, fuel_gauge.voltage()?, fuel_gauge.charge_rate()?))
        };
        match reading() {
            Ok((soc, voltage, rate)) => {
                esp_println::println!(
                    "[{}ms] Battery: {:.1}% | {:.3}V | {:.2}%/hr",
                    Instant::now().as_millis(), soc, voltage, rate
                );
            }
            Err(e) => {
                esp_println::println!("[{}ms] Battery: read error: {:?}", Instant::now().as_millis(), e);
            }
        }

        Timer::after(Duration::from_secs(60)).await;
    }
}
