use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_time::{Duration, Timer};
use max170xx::Max17048;

use crate::I2cBusBlocking;

#[embassy_executor::task]
pub async fn fuel_gauge_task(i2c_bus: &'static I2cBusBlocking) {
    esp_println::println!("Initializing MAX17048 fuel gauge...");

    let i2c = I2cDevice::new(i2c_bus);
    let mut fuel_gauge = Max17048::new(i2c);

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
