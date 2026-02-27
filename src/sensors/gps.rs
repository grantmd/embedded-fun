use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_time::{Duration, Timer};
use embedded_hal::i2c::I2c;
use ublox::Parser;

use crate::I2cBusBlocking;

const GPS_ADDR: u8 = 0x42;
const REG_DATA_STREAM: u8 = 0xFF;
const REG_AVAIL_MSB: u8 = 0xFD;

#[embassy_executor::task]
pub async fn gps_task(i2c_bus: &'static I2cBusBlocking) {
    esp_println::println!("Initializing MAX-M10S GPS...");

    let mut i2c = I2cDevice::new(i2c_bus);
    let mut parser = Parser::<ublox::FixedBuffer<512>>::new_fixed();

    // Give GPS module time to boot up
    Timer::after(Duration::from_secs(2)).await;

    // Tickle the device to start I2C broadcasting by sending UBX-MON-VER command
    // This command requests the device to send its version information
    // Retry until successful
    esp_println::println!("GPS: Sending initialization command...");
    let init_cmd: [u8; 8] = [0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34];
    loop {
        match i2c.write(GPS_ADDR, &init_cmd) {
            Ok(_) => {
                esp_println::println!("GPS: Initialization command sent successfully");
                break;
            }
            Err(e) => {
                esp_println::println!("GPS: Failed to send init command: {:?}, retrying...", e);
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }

    Timer::after(Duration::from_millis(100)).await;

    esp_println::println!("GPS: Starting data polling...");

    loop {
        // Read the number of available bytes from REG_AVAIL_MSB (2 bytes, big-endian)
        let mut avail_bytes = [0u8; 2];
        match i2c.write_read(GPS_ADDR, &[REG_AVAIL_MSB], &mut avail_bytes) {
            Ok(_) => {
                let available = u16::from_be_bytes(avail_bytes) as usize;

                if available > 0 {
                    // Cap the read size to our buffer capacity
                    let read_size = available.min(256);
                    let mut buffer = [0u8; 256];

                    // Read from REG_DATA_STREAM
                    match i2c.write_read(GPS_ADDR, &[REG_DATA_STREAM], &mut buffer[..read_size]) {
                        Ok(_) => {
                            // Filter out trailing 0xFF padding bytes
                            // REG_DATA_STREAM returns 0xFF when no data is available
                            let mut actual_size = read_size;
                            while actual_size > 0 && buffer[actual_size - 1] == 0xFF {
                                actual_size -= 1;
                            }

                            if actual_size > 0 {
                                // Print raw bytes
                                esp_println::print!("GPS: Received {} bytes", actual_size);
                                if read_size > actual_size {
                                    esp_println::print!(
                                        " ({} padding bytes stripped)",
                                        read_size - actual_size
                                    );
                                }
                                if available > read_size {
                                    esp_println::print!(
                                        " ({} available, truncated)",
                                        available
                                    );
                                }
                                esp_println::print!(": ");
                                for i in 0..actual_size {
                                    esp_println::print!("{:02X} ", buffer[i]);
                                }
                                esp_println::println!();

                                // Parse UBX packets (only the actual data, not padding)
                                let mut packets = parser.consume_ubx(&buffer[..actual_size]);

                                // Print parsed packets
                                while let Some(packet_result) = packets.next() {
                                    match packet_result {
                                        Ok(packet) => {
                                            esp_println::println!(
                                                "GPS: Received packet: {:?}",
                                                packet
                                            );
                                        }
                                        Err(e) => {
                                            esp_println::println!("GPS: Parser error: {:?}", e);
                                        }
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            esp_println::println!("GPS: I2C read error: {:?}", e);
                        }
                    }
                }
            }
            Err(e) => {
                esp_println::println!("GPS: I2C availability check error: {:?}", e);
            }
        }

        // Poll at reasonable interval
        Timer::after(Duration::from_millis(250)).await;
    }
}
