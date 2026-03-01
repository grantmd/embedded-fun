use core::fmt::Write as FmtWrite;
use core::sync::atomic::Ordering;

use embassy_net::tcp::TcpSocket;
use embassy_net::dns::DnsQueryType;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write;
use heapless::String;

use crate::network::wifi::NETWORK_CONNECTED;
use crate::sensors::fuel_gauge::{BATTERY_DATA, BATTERY_READY};
use crate::sensors::gps::GPS_DATA;
use crate::sensors::imu::{IMU_DATA, IMU_READY};

const THINGSPEAK_HOST: &str = "api.thingspeak.com";
const UPLOAD_INTERVAL_SECS: u64 = 30;

fn ms() -> u64 {
    Instant::now().as_millis()
}

#[embassy_executor::task]
pub async fn telemetry_task(stack: &'static embassy_net::Stack<'static>, api_key: &'static str) {
    esp_println::println!("[{}ms] Telemetry: Waiting for network...", ms());
    NETWORK_CONNECTED.wait().await;
    esp_println::println!("[{}ms] Telemetry: Network ready, starting uploads every {}s", ms(), UPLOAD_INTERVAL_SECS);

    // Give sensors a moment to produce initial data
    Timer::after(Duration::from_secs(5)).await;

    let mut rx_buf = [0u8; 512];
    let mut tx_buf = [0u8; 512];

    loop {
        // Read battery data
        let (bat_soc, bat_voltage) = if BATTERY_READY.load(Ordering::Relaxed) {
            BATTERY_DATA.lock(|cell| {
                let d = cell.borrow();
                (d.soc, d.voltage)
            })
        } else {
            (0.0f32, 0.0f32)
        };

        // Read GPS data
        let (fix_type, satellites, lat_deg, lon_deg) = GPS_DATA.lock(|cell| {
            let d = cell.borrow();
            let lat = d.latitude as f32 / 1e7;
            let lon = d.longitude as f32 / 1e7;
            (d.fix_type, d.satellites, lat, lon)
        });

        // Read IMU data
        let (accel_mag, gyro_mag) = if IMU_READY.load(Ordering::Relaxed) {
            IMU_DATA.lock(|cell| {
                let d = cell.borrow();
                let am = libm::sqrtf(d.accel_x * d.accel_x + d.accel_y * d.accel_y + d.accel_z * d.accel_z);
                let gm = libm::sqrtf(d.gyro_x * d.gyro_x + d.gyro_y * d.gyro_y + d.gyro_z * d.gyro_z);
                (am, gm)
            })
        } else {
            (0.0f32, 0.0f32)
        };

        // Format the POST body
        let mut body: String<256> = String::new();
        let _ = write!(
            body,
            "api_key={}&field1={:.1}&field2={:.3}&field3={}&field4={}&field5={:.7}&field6={:.7}&field7={:.3}&field8={:.3}",
            api_key,
            bat_soc,
            bat_voltage,
            fix_type,
            satellites,
            lat_deg,
            lon_deg,
            accel_mag,
            gyro_mag,
        );

        // DNS resolve
        let addr = match stack.dns_query(THINGSPEAK_HOST, DnsQueryType::A).await {
            Ok(addrs) => addrs[0],
            Err(e) => {
                esp_println::println!("[{}ms] Telemetry: DNS failed: {:?}", ms(), e);
                Timer::after(Duration::from_secs(UPLOAD_INTERVAL_SECS)).await;
                continue;
            }
        };

        let remote = embassy_net::IpEndpoint::new(addr, 80);

        // Open TCP socket
        let mut socket = TcpSocket::new(*stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(Duration::from_secs(10)));

        if let Err(e) = socket.connect(remote).await {
            esp_println::println!("[{}ms] Telemetry: TCP connect failed: {:?}", ms(), e);
            Timer::after(Duration::from_secs(UPLOAD_INTERVAL_SECS)).await;
            continue;
        }

        // Format HTTP request
        let mut request: String<512> = String::new();
        let _ = write!(
            request,
            "POST /update HTTP/1.1\r\nHost: {}\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
            THINGSPEAK_HOST,
            body.len(),
            body,
        );

        // Send request
        if let Err(e) = socket.write_all(request.as_bytes()).await {
            esp_println::println!("[{}ms] Telemetry: Write failed: {:?}", ms(), e);
            Timer::after(Duration::from_secs(UPLOAD_INTERVAL_SECS)).await;
            continue;
        }

        // Read response status line
        let mut resp_buf = [0u8; 128];
        match embedded_io_async::Read::read(&mut socket, &mut resp_buf).await {
            Ok(n) if n > 0 => {
                // Find the status code in "HTTP/1.1 200 OK"
                let resp = &resp_buf[..n];
                if let Some(status_start) = resp.windows(5).position(|w| w == b"HTTP/") {
                    let status_line_end = resp[status_start..]
                        .iter()
                        .position(|&b| b == b'\r' || b == b'\n')
                        .unwrap_or(n - status_start);
                    let status_line = &resp[status_start..status_start + status_line_end];
                    // Check for 200
                    if status_line.len() >= 12 && &status_line[9..12] == b"200" {
                        esp_println::println!("[{}ms] Telemetry: OK (bat={:.1}% sats={})", ms(), bat_soc, satellites);
                    } else {
                        let mut status_str: String<64> = String::new();
                        for &b in status_line {
                            let _ = write!(status_str, "{}", b as char);
                        }
                        esp_println::println!("[{}ms] Telemetry: Server error: {}", ms(), status_str);
                    }
                } else {
                    esp_println::println!("[{}ms] Telemetry: Unexpected response", ms());
                }
            }
            Ok(_) => {
                esp_println::println!("[{}ms] Telemetry: Empty response", ms());
            }
            Err(e) => {
                esp_println::println!("[{}ms] Telemetry: Read error: {:?}", ms(), e);
            }
        }

        socket.close();
        Timer::after(Duration::from_secs(UPLOAD_INTERVAL_SECS)).await;
    }
}
