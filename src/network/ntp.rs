use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_net::dns::DnsQueryType;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};


pub static NTP_SYNCED: AtomicBool = AtomicBool::new(false);

struct NtpTime {
    epoch_secs: u64,
    synced_at_ms: u64,
}

static NTP_TIME: BlockingMutex<CriticalSectionRawMutex, RefCell<NtpTime>> =
    BlockingMutex::new(RefCell::new(NtpTime {
        epoch_secs: 0,
        synced_at_ms: 0,
    }));

const NTP_EPOCH_OFFSET: u64 = 2_208_988_800;
const RESYNC_INTERVAL_SECS: u64 = 3600;

/// Returns current UTC time as (year, month, day, hour, minute, second), or None if not synced.
pub fn ntp_datetime() -> Option<(u16, u8, u8, u8, u8, u8)> {
    if !NTP_SYNCED.load(Ordering::Relaxed) {
        return None;
    }
    let now_ms = Instant::now().as_millis();
    let current_epoch = NTP_TIME.lock(|cell| {
        let t = cell.borrow();
        let elapsed_secs = (now_ms.saturating_sub(t.synced_at_ms)) / 1000;
        t.epoch_secs + elapsed_secs
    });
    Some(epoch_to_datetime(current_epoch))
}

/// Convert Unix epoch seconds to (year, month, day, hour, minute, second).
fn epoch_to_datetime(secs: u64) -> (u16, u8, u8, u8, u8, u8) {
    let seconds_in_day = secs % 86400;
    let hour = (seconds_in_day / 3600) as u8;
    let minute = ((seconds_in_day % 3600) / 60) as u8;
    let second = (seconds_in_day % 60) as u8;

    // Days since 1970-01-01
    let mut days = (secs / 86400) as u32;

    // Standard civil date algorithm
    let mut year: u16 = 1970;
    loop {
        let days_in_year = if is_leap(year) { 366 } else { 365 };
        if days < days_in_year {
            break;
        }
        days -= days_in_year;
        year += 1;
    }

    let leap = is_leap(year);
    let month_days: [u32; 12] = [
        31,
        if leap { 29 } else { 28 },
        31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
    ];
    let mut month: u8 = 1;
    for &md in &month_days {
        if days < md {
            break;
        }
        days -= md;
        month += 1;
    }
    let day = days as u8 + 1;

    (year, month, day, hour, minute, second)
}

fn is_leap(y: u16) -> bool {
    (y % 4 == 0 && y % 100 != 0) || y % 400 == 0
}

fn ms() -> u64 {
    Instant::now().as_millis()
}

#[embassy_executor::task]
pub async fn ntp_sync_task(stack: &'static embassy_net::Stack<'static>) {
    esp_println::println!("[{}ms] NTP: Waiting for network...", ms());
    loop {
        if stack.config_v4().is_some() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    esp_println::println!("[{}ms] NTP: Network ready", ms());

    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buf = [0u8; 64];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_buf = [0u8; 64];

    loop {
        // DNS resolve pool.ntp.org
        let addr = match stack.dns_query("pool.ntp.org", DnsQueryType::A).await {
            Ok(addrs) => addrs[0],
            Err(e) => {
                esp_println::println!("[{}ms] NTP: DNS failed: {:?}", ms(), e);
                Timer::after(Duration::from_secs(30)).await;
                continue;
            }
        };

        let mut socket = UdpSocket::new(
            *stack,
            &mut rx_meta,
            &mut rx_buf,
            &mut tx_meta,
            &mut tx_buf,
        );
        if let Err(e) = socket.bind(0) {
            esp_println::println!("[{}ms] NTP: Bind failed: {:?}", ms(), e);
            Timer::after(Duration::from_secs(30)).await;
            continue;
        }

        // Build SNTP request: 48 bytes, byte 0 = 0x1B (LI=0, VN=3, Mode=3)
        let mut request = [0u8; 48];
        request[0] = 0x1B;

        let remote = embassy_net::IpEndpoint::new(addr, 123);
        if let Err(e) = socket.send_to(&request, remote).await {
            esp_println::println!("[{}ms] NTP: Send failed: {:?}", ms(), e);
            socket.close();
            Timer::after(Duration::from_secs(30)).await;
            continue;
        }

        // Read response with timeout
        let mut response = [0u8; 48];
        let recv_result = embassy_time::with_timeout(
            Duration::from_secs(5),
            socket.recv_from(&mut response),
        )
        .await;

        socket.close();

        match recv_result {
            Ok(Ok((len, _remote))) if len >= 48 => {
                // Extract transmit timestamp (seconds) at offset 40
                let ntp_secs = u32::from_be_bytes([
                    response[40],
                    response[41],
                    response[42],
                    response[43],
                ]) as u64;

                if ntp_secs < NTP_EPOCH_OFFSET {
                    esp_println::println!("[{}ms] NTP: Invalid timestamp", ms());
                    Timer::after(Duration::from_secs(30)).await;
                    continue;
                }

                let epoch_secs = ntp_secs - NTP_EPOCH_OFFSET;
                let now_ms = Instant::now().as_millis();

                NTP_TIME.lock(|cell| {
                    let mut t = cell.borrow_mut();
                    t.epoch_secs = epoch_secs;
                    t.synced_at_ms = now_ms;
                });
                NTP_SYNCED.store(true, Ordering::Relaxed);

                let (y, mo, d, h, mi, s) = epoch_to_datetime(epoch_secs);
                esp_println::println!(
                    "[{}ms] NTP: Synced — {:04}-{:02}-{:02} {:02}:{:02}:{:02} UTC",
                    ms(), y, mo, d, h, mi, s
                );
            }
            Ok(Ok((len, _))) => {
                esp_println::println!("[{}ms] NTP: Short response ({} bytes)", ms(), len);
                Timer::after(Duration::from_secs(30)).await;
                continue;
            }
            Ok(Err(e)) => {
                esp_println::println!("[{}ms] NTP: Recv error: {:?}", ms(), e);
                Timer::after(Duration::from_secs(30)).await;
                continue;
            }
            Err(_) => {
                esp_println::println!("[{}ms] NTP: Timeout", ms());
                Timer::after(Duration::from_secs(30)).await;
                continue;
            }
        }

        Timer::after(Duration::from_secs(RESYNC_INTERVAL_SECS)).await;
    }
}
