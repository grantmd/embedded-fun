use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

// Signals to notify tasks that WiFi network is ready
pub static WIFI_CONNECTED: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static NETWORK_CONNECTED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task]
pub async fn wifi_task(stack: &'static embassy_net::Stack<'static>) {
    loop {
        if stack.is_link_up() {
            WIFI_CONNECTED.signal(());
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // Keep monitoring connection
    loop {
        Timer::after(Duration::from_secs(10)).await;
        if !stack.is_link_up() {
            esp_println::println!("[{}ms] WiFi: Connection lost!", Instant::now().as_millis());
        }
    }
}

#[embassy_executor::task]
pub async fn net_task(
    mut runner: embassy_net::Runner<'static, esp_wifi::wifi::WifiDevice<'static>>,
) {
    runner.run().await
}

#[embassy_executor::task]
pub async fn ping_task(stack: &'static embassy_net::Stack<'static>) {
    esp_println::println!("[{}ms] Ping: Waiting for WiFi...", Instant::now().as_millis());
    WIFI_CONNECTED.wait().await;

    esp_println::println!("[{}ms] Ping: WiFi connected, starting ping loop...", Instant::now().as_millis());

    // Wait a bit more to ensure we have an IP address
    Timer::after(Duration::from_secs(2)).await;

    let mut seq = 0u16;

    loop {
        if !stack.is_link_up() {
            esp_println::println!("[{}ms] Ping: Network down, waiting...", Instant::now().as_millis());
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        if stack.config_v4().is_none() {
            esp_println::println!("[{}ms] Ping: No IP address yet, waiting...", Instant::now().as_millis());
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        // This probably means we have a network connection
        NETWORK_CONNECTED.signal(());

        seq = seq.wrapping_add(1);

        match stack
            .dns_query("8.8.8.8", embassy_net::dns::DnsQueryType::A)
            .await
        {
            Ok(_) => {
                esp_println::println!("[{}ms] Ping #{}: 8.8.8.8 reachable", Instant::now().as_millis(), seq);
            }
            Err(e) => {
                esp_println::println!("[{}ms] Ping #{}: DNS query failed: {:?}", Instant::now().as_millis(), seq, e);
            }
        }

        Timer::after(Duration::from_secs(60)).await;
    }
}
