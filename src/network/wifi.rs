use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

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
            esp_println::println!("WiFi connection lost!");
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
    esp_println::println!("Ping task waiting for WiFi connection...");
    WIFI_CONNECTED.wait().await;

    esp_println::println!("WiFi connected! Starting ping loop...");

    // Wait a bit more to ensure we have an IP address
    Timer::after(Duration::from_secs(2)).await;

    let mut seq = 0u16;

    loop {
        if !stack.is_link_up() {
            esp_println::println!("Network down, waiting for connection...");
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        if stack.config_v4().is_none() {
            esp_println::println!("No IP address yet, waiting...");
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
                esp_println::println!("Ping #{}: 8.8.8.8 is reachable (via DNS query)", seq);
            }
            Err(e) => {
                esp_println::println!("Ping #{}: DNS query failed: {:?}", seq, e);
            }
        }

        Timer::after(Duration::from_secs(60)).await;
    }
}
