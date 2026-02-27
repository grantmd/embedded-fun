use embassy_time::{Duration, Timer};
use esp_hal::gpio::Output;

use crate::network::wifi::NETWORK_CONNECTED;

#[embassy_executor::task]
pub async fn led_blink_task(mut led: Output<'static>) {
    // Blink while waiting for WiFi connection
    loop {
        led.toggle();

        match embassy_futures::select::select(
            Timer::after(Duration::from_millis(250)),
            NETWORK_CONNECTED.wait(),
        )
        .await
        {
            embassy_futures::select::Either::First(_) => {
                continue;
            }
            embassy_futures::select::Either::Second(_) => {
                // WiFi connected! Turn LED on solid
                led.set_high();
                break;
            }
        }
    }

    // Keep LED on solid
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
