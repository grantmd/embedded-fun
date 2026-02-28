use core::sync::atomic::Ordering;

use embassy_time::{Duration, Timer};
use esp_hal::gpio::Output;

use crate::network::wifi::NETWORK_CONNECTED;
use crate::sensors::gps::GPS_HAS_FIX;

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
                break;
            }
        }
    }

    // WiFi connected — now wait for GPS fix too
    // Slow blink (1s) to indicate "connected, waiting for GPS"
    loop {
        if GPS_HAS_FIX.load(Ordering::Relaxed) {
            led.set_high();
            break;
        }
        led.toggle();
        Timer::after(Duration::from_secs(1)).await;
    }

    // Both connected — keep LED on solid
    // Periodically check if GPS fix is lost
    loop {
        Timer::after(Duration::from_secs(5)).await;
        if GPS_HAS_FIX.load(Ordering::Relaxed) {
            led.set_high();
        } else {
            led.toggle();
        }
    }
}
