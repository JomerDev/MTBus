#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;

use {defmt_rtt as _, panic_probe as _};
use mtbus_shared::led_settings::{led_runner, set_led, LedStatus};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let _ = spawner.spawn(led_runner(
        p.PIN_11, p.PIN_12, p.PIN_16, p.PIN_17, p.PIN_25, p.PIO0, p.DMA_CH0,
    ));

    set_led(LedStatus::Green, true);

}
