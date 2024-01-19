#![no_std]
#![no_main]

use cortex_m_rt::{exception, ExceptionFrame};
use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Level, Output},
    Peripherals,
};
use embassy_time::{Duration, Instant, Timer};
use led_settings::{set_led, LedStatus};

use crate::led_settings::led_runner;
use {defmt_rtt as _, panic_probe as _};

mod led_settings;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let _ = spawner.spawn(led_runner(
        p.PIN_11, p.PIN_12, p.PIN_16, p.PIN_17, p.PIN_25, p.PIO0, p.DMA_CH0,
    ));

    let fut = async {
        set_led(LedStatus::Red, true);

        Timer::after_secs(5).await;

        set_led(LedStatus::Green, true);

        Timer::after_secs(5).await;

        set_led(LedStatus::Blue, true);

        Timer::after_secs(5).await;

        set_led(LedStatus::Off, false);
    };

    fut.await;
}

#[exception]
unsafe fn HardFault(_frame: &ExceptionFrame) -> ! {
    let p = unsafe { Peripherals::steal() };
    let mut led1 = Output::new(p.PIN_16, Level::Low);
    let mut led2 = Output::new(p.PIN_17, Level::Low);
    let mut led3 = Output::new(p.PIN_25, Level::Low);
    led1.set_low();
    led2.set_low();
    led3.set_low();
    let mut instance = Instant::now();
    let duration = Duration::from_millis(2000);
    let mut low = true;
    loop {
        if Instant::now() == instance + duration {
            instance = Instant::now();
            low = !low;
            if low {
                led1.set_low();
                led2.set_low();
                led3.set_low();
            } else {
                led1.set_high();
                led2.set_high();
                led3.set_high();
            }
        }
    }
}
