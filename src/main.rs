
#![no_std]
#![no_main]

use defmt::debug;
use embassy_executor::Spawner;
use embassy_rp::{gpio::{Level, Output}, pio::{Pio, InterruptHandler}, bind_interrupts, peripherals::PIO0};
use embassy_time::Timer;
use smart_leds::RGB8;
use {defmt_rtt as _, panic_probe as _};

mod ws2812;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);

    // PIN_16: User Led Green
    // PIN_17: User Led Red
    // PIN_25: User Led Blue

    let _ = Output::new(p.PIN_16, Level::Low);
    let _ = Output::new(p.PIN_17, Level::Low);
    let _ = Output::new(p.PIN_25, Level::Low);
    let _led_11 = Output::new(p.PIN_11, Level::High);

    const NUM_LEDS: usize = 1;
    let mut data = [RGB8::default(); NUM_LEDS];

    let mut ws = ws2812::Ws2812::new( &mut common, sm0, p.DMA_CH0, p.PIN_12);

    loop {
        // info!("led on!");
        // led.set_high();
        // Timer::after_secs(1).await;

        // info!("led off!");
        // led.set_low();
        // Timer::after_secs(1).await;

        for j in 0..(256 * 5) {
            debug!("New Colors:");
            for i in 0..NUM_LEDS {
                data[i] = wheel((((i * 256) as u16 / NUM_LEDS as u16 + j as u16) & 255) as u8);
                debug!("R: {} G: {} B: {}", data[i].r, data[i].g, data[i].b);
            }
            ws.write(&data).await;

            Timer::after_millis(10).await;
        }
    }
}

