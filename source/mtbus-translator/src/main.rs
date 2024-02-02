#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_executor::Spawner;
// use log::info;

use crate::uart_runner::run_uart;

// use defmt_rtt as _;
use panic_probe as _;
use embassy_rp::{usb::{Driver, InterruptHandler as InterruptHandlerUSB}, peripherals::{USB, UART0}, bind_interrupts, uart::{InterruptHandler as InterruptHandlerUART, self}, gpio::{Output, AnyPin, Level}};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::{Channel, Sender, Receiver}, mutex::Mutex};
use embassy_time::{Timer, Duration, Ticker};
use erdnuss_comms::{target::TgtCfg, frame_pool::{ FrameBox, FrameStorage, RawFrameSlice }};
use erdnuss_rp2040::{Rs485Uart, get_rand};
use mtbus_shared::led_settings::{led_runner, set_led, LedStatus};
use grounded::uninit::GroundedArrayCell;
use rand_chacha::ChaCha8Rng;
use static_cell::StaticCell;
use heapless::String;

mod uart_runner;

bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandlerUART<UART0>;
});

static POOL: FrameStorage<16> = FrameStorage::new();
static OUTGOING: Channel<ThreadModeRawMutex, FrameBox, 8> = Channel::new();
static INCOMING: Channel<ThreadModeRawMutex, FrameBox, 8> = Channel::new();

static OUTIE: StaticCell<Mutex<ThreadModeRawMutex, Outie>> = StaticCell::new();
pub type OutieRef = &'static Mutex<ThreadModeRawMutex, Outie>;
pub type Innie = Receiver<'static, ThreadModeRawMutex, FrameBox, 8>;

pub struct Outie {
    pub tx: Sender<'static, ThreadModeRawMutex, FrameBox, 8>,
    pub opool: RawFrameSlice,
}

impl Outie {
    pub async fn alloc_buf(&mut self) -> FrameBox {
        // lol
        loop {
            let Some(buf) = self.opool.allocate_raw() else {
                Timer::after(Duration::from_millis(5)).await;
                continue;
            };
            break buf;
        }
    }
}

struct Config;

impl TgtCfg for Config {
    type Mutex = ThreadModeRawMutex;
    type Serial = Rs485Uart<UART0>;
    type Rand = ChaCha8Rng;
    const ADDRESS_CLAIM_TIMEOUT: Duration = Duration::from_secs(3);
    const TURNAROUND_DELAY: Duration = Duration::from_micros(5);
    const SELECT_TIMEOUT: Duration = Duration::from_secs(3);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());

    spawner.spawn(run_uart(p.USB)).unwrap();
    Timer::after_secs(8).await; // Wait two seconds so the usb connection can restart

    let _ = spawner.spawn(led_runner(
        p.PIN_11, p.PIN_12, p.PIN_16, p.PIN_17, p.PIN_25, p.PIO0, p.DMA_CH2,
    ));

    set_led(LedStatus::Red, true);

    defmt::info!("Startup\n\n");

    let unique = erdnuss_rp2040::get_unique_id(&mut p.FLASH).unwrap();
    // defmt::info!("MAC: {0=0..7:02X}:{0=8..15:02X}:{0=16..23:02X}:{0=24..31:02X}:{0=32..39:02X}:{0=40..47:02X}:{0=48..55:02X}:{0=56..63:02X}", unique);
    defmt::info!("MAC: {=u64:08X}", unique);

    let mut config = uart::Config::default();
    config.baudrate = 7_812_500;
    // config.baudrate = 115200;
    
    // Pico:
    // let uart = uart::Uart::new(
    //     p.UART0, p.PIN_16, p.PIN_17, Irqs, p.DMA_CH0, p.DMA_CH1, config,
    // );

    // let io_pin1 = Output::new(AnyPin::from(p.PIN_15), Level::Low);

    // xiao-rp2040:
    let uart = uart::Uart::new(
        p.UART0, p.PIN_0, p.PIN_1, Irqs, p.DMA_CH0, p.DMA_CH1, config,
    );

    let io_pin1 = Output::new(AnyPin::from(p.PIN_29), Level::Low);
    let rs485 = Rs485Uart::new(uart, io_pin1);

    let mut whole_pool = POOL.take().unwrap();
    let app_pool = whole_pool.split(8).unwrap();

    let sub: erdnuss_comms::target::Target<'_, Config, 8> = erdnuss_comms::target::Target::new(
        rs485,
        INCOMING.sender(),
        OUTGOING.receiver(),
        app_pool,
        unique.to_le_bytes(),
        get_rand(unique),
    );

    // let outie_ref = OUTIE.init(Mutex::new(Outie {
    //     tx: OUTGOING.sender(),
    //     opool: whole_pool,
    // }));

    spawner.must_spawn(run_sub(sub));

    let fut = async {
        loop {
            Timer::after_secs(10).await;
            defmt::info!("I'm alive!");
        }
    };

    fut.await;

}

#[embassy_executor::task]
async fn run_sub(mut sub: erdnuss_comms::target::Target<'static, Config, 8>) {
    sub.run().await;
}