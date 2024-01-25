#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_executor::Spawner;

use crate::uart_runner::run_uart;

// defmt_rtt as _, 
use panic_probe as _;
use embassy_rp::{usb::{Driver, InterruptHandler as InterruptHandlerUSB}, peripherals::{USB, UART0}, bind_interrupts, uart::{InterruptHandler as InterruptHandlerUART, self}, gpio::{Output, AnyPin, Level}};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::{Channel, Sender, Receiver}, mutex::Mutex};
use embassy_time::{Timer, Duration, Ticker};
use erdnuss_comms::{target::TgtCfg, frame_pool::{ FrameBox, RawFrame, RawFrameSlice }};
use erdnuss_rp2040::{Rs485Uart, get_rand};
use mtbus_shared::led_settings::{led_runner, set_led, LedStatus};
use grounded::uninit::GroundedArrayCell;
use rand_chacha::ChaCha8Rng;
use static_cell::StaticCell;
use heapless::String;

mod uart_runner;

bind_interrupts!(struct Irqs {
    // USBCTRL_IRQ => InterruptHandlerUSB<USB>;
    UART0_IRQ => InterruptHandlerUART<UART0>;
});

static POOL: GroundedArrayCell<RawFrame, 16> = GroundedArrayCell::const_init();
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
    const ADDRESS_CLAIM_TIMEOUT: Duration = Duration::from_millis(10);
    const TURNAROUND_DELAY: Duration = Duration::from_millis(0);
    const SELECT_TIMEOUT: Duration = Duration::from_millis(10);
}

// #[embassy_executor::task]
// async fn logger_task(driver: Driver<'static, USB>) {
//     embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
// }

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());

    spawner.spawn(run_uart(p.USB)).unwrap();
    Timer::after_secs(2).await; // Wait two seconds so the usb connection can restart

    let _ = spawner.spawn(led_runner(
        p.PIN_11, p.PIN_12, p.PIN_16, p.PIN_17, p.PIN_25, p.PIO0, p.DMA_CH0,
    ));

    set_led(LedStatus::Red, true);

    defmt::info!("Startup");

    let unique = erdnuss_rp2040::get_unique_id(&mut p.FLASH).unwrap();
    defmt::info!("MAC: {=u64:08X}", unique);

    let mut config = uart::Config::default();
    config.baudrate = 7812500;
    let uart = uart::Uart::new(
        p.UART0, p.PIN_0, p.PIN_1, Irqs, p.DMA_CH1, p.DMA_CH2, config,
    );

    let io_pin1 = Output::new(AnyPin::from(p.PIN_4), Level::Low);
    let rs485 = Rs485Uart::new(uart, io_pin1);

    defmt::info!("Here we go!");

    let mut whole_pool = unsafe { RawFrameSlice::from_static(&POOL) };
    let app_pool = whole_pool.split(8).unwrap();

    let sub: erdnuss_comms::target::Target<'_, Config, 8> = erdnuss_comms::target::Target::new(
        rs485,
        INCOMING.sender(),
        OUTGOING.receiver(),
        app_pool,
        unique.to_le_bytes(),
        get_rand(unique),
    );

    let outie_ref = OUTIE.init(Mutex::new(Outie {
        tx: OUTGOING.sender(),
        opool: whole_pool,
    }));

    spawner.must_spawn(run_sub(sub));
    spawner.must_spawn(chatty(unique, outie_ref));

    defmt::info!("Running!");
    // set_led(LedStatus::Blue, true);

}

#[embassy_executor::task]
async fn run_sub(mut sub: erdnuss_comms::target::Target<'static, Config, 8>) {
    sub.run().await;
}


#[embassy_executor::task]
async fn chatty(unique: u64, outie: OutieRef) {
    // use usb_icd::rs485::Log;

    let mut strbuf = String::<128>::new();
    let mut tick = Ticker::every(Duration::from_secs(3));
    let mut seq: u32 = 0;
    write!(&mut strbuf, "Device {unique:016X} says hello!").ok();

    loop {
        tick.next().await;
        let mut out = outie.lock().await;
        let msg = out.alloc_buf().await;
        // let Some(msg) = send_topic::<Log>(buf, seq, &strbuf) else {
        //     continue;
        // };
        out.tx.send(msg).await;
        seq = seq.wrapping_add(1);
    }
}