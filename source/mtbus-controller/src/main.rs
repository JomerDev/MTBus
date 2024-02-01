#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, gpio::{AnyPin, Level, Output}, peripherals::UART0, uart::{self, InterruptHandler}};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::{Duration, Instant, Ticker, Timer};

use erdnuss_comms::{frame_pool::{FrameStorage, RawFrameSlice}, Controller, LastChange, MAX_TARGETS};
use erdnuss_rp2040::{get_rand, Rs485Uart};
use heapless::Vec;
use panic_probe as _;
use mtbus_shared::led_settings::{led_runner, set_led, LedStatus};
use uart_runner::run_uart;

mod uart_runner;

bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandler<UART0>;
});

static CON: Controller<ThreadModeRawMutex> = Controller::uninit();
static POOL: FrameStorage<256> = FrameStorage::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());

    spawner.spawn(run_uart(p.USB)).unwrap();
    Timer::after_secs(8).await; // Wait two seconds so the usb connection can restart

    let _ = spawner.spawn(led_runner(
        p.PIN_11, p.PIN_12, p.PIN_16, p.PIN_17, p.PIN_25, p.PIO0, p.DMA_CH2,
    ));

    set_led(LedStatus::Green, false);

    defmt::info!("Startup\n\n");

    let mac = erdnuss_rp2040::get_unique_id(&mut p.FLASH).unwrap();
    // defmt::info!("MAC: {0=0..7:02X}:{0=8..15:02X}:{0=16..23:02X}:{0=24..31:02X}:{0=32..39:02X}:{0=40..47:02X}:{0=48..55:02X}:{0=56..63:02X}", unique);
    defmt::info!("MAC: {=u64:08X}", mac);

    let mut config = uart::Config::default();
    config.baudrate = 7_812_500;
    // config.baudrate = 115200;
    let uart = uart::Uart::new(
        p.UART0, p.PIN_0, p.PIN_1, Irqs, p.DMA_CH0, p.DMA_CH1, config,
    );

    let io_pin1 = Output::new(AnyPin::from(p.PIN_29), Level::Low);
    let rs485 = Rs485Uart::new(uart, io_pin1);

    let mut pool = POOL.take().unwrap();
    let dom_pool = pool.split(128).unwrap();

    spawner.must_spawn(control(mac, rs485, dom_pool));
}

#[embassy_executor::task]
async fn control(unique: u64, mut rs485: Rs485Uart<UART0>, mut rfs: RawFrameSlice) {
    let mut tick = Ticker::every(Duration::from_millis(100));

    let dom = &CON;

    let mut known_macs: heapless::Vec<u64, {MAX_TARGETS}> = heapless::Vec::new();
    let _ = known_macs.push(0x4250304638313305);

    dom.init(&mut rfs).await;
    dom.add_known_macs(known_macs).await;
    let mut rand = get_rand(unique);
    let mut buf = [0u8; 256];
    // let mut last_routing = Instant::now();
    // let mut routing_seq = 0;

    loop {
        // defmt::info!("Dom step");
        tick.next().await;
        if let Ok(changes) = dom.step(&mut rs485, &mut rand).await {
            for change in changes {
                match change {
                    LastChange::Active(mac) => defmt::info!("Connected: {=u64:08X}", mac),
                    LastChange::Disconnected(mac) => defmt::info!("Disconnected: {=u64:08X}", mac),
                    LastChange::Known(mac) => defmt::info!("Known: {=u64:08X}", mac),
                    LastChange::Pending(mac) => defmt::info!("Pending: {=u64:08X}", mac),
                }
            }
        } else {
            defmt::error!("Dom step err :(");
        };

        let conn = dom.connected().await;

        for d in conn {
            while let Ok(m) = dom.recv_from(d).await {
                if m.len() <= 1 {
                    continue;
                }
                let m = m.payload();
                buf[..8].copy_from_slice(&d.to_le_bytes());
                buf[8..][..m.len()].copy_from_slice(m);
                defmt::println!("Got message len {=usize} from {=u64}: {}", m.len(), d, &buf[..8 + m.len()]);
            }
        }
    }
}
