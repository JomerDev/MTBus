use core::borrow::Borrow;

use embassy_futures::join::{join, join3};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use mtbus_shared::defmt_usb_logger::defmt_serial;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

const MAX_PACKET_SIZE: u8 = 8;

static mut BUFFER: Pipe<CriticalSectionRawMutex, 128> = Pipe::new();

#[embassy_executor::task]
pub async fn run_uart(usb: USB) {
    let driver = Driver::new(usb, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("PIO UART example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();
    // let mut state2 = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut builder, &mut state, MAX_PACKET_SIZE as u16);
    // let class2 = CdcAcmClass::new(&mut builder, &mut state2, 64);

    let (mut sender, mut receiver) = class.split();

    // let buffer: Pipe<CriticalSectionRawMutex, 1024> = Pipe::new();

    let (reader, writer) = unsafe { BUFFER.split() };

    defmt_serial::<CriticalSectionRawMutex, 128>(writer);


    let mut device = builder.build();
    loop {
        let run_fut = device.run();
        let log_fut = async {
            let mut rx: [u8; MAX_PACKET_SIZE as usize] = [0; MAX_PACKET_SIZE as usize];
            sender.wait_connection().await; 
            loop {
                let len = reader.read(&mut rx[..]).await;
                let _ = sender.write_packet(&rx[..len]).await;
                if len as u8 == MAX_PACKET_SIZE {
                    let _ = sender.write_packet(&[]).await;
                }
            }
        };
        let discard_fut = async {
            let mut discard_buf: [u8; MAX_PACKET_SIZE as usize] = [0; MAX_PACKET_SIZE as usize];
            receiver.wait_connection().await;
            loop {
                let _ = receiver.read_packet(&mut discard_buf).await;
            }
        };
        join3(run_fut, log_fut, discard_fut).await;
    }
}