use cortex_m_rt::{exception, ExceptionFrame};
use embassy_rp::{gpio::{Level, Output}, Peripherals};
use embassy_time::{Instant, Duration};



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
