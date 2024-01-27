use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::asm::bkpt;
use defmt::global_logger;
use embassy_sync::{blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex}, pipe::Writer};

static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static TAKEN: AtomicBool = AtomicBool::new(false);
static mut CS_RESTORE: critical_section::RestoreState = critical_section::RestoreState::invalid();

/// All of this nonsense is to try and erase the Error type of the `embedded_hal::serial::nb::Write` implementor.
// type WriterStore = Writer<'static, CriticalSectionRawMutex, 1024>;
static mut WRITEFN: Option<Writer<CriticalSectionRawMutex, 128>> = None;

enum SFn<'a> {
    Buf(&'a [u8]),
    Flush,
}

// unsafe fn trampoline<F>(buf: SFn)
// where
//     F: FnMut(SFn),
// {
//     if let Some(wfn) = WRITEFN {
//         let wfn = &mut *(wfn as *mut F);
//         wfn(buf);
//     }
// }

// fn get_trampoline<F>(_closure: &F) -> WriteCB
// where
//     F: FnMut(SFn),
// {
//     trampoline::<F>
// }

/// Assign a serial peripheral to receive defmt-messages.
///
///
/// ```no_run
///     let mut serial = hal::uart::Uart0::new(dp.UART0, pins.tx0, pins.rx0);
///     defmt_serial::defmt_serial(serial);
///
///     defmt::info!("Hello from defmt!");
/// ```
///
/// The peripheral should implement the [`embedded_hal::blocking::serial::Write`] trait. If your HAL
/// already has the non-blocking [`Write`](embedded_hal::serial::Write) implemented, it can opt-in
/// to the [default implementation](embedded_hal::blocking::serial::write::Default).
pub fn defmt_serial<'p, M, const N: usize>(writer: Writer<'static, CriticalSectionRawMutex, 128>)
where
    M: RawMutex,
{
    // let writer = core::mem::ManuallyDrop::new(writer);

    // let wfn = move |a: SFn| {
    //     match a {
    //         SFn::Buf(buf) => {
    //             bkpt();
    //             let _ = writer.try_write(buf);
    //             // for b in buf {
    //             // }
    //         }
    //         SFn::Flush => {
    //         }
    //     };
    // };

    // let trampoline = get_trampoline(&wfn);

    unsafe {
        let token = critical_section::acquire();
        WRITEFN = Some(writer);
        critical_section::release(token);
    }
}

#[global_logger]
struct GlobalSerialLogger;

unsafe impl defmt::Logger for GlobalSerialLogger {
    fn acquire() {
        let restore = unsafe { critical_section::acquire() };

        if TAKEN.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly");
        }

        TAKEN.store(true, Ordering::Relaxed);

        unsafe {
            CS_RESTORE = restore;
        }

        unsafe { ENCODER.start_frame(write_serial) }
    }

    unsafe fn release() {
        ENCODER.end_frame(write_serial);
        TAKEN.store(false, Ordering::Relaxed);

        let restore = CS_RESTORE;
        critical_section::release(restore);
    }

    unsafe fn write(bytes: &[u8]) {
        ENCODER.write(bytes, write_serial);
    }

    unsafe fn flush() {
        // if let Some(wfn) = WRITEFN {
        //     wfn(SFn::Flush);
        // }
    }
}

/// Write to serial using proxy function. We must ensure this function is not called
/// several times in parallel.
fn write_serial(remaining: &[u8]) {
    unsafe {
        if let Some(wfn) = WRITEFN {
            let _ = wfn.try_write(remaining);
        }
    }
}