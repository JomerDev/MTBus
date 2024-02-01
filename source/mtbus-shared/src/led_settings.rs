use embassy_rp::{
    clocks,
    dma::{AnyChannel, Channel},
    into_ref,
    pio::{
        Common, Config, FifoJoin, Instance, PioPin, ShiftConfig, ShiftDirection,
        StateMachine, Pio, InterruptHandler,
    },
    Peripheral, PeripheralRef, gpio::{Output, Level, AnyPin}, bind_interrupts, peripherals::{DMA_CH0, DMA_CH2, PIN_11, PIN_12, PIN_16, PIN_17, PIN_25, PIO0},
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::Timer;
use fixed::types::U24F8;
use fixed_macro::fixed;
use smart_leds::{RGB8, RGB};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});


static LED_SETTINGS: Signal<ThreadModeRawMutex, LedSettings> = Signal::new();

pub struct Ws2812<'d, P: Instance, const S: usize, const N: usize> {
    dma: PeripheralRef<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize, const N: usize> Ws2812<'d, P, S, N> {
    pub fn new(
        pio: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        dma: impl Peripheral<P = impl Channel> + 'd,
        pin: impl PioPin,
    ) -> Self {
        into_ref!(dma);

        // Setup sm0

        // prepare the PIO program
        let side_set = pio::SideSet::new(false, 1, false);
        let mut a: pio::Assembler<32> = pio::Assembler::new_with_side_set(side_set);

        const T1: u8 = 2; // start bit
        const T2: u8 = 5; // data bit
        const T3: u8 = 3; // stop bit
        const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;

        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_zero = a.label();
        a.set_with_side_set(pio::SetDestination::PINDIRS, 1, 0);
        a.bind(&mut wrap_target);
        // Do stop bit
        a.out_with_delay_and_side_set(pio::OutDestination::X, 1, T3 - 1, 0);
        // Do start bit
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
        // Do data bit = 1
        a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
        a.bind(&mut do_zero);
        // Do data bit = 0
        a.nop_with_delay_and_side_set(T2 - 1, 0);
        a.bind(&mut wrap_source);

        let prg = a.assemble_with_wrap(wrap_source, wrap_target);
        let mut cfg = Config::default();

        // Pin config
        let out_pin = pio.make_pio_pin(pin);
        cfg.set_out_pins(&[&out_pin]);
        cfg.set_set_pins(&[&out_pin]);

        cfg.use_program(&pio.load_program(&prg), &[&out_pin]);

        // Clock config, measured in kHz to avoid overflows
        // TODO CLOCK_FREQ should come from embassy_rp
        let clock_freq = U24F8::from_num(clocks::clk_sys_freq() / 1000);
        let ws2812_freq = fixed!(800: U24F8);
        let bit_freq = ws2812_freq * CYCLES_PER_BIT;
        cfg.clock_divider = clock_freq / bit_freq;

        // FIFO config
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 24,
            direction: ShiftDirection::Left,
        };

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self {
            dma: dma.map_into(),
            sm,
        }
    }

    pub async fn write(&mut self, colors: &[RGB8; N]) {
        // Precompute the word bytes from the colors
        let mut words = [0u32; N];
        for i in 0..N {
            let word = (u32::from(colors[i].g) << 24)
                | (u32::from(colors[i].r) << 16)
                | (u32::from(colors[i].b) << 8);
            words[i] = word;
        }

        // DMA transfer
        self.sm.tx().dma_push(self.dma.reborrow(), &words).await;
    }
}

pub enum RGBStatus {
    Blue,
    Red,
    Green,
    Color(u8, u8, u8),
    Off
}

impl RGBStatus {
    pub fn to_rgb(&self) -> RGB<u8> {
        match self {
            RGBStatus::Off => (0,0,0).into(),
            RGBStatus::Red => (255,0,0).into(),
            RGBStatus::Green => (0,255,0).into(),
            RGBStatus::Blue => (0,0,255).into(),
            RGBStatus::Color(r, g, b) => (*r, *g, *b).into()
        }
    }
}

pub enum LedStatus {
    Off,
    Red,
    Green,
    Blue,
    Yellow,
}

impl LedStatus {
    pub fn to_bits(&self) -> [bool; 3] {
        match self {
            LedStatus::Off => [false, false, false],
            LedStatus::Red => [true, false, false],
            LedStatus::Green => [false, true, false],
            LedStatus::Blue => [false, false, true],
            LedStatus::Yellow => [true, true, false],
        }
    }
}

pub struct LedSettings {
    pub blink_ms: Option<u64>,
    pub led_status: Option<LedStatus>,
    pub rgb_status: Option<RGBStatus>,
    pub led_blinking: Option<bool>,
    pub rgb_blinking: Option<bool>,
}

impl Default for LedSettings {
    fn default() -> Self {
        LedSettings {
            led_blinking: None,
            rgb_blinking: None,
            blink_ms: None,
            led_status: None,
            rgb_status: None
        }
    }
}


pub fn set_led_settings(set: LedSettings) {
    LED_SETTINGS.signal(set);
}

pub fn set_led(status: LedStatus, blink: bool) {
    set_led_settings(LedSettings {
        led_blinking: Some(blink),
        led_status: Some(status),
        ..Default::default()
    });
} 

pub fn set_rgb(status: RGBStatus, blink: bool) {
    set_led_settings(LedSettings {
        rgb_blinking: Some(blink),
        rgb_status: Some(status),
        ..Default::default()
    });
} 

struct Pins([Output<'static>; 3]);

#[embassy_executor::task]
pub async fn led_runner(
    p11: PIN_11,
    p12: PIN_12,
    p16: PIN_16,
    p17: PIN_17,
    p25: PIN_25,
    pio0: PIO0,
    dma_ch0: DMA_CH2,
) {

    let _pow = Output::new(p11, Level::High);

    let Pio { mut common, sm0, .. } = Pio::new(pio0, Irqs);

    const NUM_LEDS: usize = 1;
    let mut rgb_data = [RGB8::default(); NUM_LEDS];
    let rgb_data_off = [RGB8::default(); NUM_LEDS];

    let mut ws = Ws2812::new(&mut common, sm0, dma_ch0, p12);

    let led_green = Output::new(AnyPin::from(p16), Level::Low); // PIN_16: User Led Green
    let led_red = Output::new(AnyPin::from(p17), Level::Low); // PIN_17: User Led Red
    let led_blue = Output::new(AnyPin::from(p25), Level::Low); // PIN_25: User Led Blue

    let mut pins = Pins([led_red, led_green, led_blue]);

    let mut led_blinking = false;
    let mut rgb_blinking: bool = false;
    let mut blink_ms: u64 = 500;
    let mut led_list: [bool; 3] = LedStatus::Off.to_bits();
    

    loop {
        if let Some(settings) = LED_SETTINGS.try_take() {
            if let Some(new_led_blinking) = settings.led_blinking {
                led_blinking = new_led_blinking;
            }
            if let Some(new_rgb_blinking) = settings.rgb_blinking {
                rgb_blinking = new_rgb_blinking;
            }
            if let Some(new_led_status) = settings.led_status {
                led_list = new_led_status.to_bits();
                if !led_blinking {
                    pins.0.iter_mut()
                        .zip(led_list.into_iter())
                        .for_each(|(p, c)| if c { p.set_low() } else { p.set_high() });
                }
            }
            if let Some(ms) = settings.blink_ms {
                blink_ms = ms;
            }
            if let Some(status) = settings.rgb_status {
                rgb_data[0] = status.to_rgb();
            }

            LED_SETTINGS.reset();
        }

        if led_blinking {
            pins.0.iter_mut()
                .zip(led_list.into_iter())
                .for_each(|(p, c)| if c { p.set_low() } else { p.set_high() });
        }
        if rgb_blinking {
            ws.write(&rgb_data).await;
        }
        Timer::after_millis(blink_ms).await;

        if led_blinking {
            pins.0.iter_mut()
                .for_each(|p| p.set_high());
        }
        if rgb_blinking {
            ws.write(&rgb_data_off).await;
        }
        Timer::after_millis(blink_ms).await;
    }
}