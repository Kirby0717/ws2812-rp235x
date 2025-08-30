#![no_std]
//! これはWS2812のRP235x用ドライバです。
//! WS2812とは、マイコン内蔵RGBLEDでNeoPixel LEDとも呼ばれます。
//!
//! このクレートはws2812-pioを大いに参考にしています。

use cortex_m::prelude::_embedded_hal_timer_CountDown;
use rp235x_hal::{
    fugit::{ExtU32, HertzU32},
    gpio::AnyPin,
    pio::{Buffers, PIOBuilder, PinDir, ShiftDirection},
    pio::{PIO, PIOExt, StateMachineIndex, Tx, UninitStateMachine},
    timer::{CountDown, TimerDevice},
};
use smart_leds::SmartLedsWrite;

pub struct Ws2812Direct<P, SM, I>
where
    I: AnyPin<Function = P::PinFunction>,
    P: PIOExt,
    SM: StateMachineIndex,
{
    tx: Tx<(P, SM)>,
    _pin: I,
}

impl<P, SM, I> Ws2812Direct<P, SM, I>
where
    I: AnyPin<Function = P::PinFunction>,
    P: PIOExt,
    SM: StateMachineIndex,
{
    pub fn new(
        pin: I,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        clock_freq: HertzU32,
    ) -> Self {
        const T1: u8 = 2; // start bit
        const T2: u8 = 5; // data bit
        const T3: u8 = 3; // stop bit
        const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;
        const FREQ: HertzU32 = HertzU32::kHz(800);

        // PIOに入れるプログラム
        let program = pio_proc::pio_asm!(
            ".side_set 1",
            ".define public T1 2",
            ".define public T2 5",
            ".define public T3 3",
            ".wrap_target",
            "bitloop:",
            "    out x, 1           side 0 [T3 - 1]",
            "    jmp !x do_zero     side 1 [T1 - 1]",
            "    jmp bitloop        side 1 [T2 - 1]",
            "do_zero:",
            "    nop                side 0 [T2 - 1]",
            ".wrap",
        );

        let installed = pio.install(&program.program).unwrap();

        // 周波数の計算
        let bit_freq = FREQ * CYCLES_PER_BIT;
        let mut int = clock_freq / bit_freq;
        let rem = clock_freq - (int * bit_freq);
        let frac = (rem * 256) / bit_freq;
        assert!(
            (1..=65536).contains(&int) && (int != 65536 || frac == 0),
            "(System Clock / {}) must be within [1.0, 65536.0].",
            bit_freq.to_kHz()
        );
        if int == 65536 {
            int = 0;
        }
        let int = int as u16;
        let frac = frac as u8;

        let pin = pin.into();
        let (mut sm, _, tx) = PIOBuilder::from_installed_program(installed)
            .buffers(Buffers::OnlyTx)
            .side_set_pin_base(pin.id().num)
            .out_shift_direction(ShiftDirection::Left)
            .autopull(true)
            .pull_threshold(24)
            .clock_divisor_fixed_point(int, frac)
            .build(sm);
        sm.set_pindirs([(pin.id().num, PinDir::Output)]);
        sm.start();

        Self {
            tx,
            _pin: I::from(pin),
        }
    }
}

impl<P, SM, I> SmartLedsWrite for Ws2812Direct<P, SM, I>
where
    I: AnyPin<Function = P::PinFunction>,
    P: PIOExt,
    SM: StateMachineIndex,
{
    type Color = smart_leds::RGB8;
    type Error = ();
    fn write<T, C>(&mut self, iterator: T) -> Result<(), ()>
    where
        T: IntoIterator<Item = C>,
        C: Into<Self::Color>,
    {
        for item in iterator {
            let color: Self::Color = item.into();
            let (r, g, b) = (color.r as u32, color.g as u32, color.b as u32);
            let word: u32 = g << 24 | r << 16 | b << 8;

            while !self.tx.write(word) {
                cortex_m::asm::nop();
            }
        }
        Ok(())
    }
}

pub struct Ws2812<'timer, D, P, SM, I>
where
    D: TimerDevice,
    I: AnyPin<Function = P::PinFunction>,
    P: PIOExt,
    SM: StateMachineIndex,
{
    cd: CountDown<'timer, D>,
    driver: Ws2812Direct<P, SM, I>,
}

impl<'timer, D, P, SM, I> Ws2812<'timer, D, P, SM, I>
where
    D: TimerDevice,
    I: AnyPin<Function = P::PinFunction>,
    P: PIOExt,
    SM: StateMachineIndex,
{
    /// Creates a new instance of this driver.
    pub fn new(
        pin: I,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        clock_freq: HertzU32,
        cd: CountDown<'timer, D>,
    ) -> Ws2812<'timer, D, P, SM, I> {
        let driver = Ws2812Direct::new(pin, pio, sm, clock_freq);
        Self { driver, cd }
    }
}

impl<'timer, D, P, SM, I> SmartLedsWrite for Ws2812<'timer, D, P, SM, I>
where
    D: TimerDevice,
    I: AnyPin<Function = P::PinFunction>,
    P: PIOExt,
    SM: StateMachineIndex,
{
    type Color = smart_leds::RGB8;
    type Error = ();
    fn write<T, J>(&mut self, iterator: T) -> Result<(), ()>
    where
        T: IntoIterator<Item = J>,
        J: Into<Self::Color>,
    {
        self.driver.tx.clear_stalled_flag();
        while !self.driver.tx.is_empty() && !self.driver.tx.has_stalled() {}

        self.cd.start(70_u32.micros());
        let _ = nb::block!(self.cd.wait());

        SmartLedsWrite::write(&mut self.driver, iterator)
    }
}
