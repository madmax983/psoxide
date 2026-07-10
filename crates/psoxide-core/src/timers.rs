//! Root counters (hardware timers 0/1/2).
//!
//! The PlayStation has three 16-bit "root counters" mapped into the I/O window
//! at `0x1F80_1100..=0x1F80_112F`. Each timer occupies a 0x10 stride with three
//! registers:
//!
//! | Offset | Register        | Notes                                          |
//! |--------|-----------------|------------------------------------------------|
//! | `0x00` | Current value   | 16-bit, wraps                                  |
//! | `0x04` | Mode / control  | see the bit table below (read clears 11/12)    |
//! | `0x08` | Target value    | 16-bit compare target                          |
//!
//! ### Mode register bits (per Nocash PSX spec)
//!
//! * bit 0     — synchronization enable
//! * bits 1-2  — synchronization mode
//! * bit 3     — reset counter to 0 on **target** (1) vs on **0xFFFF** overflow (0)
//! * bit 4     — IRQ when counter == target
//! * bit 5     — IRQ when counter == 0xFFFF (overflow)
//! * bit 6     — IRQ repeat (1) vs one-shot (0)
//! * bit 7     — IRQ toggle (1) vs pulse (0)
//! * bits 8-9  — clock source (timer-specific; see [`Counter::clock_source`])
//! * bit 10    — IRQ request (0 = an IRQ is currently being requested); this
//!   emulator models it as "1 = armed / no IRQ pending yet". It is set on a mode
//!   write and cleared while an IRQ is asserted.
//! * bit 11    — reached-target flag (set by hardware, cleared on mode read)
//! * bit 12    — reached-overflow flag (set by hardware, cleared on mode read)
//!
//! ### Timing approximation
//!
//! The core steps one CPU cycle per instruction, so [`Timers::tick`] is called
//! with `cycles = 1` at every instruction boundary. Clock sources other than the
//! full system clock are approximated with fixed integer divisors:
//!
//! * **sysclk** — 1 timer tick per CPU cycle.
//! * **sysclk/8** (timer 2) — 1 tick per 8 CPU cycles (fractional carry kept).
//! * **dotclock** (timer 0) — approximated as sysclk/6 (~320-wide dot clock).
//! * **hblank** (timer 1) — approximated as one tick per ~2172 CPU cycles, the
//!   NTSC scanline length in CPU cycles.
//!
//! These divisors are deliberately pragmatic: the goal is correct counting
//! semantics and IRQ delivery (enough for ps1-tests `access-time`), not
//! cycle-perfect video timing.

use serde::{Deserialize, Serialize};

use crate::irq::{Irq, IrqLine};

/// Physical base address of the timer register block.
pub const TIMERS_BASE: u32 = 0x1F80_1100;
/// One past the last timer register (`0x1F80_112F`).
pub const TIMERS_END: u32 = 0x1F80_112F;

/// Approximate CPU cycles per NTSC scanline (used for the hblank clock source).
const CYCLES_PER_SCANLINE: u32 = 2172;
/// Approximate CPU-cycle divisor for the dot clock (~320-wide).
const DOTCLOCK_DIV: u32 = 6;

/// A single root counter.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Counter {
    /// Current 16-bit counter value.
    pub value: u16,
    /// Mode / control register.
    pub mode: u16,
    /// Compare target.
    pub target: u16,
    /// Fractional-cycle accumulator for sub-sysclk clock sources (sysclk/8,
    /// dotclock, hblank). Counts CPU cycles not yet converted into timer ticks.
    accumulator: u32,
    /// One-shot latch: set once an IRQ has fired in one-shot mode; blocks
    /// further IRQs until the mode register is rewritten (re-armed).
    irq_fired: bool,
}

impl Default for Counter {
    fn default() -> Self {
        Self::new()
    }
}

impl Counter {
    /// Creates a zeroed counter.
    #[must_use]
    pub fn new() -> Self {
        Self {
            value: 0,
            mode: 0,
            target: 0,
            accumulator: 0,
            irq_fired: false,
        }
    }

    // ── Mode-bit helpers ────────────────────────────────────────────────
    #[inline]
    fn reset_on_target(&self) -> bool {
        self.mode & (1 << 3) != 0
    }
    #[inline]
    fn irq_on_target(&self) -> bool {
        self.mode & (1 << 4) != 0
    }
    #[inline]
    fn irq_on_overflow(&self) -> bool {
        self.mode & (1 << 5) != 0
    }
    #[inline]
    fn irq_repeat(&self) -> bool {
        self.mode & (1 << 6) != 0
    }

    /// Returns the clock divisor (in CPU cycles per timer tick) for this
    /// counter, given its 0-based `index` and the clock-source select bits.
    fn clock_source(&self, index: usize) -> u32 {
        let sel = (self.mode >> 8) & 0x3;
        match index {
            // Timer 0: 0/2 = sysclk, 1/3 = dotclock.
            0 => {
                if sel == 1 || sel == 3 {
                    DOTCLOCK_DIV
                } else {
                    1
                }
            }
            // Timer 1: 0/2 = sysclk, 1/3 = hblank.
            1 => {
                if sel == 1 || sel == 3 {
                    CYCLES_PER_SCANLINE
                } else {
                    1
                }
            }
            // Timer 2: 0/1 = sysclk, 2/3 = sysclk/8.
            _ => {
                if sel == 2 || sel == 3 {
                    8
                } else {
                    1
                }
            }
        }
    }

    /// Reads the mode register, clearing the reached-target/overflow flags
    /// (bits 11/12) as a side effect (hardware clears them on read).
    fn read_mode(&mut self) -> u16 {
        let v = self.mode;
        self.mode &= !((1 << 11) | (1 << 12));
        v
    }

    /// Writes the mode register: resets the counter value to 0, re-arms the IRQ
    /// (bit 10 = 1, one-shot latch cleared), and clears the reached flags.
    fn write_mode(&mut self, val: u16) {
        // Bit 10 (IRQ request) reads as 1 when armed; bits 11/12 clear on write.
        self.mode = (val & !((1 << 11) | (1 << 12))) | (1 << 10);
        self.value = 0;
        self.accumulator = 0;
        self.irq_fired = false;
    }

    /// Advances this counter by `cycles` CPU cycles, raising `irq` (via
    /// interrupt line `line`) on the configured target/overflow conditions.
    fn tick(&mut self, index: usize, cycles: u32, irq: &mut Irq, line: IrqLine) {
        let div = self.clock_source(index);
        self.accumulator += cycles;
        // Divider math, preserved exactly. The full-system-clock source
        // (`div == 1`) is the reset-state configuration of all three counters
        // and is ticked once per CPU cycle, so the `accumulator / div` +
        // `accumulator %= div` divide/modulo runs on every instruction for every
        // counter — the single hottest cost in `Timers::tick`. For `div == 1`
        // the accumulator is invariantly 0 on entry (the clock source only
        // changes via a mode write, and `write_mode` zeroes the accumulator), so
        // `accumulator / 1 == accumulator` and `accumulator %= 1 == 0`: take that
        // branch without an actual hardware division. This is byte-for-byte
        // equivalent to the divide/modulo (see `batched_tick_matches_per_cycle`).
        let ticks = if div == 1 {
            let t = self.accumulator;
            self.accumulator = 0;
            t
        } else {
            let t = self.accumulator / div;
            self.accumulator %= div;
            t
        };
        for _ in 0..ticks {
            self.step_one(irq, line);
        }
    }

    /// Advances the counter by exactly one timer tick.
    fn step_one(&mut self, irq: &mut Irq, line: IrqLine) {
        let (next, overflowed) = self.value.overflowing_add(1);

        // Target reached: current value equals the target (checked on the value
        // *before* a target-reset wraps it back to 0).
        let hit_target = next == self.target;

        if hit_target {
            self.mode |= 1 << 11;
            if self.irq_on_target() {
                self.raise(irq, line);
            }
            if self.reset_on_target() {
                self.value = 0;
                return;
            }
        }

        if overflowed {
            self.mode |= 1 << 12;
            if self.irq_on_overflow() {
                self.raise(irq, line);
            }
            // Wrapping to 0 happens naturally via overflowing_add's result.
        }

        self.value = next;
    }

    /// Number of CPU cycles from the current state until this counter would
    /// next raise its interrupt, or `None` if the current configuration raises
    /// no further interrupt (IRQ disabled, or a spent one-shot).
    ///
    /// This is the counter's contribution to the lazy device scheduler's
    /// next-event deadline. It is **conservative**: it may return a value less
    /// than or equal to the true next-event cycle, never larger. In particular
    /// the overflow calculation ignores `reset_on_target` (which can only delay
    /// or cancel an overflow), so the returned offset is a safe lower bound.
    fn cycles_to_next_event(&self, index: usize) -> Option<u64> {
        let want_target = self.irq_on_target();
        let want_overflow = self.irq_on_overflow();
        if !want_target && !want_overflow {
            return None;
        }
        // A spent one-shot (bit 6 clear, already fired) raises no further IRQ
        // until the mode register is rewritten (which re-arms via `write_mode`).
        if !self.irq_repeat() && self.irq_fired {
            return None;
        }

        let v = u32::from(self.value);
        let mut steps: Option<u64> = None;
        if want_target {
            // `step_one` sets `next = value + 1` and hits when `next == target`.
            // After `s` increments the value is `v + s`, so the first hit is at
            // `s = (target - v) mod 2^16`, or a full `2^16` when they are equal.
            let d = (u32::from(self.target).wrapping_sub(v)) & 0xFFFF;
            let s = if d == 0 { 0x1_0000 } else { u64::from(d) };
            steps = Some(steps.map_or(s, |b: u64| b.min(s)));
        }
        if want_overflow {
            // Overflow occurs when the value increments from 0xFFFF to 0, i.e.
            // when the pre-increment value is 0xFFFF: `s = 0x10000 - v`.
            let s = 0x1_0000u64 - u64::from(v);
            steps = Some(steps.map_or(s, |b: u64| b.min(s)));
        }
        let steps = steps?;

        // The n-th timer tick occurs at CPU-cycle offset `n * div - accumulator`
        // (the accumulator holds `< div` residual cycles on entry). Clamp to a
        // minimum of 1 so the scheduler always makes forward progress.
        let div = u64::from(self.clock_source(index));
        let acc = u64::from(self.accumulator);
        let cycles = steps.saturating_mul(div).saturating_sub(acc);
        Some(cycles.max(1))
    }

    /// Asserts the timer interrupt, honoring the one-shot (bit 6 clear) latch.
    fn raise(&mut self, irq: &mut Irq, line: IrqLine) {
        if !self.irq_repeat() {
            if self.irq_fired {
                return;
            }
            self.irq_fired = true;
        }
        // Bit 10 low indicates an IRQ is being requested.
        self.mode &= !(1 << 10);
        irq.set(line);
    }
}

/// The three PlayStation root counters.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Timers {
    /// Counters 0, 1, 2.
    pub counters: [Counter; 3],
}

impl Default for Timers {
    fn default() -> Self {
        Self::new()
    }
}

impl Timers {
    /// Creates three zeroed counters.
    #[must_use]
    pub fn new() -> Self {
        Self {
            counters: [Counter::new(), Counter::new(), Counter::new()],
        }
    }

    /// Returns the IRQ line for counter `index`.
    #[inline]
    fn line(index: usize) -> IrqLine {
        match index {
            0 => IrqLine::Timer0,
            1 => IrqLine::Timer1,
            _ => IrqLine::Timer2,
        }
    }

    /// Decodes a physical address into `(counter index, register offset)`.
    /// Returns `None` for addresses outside the timer block.
    #[inline]
    fn decode(phys: u32) -> Option<(usize, u32)> {
        if !(TIMERS_BASE..=TIMERS_END).contains(&phys) {
            return None;
        }
        let index = ((phys - TIMERS_BASE) / 0x10) as usize;
        let which = phys & 0xF;
        if index < 3 {
            Some((index, which))
        } else {
            None
        }
    }

    /// Reads a 16-bit timer register.
    #[must_use]
    pub fn read16(&mut self, phys: u32) -> u16 {
        let Some((index, which)) = Self::decode(phys) else {
            return 0;
        };
        let c = &mut self.counters[index];
        match which {
            0x0 => c.value,
            0x4 => c.read_mode(),
            0x8 => c.target,
            _ => 0,
        }
    }

    /// Writes a 16-bit timer register.
    pub fn write16(&mut self, phys: u32, val: u16) {
        let Some((index, which)) = Self::decode(phys) else {
            return;
        };
        let c = &mut self.counters[index];
        match which {
            0x0 => c.value = val,
            0x4 => c.write_mode(val),
            0x8 => c.target = val,
            _ => {}
        }
    }

    /// Reads a 32-bit timer register (the counters are 16-bit; the high half
    /// reads back as 0).
    #[must_use]
    pub fn read32(&mut self, phys: u32) -> u32 {
        u32::from(self.read16(phys))
    }

    /// Writes a 32-bit timer register (only the low 16 bits are significant).
    pub fn write32(&mut self, phys: u32, val: u32) {
        self.write16(phys, val as u16);
    }

    /// Advances all three counters by `cycles` CPU cycles, delivering any timer
    /// interrupts through `irq`.
    pub fn tick(&mut self, cycles: u32, irq: &mut Irq) {
        for index in 0..3 {
            let line = Self::line(index);
            self.counters[index].tick(index, cycles, irq, line);
        }
    }

    /// Number of CPU cycles until the earliest of the three counters would next
    /// raise its interrupt, or `None` if no counter has an armed interrupt.
    /// Conservative (never larger than the true next event); see
    /// [`Counter::cycles_to_next_event`].
    #[must_use]
    pub fn cycles_to_next_event(&self) -> Option<u64> {
        let mut best: Option<u64> = None;
        for index in 0..3 {
            if let Some(c) = self.counters[index].cycles_to_next_event(index) {
                best = Some(best.map_or(c, |b: u64| b.min(c)));
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const T0_VALUE: u32 = 0x1F80_1100;
    const T0_MODE: u32 = 0x1F80_1104;
    const T0_TARGET: u32 = 0x1F80_1108;
    const T2_MODE: u32 = 0x1F80_1124;

    #[test]
    fn counting_increments_value() {
        let mut t = Timers::new();
        let mut irq = Irq::new();
        t.tick(5, &mut irq);
        assert_eq!(t.read16(T0_VALUE), 5);
    }

    #[test]
    fn write_mode_zeroes_counter() {
        let mut t = Timers::new();
        let mut irq = Irq::new();
        t.tick(100, &mut irq);
        assert_eq!(t.read16(T0_VALUE), 100);
        t.write16(T0_MODE, 0);
        assert_eq!(t.read16(T0_VALUE), 0);
    }

    #[test]
    fn target_reached_wraps_when_reset_bit_set() {
        let mut t = Timers::new();
        let mut irq = Irq::new();
        // Reset-on-target (bit 3), no IRQ.
        t.write16(T0_MODE, 1 << 3);
        t.write16(T0_TARGET, 4);
        t.tick(4, &mut irq); // value reaches 4 == target -> reset to 0
        assert_eq!(t.read16(T0_VALUE), 0);
    }

    #[test]
    fn target_reached_keeps_counting_when_reset_bit_clear() {
        let mut t = Timers::new();
        let mut irq = Irq::new();
        t.write16(T0_MODE, 0); // no reset-on-target
        t.write16(T0_TARGET, 4);
        t.tick(6, &mut irq); // passes target, keeps counting to 6
        assert_eq!(t.read16(T0_VALUE), 6);
    }

    #[test]
    fn overflow_wraps_at_ffff() {
        let mut t = Timers::new();
        let mut irq = Irq::new();
        t.write16(T0_MODE, 0);
        t.write16(T0_VALUE, 0xFFFF);
        t.tick(2, &mut irq); // 0xFFFF -> 0 -> 1
        assert_eq!(t.read16(T0_VALUE), 1);
    }

    #[test]
    fn irq_raised_on_target_when_enabled() {
        let mut t = Timers::new();
        let mut irq = Irq::new();
        // IRQ-on-target (bit 4), one-shot.
        t.write16(T0_MODE, 1 << 4);
        t.write16(T0_TARGET, 3);
        t.tick(3, &mut irq);
        assert_ne!(
            irq.read_stat() & (1 << IrqLine::Timer0.bit()),
            0,
            "Timer0 IRQ should be pending"
        );
    }

    #[test]
    fn no_irq_when_disabled() {
        let mut t = Timers::new();
        let mut irq = Irq::new();
        t.write16(T0_MODE, 0); // no IRQ bits
        t.write16(T0_TARGET, 3);
        t.tick(10, &mut irq);
        assert_eq!(irq.read_stat(), 0);
    }

    #[test]
    fn one_shot_fires_once_repeat_fires_repeatedly() {
        // One-shot: IRQ only once even across multiple target hits.
        let mut t = Timers::new();
        let mut irq = Irq::new();
        t.write16(T0_MODE, (1 << 4) | (1 << 3)); // irq-on-target + reset-on-target, one-shot
        t.write16(T0_TARGET, 2);
        t.tick(2, &mut irq); // hit #1 -> IRQ
        assert_ne!(irq.read_stat() & (1 << IrqLine::Timer0.bit()), 0);
        // Ack and hit target again; one-shot must NOT re-raise.
        irq.write_stat(0);
        t.tick(3, &mut irq); // wraps and hits target again
        assert_eq!(
            irq.read_stat() & (1 << IrqLine::Timer0.bit()),
            0,
            "one-shot must not re-fire until re-armed"
        );

        // Repeat mode re-fires.
        let mut t = Timers::new();
        let mut irq = Irq::new();
        t.write16(T0_MODE, (1 << 4) | (1 << 3) | (1 << 6)); // + repeat
        t.write16(T0_TARGET, 2);
        t.tick(2, &mut irq);
        assert_ne!(irq.read_stat() & (1 << IrqLine::Timer0.bit()), 0);
        irq.write_stat(0);
        t.tick(3, &mut irq); // wrap + hit target again
        assert_ne!(
            irq.read_stat() & (1 << IrqLine::Timer0.bit()),
            0,
            "repeat mode should re-fire"
        );
    }

    #[test]
    fn read_mode_clears_reached_flags() {
        let mut t = Timers::new();
        let mut irq = Irq::new();
        t.write16(T0_MODE, 0);
        t.write16(T0_TARGET, 2);
        t.tick(2, &mut irq);
        let mode = t.read16(T0_MODE);
        assert_ne!(mode & (1 << 11), 0, "reached-target flag should be set");
        // Second read: flag cleared.
        let mode2 = t.read16(T0_MODE);
        assert_eq!(mode2 & (1 << 11), 0, "reached flags clear on read");
    }

    #[test]
    fn timer2_sysclk_div8_source() {
        let mut t = Timers::new();
        let mut irq = Irq::new();
        // Timer 2 clock source sysclk/8 (sel bits = 2).
        t.write16(T2_MODE, 2 << 8);
        t.tick(16, &mut irq); // 16 / 8 = 2 ticks
        assert_eq!(t.read16(0x1F80_1120), 2);
    }

    #[test]
    fn write_mode_rearms_irq_bit10() {
        let mut t = Timers::new();
        let mut irq = Irq::new();
        t.write16(T0_MODE, 1 << 4); // irq on target, one-shot
        t.write16(T0_TARGET, 1);
        t.tick(1, &mut irq);
        // After firing, bit 10 is cleared (request asserted).
        // read_mode would clear reached flags; check raw via a fresh read.
        assert_eq!(t.counters[0].mode & (1 << 10), 0);
        // Rewriting mode re-arms bit 10.
        t.write16(T0_MODE, 1 << 4);
        assert_ne!(t.counters[0].mode & (1 << 10), 0);
    }

    /// Equivalence guard for the `Counter::tick` divider fast path: advancing a
    /// counter with a single `tick(N)` must land on byte-identical state (value,
    /// mode/flags, target, accumulator, one-shot latch) *and* identical IRQ
    /// delivery as advancing an identical counter with `N` separate `tick(1)`
    /// calls — across a matrix of modes, targets, start values, clock sources,
    /// and counts (including spans that cross multiple target hits and the
    /// 0xFFFF overflow). This proves the `div == 1` shortcut and the
    /// accumulator/modulo handling stay consistent with per-cycle stepping.
    #[test]
    fn batched_tick_matches_per_cycle() {
        fn fresh(mode: u16, target: u16, start: u16) -> Counter {
            Counter {
                value: start,
                mode,
                target,
                accumulator: 0,
                irq_fired: false,
            }
        }

        // Event-relevant mode bits: reset-on-target(3), irq-on-target(4),
        // irq-on-overflow(5), irq-repeat(6), in the combinations that matter.
        let modes = [
            0u16,
            1 << 3,
            1 << 4,
            (1 << 4) | (1 << 3),
            (1 << 4) | (1 << 3) | (1 << 6),
            1 << 5,
            (1 << 5) | (1 << 6),
            (1 << 4) | (1 << 5),
            (1 << 4) | (1 << 5) | (1 << 6),
        ];
        let targets = [0u16, 1, 2, 3, 0x00FF, 0xFFFE, 0xFFFF];
        let starts = [0u16, 1, 0x00FE, 0xFFFD, 0xFFFF];

        // (index, clock-source-select-bits) pairs covering every divisor the
        // three counters can select: 1 (sysclk), 6 (dotclock), 2172 (hblank),
        // 8 (sysclk/8).
        let sources = [
            (0usize, 0u16),   // div 1
            (0usize, 1 << 8), // div 6
            (1usize, 1 << 8), // div 2172
            (2usize, 2 << 8), // div 8
            (2usize, 0u16),   // div 1
        ];

        // Small counts exercise event/divisor handling for every source cheaply.
        let small_counts = [0u32, 1, 2, 3, 7, 8, 9, 16, 17, 100, 200, 4344];
        // Large counts (that cross the 0xFFFF overflow) are only feasible to
        // check per-cycle for the sysclk (div 1) sources.
        let big_counts = [0xFFFEu32, 0xFFFF, 0x1_0000, 0x1_0001, 0x1_0003, 0x2_0007];

        let run = |index: usize, mode: u16, target: u16, start: u16, total: u32| {
            // Batched: one tick(total).
            let mut ca = fresh(mode, target, start);
            let mut ia = Irq::new();
            ca.tick(index, total, &mut ia, Timers::line(index));

            // Per-cycle: total × tick(1).
            let mut cb = fresh(mode, target, start);
            let mut ib = Irq::new();
            for _ in 0..total {
                cb.tick(index, 1, &mut ib, Timers::line(index));
            }

            assert_eq!(
                ca, cb,
                "state mismatch idx={index} mode={mode:#06x} target={target} start={start} total={total}"
            );
            assert_eq!(
                ia.read_stat(),
                ib.read_stat(),
                "irq mismatch idx={index} mode={mode:#06x} target={target} start={start} total={total}"
            );
        };

        for &(index, sel) in &sources {
            for &base in &modes {
                let mode = base | sel;
                for &target in &targets {
                    for &start in &starts {
                        for &total in &small_counts {
                            run(index, mode, target, start, total);
                        }
                        // Overflow-spanning counts: sysclk (div 1) sources only.
                        let div = fresh(mode, target, start).clock_source(index);
                        if div == 1 {
                            for &total in &big_counts {
                                run(index, mode, target, start, total);
                            }
                        }
                    }
                }
            }
        }
    }

    /// The scheduler deadline `cycles_to_next_event` must be a safe lower bound
    /// on the cycle a per-cycle tick loop actually raises the interrupt, and
    /// exact except in the documented `reset_on_target + overflow` conservative
    /// case. Drives each config per-cycle and checks the first firing cycle.
    #[test]
    fn cycles_to_next_event_predicts_first_irq() {
        fn fresh(mode: u16, target: u16, start: u16) -> Counter {
            Counter {
                value: start,
                mode,
                target,
                accumulator: 0,
                irq_fired: false,
            }
        }

        let modes = [
            0u16,
            1 << 3,
            1 << 4,
            (1 << 4) | (1 << 3),
            (1 << 4) | (1 << 3) | (1 << 6),
            1 << 5,
            (1 << 5) | (1 << 6),
            (1 << 5) | (1 << 3),
            (1 << 4) | (1 << 5),
            (1 << 4) | (1 << 5) | (1 << 6),
            (1 << 4) | (1 << 5) | (1 << 3),
        ];
        let targets = [0u16, 1, 2, 0x00FF, 0xFFFF];
        let starts = [0u16, 1, 0x00FE, 0xFFFF];
        // Divisors 1 (sysclk), 6 (dotclock), 8 (sysclk/8).
        let sources = [(0usize, 0u16), (0usize, 1 << 8), (2usize, 2 << 8)];

        for &(index, sel) in &sources {
            let line = Timers::line(index);
            for &base in &modes {
                let mode = base | sel;
                for &target in &targets {
                    for &start in &starts {
                        let c = fresh(mode, target, start);
                        let div = c.clock_source(index);
                        let predicted = c.cycles_to_next_event(index);

                        // A config is exact unless the conservative overflow
                        // path (overflow IRQ + reset-on-target) is engaged.
                        let conservative = (mode & (1 << 5) != 0) && (mode & (1 << 3) != 0);

                        match predicted {
                            None => {
                                // IRQ disabled or spent one-shot: no fire within
                                // a generous window (a full 16-bit wrap).
                                let mut cc = fresh(mode, target, start);
                                let mut irq = Irq::new();
                                let bound = 0x1_0000u64 * u64::from(div) + 8;
                                for _ in 0..bound {
                                    cc.tick(index, 1, &mut irq, line);
                                    assert_eq!(
                                        irq.read_stat(),
                                        0,
                                        "None but fired: idx={index} mode={mode:#06x} target={target} start={start}"
                                    );
                                }
                            }
                            Some(n) => {
                                // No fire strictly before the predicted cycle.
                                let mut cc = fresh(mode, target, start);
                                let mut irq = Irq::new();
                                for _ in 0..(n - 1) {
                                    cc.tick(index, 1, &mut irq, line);
                                }
                                assert_eq!(
                                    irq.read_stat(),
                                    0,
                                    "fired before predicted n={n}: idx={index} mode={mode:#06x} target={target} start={start}"
                                );
                                // Exactly at the predicted cycle it fires (unless
                                // conservative, where it fires at or after).
                                cc.tick(index, 1, &mut irq, line);
                                if !conservative {
                                    assert_ne!(
                                        irq.read_stat(),
                                        0,
                                        "did not fire at predicted n={n}: idx={index} mode={mode:#06x} target={target} start={start}"
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// A spent one-shot (bit 6 clear) reports no further deadline until re-armed.
    #[test]
    fn cycles_to_next_event_spent_one_shot_is_none() {
        // One-shot IRQ-on-target at target 4.
        let mut c = Counter {
            value: 0,
            mode: 1 << 4,
            target: 4,
            accumulator: 0,
            irq_fired: false,
        };
        assert_eq!(c.cycles_to_next_event(2), Some(4));
        let mut irq = Irq::new();
        for _ in 0..4 {
            c.tick(2, 1, &mut irq, Timers::line(2));
        }
        assert_ne!(irq.read_stat(), 0, "one-shot fired");
        assert!(c.irq_fired, "one-shot latched");
        assert_eq!(
            c.cycles_to_next_event(2),
            None,
            "spent one-shot: no deadline"
        );
    }
}
