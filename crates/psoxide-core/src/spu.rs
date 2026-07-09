//! Sound Processing Unit (SPU).
//!
//! The PlayStation SPU is a 24-voice ADPCM sample player with a per-voice
//! ADSR envelope, hardware pitch (with pitch modulation), a noise generator,
//! per-voice and main stereo volume, key-on/key-off, an IRQ-on-address unit,
//! and a 512 KB sample RAM reached by CPU FIFO transfers or DMA channel 4.
//!
//! This module is a real, audible implementation of that datapath:
//!
//! * **ADPCM voices** — the classic 16-byte → 28-sample block codec (shift +
//!   4-tap filter), block looping (`LoopStart`/`LoopEnd`/`LoopRepeat`), and the
//!   `ENDX` end flags.
//! * **ADSR envelope** — the integer PSX-SPX attack/decay/sustain/release model
//!   (linear and exponential slopes, per-phase shift/step rate divider).
//! * **Pitch** — a 12-bit fractional phase counter with linear interpolation
//!   between the two most recently decoded samples, plus pitch modulation
//!   (`PMON`) from the previous voice.
//! * **Noise** — an LFSR clocked from the `SPUCNT` noise-frequency field,
//!   selectable per voice through `NON`.
//! * **Mixing** — per-voice L/R volume, main L/R volume, and a CD-audio input
//!   hook (present but silent this pass), producing an interleaved 44.1 kHz
//!   stereo `i16` stream drained by the frontend.
//!
//! Deliberately simplified / stubbed this pass (all documented inline):
//! reverb (registers are stored and read back but no reverb DSP runs),
//! CD-DA / XA audio mixing (the input hook exists but is fed silence),
//! volume *sweep* envelopes (fixed-volume mode is exact; sweep mode is
//! approximated), and cycle-exact transfer/seek timing.
//!
//! All persisted state is integer-typed so the containing snapshot can derive
//! `Eq`; there is no floating-point math anywhere in the datapath.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use crate::irq::{Irq, IrqLine};

/// Physical base of the SPU register window.
pub const SPU_BASE: u32 = 0x1F80_1C00;
/// Physical end (inclusive) of the SPU register window.
pub const SPU_END: u32 = 0x1F80_1FFF;

/// Size of the SPU sample RAM (512 KB).
pub const SPU_RAM_BYTES: usize = 512 * 1024;
/// Address mask that folds a byte address into [`SPU_RAM_BYTES`].
const RAM_MASK: u32 = (SPU_RAM_BYTES as u32) - 1;

/// Size of the register-file readback store (1 KB window).
pub const SPU_REG_BYTES: usize = 1024;

/// Number of hardware voices.
pub const VOICES: usize = 24;

/// CPU cycles per output sample (33_868_800 / 44_100 = 768 exactly).
pub const CYCLES_PER_SAMPLE: u32 = 768;

/// Output sample rate in Hz.
pub const SAMPLE_RATE: u32 = 44_100;

/// Maximum queued interleaved-stereo samples before the oldest are dropped
/// (~1 second of stereo audio).
const MAX_QUEUED: usize = 88_200;

/// ADPCM decode filter coefficients (positive tap, divided by 64).
const FILTER_POS: [i32; 5] = [0, 60, 115, 98, 122];
/// ADPCM decode filter coefficients (negative tap, divided by 64).
const FILTER_NEG: [i32; 5] = [0, 0, -52, -55, -60];

/// An ADSR envelope phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum AdsrPhase {
    /// Silent / not playing.
    #[default]
    Off,
    /// Rising toward the peak (0x7FFF).
    Attack,
    /// Falling from the peak toward the sustain level.
    Decay,
    /// Held at (or drifting around) the sustain level.
    Sustain,
    /// Falling toward zero after key-off.
    Release,
}

/// Per-voice running state. The static registers (volume, pitch, start address,
/// ADSR configuration) live in the register-file readback store and are read on
/// demand; this struct holds only the live playback state.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
struct Voice {
    /// Current byte address in SPU RAM being decoded.
    cur_addr: u32,
    /// Loop/repeat byte address (latched from a `LoopStart` block or key-on).
    repeat_addr: u32,
    /// 12-bit fractional pitch phase counter.
    counter: u32,
    /// Index of the next sample to consume within the decoded block (0..=28).
    block_pos: u32,
    /// The 28 decoded samples of the current ADPCM block.
    decoded: [i16; 28],
    /// ADPCM decoder history (previous output).
    hist0: i32,
    /// ADPCM decoder history (output before [`Voice::hist0`]).
    hist1: i32,
    /// Older of the two samples used for linear interpolation.
    s0: i16,
    /// Newer of the two samples used for linear interpolation.
    s1: i16,
    /// ADSR envelope phase.
    phase: AdsrPhase,
    /// ADSR envelope level (0..=0x7FFF).
    level: i32,
    /// Sample-ticks remaining until the next envelope update.
    adsr_cycles: i32,
    /// Whether the voice is keyed on (advancing).
    on: bool,
    /// Whether a `LoopEnd` block has been reached (drives `ENDX`).
    ended: bool,
    /// Last per-voice left contribution (readback for 0x1E00..).
    cur_vol_l: i16,
    /// Last per-voice right contribution (readback for 0x1E00..).
    cur_vol_r: i16,
}

impl Voice {
    /// Keys the voice on: resets the decoder to the start address, primes the
    /// interpolation samples, and starts the attack phase.
    fn key_on(&mut self, ram: &[u8], start_reg: u16) {
        self.on = true;
        self.phase = AdsrPhase::Attack;
        self.level = 0;
        self.adsr_cycles = 1;
        self.cur_addr = ((u32::from(start_reg)) << 3) & RAM_MASK;
        self.repeat_addr = self.cur_addr;
        self.counter = 0;
        self.block_pos = 28; // force a decode on the first fetch
        self.hist0 = 0;
        self.hist1 = 0;
        self.ended = false;
        self.s0 = self.next_adpcm(ram).0;
        self.s1 = self.next_adpcm(ram).0;
    }

    /// Keys the voice off: enters the release phase (playback continues to
    /// silence).
    fn key_off(&mut self) {
        if self.on {
            self.phase = AdsrPhase::Release;
            self.adsr_cycles = 1;
        }
    }

    /// Decodes the 16-byte ADPCM block at [`Voice::cur_addr`], advancing
    /// `cur_addr` to the next block (honouring loop flags). Returns the address
    /// of the block that was decoded (for IRQ-address matching).
    fn decode_block(&mut self, ram: &[u8]) -> u32 {
        let addr = self.cur_addr & RAM_MASK;
        let b0 = ram[addr as usize];
        let b1 = ram[((addr + 1) & RAM_MASK) as usize];
        let mut shift = i32::from(b0 & 0x0F);
        if shift > 12 {
            shift = 9;
        }
        let mut filter = usize::from((b0 >> 4) & 0x07);
        if filter > 4 {
            filter = 4;
        }
        let loop_end = b1 & 0x01 != 0;
        let loop_repeat = b1 & 0x02 != 0;
        let loop_start = b1 & 0x04 != 0;

        for i in 0..14usize {
            let byte = ram[((addr + 2 + i as u32) & RAM_MASK) as usize];
            for half in 0..2usize {
                let nib = if half == 0 {
                    byte & 0x0F
                } else {
                    (byte >> 4) & 0x0F
                };
                // Sign-extend the 4-bit nibble into the top of a 16-bit word.
                let raw = i32::from((u16::from(nib) << 12) as i16);
                let mut s = raw >> shift;
                s += (self.hist0 * FILTER_POS[filter] + self.hist1 * FILTER_NEG[filter]) / 64;
                s = s.clamp(-32768, 32767);
                self.decoded[i * 2 + half] = s as i16;
                self.hist1 = self.hist0;
                self.hist0 = s;
            }
        }

        if loop_start {
            self.repeat_addr = addr;
        }
        if loop_end {
            self.ended = true;
            self.cur_addr = self.repeat_addr & RAM_MASK;
            if !loop_repeat {
                // End without repeat: real hardware forces the envelope to
                // release and mutes the voice immediately.
                self.phase = AdsrPhase::Release;
                self.level = 0;
            }
        } else {
            self.cur_addr = (addr + 16) & RAM_MASK;
        }
        addr
    }

    /// Returns the next decoded ADPCM sample, decoding a new block when the
    /// current one is exhausted. The second element is the decoded block
    /// address when a decode occurred (for IRQ matching), else `None`.
    fn next_adpcm(&mut self, ram: &[u8]) -> (i16, Option<u32>) {
        let mut decoded_addr = None;
        if self.block_pos >= 28 {
            decoded_addr = Some(self.decode_block(ram));
            self.block_pos = 0;
        }
        let s = self.decoded[self.block_pos as usize];
        self.block_pos += 1;
        (s, decoded_addr)
    }

    /// Advances the ADSR envelope by one sample tick.
    fn tick_adsr(&mut self, adsr_lo: u16, adsr_hi: u16) {
        if self.phase == AdsrPhase::Off {
            return;
        }
        if self.adsr_cycles > 1 {
            self.adsr_cycles -= 1;
            return;
        }

        // Decode the active phase's mode/direction/shift/step and its target.
        let (mode_exp, dir_dec, shift, step_val, target) = match self.phase {
            AdsrPhase::Attack => {
                let mode_exp = adsr_lo & 0x8000 != 0;
                let shift = i32::from((adsr_lo >> 10) & 0x1F);
                let step = 7 - i32::from((adsr_lo >> 8) & 0x3); // +7..+4
                (mode_exp, false, shift, step, 0x7FFF)
            }
            AdsrPhase::Decay => {
                let shift = i32::from((adsr_lo >> 4) & 0x0F);
                let sl = i32::from(adsr_lo & 0x0F);
                let target = ((sl + 1) * 0x800).min(0x7FFF);
                (true, true, shift, -8, target)
            }
            AdsrPhase::Sustain => {
                let mode_exp = adsr_hi & 0x8000 != 0;
                let dir_dec = adsr_hi & 0x4000 != 0;
                let shift = i32::from((adsr_hi >> 8) & 0x1F);
                let ss = i32::from((adsr_hi >> 6) & 0x3);
                let step = if dir_dec { -(8 - ss) } else { 7 - ss };
                (mode_exp, dir_dec, shift, step, 0)
            }
            AdsrPhase::Release => {
                let mode_exp = adsr_hi & 0x0020 != 0;
                let shift = i32::from(adsr_hi & 0x1F);
                (mode_exp, true, shift, -8, 0)
            }
            AdsrPhase::Off => unreachable!(),
        };

        let mut cycles = 1i32 << (shift - 11).max(0);
        let mut delta = step_val << (11 - shift).max(0);
        if mode_exp {
            if dir_dec {
                delta = (delta * self.level) >> 15;
            } else if self.level > 0x6000 {
                cycles *= 4;
            }
        }
        self.level = (self.level + delta).clamp(0, 0x7FFF);
        self.adsr_cycles = cycles.max(1);

        match self.phase {
            AdsrPhase::Attack if self.level >= 0x7FFF => {
                self.level = 0x7FFF;
                self.phase = AdsrPhase::Decay;
                self.adsr_cycles = 1;
            }
            AdsrPhase::Decay if self.level <= target => {
                self.phase = AdsrPhase::Sustain;
                self.adsr_cycles = 1;
            }
            AdsrPhase::Release if self.level <= 0 => {
                self.level = 0;
                self.phase = AdsrPhase::Off;
                self.on = false;
            }
            _ => {}
        }
    }

    /// Advances the voice by one output sample and returns the post-envelope
    /// (pre-pan) voice output, plus the address of any block decoded this
    /// sample (for IRQ matching).
    fn advance(&mut self, ram: &[u8], p: &VoiceStep) -> (i16, Option<u32>) {
        if !self.on {
            return (0, None);
        }

        // Compute the phase step (with optional pitch modulation).
        let mut step = u32::from(p.pitch);
        if p.pmon {
            let factor = i32::from(p.prev_out) + 0x8000;
            let modulated = (i32::from(p.pitch as i16) * factor) >> 15;
            step = modulated.clamp(0, 0x3FFF) as u32;
        } else if step > 0x4000 {
            step = 0x4000;
        }

        self.counter += step;
        let mut decoded_addr = None;
        while self.counter >= 0x1000 {
            self.counter -= 0x1000;
            self.s0 = self.s1;
            let (ns, da) = self.next_adpcm(ram);
            self.s1 = ns;
            if da.is_some() {
                decoded_addr = da;
            }
        }

        let frac = (self.counter & 0x0FFF) as i32;
        let interp =
            i32::from(self.s0) + (((i32::from(self.s1) - i32::from(self.s0)) * frac) >> 12);
        let raw = if p.noise_on {
            i32::from(p.noise_level)
        } else {
            interp
        };

        self.tick_adsr(p.adsr_lo, p.adsr_hi);
        let out = ((raw * self.level) >> 15) as i16;
        (out, decoded_addr)
    }
}

/// The per-sample inputs a voice needs to advance, bundled to keep the call
/// site (and the borrow of the register file) tidy.
struct VoiceStep {
    /// ADPCM sample-rate / pitch (register +0x4).
    pitch: u16,
    /// Whether pitch modulation from the previous voice is enabled.
    pmon: bool,
    /// The previous voice's output (for pitch modulation).
    prev_out: i16,
    /// ADSR configuration low word (register +0x8).
    adsr_lo: u16,
    /// ADSR configuration high word (register +0xA).
    adsr_hi: u16,
    /// Whether this voice outputs the noise generator instead of ADPCM.
    noise_on: bool,
    /// Current noise generator level.
    noise_level: i16,
}

/// The Sound Processing Unit.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Spu {
    /// Register-file readback store (last value written to each register).
    #[serde(with = "boxed_bytes")]
    regs: Box<[u8; SPU_REG_BYTES]>,
    /// 512 KB sample RAM.
    #[serde(with = "boxed_bytes")]
    ram: Box<[u8; SPU_RAM_BYTES]>,
    /// Live per-voice playback state.
    voices: [Voice; VOICES],
    /// SPU control register (SPUCNT, 0x1F80_1DAA).
    spucnt: u16,
    /// Current SPU RAM byte address for CPU/DMA transfers.
    transfer_addr: u32,
    /// SPU IRQ byte address.
    irq_addr: u32,
    /// Whether the IRQ-on-address flag is currently latched (SPUSTAT bit 6).
    irq_flag: bool,
    /// Edge latch: an IRQ needs to be raised on the next tick.
    irq_pending_raise: bool,
    /// CPU-cycle accumulator toward the next output sample.
    sample_timer: u32,
    /// Noise generator LFSR level.
    noise_level: i16,
    /// Noise generator timer.
    noise_timer: i32,
    /// CD-audio left input sample (fed by [`Spu::push_cd_audio`]; silent).
    cd_sample_l: i16,
    /// CD-audio right input sample (fed by [`Spu::push_cd_audio`]; silent).
    cd_sample_r: i16,
    /// Queued interleaved-stereo output samples (L, R, L, R, ...).
    samples: VecDeque<i16>,
}

impl Default for Spu {
    fn default() -> Self {
        Self::new()
    }
}

impl Spu {
    /// Creates a fresh SPU with power-on defaults.
    #[must_use]
    pub fn new() -> Self {
        Self {
            regs: Box::new([0; SPU_REG_BYTES]),
            ram: Box::new([0; SPU_RAM_BYTES]),
            voices: std::array::from_fn(|_| Voice::default()),
            spucnt: 0,
            transfer_addr: 0,
            irq_addr: 0,
            irq_flag: false,
            irq_pending_raise: false,
            sample_timer: 0,
            noise_level: 1,
            noise_timer: 0,
            cd_sample_l: 0,
            cd_sample_r: 0,
            samples: VecDeque::new(),
        }
    }

    /// Returns `true` if `phys` falls in the SPU register window.
    #[must_use]
    pub fn contains(phys: u32) -> bool {
        matches!(phys, SPU_BASE..=SPU_END)
    }

    /// Feeds a CD-audio input sample pair into the mixer. Currently unused by
    /// the core (CD-DA / XA decoding is not implemented), but the datapath is
    /// present: the samples are scaled by the CD volume and added to the mix.
    pub fn push_cd_audio(&mut self, left: i16, right: i16) {
        self.cd_sample_l = left;
        self.cd_sample_r = right;
    }

    /// Drains all queued interleaved-stereo output samples.
    pub fn drain_samples(&mut self) -> Vec<i16> {
        self.samples.drain(..).collect()
    }

    // ---- register readback helpers ---------------------------------------

    /// Reads a raw 16-bit register from the readback store.
    fn reg16(&self, phys: u32) -> u16 {
        self.read_reg16_off((phys - SPU_BASE) as usize)
    }

    /// Reads a raw 16-bit register at a window offset.
    fn read_reg16_off(&self, off: usize) -> u16 {
        u16::from_le_bytes([self.regs[off], self.regs[off + 1]])
    }

    /// Writes a raw 16-bit register at a window offset.
    fn write_reg16_off(&mut self, off: usize, val: u16) {
        let b = val.to_le_bytes();
        self.regs[off] = b[0];
        self.regs[off + 1] = b[1];
    }

    /// Computes the `ENDX` end-flag bitfield from the voices.
    #[must_use]
    fn endx(&self) -> u32 {
        let mut e = 0u32;
        for (v, voice) in self.voices.iter().enumerate() {
            if voice.ended {
                e |= 1u32 << v;
            }
        }
        e
    }

    /// Synthesizes SPUSTAT (0x1F80_1DAE).
    #[must_use]
    fn spustat(&self) -> u16 {
        // Low six bits mirror SPUCNT bits 5-0; bit 6 is the IRQ flag. Transfer
        // completes synchronously here, so the busy bit (10) is never held.
        let mut s = self.spucnt & 0x003F;
        if self.irq_flag {
            s |= 1 << 6;
        }
        s
    }

    // ---- sized reads (compose from 16-bit) -------------------------------

    /// Reads an 8-bit value.
    #[must_use]
    pub fn read8(&self, phys: u32) -> u8 {
        let base = phys & !1;
        let v = self.read16(base);
        if phys & 1 == 0 {
            v as u8
        } else {
            (v >> 8) as u8
        }
    }

    /// Reads a 16-bit value.
    #[must_use]
    pub fn read16(&self, phys: u32) -> u16 {
        let off = (phys - SPU_BASE) as usize;
        match phys {
            0x1F80_1DAE => self.spustat(),
            0x1F80_1D9C => (self.endx() & 0xFFFF) as u16,
            0x1F80_1D9E => (self.endx() >> 16) as u16,
            // Voice current ADSR volume (+0xC): reflect the live envelope.
            _ if off < 0x180 && off % 16 == 0x0C => self.voices[off / 16].level as u16,
            // Voice repeat address (+0xE): reflect the live loop address.
            _ if off < 0x180 && off % 16 == 0x0E => (self.voices[off / 16].repeat_addr >> 3) as u16,
            // Per-voice current volume readback (0x1E00..0x1E60).
            _ if (0x200..0x260).contains(&off) => {
                let idx = off - 0x200;
                let v = idx / 4;
                if idx.is_multiple_of(4) {
                    self.voices[v].cur_vol_l as u16
                } else {
                    self.voices[v].cur_vol_r as u16
                }
            }
            _ => self.read_reg16_off(off),
        }
    }

    /// Reads a 32-bit value.
    #[must_use]
    pub fn read32(&self, phys: u32) -> u32 {
        u32::from(self.read16(phys)) | (u32::from(self.read16(phys.wrapping_add(2))) << 16)
    }

    // ---- sized writes ----------------------------------------------------

    /// Writes an 8-bit value (read-modify-write of the containing halfword).
    pub fn write8(&mut self, phys: u32, val: u8) {
        let base = phys & !1;
        let cur = self.read16(base);
        let nv = if phys & 1 == 0 {
            (cur & 0xFF00) | u16::from(val)
        } else {
            (cur & 0x00FF) | (u16::from(val) << 8)
        };
        self.write16(base, nv);
    }

    /// Writes a 16-bit value, applying register side effects.
    pub fn write16(&mut self, phys: u32, val: u16) {
        let off = (phys - SPU_BASE) as usize;
        match phys {
            0x1F80_1DA8 => {
                // Transfer FIFO: push a halfword into SPU RAM.
                self.transfer_fifo_write(val);
                return;
            }
            0x1F80_1DA6 => {
                self.transfer_addr = (u32::from(val) << 3) & RAM_MASK;
                self.write_reg16_off(off, val);
                return;
            }
            0x1F80_1DA4 => {
                self.irq_addr = (u32::from(val) << 3) & RAM_MASK;
                self.write_reg16_off(off, val);
                return;
            }
            0x1F80_1DAA => {
                self.spucnt = val;
                // Acknowledging the SPU IRQ: clearing the IRQ-enable bit clears
                // the latched IRQ flag (per PSX-SPX).
                if val & (1 << 6) == 0 {
                    self.irq_flag = false;
                }
                self.write_reg16_off(off, val);
                return;
            }
            0x1F80_1D88 => {
                self.write_reg16_off(off, val);
                self.key_on_mask(u32::from(val));
                return;
            }
            0x1F80_1D8A => {
                self.write_reg16_off(off, val);
                self.key_on_mask(u32::from(val) << 16);
                return;
            }
            0x1F80_1D8C => {
                self.write_reg16_off(off, val);
                self.key_off_mask(u32::from(val));
                return;
            }
            0x1F80_1D8E => {
                self.write_reg16_off(off, val);
                self.key_off_mask(u32::from(val) << 16);
                return;
            }
            0x1F80_1D9C => {
                // ENDX lo: writing clears the low-16 voice end flags.
                for voice in &mut self.voices[0..16] {
                    voice.ended = false;
                }
                return;
            }
            0x1F80_1D9E => {
                // ENDX hi: writing clears the high voice end flags.
                for voice in &mut self.voices[16..VOICES] {
                    voice.ended = false;
                }
                return;
            }
            0x1F80_1DAE => return, // SPUSTAT is read-only.
            _ => {}
        }

        // Voice current ADSR volume (+0xC): writing sets the envelope level.
        if off < 0x180 && off % 16 == 0x0C {
            self.voices[off / 16].level = i32::from(val as i16).clamp(0, 0x7FFF);
        }
        // Voice repeat address (+0xE): writing sets the loop address.
        if off < 0x180 && off % 16 == 0x0E {
            self.voices[off / 16].repeat_addr = (u32::from(val) << 3) & RAM_MASK;
        }
        // Per-voice current volume (0x1E00..0x1E60) readback store.
        if (0x200..0x260).contains(&off) {
            let idx = off - 0x200;
            let v = idx / 4;
            if idx.is_multiple_of(4) {
                self.voices[v].cur_vol_l = val as i16;
            } else {
                self.voices[v].cur_vol_r = val as i16;
            }
        }

        self.write_reg16_off(off, val);
    }

    /// Writes a 32-bit value (as two halfword writes).
    pub fn write32(&mut self, phys: u32, val: u32) {
        self.write16(phys, val as u16);
        self.write16(phys.wrapping_add(2), (val >> 16) as u16);
    }

    // ---- key on / key off ------------------------------------------------

    /// Keys on every voice whose bit is set in `mask`.
    fn key_on_mask(&mut self, mask: u32) {
        for v in 0..VOICES {
            if mask & (1u32 << v) != 0 {
                let start = self.read_reg16_off(v * 16 + 6);
                self.voices[v].key_on(&self.ram[..], start);
            }
        }
    }

    /// Keys off every voice whose bit is set in `mask`.
    fn key_off_mask(&mut self, mask: u32) {
        for v in 0..VOICES {
            if mask & (1u32 << v) != 0 {
                self.voices[v].key_off();
            }
        }
    }

    // ---- transfers -------------------------------------------------------

    /// Pushes a 16-bit halfword through the transfer FIFO into SPU RAM.
    fn transfer_fifo_write(&mut self, val: u16) {
        let b = val.to_le_bytes();
        let i0 = (self.transfer_addr & RAM_MASK) as usize;
        let i1 = ((self.transfer_addr + 1) & RAM_MASK) as usize;
        self.ram[i0] = b[0];
        self.ram[i1] = b[1];
        self.note_transfer_irq(self.transfer_addr);
        self.transfer_addr = (self.transfer_addr + 2) & RAM_MASK;
    }

    /// Moves one 32-bit word from main RAM into SPU RAM (DMA channel 4,
    /// RAM→SPU direction), advancing the transfer address.
    pub fn dma_write_word(&mut self, val: u32) {
        for b in val.to_le_bytes() {
            let i = (self.transfer_addr & RAM_MASK) as usize;
            self.ram[i] = b;
            self.note_transfer_irq(self.transfer_addr);
            self.transfer_addr = (self.transfer_addr + 1) & RAM_MASK;
        }
    }

    /// Reads one 32-bit word from SPU RAM (DMA channel 4, SPU→RAM direction),
    /// advancing the transfer address.
    pub fn dma_read_word(&mut self) -> u32 {
        let mut bytes = [0u8; 4];
        for byte in &mut bytes {
            let i = (self.transfer_addr & RAM_MASK) as usize;
            *byte = self.ram[i];
            self.note_transfer_irq(self.transfer_addr);
            self.transfer_addr = (self.transfer_addr + 1) & RAM_MASK;
        }
        u32::from_le_bytes(bytes)
    }

    /// Latches an SPU IRQ if `addr` matches the IRQ address and IRQ delivery is
    /// enabled (SPUCNT bit 6).
    fn note_transfer_irq(&mut self, addr: u32) {
        if self.spucnt & (1 << 6) != 0 && (addr & RAM_MASK) == (self.irq_addr & RAM_MASK) {
            self.irq_flag = true;
            self.irq_pending_raise = true;
        }
    }

    /// Latches an SPU IRQ if the decoded block at `block_addr` covers the IRQ
    /// address and IRQ delivery is enabled.
    fn note_decode_irq(&mut self, block_addr: u32) {
        if self.spucnt & (1 << 6) == 0 {
            return;
        }
        let ia = self.irq_addr & RAM_MASK;
        let ba = block_addr & RAM_MASK;
        if ia >= ba && ia < ba + 16 {
            self.irq_flag = true;
            self.irq_pending_raise = true;
        }
    }

    // ---- per-cycle tick + sample generation ------------------------------

    /// Advances the SPU by `cycles` CPU cycles, emitting one interleaved stereo
    /// sample every [`CYCLES_PER_SAMPLE`] cycles and raising [`IrqLine::Spu`]
    /// when an enabled IRQ latches.
    pub fn tick(&mut self, cycles: u32, irq: &mut Irq) {
        for _ in 0..cycles {
            self.sample_timer += 1;
            if self.sample_timer >= CYCLES_PER_SAMPLE {
                self.sample_timer -= CYCLES_PER_SAMPLE;
                self.generate_sample();
            }
            if self.irq_pending_raise {
                irq.set(IrqLine::Spu);
                self.irq_pending_raise = false;
            }
        }
    }

    /// Returns `true` if voice `v`'s pitch-modulation (PMON) bit is set.
    fn pmon_bit(&self, v: usize) -> bool {
        if v == 0 {
            return false; // voice 0 has no previous voice to modulate from
        }
        let m = u32::from(self.reg16(0x1F80_1D90)) | (u32::from(self.reg16(0x1F80_1D92)) << 16);
        m & (1u32 << v) != 0
    }

    /// Returns `true` if voice `v`'s noise (NON) bit is set.
    fn non_bit(&self, v: usize) -> bool {
        let m = u32::from(self.reg16(0x1F80_1D94)) | (u32::from(self.reg16(0x1F80_1D96)) << 16);
        m & (1u32 << v) != 0
    }

    /// Clocks the noise LFSR from the SPUCNT noise-frequency field. The exact
    /// hardware timing is approximated (documented): the step/shift map onto a
    /// down-counter that clocks a Galois-style LFSR.
    fn step_noise(&mut self) {
        let step = 4 + i32::from((self.spucnt >> 8) & 0x3);
        let shift = i32::from((self.spucnt >> 10) & 0x0F);
        self.noise_timer -= step;
        if self.noise_timer <= 0 {
            self.noise_timer += 0x2_0000 >> shift;
            let l = u32::from(self.noise_level as u16);
            let parity = ((l >> 15) ^ (l >> 12) ^ (l >> 11) ^ (l >> 10) ^ 1) & 1;
            let next = ((self.noise_level as u16) << 1) | parity as u16;
            self.noise_level = next as i16;
        }
    }

    /// Generates one interleaved stereo output sample and queues it.
    fn generate_sample(&mut self) {
        self.step_noise();
        let enabled = self.spucnt & 0x8000 != 0;

        let mut left = 0i32;
        let mut right = 0i32;
        let mut prev_out = 0i16;

        for v in 0..VOICES {
            let base = v * 16;
            let pitch = self.read_reg16_off(base + 4);
            let adsr_lo = self.read_reg16_off(base + 8);
            let adsr_hi = self.read_reg16_off(base + 0x0A);
            let vol_l = i32::from(fixed_vol(self.read_reg16_off(base)));
            let vol_r = i32::from(fixed_vol(self.read_reg16_off(base + 2)));
            let params = VoiceStep {
                pitch,
                pmon: self.pmon_bit(v),
                prev_out,
                adsr_lo,
                adsr_hi,
                noise_on: self.non_bit(v),
                noise_level: self.noise_level,
            };

            let (out, decoded) = self.voices[v].advance(&self.ram[..], &params);
            if let Some(addr) = decoded {
                self.note_decode_irq(addr);
            }

            let l = (i32::from(out) * vol_l) >> 15;
            let r = (i32::from(out) * vol_r) >> 15;
            left += l;
            right += r;
            self.voices[v].cur_vol_l = l as i16;
            self.voices[v].cur_vol_r = r as i16;
            prev_out = out;
        }

        let (out_l, out_r) = if enabled {
            let main_l = i32::from(fixed_vol(self.reg16(0x1F80_1D80)));
            let main_r = i32::from(fixed_vol(self.reg16(0x1F80_1D82)));
            let cd_l = i32::from(fixed_vol(self.reg16(0x1F80_1DB0)));
            let cd_r = i32::from(fixed_vol(self.reg16(0x1F80_1DB2)));
            let mut l = (left * main_l) >> 15;
            let mut r = (right * main_r) >> 15;
            // CD-audio input hook (silent this pass).
            l += (i32::from(self.cd_sample_l) * cd_l) >> 15;
            r += (i32::from(self.cd_sample_r) * cd_r) >> 15;
            (l.clamp(-32768, 32767) as i16, r.clamp(-32768, 32767) as i16)
        } else {
            (0, 0)
        };

        self.push_sample(out_l, out_r);
    }

    /// Queues an interleaved stereo sample, dropping the oldest pair when the
    /// queue exceeds [`MAX_QUEUED`].
    fn push_sample(&mut self, left: i16, right: i16) {
        self.samples.push_back(left);
        self.samples.push_back(right);
        while self.samples.len() > MAX_QUEUED {
            self.samples.pop_front();
            self.samples.pop_front();
        }
    }
}

/// Converts a raw PSX volume register into a signed 16-bit multiplier.
///
/// Fixed-volume mode (bit 15 clear) is exact: bits 0-14 are a signed 15-bit
/// value that hardware doubles. Sweep mode (bit 15 set) is simplified this pass
/// to a near-full-scale constant (envelope sweeps are not modelled).
fn fixed_vol(reg: u16) -> i16 {
    if reg & 0x8000 != 0 {
        0x3FFF // simplified sweep: hold near full scale
    } else {
        ((reg & 0x7FFF) << 1) as i16
    }
}

/// Serde helper for boxed fixed-size byte arrays (register file + sample RAM).
mod boxed_bytes {
    use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error};

    /// Serializes a boxed byte array as a plain byte slice.
    pub fn serialize<const N: usize, S: Serializer>(v: &[u8; N], s: S) -> Result<S::Ok, S::Error> {
        v.as_slice().serialize(s)
    }

    /// Deserializes a boxed byte array, rejecting a wrong-length payload.
    pub fn deserialize<'de, const N: usize, D: Deserializer<'de>>(
        d: D,
    ) -> Result<Box<[u8; N]>, D::Error> {
        let v: Vec<u8> = Vec::deserialize(d)?;
        if v.len() != N {
            return Err(D::Error::custom("byte array has wrong length"));
        }
        let mut boxed = Box::new([0u8; N]);
        boxed.copy_from_slice(&v);
        Ok(boxed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Register addresses used by the tests.
    const SPUCNT: u32 = 0x1F80_1DAA;
    const SPUSTAT: u32 = 0x1F80_1DAE;
    const TRANSFER_ADDR: u32 = 0x1F80_1DA6;
    const TRANSFER_FIFO: u32 = 0x1F80_1DA8;
    const IRQ_ADDR: u32 = 0x1F80_1DA4;
    const KON_LO: u32 = 0x1F80_1D88;
    const MAIN_VOL_L: u32 = 0x1F80_1D80;
    const MAIN_VOL_R: u32 = 0x1F80_1D82;

    fn voice_reg(v: u32, off: u32) -> u32 {
        SPU_BASE + v * 16 + off
    }

    #[test]
    fn adpcm_block_decodes_known_samples() {
        // shift=0, filter=0 (no history term): each nibble sign-extends into
        // the top of a 16-bit word, so the decoded value is nibble<<12.
        let mut ram = vec![0u8; SPU_RAM_BYTES];
        ram[0] = 0x00; // shift 0, filter 0
        ram[1] = 0x00; // no loop flags
        ram[2] = 0x21; // low nibble 1 -> 0x1000, high nibble 2 -> 0x2000
        ram[3] = 0xF0; // low nibble 0 -> 0, high nibble F -> -0x1000

        let mut voice = Voice::default();
        let addr = voice.decode_block(&ram);
        assert_eq!(addr, 0);
        assert_eq!(voice.decoded[0], 0x1000);
        assert_eq!(voice.decoded[1], 0x2000);
        assert_eq!(voice.decoded[2], 0x0000);
        assert_eq!(voice.decoded[3], -0x1000);
        // No loop flags: cur_addr advances to the next block.
        assert_eq!(voice.cur_addr, 16);
    }

    #[test]
    fn adsr_rises_then_releases_to_zero() {
        let ram = vec![0u8; SPU_RAM_BYTES];
        let mut voice = Voice::default();
        // Fast linear attack, sustain level 4, moderate decay.
        let adsr_lo: u16 = 0x0040 | 0x4; // decay shift 4, sustain level 4
        let adsr_hi: u16 = 0x0000; // linear sustain up, fast release
        voice.key_on(&ram, 0);
        assert_eq!(voice.phase, AdsrPhase::Attack);
        assert_eq!(voice.level, 0);

        voice.tick_adsr(adsr_lo, adsr_hi);
        assert!(voice.level > 0, "level should rise during attack");

        // Run to the sustain phase.
        let mut reached_sustain = false;
        for _ in 0..10_000 {
            voice.tick_adsr(adsr_lo, adsr_hi);
            if voice.phase == AdsrPhase::Sustain {
                reached_sustain = true;
                break;
            }
        }
        assert!(reached_sustain, "should reach sustain");
        assert!(voice.level > 0);

        // Key off and run to silence.
        voice.key_off();
        assert_eq!(voice.phase, AdsrPhase::Release);
        let mut reached_off = false;
        for _ in 0..10_000 {
            voice.tick_adsr(adsr_lo, adsr_hi);
            if voice.phase == AdsrPhase::Off {
                reached_off = true;
                break;
            }
        }
        assert!(reached_off, "release should reach Off");
        assert_eq!(voice.level, 0);
    }

    #[test]
    fn noise_lfsr_produces_varying_output() {
        let mut spu = Spu::new();
        // Enable the SPU with a fast noise clock (large shift).
        spu.spucnt = 0x8000 | (0x0F << 10) | (0x3 << 8);
        let mut seen = std::collections::HashSet::new();
        let mut any_nonzero = false;
        for _ in 0..4_000 {
            spu.step_noise();
            seen.insert(spu.noise_level);
            if spu.noise_level != 0 {
                any_nonzero = true;
            }
        }
        assert!(any_nonzero, "noise should produce nonzero output");
        assert!(seen.len() > 4, "noise output should vary");
    }

    #[test]
    fn register_readback_and_spustat_mirror() {
        let mut spu = Spu::new();
        // Voice 0 volume registers read back.
        spu.write16(voice_reg(0, 0), 0x1234);
        spu.write16(voice_reg(0, 2), 0x5678);
        assert_eq!(spu.read16(voice_reg(0, 0)), 0x1234);
        assert_eq!(spu.read16(voice_reg(0, 2)), 0x5678);
        // 32-bit compose.
        assert_eq!(spu.read32(voice_reg(0, 0)), 0x5678_1234);

        // SPUSTAT mirrors the low six bits of SPUCNT.
        spu.write16(SPUCNT, 0x8035);
        assert_eq!(spu.read16(SPUSTAT) & 0x3F, 0x35);
        assert_eq!(spu.read16(SPUSTAT) & (1 << 6), 0, "no IRQ latched");
    }

    #[test]
    fn transfer_fifo_writes_ram_and_wraps() {
        let mut spu = Spu::new();
        spu.write16(TRANSFER_ADDR, 0x10); // byte addr 0x80
        spu.write16(TRANSFER_FIFO, 0xBEEF);
        spu.write16(TRANSFER_FIFO, 0xDEAD);
        assert_eq!(spu.ram[0x80], 0xEF);
        assert_eq!(spu.ram[0x81], 0xBE);
        assert_eq!(spu.ram[0x82], 0xAD);
        assert_eq!(spu.ram[0x83], 0xDE);

        // Wrap at 512KB. The transfer address is in 8-byte units, so point at
        // the last 8-byte granule (0x7FFF8) and write four halfwords to fill it,
        // then a fifth that wraps back to offset 0.
        let last_granule = (SPU_RAM_BYTES as u32 - 8) >> 3;
        spu.write16(TRANSFER_ADDR, last_granule as u16);
        spu.write16(TRANSFER_FIFO, 0xAAAA); // 0x7FFF8
        spu.write16(TRANSFER_FIFO, 0xBBBB); // 0x7FFFA
        spu.write16(TRANSFER_FIFO, 0xCCCC); // 0x7FFFC
        spu.write16(TRANSFER_FIFO, 0x1122); // 0x7FFFE (last two bytes)
        spu.write16(TRANSFER_FIFO, 0x3344); // wraps to offset 0
        assert_eq!(spu.ram[SPU_RAM_BYTES - 2], 0x22);
        assert_eq!(spu.ram[SPU_RAM_BYTES - 1], 0x11);
        assert_eq!(spu.ram[0], 0x44);
        assert_eq!(spu.ram[1], 0x33);
    }

    #[test]
    fn spu_irq_on_transfer_address() {
        let mut spu = Spu::new();
        let mut irq = Irq::new();
        spu.write16(SPUCNT, 1 << 6); // enable SPU IRQ
        spu.write16(IRQ_ADDR, 0x100); // byte addr 0x800
        spu.write16(TRANSFER_ADDR, 0x100); // transfer addr 0x800
        spu.write16(TRANSFER_FIFO, 0x1234); // write at 0x800 -> match
        spu.tick(1, &mut irq);
        assert_ne!(
            irq.read_stat() & (1 << IrqLine::Spu.bit()),
            0,
            "SPU IRQ should be raised"
        );
    }

    #[test]
    fn voice_mixing_respects_pan() {
        let mut spu = Spu::new();
        // Two voices with fixed non-zero output and mirrored panning.
        for v in 0..2usize {
            spu.voices[v].on = true;
            spu.voices[v].phase = AdsrPhase::Sustain;
            spu.voices[v].level = 0x7FFF;
            spu.voices[v].s0 = 10_000;
            spu.voices[v].s1 = 10_000;
        }
        // Voice 0 hard left, voice 1 hard right.
        spu.write16(voice_reg(0, 0), 0x3FFF); // vol L
        spu.write16(voice_reg(0, 2), 0x0000); // vol R
        spu.write16(voice_reg(1, 0), 0x0000); // vol L
        spu.write16(voice_reg(1, 2), 0x3FFF); // vol R
        // Sustain config that holds the level (linear up, clamped at max).
        spu.write16(voice_reg(0, 0x0A), 0x0000);
        spu.write16(voice_reg(1, 0x0A), 0x0000);
        spu.write16(SPUCNT, 0x8000); // enable
        spu.write16(MAIN_VOL_L, 0x3FFF);
        spu.write16(MAIN_VOL_R, 0x3FFF);

        spu.generate_sample();
        let out = spu.drain_samples();
        assert_eq!(out.len(), 2, "one interleaved stereo pair");
        let (l, r) = (out[0], out[1]);
        assert!(l > 1_000, "left channel driven by voice 0: {l}");
        assert!(r > 1_000, "right channel driven by voice 1: {r}");
        // Voice 0 current-volume readback: left non-zero, right zero.
        assert!(spu.voices[0].cur_vol_l > 0);
        assert_eq!(spu.voices[0].cur_vol_r, 0);
    }

    #[test]
    fn keyed_voice_generates_nonzero_audio() {
        let mut spu = Spu::new();
        let mut irq = Irq::new();
        // Stage an ADPCM block at SPU RAM offset 0 with a ramp.
        spu.write16(TRANSFER_ADDR, 0);
        // header: shift 0, filter 0; flags: LoopStart+LoopRepeat so it loops.
        spu.write16(TRANSFER_FIFO, 0x0600); // b0=0x00, b1=0x06
        for _ in 0..7 {
            spu.write16(TRANSFER_FIFO, 0x1111);
        }
        // Program voice 0: start addr 0, full pitch, full volume, fast attack.
        spu.write16(voice_reg(0, 4), 0x1000); // pitch = 44.1kHz
        spu.write16(voice_reg(0, 6), 0); // start addr
        spu.write16(voice_reg(0, 8), 0x00FF); // adsr lo: fast attack, sustain hi
        spu.write16(voice_reg(0, 0x0A), 0x0000); // adsr hi
        spu.write16(voice_reg(0, 0), 0x3FFF); // vol L
        spu.write16(voice_reg(0, 2), 0x3FFF); // vol R
        spu.write16(SPUCNT, 0x8000); // enable
        spu.write16(MAIN_VOL_L, 0x3FFF);
        spu.write16(MAIN_VOL_R, 0x3FFF);
        spu.write16(KON_LO, 0x0001); // key on voice 0

        // Generate a batch of samples.
        spu.tick(CYCLES_PER_SAMPLE * 200, &mut irq);
        let out = spu.drain_samples();
        assert!(!out.is_empty(), "should produce samples");
        assert!(
            out.iter().any(|&s| s != 0),
            "keyed voice should produce nonzero audio"
        );
    }

    #[test]
    fn snapshot_serde_round_trip() {
        let mut spu = Spu::new();
        spu.write16(voice_reg(3, 4), 0x0ABC);
        spu.write16(TRANSFER_ADDR, 0x20);
        spu.write16(TRANSFER_FIFO, 0xCAFE);
        spu.voices[3].on = true;
        spu.voices[3].level = 0x1234;
        spu.push_sample(100, -100);

        let json = serde_json::to_string(&spu).unwrap();
        let back: Spu = serde_json::from_str(&json).unwrap();
        assert_eq!(spu, back);
    }

    #[test]
    fn drain_clears_the_queue() {
        let mut spu = Spu::new();
        spu.push_sample(1, 2);
        spu.push_sample(3, 4);
        assert_eq!(spu.drain_samples(), vec![1, 2, 3, 4]);
        assert!(spu.drain_samples().is_empty());
    }
}
