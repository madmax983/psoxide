//! End-to-end SPU integration test.
//!
//! Unlike the in-module unit tests in `psoxide-core/src/spu.rs` (which drive the
//! voice engine in isolation), this exercises the *assembled* core: the real
//! system bus routing to `0x1F80_1C00..=0x1F80_1FFF`, the per-cycle `Spu::tick`
//! wired into the step loop, and the audio queue drained through
//! [`psoxide_core::PsxCore::drain_audio`].
//!
//! A tiny ADPCM block is staged into SPU RAM through the transfer FIFO, a voice
//! is keyed on with a non-zero pitch, full volume, and a fast ADSR, and the core
//! is stepped long enough to synthesise samples. Interrupts stay masked
//! (`I_MASK` = 0) so the looping CPU program is never diverted.
//!
//! Note: the upstream JaCzekanski `ps1-tests` SPU binaries are not used here —
//! they are not cleanly headless-gateable against an approximate-timing SPU
//! (per the project's CD-ROM precedent), so the SPU is covered by these
//! synthetic fixtures plus the `spu.rs` unit tests instead.

use psoxide_test_harness::Harness;

// ---- SPU register map (physical) -----------------------------------------

const SPU_BASE: u32 = 0x1F80_1C00;
const SPUCNT: u32 = 0x1F80_1DAA;
const MAIN_VOL_L: u32 = 0x1F80_1D80;
const MAIN_VOL_R: u32 = 0x1F80_1D82;
const KON_LO: u32 = 0x1F80_1D88;
const TRANSFER_ADDR: u32 = 0x1F80_1DA6;
const TRANSFER_FIFO: u32 = 0x1F80_1DA8;

/// Voice register address (voice `v`, byte offset `off`).
fn voice_reg(v: u32, off: u32) -> u32 {
    SPU_BASE + v * 16 + off
}

/// Puts the CPU in a tight self-loop in RAM so each `StepCpu` ticks the SPU
/// without the PC leaving mapped memory.
fn spin_cpu(h: &mut Harness) {
    // 0x0000: j 0x0000 ; 0x0004: nop (delay slot) — loops forever at PC 0.
    h.load_program(&[0x0800_0000, 0x0000_0000]);
}

#[test]
fn keyed_voice_emits_nonzero_audio_through_the_bus() {
    let mut h = Harness::new();
    spin_cpu(&mut h);

    // Stage an ADPCM block at SPU RAM offset 0 via the transfer FIFO.
    // Header byte0 = 0x00 (shift 0, filter 0); byte1 = 0x06 (LoopStart +
    // LoopRepeat) so the block loops forever. The 28 nibbles are a non-zero
    // pattern, so decoded samples are non-zero.
    h.core_mut().store16(TRANSFER_ADDR, 0);
    h.core_mut().store16(TRANSFER_FIFO, 0x0600); // b0 = 0x00, b1 = 0x06
    for _ in 0..7 {
        h.core_mut().store16(TRANSFER_FIFO, 0x2413);
    }

    // Program voice 0: start addr 0, full-speed pitch, full L/R volume, and a
    // fast attack that reaches a high sustain quickly.
    h.core_mut().store16(voice_reg(0, 4), 0x1000); // pitch = 44.1 kHz
    h.core_mut().store16(voice_reg(0, 6), 0); // ADPCM start addr (x8 bytes)
    h.core_mut().store16(voice_reg(0, 8), 0x00FF); // ADSR lo: fast attack
    h.core_mut().store16(voice_reg(0, 0x0A), 0x0000); // ADSR hi
    h.core_mut().store16(voice_reg(0, 0), 0x3FFF); // volume L
    h.core_mut().store16(voice_reg(0, 2), 0x3FFF); // volume R

    // Enable the SPU and open the main volume.
    h.core_mut().store16(SPUCNT, 0x8000);
    h.core_mut().store16(MAIN_VOL_L, 0x3FFF);
    h.core_mut().store16(MAIN_VOL_R, 0x3FFF);

    // Key on voice 0.
    h.core_mut().store16(KON_LO, 0x0001);

    // Step the core: one output sample is produced every 768 CPU cycles, so a
    // few hundred thousand steps yields several hundred samples.
    h.run(400_000);

    let audio = h.core_mut().drain_audio();
    assert!(!audio.is_empty(), "SPU should have queued audio samples");
    // Interleaved stereo: an even number of i16 values.
    assert_eq!(audio.len() % 2, 0, "audio must be interleaved stereo");
    assert!(
        audio.iter().any(|&s| s != 0),
        "a keyed voice should produce at least one non-zero sample"
    );
}

#[test]
fn disabled_spu_stays_silent() {
    let mut h = Harness::new();
    spin_cpu(&mut h);

    // Same voice setup, but leave the SPU disabled (SPUCNT bit 15 clear).
    h.core_mut().store16(TRANSFER_ADDR, 0);
    h.core_mut().store16(TRANSFER_FIFO, 0x0600);
    for _ in 0..7 {
        h.core_mut().store16(TRANSFER_FIFO, 0x2413);
    }
    h.core_mut().store16(voice_reg(0, 4), 0x1000);
    h.core_mut().store16(voice_reg(0, 8), 0x00FF);
    h.core_mut().store16(voice_reg(0, 0), 0x3FFF);
    h.core_mut().store16(voice_reg(0, 2), 0x3FFF);
    h.core_mut().store16(MAIN_VOL_L, 0x3FFF);
    h.core_mut().store16(MAIN_VOL_R, 0x3FFF);
    h.core_mut().store16(KON_LO, 0x0001);
    // SPUCNT left at 0 (disabled).

    h.run(50_000);
    let audio = h.core_mut().drain_audio();
    assert!(
        !audio.is_empty(),
        "samples are still emitted while disabled"
    );
    assert!(
        audio.iter().all(|&s| s == 0),
        "a disabled SPU must output silence"
    );
}
