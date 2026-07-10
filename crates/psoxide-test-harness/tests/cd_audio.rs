//! End-to-end CD-audio (CD-DA) → SPU integration test.
//!
//! This exercises the *assembled* core wiring added for CD audio: the CD-ROM
//! controller decodes an audio-track sector into 44.1 kHz stereo frames, the
//! per-cycle bridge in `PsxCore::step_cpu` hands those frames to the SPU via
//! `Spu::push_cd_audio_samples`, and the SPU mixes them through its CD input
//! (CD volume matrix on the CD side, SPU CD input volume + main volume on the
//! SPU side) into the output stream drained by `PsxCore::drain_audio`.
//!
//! No real BIOS or game data is needed: a synthetic single-track AUDIO disc is
//! mounted through the CUE/BIN parser in [`psoxide_config::disc`], carrying a
//! constant, unmistakably non-silent PCM waveform. Playback is driven through
//! the real system bus (Setmode + Play written to `0x1F80_1800..=0x1F80_1803`,
//! exactly as a program would), while a tight self-loop keeps the CPU stepping
//! so the controller and SPU tick.
//!
//! The routing is proven both ways: with the SPU CD input volume open the drain
//! is non-silent; with it muted (CD volume 0) the identical run is pure silence,
//! so the audio can only have arrived through the CD → SPU path under test.

use std::path::PathBuf;

use psoxide_config::disc::{SECTOR_RAW, parse_cue};
use psoxide_test_harness::Harness;

// ---- CD-ROM register map (physical) --------------------------------------

const CD_STATUS: u32 = 0x1F80_1800; // write: index
const CD_1801: u32 = 0x1F80_1801; // write(idx0): command
const CD_1802: u32 = 0x1F80_1802; // write(idx0): parameter

// ---- SPU register map (physical) -----------------------------------------

const SPUCNT: u32 = 0x1F80_1DAA; // SPU control (bit15 enable, bit0 CD audio)
const MAIN_VOL_L: u32 = 0x1F80_1D80;
const MAIN_VOL_R: u32 = 0x1F80_1D82;
const CD_VOL_L: u32 = 0x1F80_1DB0; // SPU CD input volume, left
const CD_VOL_R: u32 = 0x1F80_1DB2; // SPU CD input volume, right

// Known, unmistakably non-silent PCM the audio track carries on every frame.
const PCM_L: i16 = 8000;
const PCM_R: i16 = -8000;

/// One full CD-DA sector is 451_584 cycles (588 frames × 768 cycles). Stepping
/// several sectors' worth guarantees decoded frames have been handed to the SPU
/// and clocked out into the drain.
const STEP_CYCLES: usize = 1_600_000;

/// Writes an `n`-sector single-track AUDIO BIN plus a matching CUE into `dir`,
/// returning the CUE path. Every 2352-byte sector is 588 stereo i16 LE PCM
/// frames of the constant [`PCM_L`]/[`PCM_R`] waveform.
fn write_audio_disc(dir: &std::path::Path, n_sectors: usize) -> PathBuf {
    let mut data = vec![0u8; n_sectors * SECTOR_RAW];
    let frames = n_sectors * 588;
    for f in 0..frames {
        let b = f * 4;
        data[b..b + 2].copy_from_slice(&PCM_L.to_le_bytes());
        data[b + 2..b + 4].copy_from_slice(&PCM_R.to_le_bytes());
    }
    let bin_path = dir.join("audio.bin");
    let cue_path = dir.join("audio.cue");
    std::fs::write(&bin_path, &data).expect("write bin");
    std::fs::write(
        &cue_path,
        "FILE \"audio.bin\" BINARY\r\n  TRACK 01 AUDIO\r\n    INDEX 01 00:00:00\r\n",
    )
    .expect("write cue");
    cue_path
}

/// Puts the CPU in a tight self-loop in RAM so each `StepCpu` ticks the CD-ROM
/// controller and SPU without the PC leaving mapped memory.
fn spin_cpu(h: &mut Harness) {
    // 0x0000: j 0x0000 ; 0x0004: nop (delay slot) — loops forever at PC 0.
    h.load_program(&[0x0800_0000, 0x0000_0000]);
}

fn set_index(h: &mut Harness, index: u8) {
    h.core_mut().store8(CD_STATUS, index);
}

fn write_param(h: &mut Harness, val: u8) {
    set_index(h, 0);
    h.core_mut().store8(CD_1802, val);
}

fn send_command(h: &mut Harness, cmd: u8) {
    set_index(h, 0);
    h.core_mut().store8(CD_1801, cmd);
}

/// Mounts the synthetic audio disc, programs the SPU with the given CD input
/// volume, starts CD-DA playback, steps the machine, and returns the drained
/// interleaved-stereo audio.
fn drive_cd_audio(dir: &std::path::Path, cd_input_vol: u16) -> Vec<i16> {
    let cue = write_audio_disc(dir, 5);
    let disc = parse_cue(&cue).expect("parse cue");

    let mut h = Harness::new();
    spin_cpu(&mut h);
    h.load_disc(disc);

    // Enable the SPU (bit15) with CD-audio dry mix (bit0); no reverb. Open the
    // main output and set the SPU CD input volume to the requested level.
    h.core_mut().store16(SPUCNT, 0x8000 | 0x0001);
    h.core_mut().store16(MAIN_VOL_L, 0x3FFF);
    h.core_mut().store16(MAIN_VOL_R, 0x3FFF);
    h.core_mut().store16(CD_VOL_L, cd_input_vol);
    h.core_mut().store16(CD_VOL_R, cd_input_vol);
    // Drain any samples produced while registers were being programmed so the
    // measured window is purely playback.
    let _ = h.core_mut().drain_audio();

    // Setmode with the CDDA routing bit (bit0), then Play track 1. The CD side
    // keeps its power-on unity volume matrix, so a non-zero SPU CD input volume
    // passes the PCM straight through.
    write_param(&mut h, 0x01);
    send_command(&mut h, 0x0E); // Setmode(CDDA)
    write_param(&mut h, 0x01);
    send_command(&mut h, 0x03); // Play(track 1)

    h.run(STEP_CYCLES);
    h.core_mut().drain_audio()
}

/// The high-value end-to-end path: CD-DA decoded by the CD-ROM controller is
/// mixed into the SPU CD input and reaches the audio drain non-silent.
#[test]
fn cd_da_reaches_spu_output() {
    let dir = std::env::temp_dir().join(format!("psoxide-cd-audio-on-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();

    let out = drive_cd_audio(&dir, 0x3FFF);
    assert!(!out.is_empty(), "the SPU produced output samples");
    let peak = out.iter().map(|&s| i32::from(s).abs()).max().unwrap_or(0);
    assert!(
        peak > 1000,
        "CD-DA must reach the SPU output non-silent (peak |sample| = {peak})"
    );

    std::fs::remove_dir_all(&dir).ok();
}

/// The routing control: the identical run with the SPU CD input volume at zero
/// produces pure silence, proving the audio arrives only through the CD → SPU
/// path (no voices are keyed, reverb is off).
#[test]
fn cd_input_volume_zero_is_silent() {
    let dir = std::env::temp_dir().join(format!("psoxide-cd-audio-off-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();

    let out = drive_cd_audio(&dir, 0x0000);
    assert!(!out.is_empty(), "the SPU still clocks output samples");
    assert!(
        out.iter().all(|&s| s == 0),
        "CD input at zero volume must leave the SPU output silent"
    );

    std::fs::remove_dir_all(&dir).ok();
}
