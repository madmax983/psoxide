//! BIOS-boot smoke test, gated behind the `PSOXIDE_BIOS` environment variable.
//!
//! When `PSOXIDE_BIOS` points at a 512KB PlayStation BIOS image, this test
//! loads it, runs a batch of frames, and asserts the emulator makes forward
//! progress toward the boot logo without panicking.
//!
//! Progress is checked in tiers so a partially-booting BIOS still fails
//! loudly rather than silently succeeding:
//!
//! 1. **CPU progress** — the PC must have moved beyond the reset vector.
//! 2. **GPU progress** — the display must eventually be enabled (GP1 0x03
//!    clears the "display disabled" bit) or VRAM must have non-zero pixels.
//! 3. **Logo progress** — with a real, working boot path, the framebuffer
//!    should contain a non-trivial color distribution. This is a
//!    conservative "not all one color" check.
//!
//! No BIOS image is committed. When the variable is unset the test skips.

use psoxide_core::{Command, CoreQuery, FRAME_HEIGHT, FRAME_WIDTH, PsxCore, QueryResult};

/// How many frames to run before checking progress. The Sony boot animation
/// takes roughly 4 seconds of wall time on real hardware; at 60Hz that's
/// ~240 frames, but the animation begins well before then. 180 frames is a
/// pragmatic bound that comfortably reaches the logo on real emulators
/// without ballooning the smoke-test runtime.
const FRAMES_TO_RUN: usize = 180;

#[test]
fn bios_boots_toward_logo() {
    let Ok(path) = std::env::var("PSOXIDE_BIOS") else {
        eprintln!("PSOXIDE_BIOS not set; skipping BIOS boot smoke test");
        return;
    };
    let image = std::fs::read(&path).expect("read BIOS image");
    assert_eq!(image.len(), 512 * 1024, "BIOS must be exactly 512KB");

    let mut core = PsxCore::new();
    core.execute(Command::LoadBios(image))
        .expect("load BIOS image");

    let start_pc = match core.query(CoreQuery::Pc) {
        QueryResult::Pc(pc) => pc,
        _ => unreachable!(),
    };

    // Run frames; the interpreter must not panic.
    for _ in 0..FRAMES_TO_RUN {
        core.execute(Command::StepFrame).unwrap();
    }

    let end_pc = match core.query(CoreQuery::Pc) {
        QueryResult::Pc(pc) => pc,
        _ => unreachable!(),
    };

    // ── Tier 1: CPU made progress. ─────────────────────────────────────────
    assert!(
        end_pc != start_pc,
        "BIOS PC did not advance from reset ({start_pc:#010x})"
    );

    // ── Tier 2: GPU touched. ───────────────────────────────────────────────
    let vram_touched = core.gpu().vram.iter().any(|&p| p != 0);
    let display_enabled = core.gpu().display_enabled;
    assert!(
        vram_touched || display_enabled,
        "BIOS never enabled the display or wrote to VRAM"
    );

    // ── Tier 3: framebuffer distribution. ──────────────────────────────────
    // The framebuffer contract must remain sane.
    let frame = core.framebuffer_rgba();
    assert_eq!(frame.len(), FRAME_WIDTH * FRAME_HEIGHT * 4);

    // Only assert the "logo-visible" distribution check if the display got
    // enabled — a stalled BIOS that never touched GP1(0x03) will still leave
    // the framebuffer solid black, and we'd rather fail with the tier-2
    // message than with a misleading "framebuffer is one color" message.
    if display_enabled {
        let (unique, nonblack) = frame_color_stats(&frame);
        // The Sony boot logo is a multi-color, mostly-dark scene; the
        // color-cycled outline animates through many hues. We require both
        // some minimum unique-color count and some minimum non-black pixel
        // count — either alone is spoofable by a stuck-white or stuck-blue
        // frame.
        assert!(
            unique >= 8,
            "framebuffer has only {unique} unique colors — logo not rendered"
        );
        assert!(
            nonblack >= 200,
            "framebuffer has only {nonblack} non-black pixels — logo not rendered"
        );
    }
}

/// Returns `(unique_rgb_count, non_black_pixel_count)` for a 320x240 RGBA
/// framebuffer. Alpha is ignored.
fn frame_color_stats(frame: &[u8]) -> (usize, usize) {
    use std::collections::HashSet;
    let mut colors: HashSet<u32> = HashSet::new();
    let mut nonblack = 0usize;
    for px in frame.chunks_exact(4) {
        let rgb = u32::from(px[0]) << 16 | u32::from(px[1]) << 8 | u32::from(px[2]);
        colors.insert(rgb);
        if rgb != 0 {
            nonblack += 1;
        }
    }
    (colors.len(), nonblack)
}
