//! BIOS-boot smoke test, gated behind the `PSOXIDE_BIOS` environment variable.
//!
//! When `PSOXIDE_BIOS` points at a 512KB PlayStation BIOS image, this test
//! loads it, runs several frames, and asserts the emulator makes forward
//! progress (GPUSTAT is polled / VRAM or the PC advance) without panicking.
//!
//! No BIOS image is committed. When the variable is unset the test skips.

use psoxide_core::{Command, CoreQuery, PsxCore, QueryResult};

#[test]
fn bios_boots_a_few_frames() {
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

    // Run several frames; the interpreter must not panic.
    for _ in 0..30 {
        core.execute(Command::StepFrame).unwrap();
    }

    let end_pc = match core.query(CoreQuery::Pc) {
        QueryResult::Pc(pc) => pc,
        _ => unreachable!(),
    };

    // Best-effort progress signal: the PC moved on from reset, or the GPU has
    // been written (non-empty VRAM / display touched).
    let vram_touched = core.gpu().vram.iter().any(|&p| p != 0);
    assert!(
        end_pc != start_pc || vram_touched,
        "BIOS made no observable progress"
    );

    // The framebuffer path must remain sane.
    assert_eq!(core.framebuffer_rgba().len(), 320 * 240 * 4);
}
