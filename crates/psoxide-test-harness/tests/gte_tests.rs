//! Always-on gate: drive the vendored JaCzekanski `ps1-tests` `gte/test-all`
//! binary through the harness and assert the GTE (cop2) datapath is bit-exact.
//!
//! Unlike the CPU gates in `ps1_tests.rs`, the GTE is now fully implemented in
//! `psoxide-core`, so this gate is tightened all the way to the on-device
//! self-check: `gte/test-all` writes, executes, and reads back every one of its
//! 1150 register/opcode vectors *through the real MTC2/CTC2/GTE-command/MFC2/CFC2
//! path* and prints a `Passed tests: N / Failed tests: M` summary. We assert the
//! full pass (`Passed tests: 1150`, `Failed tests: 0`) and that it runs to its
//! `Done.` marker without the early `Breaking.` bail-out a failing vector emits.
//!
//! The companion `gte-fuzz` binary is *not* vendored here: it is an interactive
//! program whose "VALID CMD FUZZ" golden (`gte_valid_*.log`) is only produced
//! when the user presses **Start** on a controller. The headless harness has no
//! controller-input HLE (the SIO0/joypad stub reports "no controller attached"),
//! so the binary defaults to its single-command mode and blocks on `VSync`
//! before ever reaching the valid-command phase the golden captures. Driving it
//! would require a controller-injection feature orthogonal to the GTE. The
//! `gte/test-all` gate already exercises all 1150 vectors on-device, and the
//! out-of-band replay harness cross-checks the same vectors directly against
//! `psoxide_core::Gte`.

use psoxide_test_harness::Harness;
use std::path::PathBuf;

/// Reads a vendored fixture, relative to `tests/fixtures/ps1-tests/`.
fn fixture(rel: &str) -> Vec<u8> {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("tests/fixtures/ps1-tests");
    p.push(rel);
    std::fs::read(&p).unwrap_or_else(|e| panic!("read fixture {}: {e}", p.display()))
}

/// Sideloads `rel`, runs the HLE loop for `budget` instructions, and returns the
/// captured TTY output.
fn run_exe(rel: &str, budget: usize) -> String {
    let bytes = fixture(rel);
    let mut h = Harness::new();
    h.load_exe(&bytes).expect("sideload ps1-test exe");
    h.run_hle(budget);
    h.tty()
}

#[test]
fn test_all_reports_full_pass() {
    let tty = run_exe("gte/test-all/test-all.exe", 20_000_000);
    assert!(tty.contains("gte/test-all"), "header missing:\n{tty}");
    assert!(
        tty.contains("Passed tests: 1150"),
        "expected all 1150 GTE vectors to pass:\n{tty}"
    );
    assert!(
        tty.contains("Failed tests: 0"),
        "expected zero GTE vector failures:\n{tty}"
    );
    assert!(
        !tty.contains("Breaking."),
        "test-all bailed out early (a vector failed):\n{tty}"
    );
    assert!(
        tty.contains("Done."),
        "test-all did not run to completion:\n{tty}"
    );
}
