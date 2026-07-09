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
//! when the user presses **Start** on a controller. The SIO0/joypad stub now
//! models the real digital-pad serial protocol and clocks out button input set
//! via `Command::SetControllerState` (see `tests/sio0_joypad.rs`), so a headless
//! driver *could* script the Start press the fuzzer waits on. Full headless
//! `gte-fuzz` still is not wired for two concrete reasons: (1) the `gte-fuzz`
//! binary is not vendored in `tests/fixtures/` (nor is Amidog `psxtest_gte`,
//! which is CC BY-NC-SA), so there is nothing to drive; and (2) even with the
//! binary present, its pass criterion is a golden-log capture of the
//! valid-command phase gated on `VSync`/VBlank timing rather than a TTY
//! `pass -`/`Done.` marker the harness can assert, so it needs the out-of-band
//! golden-replay path rather than an in-tree gate. Should the binary be
//! vendored later, a driver would `SetControllerState { port: 0, buttons:
//! Start }`, pump `run_hle` with periodic `raise_vblank()`, and diff the
//! captured TTY against `gte_valid_*.log`. Meanwhile the `gte/test-all` gate
//! already exercises all 1150 vectors on-device, and the out-of-band replay
//! harness cross-checks the same vectors directly against `psoxide_core::Gte`.

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
