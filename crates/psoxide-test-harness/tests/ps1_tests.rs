//! Always-on gate: drive the vendored JaCzekanski `ps1-tests` CPU binaries
//! through the harness and assert each reaches its known progress markers. The
//! harness supplies the PS-EXE sideloader, the BIOS TTY / `printf` / exception
//! HLE, the hardware timers, and an injected VBlank.
//!
//! These gates prove the real-PS-EXE execution path works end-to-end — a
//! regression guard for the sideloader, the syscall/`printf` HLE, and the timer
//! interrupt path. They intentionally do **not** assert a full byte-for-byte
//! match against the shipped `psx.log`: each suite still exercises hardware
//! psoxide has not implemented yet (the BIOS exception-dispatch chain that
//! invokes program-registered handlers, instruction/data bus-error exceptions,
//! the coprocessor-unusable exception, cycle-accurate access timing, and the
//! JOY / SIO / SPU / CD-ROM / MDEC peripherals). See `tests/README.md` and
//! `tests/fixtures/ps1-tests/README.md` for the blocker details. The `psx.log`
//! goldens are vendored so these gates can be tightened to a golden diff as that
//! hardware lands.

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
fn cop_runs_to_completion_and_reports_passes() {
    let tty = run_exe("cpu/cop/cop.exe", 1_000_000);
    assert!(tty.contains("cpu/cop"), "cop header missing:\n{tty}");
    // The coprocessor-enabled paths pass today. The core now raises the
    // Coprocessor Unusable exception (ExcCode 0x0B, CAUSE.CE) for a disabled
    // COP0/COP2 op, but the "Disabled" cases still cannot be asserted here:
    // this test registers its checker via `hookUnresolvedExceptionHandler`,
    // which needs the BIOS exception-dispatch chain to invoke it (the RAM
    // vector 0x80000080 is empty without a BIOS, so the harness HLE handles
    // the trap and never calls the program's handler). The COP1/COP3/SWCx
    // disabled cases additionally need the decoder to surface those opcodes as
    // distinct coprocessor ops (they currently decode to `Illegal`).
    assert!(
        tty.contains("pass - testCop0Enabled"),
        "expected testCop0Enabled to pass:\n{tty}"
    );
    assert!(
        tty.contains("pass - testCop2Enabled"),
        "expected testCop2Enabled to pass:\n{tty}"
    );
    assert!(
        tty.contains("Done."),
        "cop did not run to completion:\n{tty}"
    );
}

#[test]
fn code_in_io_executes_code_from_ram() {
    let tty = run_exe("cpu/code-in-io/code-in-io.exe", 1_000_000);
    assert!(tty.contains("cpu/code-in-io"), "header missing:\n{tty}");
    // Executing code out of main RAM works; the scratchpad/MDEC/IO cases need
    // instruction bus-error exceptions that psoxide does not model yet.
    assert!(
        tty.contains("pass - testCodeInRam"),
        "expected testCodeInRam to pass:\n{tty}"
    );
}

#[test]
fn io_access_bitwidth_runs_to_completion() {
    let tty = run_exe("cpu/io-access-bitwidth/io-access-bitwidth.exe", 2_000_000);
    assert!(
        tty.contains("cpu/io-access-bitwidth"),
        "header missing:\n{tty}"
    );
    // RAM / scratchpad byte/half/word write-then-read paths behave correctly;
    // the many device rows still need the missing peripherals.
    assert!(
        tty.contains("Done."),
        "io-access-bitwidth did not run to completion:\n{tty}"
    );
}

#[test]
fn access_time_runs_to_completion() {
    // access-time is a pure cycle-timing measurement with no pass/fail
    // assertions of its own; psoxide's one-cycle-per-instruction model cannot
    // reproduce the reference numbers. The gate only proves the binary loads and
    // runs its whole measurement loop to completion via the timer/HLE path.
    let tty = run_exe("cpu/access-time/access-time.exe", 2_000_000);
    assert!(tty.contains("cpu/access-time"), "header missing:\n{tty}");
    assert!(
        tty.contains("Done."),
        "access-time did not run to completion:\n{tty}"
    );
}
