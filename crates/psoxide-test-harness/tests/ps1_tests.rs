//! Always-on gate: drive the vendored JaCzekanski `ps1-tests` CPU binaries
//! through the harness and assert each reaches its known progress markers. The
//! harness supplies the PS-EXE sideloader, the BIOS TTY / `printf` / exception
//! HLE, the hardware timers, and an injected VBlank.
//!
//! These gates prove the real-PS-EXE execution path works end-to-end — a
//! regression guard for the sideloader, the syscall/`printf` HLE, and the timer
//! interrupt path. They intentionally do **not** assert a full byte-for-byte
//! match against the shipped `psx.log`: each suite still exercises hardware
//! psoxide has not implemented yet (instruction/data bus-error exceptions,
//! the COP1/COP3/LWCx/SWCx decoder split, cycle-accurate access timing, and the
//! JOY / SIO / SPU / CD-ROM / MDEC peripherals). The BIOS exception-dispatch
//! chain that invokes program-registered handlers is now HLE'd by the harness
//! (see `src/lib.rs`), so `cpu/cop`'s "Disabled" cases pass. See `tests/README.md`
//! and
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

    // The coprocessor-*enabled* COP0/COP2 paths pass today.
    for case in ["testCop0Enabled", "testCop2Enabled"] {
        assert!(
            tty.contains(&format!("pass - {case}")),
            "expected {case} to pass:\n{tty}"
        );
    }

    // The **exception-dispatch chain** now invokes the program-registered
    // "unresolved exception" handler (hooked via `hookUnresolvedExceptionHandler`
    // into RAM 0x300), so the "Disabled" cases are finally observable: each
    // faulting coprocessor op raises an exception the handler records, and the
    // test sees `wasExceptionThrown()`. Before this landed the harness serviced
    // the trap itself and never called the handler, so all six of these reported
    // `given 0x0, expected 0x1` and failed. `testCop2Disabled` additionally
    // proves the real Coprocessor-Unusable exception (ExcCode 0x0B) reaches the
    // handler; the others reach it via the reserved-instruction trap the decoder
    // currently raises for COP1/COP3/SWCx (see the follow-up note below).
    for case in [
        "testSwc0Disabled",
        "testCop1Disabled",
        "testCop2Disabled",
        "testSwc2Disabled",
        "testCop3Disabled",
        "testSwc3Disabled",
    ] {
        assert!(
            tty.contains(&format!("pass - {case}")),
            "expected {case} to pass via the exception-dispatch chain:\n{tty}"
        );
    }

    assert!(
        tty.contains("Done."),
        "cop did not run to completion:\n{tty}"
    );

    // Known follow-up (a decoder change, deliberately out of scope here): the
    // COP1/COP3 and LWCx/SWCx opcodes decode to `Illegal`, so they raise a
    // reserved-instruction trap (ExcCode 0x0A) instead of Coprocessor-Unusable
    // (0x0B). That makes:
    //   * `testDisabledCoprocessorThrowsCoprocessorUnusable` still fail (it
    //     asserts the exception *type* is 0x0B; it currently sees 0x0A), and
    //   * the coprocessor-*enabled* `testSwc0/1/2/3Enabled` and
    //     `testCop1/3Enabled` cases fail (a usable-coprocessor op should not
    //     trap, but the reserved-instruction decode makes it trap regardless),
    //   * plus `testCop0InvalidOpcode` (an unknown COP0 command should be a
    //     no-op, but decodes to `Illegal`).
    // Before the dispatch chain those enabled/invalid cases "passed" only because
    // the harness silently swallowed the spurious trap; exposing them here is the
    // dispatch chain working correctly. They turn green once the decoder surfaces
    // COP1/COP3/LWCx/SWCx as distinct coprocessor ops.
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
