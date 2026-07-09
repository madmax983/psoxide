//! Always-on gate: drive the vendored JaCzekanski `ps1-tests` CPU binaries
//! through the harness and assert each reaches its known progress markers. The
//! harness supplies the PS-EXE sideloader, the BIOS TTY / `printf` / exception
//! HLE, the hardware timers, and an injected VBlank.
//!
//! These gates prove the real-PS-EXE execution path works end-to-end — a
//! regression guard for the sideloader, the syscall/`printf` HLE, and the timer
//! interrupt path. They intentionally do **not** assert a full byte-for-byte
//! match against the shipped `psx.log`: several suites still exercise hardware
//! psoxide has not implemented yet (instruction/data bus-error exceptions,
//! cycle-accurate access timing, and the JOY / SIO / SPU / CD-ROM / MDEC
//! peripherals). The BIOS exception-dispatch chain that invokes
//! program-registered handlers is HLE'd by the harness (see `src/lib.rs`), and
//! the decoder now surfaces COP1/COP3/LWCx/SWCx (and unassigned COP0 commands)
//! as distinct coprocessor ops with correct Coprocessor-Unusable behaviour, so
//! `cpu/cop` passes in full against its golden. See `tests/README.md` and
//! `tests/fixtures/ps1-tests/README.md` for the remaining blocker details. The
//! `psx.log` goldens are vendored so these gates can be tightened to a golden
//! diff as that hardware lands.

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

    // Every `cpu/cop` case now passes, matching the vendored `psx.log` golden.
    // The decoder surfaces COP1/COP3/LWCx/SWCx (and unassigned COP0 commands) as
    // distinct coprocessor ops, so:
    //   * the coprocessor-*enabled* and *invalid-opcode* cases stay no-ops and
    //     raise no exception (a usable coprocessor op must not trap);
    //   * the *disabled* cases raise the real Coprocessor-Unusable exception
    //     (ExcCode 0x0B) with CAUSE.CE = the coprocessor number, which the
    //     program-registered "unresolved exception" handler observes via the
    //     BIOS exception-dispatch chain the harness HLEs;
    //   * `testDisabledCoprocessorThrowsCoprocessorUnusable` sees the exception
    //     *type* is 0x0B (previously it saw the reserved-instruction 0x0A).
    // COP0 register ops (MFC0) keep their kernel-mode usability exemption, so
    // `testCop0Disabled` correctly raises nothing in kernel mode, while
    // `testSwc0Disabled` still traps because coprocessor load/stores are gated
    // purely by SR.CU0 (no kernel-mode exemption).
    for case in [
        "testCop0Disabled",
        "testCop0Enabled",
        "testCop0InvalidOpcode",
        "testSwc0Disabled",
        "testSwc0Enabled",
        "testCop1Disabled",
        "testCop1Enabled",
        "testCop2Disabled",
        "testCop2Enabled",
        "testCop2InvalidOpcode",
        "testSwc2Disabled",
        "testSwc2Enabled",
        "testCop3Disabled",
        "testCop3Enabled",
        "testSwc3Disabled",
        "testSwc3Enabled",
        "testDisabledCoprocessorThrowsCoprocessorUnusable",
    ] {
        assert!(
            tty.contains(&format!("pass - {case}")),
            "expected {case} to pass:\n{tty}"
        );
    }

    assert!(
        tty.contains("Done."),
        "cop did not run to completion:\n{tty}"
    );
}

#[test]
fn code_in_io_executes_code_from_ram() {
    let tty = run_exe("cpu/code-in-io/code-in-io.exe", 1_000_000);
    assert!(tty.contains("cpu/code-in-io"), "header missing:\n{tty}");

    // Instruction Bus-Error (ExcCode 0x06) is now modelled: a code fetch from a
    // region that does not respond to a code-fetch bus cycle raises IBE, which
    // the program-registered "unresolved exception" handler observes (via the
    // BIOS exception-dispatch chain the harness HLEs), returning to `$ra`.
    //   * testCodeInRam        — main RAM is a legal code source (no exception).
    //   * testCodeInScratchpad — the D-cache scratchpad bus-errors on fetch.
    //   * testCodeInMDEC       — the MDEC I/O port bus-errors on fetch.
    //   * testCodeInInterrupts — the interrupt I/O port bus-errors on fetch.
    //   * testCodeInDMA0 / testCodeInDMAControl — the DMA register block responds
    //     to code fetch (no exception); psoxide backs those registers, so the
    //     copied `jr $ra` is read back and executes.
    //   * testCodeInSPU — the SPU register block also responds to a code fetch on
    //     hardware; psoxide now backs the SPU register file (`iostubs::Spu`), so
    //     `fetch_ok` treats it as a legal fetch source and the copied `jr $ra`
    //     reads back and executes. This is the full 7/7 set — a byte-for-byte
    //     match against the vendored `psx.log` golden.
    for case in [
        "testCodeInRam",
        "testCodeInScratchpad",
        "testCodeInMDEC",
        "testCodeInInterrupts",
        "testCodeInSPU",
        "testCodeInDMA0",
        "testCodeInDMAControl",
    ] {
        assert!(
            tty.contains(&format!("pass - {case}")),
            "expected {case} to pass:\n{tty}"
        );
    }

    assert!(
        tty.contains("Done."),
        "code-in-io did not run to completion:\n{tty}"
    );

    // Tighten to the full golden: every non-empty line of the vendored `psx.log`
    // must appear in the captured TTY (7/7).
    let golden = String::from_utf8(fixture("cpu/code-in-io/psx.log")).unwrap();
    let produced: std::collections::HashSet<&str> = tty.lines().map(str::trim_end).collect();
    for line in golden.lines().map(str::trim_end).filter(|l| !l.is_empty()) {
        assert!(
            produced.contains(line),
            "code-in-io golden line missing from output: {line:?}\n{tty}"
        );
    }
}

#[test]
fn io_access_bitwidth_runs_to_completion() {
    let tty = run_exe("cpu/io-access-bitwidth/io-access-bitwidth.exe", 2_000_000);
    assert!(
        tty.contains("cpu/io-access-bitwidth"),
        "header missing:\n{tty}"
    );
    assert!(
        tty.contains("Done."),
        "io-access-bitwidth did not run to completion:\n{tty}"
    );

    // Assert the real (currently achievable) pass set against the vendored
    // `psx.log` golden rather than just the progress marker. The generic memory
    // regions (RAM, SCRATCHPAD) reproduce the byte/half/word write-then-read
    // adaptation exactly across all three read widths, and the misaligned 32-bit
    // word accesses (JOY_CTRL / SIO_CTRL / SPUCNT at 0x…A) raise the existing
    // address-error path so the `--CRASH--` cells match. Every one of these
    // golden lines must appear verbatim (trailing whitespace ignored).
    let produced: std::collections::HashSet<&str> = tty.lines().map(str::trim_end).collect();
    for line in [
        // 32-bit read section.
        "RAM        (0x80080000)         0x78        0x5678    0x12345678",
        "SCRATCHPAD (0x1f800000)         0x78        0x5678    0x12345678",
        "JOY_CTRL   (0x1f80104a)            0             0    --CRASH--",
        // 16-bit read section.
        "RAM        (0x80080000)         0x78        0x5678",
        "SCRATCHPAD (0x1f800000)         0x78        0x5678",
        // 8-bit read section.
        "RAM        (0x80080000)         0x78",
        "SCRATCHPAD (0x1f800000)         0x78",
    ] {
        assert!(
            produced.contains(line),
            "io-access-bitwidth golden row missing from output: {line:?}\n{tty}"
        );
    }

    // Guard the aggregate count so device-accuracy work can only raise it: at
    // least 28 of the 67 golden lines match today. The remaining rows need
    // per-device narrow-access width semantics psoxide does not model yet — DMA
    // register 32-bit-only reads, JOY/SIO/IRQ/timer/CDROM/GPU/MDEC/SPU
    // width-adaptation, expansion open-bus, and a real BIOS image for the BIOS
    // row (see the test-harness README). None of these are data bus errors: the
    // only `io-access-bitwidth` traps are the misaligned-word address errors
    // above.
    let golden = String::from_utf8(fixture("cpu/io-access-bitwidth/psx.log")).unwrap();
    let matched = golden
        .lines()
        .map(str::trim_end)
        .filter(|l| produced.contains(l))
        .count();
    assert!(
        matched >= 28,
        "io-access-bitwidth golden match regressed: {matched} < 28"
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
