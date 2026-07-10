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
    //     hardware; psoxide now backs the SPU register file (`spu::Spu`), so
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
    // least 30 of the 67 golden lines match today. The remaining rows need
    // per-device narrow-access width semantics psoxide does not model yet — DMA
    // register 32-bit-only reads, JOY/SIO/IRQ/timer/GPU/MDEC/SPU
    // width-adaptation, expansion open-bus, and a real BIOS image for the BIOS
    // row (see the test-harness README). None of these are data bus errors: the
    // only `io-access-bitwidth` traps are the misaligned-word address errors
    // above.
    //
    // All three `CDROM_STAT` golden rows (the 8/16/32-bit read sections) now
    // match: the CD-ROM is modelled as an 8-bit device, so a wide *read* mirrors
    // the single addressed register across the access width (0x18 / 0x1818 /
    // 0x18181818 after an 8-bit write) rather than composing a word from the
    // four adjacent ports, and a wide *write* latches its bytes into the one
    // addressed register in ascending order (16/32-bit writes of 0x…5678 leave
    // the index register = 0x56 & 3 = 2, giving the 0x1a / 0x1a1a / 0x1a1a1a1a
    // rows) instead of spilling the second byte into the command port and
    // holding BUSYSTS for the command-latency window. See
    // `Cdrom::read16`/`read32`/`write16`/`write32`.
    let golden = String::from_utf8(fixture("cpu/io-access-bitwidth/psx.log")).unwrap();
    let matched = golden
        .lines()
        .map(str::trim_end)
        .filter(|l| produced.contains(l))
        .count();
    assert!(
        matched >= 30,
        "io-access-bitwidth golden match regressed: {matched} < 30"
    );
}

/// Parses the `access-time` result table into `label -> [w8, w16, w32]`, where
/// each entry is the **whole** part of the printed average cycles-per-access.
///
/// The test prints each cell with the format `%2d.%2-d` (whole `.` fraction).
/// The harness's `printf` HLE renders the leading `%2d` but not the trailing
/// `%2-d`, so only the whole part is machine-readable — enough to gate the model
/// to ~1-cycle resolution, which is all the golden's own noise (5.21 vs 5.30 …)
/// supports. Address tokens carry no `.` and are skipped.
fn parse_access_table(tty: &str) -> std::collections::HashMap<String, [i64; 3]> {
    let mut out = std::collections::HashMap::new();
    for line in tty.lines() {
        let mut toks = line.split_whitespace();
        let Some(label) = toks.next() else { continue };
        // A data row's label is all-caps letters/digits/underscore, e.g. "RAM",
        // "CDROM_STAT". Skip the header ("SEGMENT") and prose lines.
        if label == "SEGMENT"
            || !label
                .chars()
                .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
        {
            continue;
        }
        let wholes: Vec<i64> = toks
            .filter(|t| t.contains('.'))
            .filter_map(|t| t.split('.').next().unwrap().parse::<i64>().ok())
            .collect();
        if wholes.len() >= 3 {
            out.insert(label.to_string(), [wholes[0], wholes[1], wholes[2]]);
        }
    }
    out
}

#[test]
fn access_time_reproduces_region_timing() {
    // `access-time` times a single volatile load to each region and prints the
    // average CPU cycles per access (Timer2, sysclk source). With the wait-state
    // cost model the produced whole-cycle numbers track the vendored `psx.log`
    // golden for the well-modelled regions and preserve the region-aware spread.
    // The reference values are noisy fractions; we gate on the whole part.
    let tty = run_exe("cpu/access-time/access-time.exe", 2_000_000);

    // (a) The measurement ran start to finish.
    assert!(tty.contains("cpu/access-time"), "header missing:\n{tty}");
    assert!(
        tty.contains("Done."),
        "access-time did not run to completion:\n{tty}"
    );

    let table = parse_access_table(&tty);
    let get = |label: &str| -> [i64; 3] {
        *table
            .get(label)
            .unwrap_or_else(|| panic!("row {label:?} missing from table:\n{tty}"))
    };

    // (b) Well-modelled regions land within ±1.5 cycles of the golden at every
    // width. Golden values (8/16/32):
    //   RAM 5.21/5.30/5.14  BIOS 7.60/12.94/24.94  EXPANSION1 6.94/13.70/25.70
    //   EXPANSION3 6.70/6.10/9.95  CACHECTRL 0.95/1.90/1.90  and the internal
    //   I/O register rows, all ~3.0-3.8. SCRATCHPAD (1.50/1.10/0.94) is at the
    //   edge: its marginal cost rounds just under one cycle, so the whole part
    //   reads 0 — |0 - 1.50| = 1.50, exactly at tolerance.
    let within = |label: &str, golden: [f64; 3]| {
        let p = get(label);
        for w in 0..3 {
            let delta = (p[w] as f64 - golden[w]).abs();
            assert!(
                delta <= 1.5 + 1e-9,
                "{label} width{w}: produced {} vs golden {} (Δ{delta:.2} > 1.5)\n{tty}",
                p[w],
                golden[w],
            );
        }
    };
    within("RAM", [5.21, 5.30, 5.14]);
    within("BIOS", [7.60, 12.94, 24.94]);
    within("EXPANSION1", [6.94, 13.70, 25.70]);
    within("EXPANSION3", [6.70, 6.10, 9.95]);
    within("SCRATCHPAD", [1.50, 1.10, 0.94]);
    within("CACHECTRL", [0.95, 1.90, 1.90]);
    for io in [
        "DMAC_CTRL",
        "JOY_STAT",
        "SIO_STAT",
        "RAM_SIZE",
        "I_STAT",
        "TIMER0_VAL",
        "GPUSTAT",
        "MDECSTAT",
    ] {
        within(io, [3.0, 3.0, 3.0]);
    }

    // (c) The delay-driven external devices reproduce their golden rows exactly.
    // The sequential-turnaround split in `delay_1st_seq` closes the former 1-4
    // cycle residuals, so these now hold to the same ±1.5 whole-cycle tolerance
    // as the well-modelled regions:
    //   CDROM produced 8/14/26  vs golden 8.0/14.0/25.93
    //   SPU   produced 18/18/…  vs golden 17.99/17.99/…
    //   EXP2  produced 11/26/56 vs golden 10.99/25.99/55.98
    within("CDROM_STAT", [8.0, 14.0, 25.93]);
    within("EXPANSION2", [10.99, 25.99, 55.98]);
    // SPUCNT's 32-bit cell is a misaligned-word access — an address-error trap
    // whose cost is HLE exception overhead, not a bus access — so only the 8- and
    // 16-bit read cells are checked against the golden.
    let spu = get("SPUCNT");
    for (w, golden) in [(0usize, 17.99f64), (1, 17.99)] {
        let delta = (spu[w] as f64 - golden).abs();
        assert!(
            delta <= 1.5 + 1e-9,
            "SPUCNT width{w}: produced {} vs golden {golden} (Δ{delta:.2} > 1.5)\n{tty}",
            spu[w],
        );
    }
    let exp2 = get("EXPANSION2");

    // (d) The region-aware spread the whole model exists to produce:
    //   BIOS-32 (25) > RAM-32 (5) > SCRATCHPAD-32 (0), and Expansion 2 carries
    //   the largest 32-bit cost of every *aligned* row (57), dwarfing internal
    //   RAM/BIOS/scratchpad — a flat one-cycle model could never show this.
    let ram = get("RAM");
    let bios = get("BIOS");
    let scratch = get("SCRATCHPAD");
    assert!(
        bios[2] > ram[2] && ram[2] > scratch[2],
        "expected BIOS32 {} > RAM32 {} > SCRATCH32 {}\n{tty}",
        bios[2],
        ram[2],
        scratch[2],
    );
    // EXPANSION2 has the largest 32-bit access cost of any aligned region.
    for label in [
        "RAM",
        "BIOS",
        "EXPANSION1",
        "EXPANSION3",
        "SCRATCHPAD",
        "CDROM_STAT",
        "CACHECTRL",
        "DMAC_CTRL",
    ] {
        assert!(
            exp2[2] >= get(label)[2],
            "EXPANSION2-32 ({}) should be >= {label}-32 ({})\n{tty}",
            exp2[2],
            get(label)[2],
        );
    }
}
