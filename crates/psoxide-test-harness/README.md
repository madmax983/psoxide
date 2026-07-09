# psoxide-test-harness

Integration-test scaffolding for the psoxide PlayStation emulator, and the home
of the **tier-1 CPU test gate**.

## What's here

The [`Harness`] type drives `psoxide-core` through its public `Command` /
`CoreQuery` API. It can:

- hand-assemble MIPS programs directly into main RAM (`load_program` / `run`,
  see `tests/cpu_program.rs`), and
- **sideload PSX-EXE files** (`Harness::load_exe`) — parse the 0x800-byte
  `PS-X EXE` header, copy the body to its `t_addr`, and stage PC/GP/SP/`$ra`.

When a sideloaded EXE calls the BIOS, `Harness::run_hle` high-level-emulates the
A0h/B0h jump-table entries for `std_out_putchar`, `std_out_puts`, and
`printf` (A(0x3F), with `%d/%i/%u/%x/%X/%o/%c/%s/%p` plus flags, width, and
precision), capturing console output so it can be read back with `Harness::tty()`
/ `Harness::tty_bytes()` (`clear_tty()` resets the buffer). This lets CPU test
programs produce their pass/fail TTY log without a real BIOS image. Execution
stops when the program returns to the sentinel `HLE_RETURN_ADDR`.

`run_hle` also stands in for the **BIOS general exception handler** when a test
takes an exception and no real BIOS or test-installed handler lives at the
vector (`0x8000_0080` / `0xBFC0_0180`): it dispatches `syscall`
(`EnterCriticalSection` / `ExitCriticalSection` by `$a0`), acknowledges hardware
interrupts, and performs the `rfe`-style return (resuming at `EPC + 4` for a
syscall, `EPC` for an interrupt). If nonzero code is present at the vector — a
test that installs its own handler — the harness leaves it alone and lets the CPU
run it. Finally, `run_hle` injects a VBlank interrupt roughly once per frame's
worth of stepping so `VSync`-polling programs make progress (`StepCpu` alone
never raises VBlank).

## Always-on CPU gate (runs in CI, no external assets)

These tests are self-contained and run on every `cargo test --workspace`:

- **`tests/exe_loader.rs`** — a synthetic, in-code PS-EXE proves the sideloader
  plus TTY HLE end-to-end (it prints `"OK\n"` through the B-table
  `std_out_putchar` HLE), plus negative tests for a bad magic and an undersized
  image.
- **`tests/cpu_semantics.rs`** — spec-derived MIPS R3000A (MIPS I) corner cases:
  load-delay and branch-delay slots, `ADD`/`ADDI`/`SUB` overflow traps (ExcCode
  0x0C, destination unchanged), `DIV` `INT_MIN / -1` and divide-by-zero,
  `MULT`/`MULTU` high word, `LB`/`LH` sign-extension vs `LBU`/`LHU`
  zero-extension, `SLT` vs `SLTU` signedness, `LWL`/`LWR` merge, and `JAL`/`JALR`
  link = jump + 8. Expected values are taken from the MIPS I specification, not
  from this emulator, so they catch interpreter regressions.
- **`tests/ps1_tests.rs`** — drives the four **vendored** JaCzekanski `ps1-tests`
  CPU binaries (`cpu/{cop,code-in-io,io-access-bitwidth,access-time}`, MIT, under
  `tests/fixtures/ps1-tests/`) end-to-end through the sideloader + HLE + timers
  and asserts each loads, executes, and reaches its known progress markers (e.g.
  `cop` runs to `Done.` reporting the coprocessor-enabled passes). These are
  regression guards for the real-PS-EXE path; they do **not** yet assert a full
  `psx.log` match (each suite still needs hardware listed in the blocker table
  below — the vendored `psx.log` goldens are kept so the gate can be tightened as
  that hardware lands).

The four `ps1-tests` CPU binaries these gates use are **vendored** (MIT) under
`tests/fixtures/ps1-tests/` — see that directory's `README.md`/`LICENSE` for
attribution and per-file SHA1s. They now run every `cargo test`; the gates assert
end-to-end execution, not (yet) a full golden match.

## Amidog `psxtest_cpu` (env-gated, NOT vendored)

Amidog `psxtest_cpu` is **CC BY-NC-SA 3.0** (non-commercial + share-alike), which
is incompatible with vendoring into this project, so it stays env-gated behind the
ignored `run_real_suite` driver.

- Obtain: `https://psx.amidog.se/lib/exe/fetch.php?media=psx:download:psxtest_cpu.zip`
  → extracted `psxtest_cpu.exe` SHA1 `023aec8c92aaaf4d3b07956e26dd6c77ff397456`.
- Run:

  ```
  PSOXIDE_EXE=/path/to/psxtest_cpu.exe \
  PSOXIDE_STEPS=80000000 \
  PSOXIDE_OUT=/tmp/amidog.txt \
  cargo test -p psoxide-test-harness --release run_real_suite -- --ignored --nocapture
  ```

- Expected: with timers + the syscall/`printf`/exception HLE, the suite now runs
  end-to-end and prints its per-instruction test log (`Running <op> test` / `Done`)
  down to a final `Result: <errors>` line. Every ALU, shift, mul/div, immediate,
  branch, and **load-delay** group passes: the exhaustive back-to-back load-delay
  matrix (`nop_<load>_<load>_d`, i.e. two loads targeting the same register in
  consecutive slots — including the `LWL`/`LWR` merge and chain cases) is green
  after the R3000 load-delay-slot pipeline fix (a load in another load's delay
  slot squashes the earlier load's writeback; `LWL`/`LWR` merge with the value
  committed this cycle but still de-leak like every other load). The only
  remaining `value error` lines are the six exception-return-address cases
  (`syscall`/`rfe`/`break`, 2 each), which need the BIOS exception-dispatch chain
  (see the blocker table below); the count drops from ~594 to 6.

`PSOXIDE_EXE` is required; `PSOXIDE_STEPS` (step budget, default 50,000,000) and
`PSOXIDE_OUT` (write captured TTY to a file) are optional. The same driver runs
any PS-EXE, including the vendored `ps1-tests` binaries, for ad-hoc inspection.

## Remaining runtime blockers (why the gates aren't full golden diffs)

Milestone-2 landed hardware timers (`timers.rs`), the VBlank interrupt in the
step loop, and the BIOS TTY/`printf`/exception HLE — so all four `ps1-tests` CPU
binaries and Amidog `psxtest_cpu` now **run to completion** instead of stalling.
Turning each gate into a byte-for-byte `psx.log` comparison still needs hardware
that is out of scope for this milestone:

| Suite | Runs to completion | Remaining hardware for a full pass |
|-------|--------------------|------------------------------------|
| `cop` | yes (`Done.`, coprocessor-*enabled* cases pass) | BIOS **exception-dispatch chain** (so a program-registered handler actually runs and can observe the trap) + the **coprocessor-unusable** exception (ExcCode 0x0B). Our HLE services the trap itself but does not invoke a test-installed handler. |
| `code-in-io` | header + `testCodeInRam` pass | instruction **bus-error** exception (ExcCode 0x06) for fetches from MDEC/IO/SPU. |
| `io-access-bitwidth` | yes (`Done.`, RAM/scratchpad rows correct) | JOY / SIO / SPU / CD-ROM / MDEC registers with per-bitwidth semantics, plus data **bus-error** (`--CRASH--`) cases. |
| `access-time` | yes (`Done.`) | **cycle-accurate** per-region access timing; the one-cycle-per-instruction model cannot reproduce the reference cycle counts (this test has no self-asserted pass/fail — it is a manual comparison). |
| Amidog `psxtest_cpu` | yes (`Result:` printed; only 6 `value error` lines remain) | BIOS exception-dispatch chain for the rfe/break/syscall return-address cases. The R3000 **load-delay-slot pipeline** for back-to-back same-register loads (incl. `LWL`/`LWR`) is now modelled, so the whole load-delay matrix passes. |
