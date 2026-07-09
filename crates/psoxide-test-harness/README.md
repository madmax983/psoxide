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
A0h/B0h jump-table entries for `std_out_putchar` and `std_out_puts`, capturing
console output so it can be read back with `Harness::tty()` /
`Harness::tty_bytes()` (`clear_tty()` resets the buffer). This lets CPU test
programs produce their pass/fail TTY log without a real BIOS image. Execution
stops when the program returns to the sentinel `HLE_RETURN_ADDR`.

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

## External reference suites (env-gated, NOT vendored)

These binaries are not committed to the repo. Obtain them out-of-band and run
them through the ignored `run_real_suite` driver.

| Suite | License | How to obtain (SHA1) | Runtime feature(s) still needed |
|-------|---------|----------------------|---------------------------------|
| **JaCzekanski ps1-tests** (`cpu/`) | MIT | https://github.com/JaCzekanski/ps1-tests (prebuilt binaries in Releases `build-158`). CI-friendly mirror (GitHub egress-blocked in our env): `https://archive.org/download/tests_202203/tests.zip`, zip SHA1 `bc9d5f910cd79f86ec703f198f0bf46a12253ab6`. CPU EXEs + expected `psx.log` under `cpu/{access-time,io-access-bitwidth,code-in-io,cop}/`. Per-exe SHA1: access-time `bf3e90089b7e8a1b92ca18f2f547b205bf595559`, code-in-io `409ac92b8f77ed753a85076a926cfb37dd7431ff`, cop `74bf58ae5237263ab2580dcc5558c3e75b8b53f5`, io-access-bitwidth `9b1c1e87b7969d7c64f2c61d6bda020ab014668d`. | Output is plain ASCII TTY (psn00bsdk `printf` → BIOS putchar); verify by diffing captured TTY against the shipped `psx.log`. See blockers table below. |
| **Amidog psxtest_cpu** | CC BY-NC-SA 3.0 (non-commercial) — **not vendored** (non-commercial + share-alike is incompatible with vendoring into this project) | `https://psx.amidog.se/lib/exe/fetch.php?media=psx:download:psxtest_cpu.zip`; extracted `psxtest_cpu.exe` SHA1 `023aec8c92aaaf4d3b07956e26dd6c77ff397456`. | Reports results to the GPU and to TTY. See blockers table below. |

### How to run one manually

```
PSOXIDE_EXE=/path/to/test.exe \
PSOXIDE_STEPS=50000000 \
PSOXIDE_OUT=/tmp/out.txt \
cargo test -p psoxide-test-harness --release run_real_suite -- --ignored --nocapture
```

`PSOXIDE_EXE` is required; `PSOXIDE_STEPS` (step budget, default 50,000,000) and
`PSOXIDE_OUT` (write captured TTY to a file) are optional. The driver prints the
step count, whether it terminated via the sentinel, the final PC, and the
captured TTY.

## Why the external suites are gated (remaining runtime blockers)

These binaries assume a booting console and stall on hardware the core does not
implement yet. Each blocker below names the suites it holds back:

| Blocker | Missing hardware | Suites blocked | Provided by |
|---------|------------------|----------------|-------------|
| **GPUSTAT-ready** | GPU status poll at `0x1F801814` bit 26 during GPU init | **all** of them | in-progress GPU work (branch `gpu-command-fifo`) |
| **VBlank interrupt / frame counter** | `VSync` wait (I_STAT/I_MASK + VBlank tick) | `io-access-bitwidth`, `code-in-io`, Amidog `psxtest_cpu` | `gpu-command-fifo` |
| **BIOS syscall + exception-vector handling** | `syscall` → 0xBFC00180 (EnterCriticalSection / ExitCriticalSection) | `cop`, `access-time` | milestone-2 follow-up (needs a BIOS image or syscall HLE) |
| **Hardware timers** | root-counter timing | `access-time` (additionally) | milestone-2 follow-up |

Bottom line: once VBlank IRQ + GPUSTAT land (via `gpu-command-fifo`), enabling
ps1-tests `io-access-bitwidth` / `code-in-io` is a mechanical re-run against
their shipped `psx.log`. `cop` / `access-time` / Amidog `psxtest_cpu`
additionally need BIOS syscall/exception handling (and, for `access-time`,
timers) — a milestone-2 follow-up.
