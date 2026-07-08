# psoxide-test-harness

Integration-test scaffolding for the psoxide PlayStation emulator.

The [`Harness`] type wraps `PsxCore` and lets tests:

- stage hand-assembled MIPS programs into main RAM (`load_program`),
- load a 512KB BIOS image (`load_bios`),
- step the CPU (`run`), and
- inspect registers and memory (`reg`, `registers`, `read_word`).

## Where real test ROMs go

Drop binary CPU test ROMs under `tests/roms/` and drive them through
`Harness::load_bios` (or a future EXE side-loader). Recommended suites, in the
family's tier-1 "CPU instruction tests gate before GPU" order:

- **Amidog `psxtest_cpu`** — the canonical R3000A + cop0 instruction test.
- **JaCzekanski `ps1-tests`** — modern, granular CPU/GTE/GPU suites.
- **PeterLemon PSX** — small hand-written demos useful for bring-up.

These binaries are not vendored here (licensing / size); add them locally or via
a fetch script when wiring golden-output comparisons.
