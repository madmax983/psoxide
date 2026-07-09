# Psoxide

Sony PlayStation (PSX) emulator in Rust. Part of the oxide emulator family.

## Architecture

- **psoxide-core**: Pure emulation library. No I/O, no windowing. Owns all hardware state.
  - Frontends drive via `Command` enum, poll via `CoreQuery`
  - Extract framebuffer: `core.framebuffer_rgba()` (renders the GPU display area from VRAM, 320x240 RGBA)
  - All state serializable for snapshots (`save_state`/`load_state`)
- **psoxide-config**: TOML config, `PsxConfig::load_or_default()`
- **psoxide-desktop**: CLI frontend. Winit + Pixels. Silent audio stub.
- **psoxide-proof**: Verus proof scaffold (checked out-of-band; see below).
- **psoxide-test-harness**: Program/ROM-based integration tests.

## Hardware Emulated (v0.1)

- MIPS R3000A CPU @ ~33.8688 MHz (full MIPS I base ISA, little-endian)
- Coprocessor 0 basics: SR/CAUSE/EPC/BadVaddr, exception path, RFE
- Hardware interrupt delivery: I_STAT/I_MASK controller drives cop0 CAUSE IP2;
  the interrupt exception is taken at instruction boundaries when SR enables it
- Explicit branch delay and load delay slots
- Bus + memory map: 2MB RAM (mirrored), 1KB scratchpad, 512KB BIOS
- BIOS loading; reset vector at 0xBFC00000
- GPU (`gpu.rs`): 1024x512 BGR555 VRAM, GP0 command FIFO (multi-word
  accumulation, no desync), GP1 control port, GPUSTAT/GPUREAD. Software
  rasterizer: fill, flat/Gouraud triangles + quads, textured triangles/quads/
  rectangles (4/8/15bpp CLUT + direct sampling, colour modulation, texture
  window), monochrome/textured rectangles, flat/Gouraud lines + poly-lines,
  VRAM↔VRAM and CPU↔VRAM transfers. Honours all four semi-transparency blend
  modes, ordered dithering, the mask bit, and the top-left fill rule.
  `framebuffer_rgba()` renders the real display area from VRAM (15bpp full;
  24bpp best-effort)
- DMA (`dma.rs`): register file for all 7 channels; channel 2 (GPU: linked-list
  + block, both directions), channel 3 (CD-ROM: device→RAM block copy pulling
  sector words from the CD data FIFO), and channel 6 (OTC) execute synchronously
  and raise the DMA interrupt via DICR
- CD-ROM (`cdrom.rs`): a real sub-controller at 0x1F80_1800..0x1F80_1803 (not
  the old read-back stub). Index-banked register file with parameter/response/
  data FIFOs; a command state machine (Getstat, Setloc, Play, ReadN/ReadS,
  MotorOn, Stop, Pause, Init, Mute/Demute, Setfilter, Setmode, Getparam,
  GetlocL/GetlocP, GetTN/GetTD, SeekL/SeekP, Test, GetID, ReadTOC); and an
  ordered INT1–INT5 response queue that latches only after the previous
  interrupt is acknowledged, raising the CD interrupt (`IrqLine::CdRom` →
  I_STAT bit 2) when enabled by the IE register. Timing is approximate: first/
  second response latency `FIRST_RESP_DELAY`/`SECOND_RESP_DELAY` (50_000 CPU
  cycles each) and a per-sector read period `READ_PERIOD_SINGLE` 451_584 /
  `READ_PERIOD_DOUBLE` 225_792 cycles (1x/2x), ticked from `Cdrom::tick` in the
  step loop. Discs are BIN/CUE MODE2/2352 images mounted via
  `Command::LoadDisc(Disc)` / ejected via `Command::EjectDisc`; a sector's user
  data (2048 bytes at raw offset 24, or 2340 at offset 12 when Setmode bit5 is
  set) is delivered to the CPU through the data FIFO (BFRD request) and to RAM
  through DMA channel 3. MSF↔LBA use a 150-sector pregap; per-sector reads emit
  INT1, and GetID reports the SCEA data-disc response (INT2) or the no-disc
  error (INT5). The `disc` module (psoxide-config) parses `.cue` sheets + their
  `.bin` tracks into a core `Disc`; the desktop `--disc` flag mounts one at
  startup and the CD-ROM integration test uses it as a dev-dependency
- Interrupt controller (`irq.rs`): I_STAT/I_MASK; VBlank raised once per
  `StepFrame`
- Hardware timers (`timers.rs`): the three root counters at
  0x1F80_1100..0x1F80_112F (value/mode/target, 16- and 32-bit access), clock
  sources (sysclk, sysclk/8, approximated dotclock/hblank), target/overflow IRQ
  delivery (Timer0/1/2 → I_STAT bits 4/5/6), one-shot vs repeat, read-clears the
  reached-flags. Ticked once per CPU cycle at the top of `step_cpu`
- I/O device stubs (`iostubs.rs`) — write-then-read-back register files that
  cover the memory-mapped regions a real BIOS touches during boot but for which
  no real emulation exists yet: memory-control (0x1F80_1000..0x1F80_1023 + the
  RAM_SIZE register at 0x1F80_1060), cache-control (0xFFFE_0130), SIO0 /
  joypad (0x1F80_1040..0x1F80_105F, "no controller attached" defaults), and the
  SPU register window (0x1F80_1C00..0x1F80_1FFF). No side effects, no DMA/IRQ
  delivery — the goal is only that BIOS init sequences do not FIFO-desync or
  panic on unmapped-region reads. SPUSTAT is synthesized to mirror the low six
  bits of SPUCNT the way real hardware does. (The CD-ROM ports
  0x1F80_1800..0x1F80_1803 are no longer a stub here — see the real `cdrom.rs`
  controller above)

## Not Yet Implemented

- GTE (cop2) — decoded but ignored
- SPU (audio — register-file stub in `iostubs.rs` reads back what the BIOS
  writes but produces no audio; there is no envelope, voice, or reverb engine)
- CD-ROM audio + fine timing (the controller in `cdrom.rs` is real — see
  "Hardware Emulated"): what stays stubbed is XA-ADPCM / CD-DA audio playback
  (Play/Setfilter/Mute are accepted but decode no samples and drive no SPU),
  subchannel Q beyond the GetlocL/GetlocP position bytes, and cycle-accurate
  seek/read timing (the read/response latencies are approximate constants, not
  measured mechanics). Narrow (8/16-bit) reads of the CD ports compose from the
  four consecutive ports rather than mirroring the addressed 8-bit register, and
  BUSYSTS is held for the whole command-latency window — both visible in the
  ps1-tests `io-access-bitwidth` `CDROM_STAT` rows. The upstream JaCzekanski
  ps1-tests `cdrom` binaries are **not** vendored as a gate: none are cleanly
  headless against an approximate-timing controller (`timing` measures exact
  cycle counts, `terminal`/`volume-regs`/`disc-swap` need interactive serial /
  gamepad / lid-open input, `getloc` needs INT-ack HLE the CPU-test loop lacks);
  CD-ROM is covered instead by `cdrom.rs` unit tests + the synthetic-fixture
  integration test `crates/psoxide-test-harness/tests/cdrom.rs`
- Coprocessor-unusable exception (ExcCode 0x0B): implemented for every
  coprocessor op. The decoder gives COP1 (0x11), COP3 (0x13), the LWCx/SWCx
  coprocessor load/stores (0x30-0x33 / 0x38-0x3B), and unassigned COP0 commands
  their own instruction variants (mirrored in the Verus decoder spec,
  `crates/psoxide-proof/src/decode.rs`). Each raises Coprocessor Unusable with
  CAUSE.CE set to the coprocessor number when its `SR.CU{n}` bit is clear, and is
  a no-op when usable (COP1/COP3 and the GTE datapath are absent). COP0 register
  ops (MFC0/MTC0) keep the kernel-mode usability exemption; the LWC0/SWC0
  coprocessor load/stores do **not** (they are gated purely by SR.CU0). No op
  decodes to `Illegal` / reserved-instruction (0x0A) any more except genuinely
  unassigned opcodes
- Instruction Bus-Error exception (ExcCode 0x06, IBE): implemented on the
  instruction-fetch path. A code fetch from a region that does not respond to a
  code-fetch bus cycle (I/O ports, scratchpad, expansion, cache-control,
  unmapped) raises IBE before the opcode is decoded/executed; EPC/BD are set as
  usual and (unlike an address error) BadVaddr is left untouched. Main RAM and
  BIOS are legal code sources; the DMA register block (0x1F80_1080..0x1F80_10FF)
  and the SPU register block (0x1F80_1C00..0x1F80_1FFF) are also fetchable,
  matching real hardware (ps1-tests `code-in-io` testCodeInDMA0/
  testCodeInDMAControl and testCodeInSPU — now **7/7**) since psoxide backs both
  register files. The **data** bus-error (ExcCode 0x07, DBE) has a constant
  (`EXC_DBE`) and `enter_exception` semantics (EPC/BD set, BadVaddr untouched),
  but no live trigger: the regions the boot path and ps1-tests
  `io-access-bitwidth` touch answer with open-bus reads / dropped writes (or an
  *address* error on a misaligned word access) rather than a data bus error, so
  wiring one would regress that open-bus behaviour
- BIOS exception-dispatch chain — the core exception path (vectors/EPC/rfe/
  syscall) is complete, but there is no BIOS kernel in psoxide-core to dispatch a
  program's registered exception/interrupt handlers. The test harness HLEs the
  minimal BIOS handler (syscall EnterCriticalSection/ExitCriticalSection,
  interrupt ack) for side-loaded CPU tests, and now also HLEs the
  **exception-dispatch chain** for programs that register an "unresolved
  exception" handler via the A0[0x40]/RAM-0x300 hook (as the ps1-tests runtime
  does): it stands up the kernel Process/Thread globals (`*(*(Process**)0x108)`),
  saves context to the TCB, runs the registered handler, and resumes at the
  handler's chosen return PC. This makes the `cpu/cop` "Disabled" cases observe
  their traps. This is harness test-infra only — psoxide-core still has no BIOS
  kernel
- PSX-EXE side-loading (core `Command::LoadExe` is accepted as a no-op; the
  test harness has a standalone PS-EXE sideloader, `Harness::load_exe`, used for
  CPU tests. The core no-op is retained to avoid duplicating the harness
  sideloader — no in-core consumer needs it)

### GPU/DMA gaps (implemented but partial)

- Textured polygons/rectangles sample real textures: 4/8/15bpp CLUT + 15bpp
  direct, colour modulation vs. raw, the texture window, and per-texel
  semi-transparency (STP). Remaining texture gaps: no perspective correction (the
  GPU is affine like real hardware, so this is not a bug) and no texture cache
  timing model
- Poly-lines are parsed to their terminator and each segment is Gouraud-
  interpolated between its own two endpoints (flat/monochrome lines keep their
  single colour); line pixels go through the shared shade/plot path so they
  honour the mask bit, dithering (Gouraud segments only, per PSX-SPX), and
  semi-transparency
- 24bpp display output is best-effort
- Semi-transparency, dithering, and the mask bit are stored in GPUSTAT but not
  applied during rasterization
- DMA channels other than 2 (GPU), 3 (CD-ROM), and 6 (OTC) are register-only
  (no transfer)
- Semi-transparency (all four blend modes), ordered dithering, and the mask bit
  (check-before-draw + set-while-drawing) are applied during rasterization for
  polygons, rectangles, and lines
- DMA channels other than 2 (GPU) and 6 (OTC) are register-only (no transfer)
- Interrupt delivery uses the single cop0 IP2 line; VBlank timing is one pulse
  per `StepFrame` rather than cycle-accurate

## Patterns

- Same Command/CoreQuery API as the NES emulator (returning `Result`/`QueryResult`)
- Concrete types, no trait objects. Enum dispatch for instructions.
- Snapshot-based save states via serde Serialize/Deserialize
- Verus specs on bus mapping (`mask_region`) + decoder [proof crate checked out-of-band]
- Proptest on CPU instruction semantics
- Explicit branch + load delay slots (two-bank register file)
- Little-endian throughout (unlike the big-endian 68000 in genesoxide)

## Key Constants

- CPU: ~33.8688 MHz
- Memory: 2MB main RAM (0x0000_0000, mirrored to 0x0080_0000), 1KB scratchpad
  (0x1F80_0000), 512KB BIOS (0x1FC0_0000)
- Reset vector: 0xBFC0_0000 (KSEG1 uncached BIOS entry)
- Segment map / region masks (`addr >> 29`):
  - KUSEG 0x0000_0000..0x8000_0000 → mask 0xFFFF_FFFF
  - KSEG0 0x8000_0000..0xA000_0000 → mask 0x7FFF_FFFF (cached, strips top bit)
  - KSEG1 0xA000_0000..0xC000_0000 → mask 0x1FFF_FFFF (uncached, strips top 3 bits)
  - KSEG2 0xC000_0000..0xFFFF_FFFF → mask 0xFFFF_FFFF

## Commands

```
cargo run -p psoxide-desktop -- run <bios> --scale 2
cargo run -p psoxide-desktop -- info <bios>
cargo test -p psoxide-core
cargo test -p psoxide-test-harness
```

Verus proofs are checked out-of-band (Verus is not a Cargo dependency):

```
pwsh scripts/verus-check.ps1
```

## Test Tiers

1. CPU instruction tests **[tier-1 gate wired]** — PS-EXE sideloader + BIOS TTY/`printf`/exception HLE + hardware timers in psoxide-test-harness. Always-on gates = synthetic PS-EXE self-test, syscall-exception round-trip, spec-derived MIPS corner tests (`cpu_semantics.rs`), and the four **vendored** JaCzekanski `ps1-tests` CPU binaries (MIT, `tests/ps1_tests.rs`) driven end-to-end to their progress markers. Amidog `psxtest_cpu` (CC BY-NC-SA, not vendored) stays an env-gated `run_real_suite` driver; it now runs to completion with 0 `value error` lines: the R3000 load-delay pipeline is modelled (the whole back-to-back same-register load-delay matrix passes) and `rfe` preserves the old SR interrupt/mode pair, so the rfe/break/syscall exception groups also pass (see `crates/psoxide-test-harness/README.md`).
2. GPU rendering tests — golden-frame comparison
3. Full boot: BIOS boots to the shell/logo, then a real game boots from a disc image

## Milestones

1. CPU interpreter passes instruction tests **[tier-1 gate wired]**
2. BIOS boots to shell/logo with GPU rendering **[in progress]** —
   the memory-mapped device windows the BIOS touches during startup have
   read-back-sane stubs (`iostubs.rs`); the `PSOXIDE_BIOS`-gated smoke test
   in `crates/psoxide-test-harness/tests/bios_smoke.rs` runs 180 frames and
   asserts tiered progress (PC advance, display enable / VRAM touch, and a
   framebuffer color-distribution check). No boot claim is made without a
   real BIOS image driving the gated test to green
3. Boots a real game from a disc image **[in progress]** — a real CD-ROM
   controller (`cdrom.rs`) now mounts BIN/CUE MODE2/2352 discs via
   `Command::LoadDisc`, executes the BIOS/runtime command set, and delivers
   sectors through the data FIFO and DMA channel 3 (see the CD-ROM entry under
   "Hardware Emulated"). Still needed for an actual game boot: the BIOS kernel /
   CD-ROM boot path and SPU audio

Test resources: Amidog PSX CPU/GTE tests, JaCzekanski `ps1-tests`, PeterLemon PSX demos.
