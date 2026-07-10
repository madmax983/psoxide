# Psoxide

Sony PlayStation (PSX) emulator in Rust. Part of the oxide emulator family.

## Architecture

- **psoxide-core**: Pure emulation library. No I/O, no windowing. Owns all hardware state.
  - Frontends drive via `Command` enum, poll via `CoreQuery`
  - Extract framebuffer: `core.framebuffer_rgba()` (renders the GPU display area from VRAM, 320x240 RGBA)
  - All state serializable for snapshots (`save_state`/`load_state`)
- **psoxide-config**: TOML config, `PsxConfig::load_or_default()`
- **psoxide-desktop**: CLI frontend. Winit + Pixels + rodio (SPU audio
  playback; falls back to silent if no audio device). Keyboard and gilrs gamepad
  input both drive controller port 0 through `Command::SetControllerState`.
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
  sector words from the CD data FIFO), channel 4 (SPU: bidirectional block copy
  between main RAM and SPU sample RAM), and channel 6 (OTC) execute
  synchronously and raise the DMA interrupt via DICR
- SPU (`spu.rs`): a real 24-voice audio engine at 0x1F80_1C00..0x1F80_1FFF with
  512KB sample RAM. ADPCM block decode (16-byte → 28-sample, shift + 4-tap
  filter, LoopStart/LoopEnd/LoopRepeat + ENDX), the integer PSX-SPX ADSR
  envelope (linear/exponential attack/decay/sustain/release), a 12-bit
  fractional pitch counter with linear interpolation and pitch modulation
  (PMON), a noise LFSR (NON) clocked from SPUCNT, per-voice + main stereo
  volume, key-on/key-off, and the IRQ-on-address unit (`IrqLine::Spu` → I_STAT
  bit 9). `Spu::tick` (in the step loop) emits one interleaved-stereo 44.1kHz
  `i16` sample every 768 CPU cycles into a queue drained by
  `PsxCore::drain_audio`; the desktop plays it through rodio. Transfers arrive
  via the CPU transfer FIFO (0x1DA8) or DMA channel 4. The **reverb DSP** is a
  real PSX-SPX delay network (the 22 comb/all-pass taps + IIR/wall/APF
  coefficients out of the reverb work area, clocked at 22.05kHz — every other
  44.1kHz sample, output held between ticks — gated by SPUCNT bit7 with the
  vLOUT/vROUT master applied to the wet return); its input is the per-voice EON
  sends plus the SPUCNT-bit2 CD reverb send. **CD audio** (CD-DA + XA-ADPCM
  decoded in `cdrom.rs`) is handed in each cycle via
  `Spu::push_cd_audio_samples` and mixed through the SPU CD input: the SPU CD
  input volume (0x1DB0/0x1DB2) scales the dry mix (gated by SPUCNT bit0) and the
  reverb send (bit2). Stubbed/simplified: the reverb input is not band-limited
  (every-other-sample clocking + held output rather than a proper anti-alias
  filter), and volume sweeps are still fixed-mode (bit15-clear volumes exact)
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
  error (INT5). **CD audio** is decoded: CD-DA audio-track sectors (2352 raw
  bytes → 588 stereo 44.1kHz PCM frames, played at 1x for correct pitch) and
  XA-ADPCM sectors (decoded + resampled to 44.1kHz through a bounded queue) are
  passed through the CD volume matrix (index-2/3 ports, unity at power-on) and
  Mute/Demute, then drained by `Cdrom::take_cd_audio` and pushed into the SPU CD
  input each cycle by the `step_cpu` bridge. The `disc` module (psoxide-config)
  parses `.cue` sheets + their `.bin` tracks into a core `Disc`; the desktop
  `--disc` flag mounts one at startup and the CD-ROM integration test uses it as
  a dev-dependency
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
  RAM_SIZE register at 0x1F80_1060), cache-control (0xFFFE_0130), and SIO0 /
  joypad (0x1F80_1040..0x1F80_105F — now implements the digital-pad serial
  protocol, clocking out real controller input set via
  `Command::SetControllerState`). No side effects, no DMA/IRQ delivery — the
  goal is only that BIOS init sequences do not FIFO-desync or panic on
  unmapped-region reads. (The CD-ROM ports 0x1F80_1800..0x1F80_1803 and the SPU
  window 0x1F80_1C00..0x1F80_1FFF are no longer stubs here — see the real
  `cdrom.rs` and `spu.rs` controllers above)

## Not Yet Implemented

- GTE (cop2) — decoded but ignored
- SPU volume-sweep envelopes (the `spu.rs` voice engine, reverb DSP, and CD
  mixing are all real — see "Hardware Emulated"): fixed-volume mode (register
  bit15 clear) is exact; sweep mode (bit15 set) is approximated to a
  near-full-scale constant. The reverb DSP runs the real PSX-SPX delay network
  but its input is not band-limited (the 22.05kHz clock is modelled by running
  the DSP every other 44.1kHz sample and holding the output, not by an
  anti-alias filter)
- CD-ROM fine timing (the controller in `cdrom.rs` and its CD-DA/XA-ADPCM audio
  are real — see "Hardware Emulated"): what stays approximate is subchannel Q
  beyond the GetlocL/GetlocP position bytes, cycle-accurate seek/read timing
  (the read/response latencies are approximate constants, not measured
  mechanics), and the CD-DA report / autopause cadence (once per sector rather
  than the hardware's finer subq granularity). XA playback is rate-regulated by
  a bounded queue rather than exact sector mechanics; Setfilter file/channel
  selection is honoured but the XA SPU-boost and gapless streaming details are
  not. Narrow (8/16-bit) reads of the CD ports compose from the
  four consecutive ports rather than mirroring the addressed 8-bit register, and
  BUSYSTS is held for the whole command-latency window — both visible in the
  ps1-tests `io-access-bitwidth` `CDROM_STAT` rows. The upstream JaCzekanski
  ps1-tests `cdrom` binaries are **not** vendored as a gate: none are cleanly
  headless against an approximate-timing controller (`timing` measures exact
  cycle counts, `terminal`/`volume-regs`/`disc-swap` need interactive serial /
  gamepad / lid-open input, `getloc` needs INT-ack HLE the CPU-test loop lacks);
  CD-ROM is covered instead by `cdrom.rs` unit tests + the synthetic-fixture
  integration tests `crates/psoxide-test-harness/tests/cdrom.rs` (data path) and
  `.../tests/cd_audio.rs` (CD-DA → SPU audio path)
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
- DMA channels other than 2 (GPU), 3 (CD-ROM), 4 (SPU), and 6 (OTC) are
  register-only (no transfer)
- Semi-transparency (all four blend modes), ordered dithering, and the mask bit
  (check-before-draw + set-while-drawing) are applied during rasterization for
  polygons, rectangles, and lines
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
make verify                 # Linux/macOS (skips cleanly if verus is absent)
./scripts/verus-check.sh    # same, direct
pwsh scripts/verus-check.ps1   # Windows / CI parity
```

## Formal Verification

The `psoxide-proof` crate holds hand-written Verus specs that machine-check the
pure logic of the emulator's bus map, instruction decoder, and timing tables.
Verus is installed **out-of-band** (it is not a Cargo dependency): a prebuilt
Verus release `0.2026.07.05` running on rust toolchain `1.96.0`. Get it from
<https://github.com/verus-lang/verus/releases>.

Run the proofs with `make verify` (or `./scripts/verus-check.sh`, or
`pwsh scripts/verus-check.ps1` on Windows). All entry points discover the
`verus` binary via the `VERUS` / `VERUS_BIN` env var or `verus` on PATH, and
skip cleanly (exit 0) with a message when it is not found, so they never break a
checkout without Verus. The proof `.rs` files are **not** declared as cargo
modules (only `lib.rs` is), so they do not affect `cargo build`/`clippy`/`test`.

### Machine-VERIFIED (33 verified, 0 errors)

- **bus_map.rs — 6 verified.** `mask_region`: segment bounds, mask
  boundedness, and per-segment mask correctness (the KUSEG/KSEG0/KSEG1/KSEG2
  region masks).
- **decode.rs — 4 verified.** Opcode 6-bit bound, and that coprocessor opcodes
  never decode to `Illegal`.
- **map_region.rs — 3 verified.** `BusRegion` decode totality, disjointness,
  and 19 boundary addresses.
- **timing.rs — 20 verified.** `delay_1st_seq` field bounds & overflow-safety;
  `bus_cycles >= 1` so there is no underflow; fixed-class exact cycle values;
  width-monotonicity; and golden exact-value lemmas for BIOS/EXP1 (7/13/25),
  EXP3, SPU, CDROM, and EXP2.

### ASSERTED-BY-TEST, not proven

- The decoder spec models opcode dispatch at **CLASS granularity** — it does
  not descend into the COP0/COP2 sub-decode.
- The timing monotonicity/overflow lemmas are stated over the real caller
  **width domain `{1, 2, 4}`**, with a documented `width_bytes >= 1`
  precondition.
- The proof files **DUPLICATE** the pure logic of `bus.rs` / `timing.rs` /
  the CPU decoder (`decode.rs`) — they are hand-mirrored specs, not references
  to the shared implementation code, and are **kept in sync manually**. A spec
  can therefore drift from the impl if the impl changes without the spec being
  updated.

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
   "Hardware Emulated"), and the SPU (`spu.rs`) synthesises audio — including
   the reverb DSP and CD-DA/XA-ADPCM audio now mixed through the SPU CD input.
   Still needed for an actual game boot: the BIOS kernel / CD-ROM boot path

Test resources: Amidog PSX CPU/GTE tests, JaCzekanski `ps1-tests`, PeterLemon PSX demos.
