# Psoxide

Sony PlayStation (PSX) emulator in Rust. Part of the oxide emulator family.

## Architecture

- **psoxide-core**: Pure emulation library. No I/O, no windowing. Owns all hardware state.
  - Frontends drive via `Command` enum, poll via `CoreQuery`
  - Extract framebuffer: `core.framebuffer_rgba()` (placeholder gradient until the GPU exists)
  - All state serializable for snapshots (`save_state`/`load_state`)
- **psoxide-config**: TOML config, `PsxConfig::load_or_default()`
- **psoxide-desktop**: CLI frontend. Winit + Pixels. Silent audio stub.
- **psoxide-proof**: Verus proof scaffold (checked out-of-band; see below).
- **psoxide-test-harness**: Program/ROM-based integration tests.

## Hardware Emulated (v0.1)

- MIPS R3000A CPU @ ~33.8688 MHz (full MIPS I base ISA, little-endian)
- Coprocessor 0 basics: SR/CAUSE/EPC/BadVaddr, exception path, RFE
- Explicit branch delay and load delay slots
- Bus + memory map: 2MB RAM (mirrored), 1KB scratchpad, 512KB BIOS
- BIOS loading; reset vector at 0xBFC00000

## Not Yet Implemented

- GTE (cop2) — decoded but ignored
- GPU + VRAM (framebuffer is a placeholder gradient)
- SPU (audio — stubbed silent)
- CD-ROM
- DMA (7 channels)
- Interrupts beyond the cop0 exception path
- Hardware timers
- PSX-EXE side-loading (`LoadExe` is accepted as a no-op)

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

1. CPU instruction tests (Amidog `psxtest_cpu`, JaCzekanski `ps1-tests`) — gate before GPU
2. GPU rendering tests — golden-frame comparison
3. Full boot: BIOS boots to the shell/logo, then a real game boots from a disc image

## Milestones

1. CPU interpreter passes instruction tests **[current target]**
2. BIOS boots to shell/logo with GPU rendering
3. Boots a real game from a disc image

Test resources: Amidog PSX CPU/GTE tests, JaCzekanski `ps1-tests`, PeterLemon PSX demos.
