# psoxide

A Sony PlayStation (PSX) emulator written in Rust.

psoxide is the third emulator in the **oxide** family, after the NES core and
[genesoxide](https://github.com/madmax983/genesoxide) (Sega Genesis / Mega
Drive). Like its siblings it is a from-scratch, systems-learning project: the
goal is a clean, well-tested model of the hardware — a pure `no-I/O` core driven
by a thin desktop frontend — rather than a drop-in replacement for a mature
emulator. It is at **v0.1**: the CPU, GTE, GPU, SPU, CD-ROM, DMA, MDEC, timers,
and interrupt controller are all implemented and validated against external test
suites, but a full BIOS-to-game boot is not yet confirmed (see
[Milestones](#milestones)).

---

## Features

| Subsystem | Status | Notes |
|-----------|--------|-------|
| **CPU** — MIPS R3000A @ 33.8688 MHz | ✅ | Full MIPS I ISA, little-endian, explicit branch + load delay slots, two-bank register file |
| **COP0** (system control) | ✅ | SR/CAUSE/EPC/BadVaddr, exception vectors, RFE, hardware IRQ delivery, coprocessor-unusable + instruction bus-error exceptions |
| **GTE** (COP2 geometry engine) | ✅ | Full Nocash-spec datapath; passes all 1150 ps1-tests vectors bit-exact |
| **GPU** | ✅ | Software rasterizer: flat/Gouraud triangles & quads, textured polys/rects (4/8/15bpp CLUT + direct), all 4 semi-transparency blend modes, ordered dithering, mask bit, lines/poly-lines, VRAM↔VRAM & CPU↔VRAM transfers |
| **SPU** — 24-voice audio | ✅ | ADPCM decode, integer ADSR envelope, pitch counter w/ interpolation & modulation, noise LFSR, per-voice + main volume, IRQ-on-address, real reverb DSP, CD-audio mixing |
| **CD-ROM** | ✅ | Real sub-controller, BIN/CUE MODE2/2352 discs, full command set, INT1–INT5 queue, data-FIFO + DMA3 delivery, CD-DA + XA-ADPCM audio decode |
| **DMA** | ✅ (4 of 7 channels transfer) | GPU (ch2), CD-ROM (ch3), SPU (ch4), OTC (ch6) execute; others are register-only |
| **MDEC** | ✅ | Macroblock decoder (IDCT + colour conversion) |
| **Timers** | ✅ | Three root counters, clock sources, target/overflow IRQs, one-shot vs repeat |
| **Interrupt controller** | ✅ | I_STAT/I_MASK; VBlank once per frame |
| **Controllers** | ✅ | Digital pad, DualShock/analog, flightstick, multitap, memory cards via SIO0 |
| **BIOS kernel** | ❌ | No in-core kernel yet; the test harness HLEs the minimal handler for CPU tests |

See [CLAUDE.md](CLAUDE.md) for the exhaustive per-subsystem breakdown and the
honest [known-gaps list](#known-gaps).

---

## Accuracy & validation

psoxide is gated against external test suites. All figures below are asserted by
tests in `crates/psoxide-test-harness/tests/` — the ✅ rows run in CI on every push.

| Suite | Result | Gate |
|-------|--------|------|
| ps1-tests **GTE** `test-all` | `Passed tests: 1150 / Failed tests: 0` | ✅ always-on (`gte_tests.rs`) |
| ps1-tests **cop** (coprocessor-unusable) | 17 coprocessor cases pass, runs to `Done.` | ✅ always-on (`ps1_tests.rs`) |
| ps1-tests **code-in-io** | 7/7 cases, full byte-for-byte golden diff vs `psx.log` | ✅ always-on |
| ps1-tests **io-access-bitwidth** | ≥ 37 of 67 golden lines match (documented residuals) | ✅ always-on |
| ps1-tests **access-time** | Per-region cycle counts match golden within ±1.5 cycles | ✅ always-on |
| Amidog **psxtest_cpu** | Runs to completion, 0 value-error lines | ⚙️ env-gated (not vendored) |
| **Verus** proof lemmas | 33 verified, 0 errors | ⚙️ `make verify` (out-of-band) |

The vendored [JaCzekanski ps1-tests](https://github.com/JaCzekanski/ps1-tests)
binaries are MIT-licensed (see
`crates/psoxide-test-harness/tests/fixtures/ps1-tests/LICENSE`). The Amidog
`psxtest_cpu` suite is CC BY-NC-SA and therefore **not** vendored — it is fetched
manually and run through an env-gated driver (see [Testing](#testing)).

The 33 Verus lemmas machine-check the pure logic of the bus map, instruction
decoder, and cycle-timing tables. Verus is installed out-of-band (it is not a
Cargo dependency); `make verify` skips cleanly if it isn't present.

---

## Building & running

**System dependencies** (Debian/Ubuntu — for gamepad and audio backends):

```sh
sudo apt-get install -y pkg-config libudev-dev libasound2-dev
```

`libudev-dev` backs gilrs (gamepad); `libasound2-dev` (ALSA) backs rodio (audio).
On other platforms install the equivalent udev + ALSA/audio development packages.

**Build & test:**

```sh
cargo build --release
cargo test --workspace
```

**Run** (the desktop binary is `psoxide`):

```sh
# Boot a BIOS image
cargo run -p psoxide-desktop --release -- run <bios> --scale 2

# Boot a BIOS and mount a disc
cargo run -p psoxide-desktop --release -- run <bios> --disc game.cue

# Inspect a BIOS image
cargo run -p psoxide-desktop --release -- info <bios>
```

`run` flags: `--exe <psx-exe>` (side-load), `--disc <cue>`, `--memcard <path>`,
`--scale <n>`, `--fullscreen`, `--multitap`, `--config <toml>` (defaults to
`psoxide.toml`). CLI flags override the config file.

### BIOS & disc images

psoxide ships **no** copyrighted material. You must supply your own PlayStation
BIOS image and disc images (BIN/CUE) — dump them from hardware you own. None of
these files are committed to this repository, and none should be. The emulator
will not boot to the shell without a real BIOS.

---

## Controls

### Keyboard (gameplay)

| Key | Button | Key | Button |
|-----|--------|-----|--------|
| Arrow keys | D-pad | `Q` | L1 |
| `Z` | Cross (✕) | `W` | R1 |
| `X` | Circle (○) | `Enter` | Start |
| `A` | Square (□) | `Right Shift` | Select |
| `S` | Triangle (△) | | |

### Runtime controls

| Key | Action | Key | Action |
|-----|--------|-----|--------|
| `P` | Pause / resume | `F11` | Toggle fullscreen |
| `F` | Frame-step (while paused) | `=` / `-` | Scale window up / down |
| `Space` | Fast-forward (hold) | `1`–`9` | Select save-state slot |
| `R` | Reset | `F5` / `F9` | Save / load state |
| `Esc` | Quit | | |

A gamepad (via gilrs) also drives controller port 0: D-pad → D-pad,
South/East/West/North → ✕/○/□/△, L/R triggers → L1/R1.

The title bar shows a HUD: fps, emulation-speed %, and audio buffer fill + drop
count (all derived from core counters).

### Save states & memory cards

- **Save states** (`F5`/`F9`) are full serde snapshots written beside the content
  as `<stem>.ss<slot>` for slots 1–9. Snapshots carry identity metadata
  (core version, BIOS hash, disc hash); loading validates these and reports a
  typed mismatch instead of corrupting the machine. Legacy snapshots without
  metadata still load.
- **Memory cards** persist through the SIO0 protocol; pass `--memcard <path>` to
  choose the file. It is flushed on every exit path.

Runtime keybindings and last-used paths persist via `psoxide.toml`:

```toml
[desktop]
window_scale = 2
fullscreen = false

[keybindings]
pause = "KeyP"
# ... etc
```

---

## Testing

The always-on gate (fmt, clippy, the CPU/GTE/CD-ROM integration tests) runs with
no external assets:

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

**BIOS smoke test** (env-gated): point `PSOXIDE_BIOS` at a real BIOS image to run
the 180-frame boot-progress test:

```sh
PSOXIDE_BIOS=/path/to/bios.bin cargo test -p psoxide-test-harness bios_smoke
```

**Amidog psxtest_cpu** (env-gated, not vendored — CC BY-NC-SA): fetch
`psxtest_cpu.zip` from the Amidog site, extract, then drive the ignored
`run_real_suite` test with `PSOXIDE_EXE` / `PSOXIDE_STEPS` / `PSOXIDE_OUT`. See
`crates/psoxide-test-harness/README.md` for the exact invocation.

**Verus proofs** (out-of-band): install a Verus release and run

```sh
make verify        # or ./scripts/verus-check.sh, or pwsh scripts/verus-check.ps1
```

It skips cleanly (exit 0) when Verus is not on `PATH` / `$VERUS`.

---

## Architecture

A pure emulation core with no I/O, driven by a thin frontend through a
`Command` / `CoreQuery` API — the same shape as the other oxide emulators.

| Crate | Role |
|-------|------|
| **psoxide-core** | Pure emulation library. Owns all hardware state (`cpu/`, `gpu.rs`, `spu.rs`, `cdrom.rs`, `dma.rs`, `gte.rs`, `mdec.rs`, `timers.rs`, `irq.rs`, `bus.rs`, `timing.rs`). No I/O, no windowing. All state serializable for save states. |
| **psoxide-config** | TOML config (`PsxConfig`) + the CUE/BIN `disc` parser. |
| **psoxide-desktop** | CLI frontend (binary `psoxide`). winit + pixels + rodio + gilrs; HUD, hotkeys, save states, config persistence. |
| **psoxide-proof** | Hand-written Verus specs (bus map, decoder, timing). Checked out-of-band; not a Cargo dependency. |
| **psoxide-test-harness** | Integration tests + the tier-1 CPU gate (PS-EXE sideloader, BIOS TTY/exception HLE). |

- Concrete types, no trait objects; enum dispatch for instructions.
- Little-endian throughout (unlike the big-endian 68000 in genesoxide).
- Save states via serde `Serialize`/`Deserialize` with identity validation.

---

## Milestones

1. **CPU interpreter passes instruction tests** — ✅ done. GTE 1150/1150,
   the vendored ps1-tests CPU binaries, and the env-gated Amidog suite all pass.
2. **BIOS boots to shell/logo with GPU rendering** — 🚧 in progress. The device
   windows the BIOS touches at startup are modelled; the `PSOXIDE_BIOS`-gated
   smoke test asserts tiered boot progress. A logo/shell boot is not yet
   confirmed against a real BIOS image.
3. **Boots a real game from a disc image** — 🚧 in progress. The CD-ROM
   controller mounts and reads BIN/CUE discs and the SPU synthesises audio; what
   remains is the in-core BIOS kernel / CD-ROM boot path. No game boot yet.

---

## Known gaps

Honest residuals (see [CLAUDE.md](CLAUDE.md) for the full list):

- **No in-core BIOS kernel** — the exception-dispatch chain is HLE'd in the test
  harness only; there is no kernel in `psoxide-core` to boot a game.
- **SPU volume sweeps** — fixed-volume mode (register bit15 clear) is exact;
  sweep mode is approximated to a near-full-scale constant.
- **SPU reverb** — the real PSX-SPX delay network runs, but its input is not
  band-limited (22.05 kHz modelled by every-other-sample clocking, not an
  anti-alias filter).
- **CD-ROM fine timing** — read/seek/response latencies are approximate
  constants; subchannel-Q beyond position bytes, and the CD-DA report/autopause
  cadence, are coarse (per-sector). XA playback is rate-regulated by a bounded
  queue.
- **io-access-bitwidth** — 37 of 67 golden lines match; the CDROM_STAT narrow-read
  and BUSYSTS-hold rows are documented residuals.
- **GPU** — 24bpp display output is best-effort; no perspective correction (the
  GPU is affine like real hardware) and no texture-cache timing model. DMA
  channels other than 2/3/4/6 are register-only.
- **Data bus-error (DBE)** exception has semantics but no live trigger.
- **PSX-EXE side-loading** in the core (`Command::LoadExe`) is a no-op; the test
  harness has the standalone sideloader.

---

## License

MIT — see [LICENSE](LICENSE). Copyright (c) 2026 Mark Masterson.

Vendored test binaries retain their own licenses: the JaCzekanski ps1-tests
binaries are MIT (`crates/psoxide-test-harness/tests/fixtures/ps1-tests/LICENSE`).
The Amidog `psxtest_cpu` suite (CC BY-NC-SA) is **not** vendored; it is fetched
manually for the env-gated driver.
