//! Criterion perf-baseline suite for psoxide's hot loops.
//!
//! This is **measurement scaffolding only** — it optimizes nothing. It captures
//! a reproducible baseline for three classes of work so future perf changes can
//! be judged against a fixed reference:
//!
//!   A. `StepFrame` throughput — one full `Command::StepFrame` (the fanned-out
//!      per-cycle loop over `CYCLES_PER_FRAME` = 564,480 CPU cycles) for three
//!      synthetic guest workloads (pure arithmetic, memory traffic, GTE ops).
//!   B. GPU rasterization — a representative batch of Gouraud triangles and flat
//!      quads pushed through the public `Gpu::gp0` port.
//!   C. SPU frame mix — one frame's worth (735 stereo samples = 768*735 CPU
//!      cycles) of the 24-voice ADPCM+ADSR mixer via the public `Spu::tick`.
//!
//! Public-API paths used (the harness crate can only touch `pub` items of
//! psoxide-core):
//!   * CPU workloads: `Harness::load_program` stages hand-assembled MIPS into
//!     RAM; `PsxCore::execute(Command::StepFrame)` runs the frame loop.
//!   * GPU: a directly-constructed `psoxide_core::Gpu` driven by its public
//!     `gp0(word)` entry (equivalent to a guest store to the GP0 port
//!     `0x1F80_1810`, but isolates the rasterizer from CPU-loop overhead).
//!   * SPU: a directly-constructed `psoxide_core::Spu` programmed through its
//!     public `write16` register path (voice regs, key-on, transfer FIFO) and
//!     advanced by the public `tick(cycles, &mut Irq)` — isolating the mixer
//!     from CPU-loop overhead. (`Spu`/`Irq` are re-exported at the crate root
//!     and fully constructible; there is no `PsxCore::spu_mut`, so this is the
//!     representative public path for the mixer in isolation.)

use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;

use psoxide_core::api::CYCLES_PER_FRAME;
use psoxide_core::spu::{SPU_BASE, VOICES};
use psoxide_core::{Command, Gpu, Irq, Spu};
use psoxide_test_harness::Harness;

// ── MIPS instruction encoders (mirroring tests/cpu_program.rs) ───────────────

/// Assembles an I-type instruction word.
fn i_type(op: u32, rs: u32, rt: u32, imm: u16) -> u32 {
    (op << 26) | (rs << 21) | (rt << 16) | u32::from(imm)
}
/// Assembles an R-type (SPECIAL) instruction word.
fn r_type(rs: u32, rt: u32, rd: u32, shamt: u32, funct: u32) -> u32 {
    (rs << 21) | (rt << 16) | (rd << 11) | (shamt << 6) | funct
}
/// Assembles a COP2 move (`mtc2`/`ctc2`): `rs_sel` selects the sub-op (0x04 =
/// MTC2 → data reg, 0x06 = CTC2 → control reg).
fn cop2_move(rs_sel: u32, rt: u32, rd: u32) -> u32 {
    (0x12 << 26) | (rs_sel << 21) | (rt << 16) | (rd << 11)
}
/// Assembles a GTE command word (`CO=1`): `(0x12<<26)|(1<<25)|cmd`.
fn gte_cmd(cmd: u32) -> u32 {
    (0x12 << 26) | (1 << 25) | cmd
}

/// The general-exception vector (RAM). If a workload traps, PC parks here and
/// the frame loop measures the wrong thing — the smoke checks guard against it.
const EXC_VECTOR: u32 = 0x8000_0080;

// ── A. StepFrame CPU workloads ───────────────────────────────────────────────

/// Builds an `arith` harness: a non-terminating tight loop of `addiu`/`addu`/
/// `slt`/`beq` (with a delay slot) in cached KUSEG RAM at base 0. No memory
/// access, no traps; the unconditional backward branch keeps the frame loop
/// entirely in-loop for the whole cycle budget.
fn build_arith() -> Harness {
    // 0x00 addiu $t0,$t0,1      ; counter++
    // 0x04 addu  $t1,$t1,$t0    ; acc += counter
    // 0x08 slt   $t2,$t0,$t1    ; t2 = t0 < t1
    // 0x0C beq   $zero,$zero,-4 ; unconditional -> 0x00
    // 0x10 addiu $t3,$t3,1      ; delay slot: extra arith
    let program = [
        i_type(0x09, 8, 8, 1),      // addiu $t0,$t0,1
        r_type(9, 8, 9, 0, 0x21),   // addu  $t1,$t1,$t0
        r_type(8, 9, 10, 0, 0x2A),  // slt   $t2,$t0,$t1
        i_type(0x04, 0, 0, 0xFFFC), // beq   $zero,$zero,-4 -> 0x00
        i_type(0x09, 11, 11, 1),    // addiu $t3,$t3,1 (delay slot)
    ];
    let mut h = Harness::new();
    h.load_program(&program);
    h
}

/// Builds a `mem` harness: an `sw`/`lw` round-trip to a fixed RAM address plus a
/// value bump each iteration, with an unconditional backward branch. Exercises
/// the bus data-cost path + `extra`-tick fan-out every iteration.
fn build_mem() -> Harness {
    // 0x00 addiu $t1,$zero,0x100 ; base ptr (once)
    // 0x04 addiu $t0,$zero,0     ; value (once)
    // 0x08 sw    $t0,0($t1)      ; loop: store
    // 0x0C lw    $t2,0($t1)      ;       load back
    // 0x10 addiu $t0,$t0,1       ;       bump value
    // 0x14 beq   $zero,$zero,-4  ;       -> 0x08
    // 0x18 nop                   ; delay slot
    let program = [
        i_type(0x09, 0, 9, 0x0100),  // addiu $t1,$zero,0x100
        i_type(0x09, 0, 8, 0),       // addiu $t0,$zero,0
        i_type(0x2B, 9, 8, 0),       // sw    $t0,0($t1)
        i_type(0x23, 9, 10, 0),      // lw    $t2,0($t1)
        i_type(0x09, 8, 8, 1),       // addiu $t0,$t0,1
        i_type(0x04, 0, 0, 0xFFFC),  // beq   $zero,$zero,-4 -> 0x08
        0,                           // nop (delay slot)
    ];
    let mut h = Harness::new();
    h.load_program(&program);
    // Start at the loop head (skip the two one-shot init instructions the first
    // time is fine too, but jumping straight in keeps the loop body pure). Run
    // the two inits first, then loop forever.
    h
}

/// Builds a `gte` harness: enables COP2 (`SR.CU2`, bit 30), seeds a couple of
/// GTE data/control registers, then loops a heavy RTPT (`cmd=0x30`) via a
/// backward branch. RTPT transforms three vertices through the rotation matrix.
fn build_gte() -> Harness {
    // 0x00 lui   $t0,0x0040      ; a nonzero seed value
    // 0x04 mtc2  $t0,$0          ; GTE data reg 0  (VXY0)
    // 0x08 mtc2  $t0,$1          ; GTE data reg 1  (VZ0)
    // 0x0C mtc2  $t0,$2          ; GTE data reg 2  (VXY1)
    // 0x10 ctc2  $t0,$0          ; GTE control reg 0 (rotation matrix element)
    // 0x14 ctc2  $t0,$6          ; GTE control reg 6 (translation)
    // 0x18 RTPT (cmd 0x30)       ; loop: perspective-transform 3 verts
    // 0x1C addiu $t3,$t3,1       ;       sentinel advance
    // 0x20 beq   $zero,$zero,-3  ;       -> 0x18
    // 0x24 nop                   ; delay slot
    let program = [
        i_type(0x0F, 0, 8, 0x0040),  // lui   $t0,0x0040
        cop2_move(0x04, 8, 0),       // mtc2  $t0,$0
        cop2_move(0x04, 8, 1),       // mtc2  $t0,$1
        cop2_move(0x04, 8, 2),       // mtc2  $t0,$2
        cop2_move(0x06, 8, 0),       // ctc2  $t0,$0
        cop2_move(0x06, 8, 6),       // ctc2  $t0,$6
        gte_cmd(0x30),               // RTPT (loop head @ 0x18)
        i_type(0x09, 11, 11, 1),     // addiu $t3,$t3,1
        i_type(0x04, 0, 0, 0xFFFD),  // beq   $zero,$zero,-3 -> 0x18
        0,                           // nop (delay slot)
    ];
    let mut h = Harness::new();
    h.load_program(&program);
    // Enable COP2 usability (SR.CU2 = bit 30) so the GTE ops do not raise
    // Coprocessor-Unusable.
    h.core_mut()
        .set_cop0(psoxide_core::COP0_SR, 1 << 30);
    h
}

/// Runs a workload for `steps` `StepCpu`s and asserts it is genuinely looping —
/// PC inside the program's low RAM window and NOT parked at the exception
/// vector — then prints a one-line confirmation. `loop_hi` is an exclusive upper
/// bound on legal PC. `sentinel_reg` (if `Some`) must have advanced past 0.
fn smoke(name: &str, mut h: Harness, steps: usize, loop_hi: u32, sentinel_reg: Option<usize>) {
    h.run(steps);
    let pc = h.core_mut().pc();
    assert_ne!(
        pc, EXC_VECTOR,
        "{name}: trapped to exception vector 0x{EXC_VECTOR:08X} — bench would measure the wrong thing"
    );
    assert!(
        pc < loop_hi,
        "{name}: PC 0x{pc:08X} escaped the loop window (< 0x{loop_hi:08X})"
    );
    if let Some(r) = sentinel_reg {
        let v = h.reg(r);
        assert!(v > 0, "{name}: sentinel $r{r} did not advance (still {v})");
        println!(
            "[smoke] {name}: OK — PC=0x{pc:08X} in-loop, sentinel $r{r}={v}, no trap after {steps} steps"
        );
    } else {
        println!("[smoke] {name}: OK — PC=0x{pc:08X} in-loop, no trap after {steps} steps");
    }
}

fn bench_stepframe(c: &mut Criterion) {
    // Smoke-assert each workload actually executes its loop without trapping.
    smoke("arith", build_arith(), 200, 0x14, Some(11)); // $t3 sentinel
    smoke("mem", build_mem(), 200, 0x1C, Some(8)); // $t0 value bump
    smoke("gte", build_gte(), 200, 0x28, Some(11)); // $t3 sentinel

    let mut group = c.benchmark_group("stepframe");
    // One StepFrame is ~564k cycles of work; a handful of samples suffices.
    group.sample_size(30);

    for (name, mut h) in [
        ("arith", build_arith()),
        ("mem", build_mem()),
        ("gte", build_gte()),
    ] {
        group.bench_function(name, |b| {
            b.iter(|| {
                let _ = h.core_mut().execute(black_box(Command::StepFrame));
            });
        });
    }
    group.finish();
}

// ── B. GPU rasterization ─────────────────────────────────────────────────────

/// Number of Gouraud triangles in the GPU batch.
const GPU_TRIS: usize = 200;
/// Number of flat quads in the GPU batch (each = 2 rasterized triangles).
const GPU_QUADS: usize = 100;
/// Total rasterized triangles per batch (quads split into two).
const GPU_TRIANGLES_PER_BATCH: usize = GPU_TRIS + GPU_QUADS * 2;

/// Packs an XY vertex word (`y<<16 | x`) with small positive coordinates.
fn vert(x: i32, y: i32) -> u32 {
    ((y as u32 & 0xFFFF) << 16) | (x as u32 & 0xFFFF)
}
/// Packs a `0x00BBGGRR` colour word.
fn color(r: u8, g: u8, b: u8) -> u32 {
    (u32::from(b) << 16) | (u32::from(g) << 8) | u32::from(r)
}

/// Builds the flat list of GP0 words for one representative batch: 200 Gouraud
/// triangles and 100 flat quads scattered across the 320x240 draw area, each a
/// non-degenerate ~40px primitive so real spans are rasterized.
fn build_gpu_batch() -> Vec<u32> {
    let mut words = Vec::new();
    for i in 0..GPU_TRIS {
        // Scatter across the frame; wrap columns every ~8 primitives.
        let bx = ((i * 37) % 280) as i32;
        let by = ((i * 53) % 200) as i32;
        let c0 = color(0xFF, (i & 0xFF) as u8, 0x20);
        let c1 = color(0x20, 0xFF, (i.wrapping_mul(3) & 0xFF) as u8);
        let c2 = color((i.wrapping_mul(5) & 0xFF) as u8, 0x20, 0xFF);
        // Gouraud triangle (0x30): cmd+color0, v0, color1, v1, color2, v2.
        words.push(0x30 << 24 | c0);
        words.push(vert(bx, by));
        words.push(c1);
        words.push(vert(bx + 40, by + 5));
        words.push(c2);
        words.push(vert(bx + 8, by + 38));
    }
    for i in 0..GPU_QUADS {
        let bx = ((i * 61) % 280) as i32;
        let by = ((i * 29) % 200) as i32;
        let c = color((i.wrapping_mul(7) & 0xFF) as u8, 0x80, 0xC0);
        // Flat quad (0x28): cmd+color, v0, v1, v2, v3.
        words.push(0x28 << 24 | c);
        words.push(vert(bx, by));
        words.push(vert(bx + 38, by + 2));
        words.push(vert(bx + 2, by + 38));
        words.push(vert(bx + 38, by + 38));
    }
    words
}

/// Configures a fresh GPU's drawing environment (draw area covering the whole
/// display, zero offset) so submitted primitives actually rasterize.
fn setup_gpu_env(gpu: &mut Gpu) {
    gpu.gp0(0xE3 << 24); // draw area top-left = (0,0)
    gpu.gp0((0xE4 << 24) | (239 << 10) | 319); // bottom-right = (319,239)
    gpu.gp0(0xE5 << 24); // draw offset = (0,0)
}

fn bench_gpu(c: &mut Criterion) {
    let batch = build_gpu_batch();
    let mut gpu = Gpu::new();
    setup_gpu_env(&mut gpu);
    println!(
        "[info] gpu: batch = {GPU_TRIS} gouraud tris + {GPU_QUADS} flat quads = {GPU_TRIANGLES_PER_BATCH} rasterized triangles, {} GP0 words",
        batch.len()
    );

    let mut group = c.benchmark_group("gpu");
    group.sample_size(50);
    group.bench_function("raster_batch", |b| {
        b.iter(|| {
            for &w in &batch {
                gpu.gp0(black_box(w));
            }
            black_box(gpu.vram_at(0, 0));
        });
    });
    group.finish();
}

// ── C. SPU frame mix ─────────────────────────────────────────────────────────

/// CPU cycles per SPU frame (735 stereo samples at 768 cycles each).
const SPU_FRAME_CYCLES: u32 = 768 * 735;

/// Builds an `Spu` with all 24 voices keyed on, each playing a looping ADPCM
/// block at full pitch and volume, so `tick` mixes every voice each sample.
fn build_spu() -> Spu {
    let mut spu = Spu::new();

    // A single looping ADPCM block at SPU-RAM byte address 0x1000. b0 = shift 0
    // / filter 0; b1 = LoopStart|LoopRepeat|LoopEnd (0x07) so it loops forever;
    // the 14 data bytes alternate a -4096/0 tone.
    let mut block = [0u8; 16];
    block[0] = 0x00;
    block[1] = 0x07;
    for b in block.iter_mut().skip(2) {
        *b = 0x0F;
    }
    // Upload the block via the transfer FIFO: set transfer addr (0x1DA6, in
    // 8-byte units) then push halfwords to 0x1DA8.
    spu.write16(0x1F80_1DA6, (0x1000u32 >> 3) as u16);
    for pair in block.chunks_exact(2) {
        spu.write16(0x1F80_1DA8, u16::from(pair[0]) | (u16::from(pair[1]) << 8));
    }

    // Main volume L/R (fixed mode, bit15 clear).
    spu.write16(0x1F80_1D80, 0x3FFF);
    spu.write16(0x1F80_1D82, 0x3FFF);
    // Enable the SPU (SPUCNT bit15). Reverb (bit7) left off: this benches the
    // voice-mix path, not the reverb DSP.
    spu.write16(0x1F80_1DAA, 0x8000);

    // Program every voice: start addr = 0x1000>>3, pitch = 0x1000 (1.0), vol
    // L/R = 0x3FFF, ADSR left at defaults (fast attack -> sustain).
    for v in 0..VOICES {
        let base = SPU_BASE + (v as u32) * 16;
        spu.write16(base, 0x3FFF); // volume L
        spu.write16(base + 2, 0x3FFF); // volume R
        spu.write16(base + 4, 0x1000); // pitch (sample rate) = 1.0
        spu.write16(base + 6, (0x1000u32 >> 3) as u16); // ADPCM start addr
        spu.write16(base + 8, 0x0000); // ADSR lo
        spu.write16(base + 10, 0x0000); // ADSR hi
    }
    // Key on all 24 voices (lo 16 + hi 8).
    spu.write16(0x1F80_1D88, 0xFFFF);
    spu.write16(0x1F80_1D8A, 0x00FF);
    spu
}

fn bench_spu(c: &mut Criterion) {
    let mut spu = build_spu();
    let mut irq = Irq::new();

    // Smoke: one frame must actually emit 735 stereo pairs (1470 i16 values)
    // with nonzero audio, or the mixer isn't doing work.
    {
        let mut probe = build_spu();
        let mut pirq = Irq::new();
        probe.tick(SPU_FRAME_CYCLES, &mut pirq);
        let samples = probe.drain_samples();
        assert_eq!(
            samples.len(),
            735 * 2,
            "SPU should emit 735 stereo pairs per frame"
        );
        assert!(
            samples.iter().any(|&s| s != 0),
            "SPU frame produced only silence — voices not mixing"
        );
        println!(
            "[smoke] spu: OK — {} i16 samples/frame, nonzero audio from {VOICES} voices",
            samples.len()
        );
    }

    let mut group = c.benchmark_group("spu");
    group.sample_size(50);
    group.bench_function("frame_mix", |b| {
        b.iter(|| {
            spu.tick(black_box(SPU_FRAME_CYCLES), &mut irq);
            black_box(spu.drain_samples());
        });
    });
    group.finish();
}

criterion_group!(benches, bench_stepframe, bench_gpu, bench_spu);
criterion_main!(benches);

// Keep the CYCLES_PER_FRAME import meaningful/documented: one StepFrame runs
// exactly this many cycles, and SPU_FRAME_CYCLES equals it (768*735 = 564480).
const _: () = assert!(SPU_FRAME_CYCLES as u64 == CYCLES_PER_FRAME);
