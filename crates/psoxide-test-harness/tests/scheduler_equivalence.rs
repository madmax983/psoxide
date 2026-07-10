//! Differential equivalence tests: the lazy device scheduler
//! (`PsxCore::step_cpu_scheduled`, on by default) must be **bit-for-bit**
//! identical to the reference per-instruction device fan-out
//! (`PsxCore::step_cpu_naive`, selected via `set_scheduler_enabled(false)`).
//!
//! Both strategies are driven with the *same* workload — a device-exercising
//! MIPS loop plus a fuzzed stream of device-configuration pokes and step
//! commands — and after every step their full observable state is compared:
//! CPU registers/PC/cycles/cop0, `I_STAT`/`I_MASK`, the generated audio stream,
//! and (at the end) a complete save-state snapshot (RAM + every device's
//! internal state). Any divergence in interrupt timing, device state, or cycle
//! accounting fails the test.

use proptest::prelude::*;
use psoxide_config::disc::{SECTOR_RAW, parse_cue};
use psoxide_core::{Command, CpuSnapshot, PsxCore};
use psoxide_test_harness::Harness;

// ---- device register map (physical) --------------------------------------

const I_MASK: u32 = 0x1F80_1074;
const T0_MODE: u32 = 0x1F80_1104;
const T0_TARGET: u32 = 0x1F80_1108;
const T1_MODE: u32 = 0x1F80_1114;
const T2_MODE: u32 = 0x1F80_1124;
const T2_TARGET: u32 = 0x1F80_1128;
const CD_INDEX: u32 = 0x1F80_1800;
const CD_CMD: u32 = 0x1F80_1801;
const SPUCNT: u32 = 0x1F80_1DAA;
const SPU_IRQ_ADDR: u32 = 0x1F80_1DA4;
const SPU_XFER_ADDR: u32 = 0x1F80_1DA6;
const SPU_XFER_FIFO: u32 = 0x1F80_1DA8;
const SIO_CTRL: u32 = 0x1F80_104A;
const SIO_TXDATA: u32 = 0x1F80_1040;

/// A device-exercising self-loop staged at `PROGRAM_BASE` (KUSEG address 0).
///
/// `$8` holds the I/O base `0x1F80_0000`; the loop performs I/O reads
/// (`I_STAT`, timer0 value, CD status, an SPU register), an I/O write (SPU main
/// volume), a scratchpad store, and an ALU op, so it visits both the
/// device-register catch-up path and the plain-memory path every iteration.
fn device_loop_program() -> [u32; 10] {
    [
        0x3C08_1F80, // lui   $8, 0x1F80
        0x3409_0000, // ori   $9, $0, 0
        // loop (index 2, address 0x08):
        0x8D0A_1070, // lw    $10, 0x1070($8)   ; I_STAT (io read)
        0x910D_1800, // lbu   $13, 0x1800($8)   ; CD status (io read)
        0x950C_1C00, // lhu   $12, 0x1C00($8)   ; SPU voice0 vol L (io read)
        0x2529_0001, // addiu $9, $9, 1
        0xA509_1D80, // sh    $9, 0x1D80($8)    ; SPU main vol L (io write)
        0xAD09_0000, // sw    $9, 0x0000($8)    ; scratchpad store (mem)
        0x0800_0002, // j     0x08              ; back to loop
        0x0000_0000, // nop                     ; delay slot
    ]
}

/// Builds a harness staged with the device loop, with the scheduler either on
/// (the default, lazy path) or off (the reference per-instruction path).
fn staged(scheduler: bool) -> Harness {
    let mut h = Harness::new();
    h.load_program(&device_loop_program());
    h.core_mut().set_scheduler_enabled(scheduler);
    h
}

/// A single fuzzed operation applied identically to both cores.
#[derive(Debug, Clone)]
enum Op {
    /// Run `n` CPU instructions.
    Step(u16),
    /// Arm timer 0 (sysclk, div 1) to fire on `target`, repeating.
    ArmTimer0(u16),
    /// Arm timer 1 to fire on 0xFFFF overflow, repeating.
    ArmTimer1Overflow,
    /// Arm timer 2 (sysclk/8) to fire on `target`, repeating.
    ArmTimer2(u16),
    /// Enable the SPU (so it generates samples).
    SpuEnable,
    /// Arm and trigger an SPU address-match IRQ via a matching transfer.
    SpuTransferIrq(u16),
    /// Enable SIO ACK interrupts and transmit `byte` (schedules an ACK).
    SioTx(u8),
    /// Issue a CD-ROM Getstat command (schedules an INT3).
    CdGetstat,
    /// Unmask interrupt lines in `I_MASK` (does not enable CPU delivery).
    UnmaskIrq(u16),
    /// Set controller-port-0 buttons.
    SetButtons(u16),
    /// Advance one full frame (`StepFrame`).
    Frame,
}

fn apply(h: &mut Harness, op: &Op) {
    let core = h.core_mut();
    match *op {
        Op::Step(n) => {
            for _ in 0..n {
                let _ = core.execute(Command::StepCpu);
            }
        }
        Op::ArmTimer0(target) => {
            core.store32(T0_MODE, (1 << 4) | (1 << 6)); // irq-on-target, repeat
            core.store32(T0_TARGET, u32::from(target));
        }
        Op::ArmTimer1Overflow => {
            core.store32(T1_MODE, (1 << 5) | (1 << 6)); // irq-on-overflow, repeat
        }
        Op::ArmTimer2(target) => {
            core.store32(T2_MODE, (1 << 4) | (1 << 6) | (2 << 8)); // div8
            core.store32(T2_TARGET, u32::from(target));
        }
        Op::SpuEnable => {
            core.store16(SPUCNT, 0x8000);
        }
        Op::SpuTransferIrq(addr) => {
            let a = u32::from(addr) & 0x0FFF; // keep within SPU RAM
            core.store16(SPUCNT, 0x8000 | (1 << 6)); // enable + IRQ enable
            core.store16(SPU_IRQ_ADDR, a as u16);
            core.store16(SPU_XFER_ADDR, a as u16);
            core.store16(SPU_XFER_FIFO, 0x1234); // matching write -> arms the IRQ
        }
        Op::SioTx(byte) => {
            core.store16(SIO_CTRL, 0x1003); // TXEN | /DTR | ACK-IEN
            core.store8(SIO_TXDATA, byte);
        }
        Op::CdGetstat => {
            core.store8(CD_INDEX, 0); // index 0
            core.store8(CD_CMD, 0x01); // Getstat
        }
        Op::UnmaskIrq(mask) => {
            core.store32(I_MASK, u32::from(mask));
        }
        Op::SetButtons(buttons) => {
            let _ = core.execute(Command::SetControllerState { port: 0, buttons });
        }
        Op::Frame => {
            let _ = core.execute(Command::StepFrame);
        }
    }
}

/// Cheap per-step observable state: CPU snapshot (regs/PC/cycles/cop0/hi/lo),
/// `I_STAT`, `I_MASK`, and the drained audio stream. Flushes the lazy devices
/// first so the scheduled and reference cores are compared at the same
/// rest-consistent point.
fn observe(core: &mut PsxCore) -> (CpuSnapshot, u32, u32, Vec<i16>) {
    core.sync_devices();
    let cpu = core.cpu_snapshot();
    let stat = core.irq().read_stat();
    let mask = core.irq().read_mask();
    let audio = core.drain_audio();
    (cpu, stat, mask, audio)
}

fn assert_observations_equal(a: &mut PsxCore, b: &mut PsxCore, ctx: &str) {
    let (ca, sa, ma, aa) = observe(a);
    let (cb, sb, mb, ab) = observe(b);
    assert_eq!(ca.cycles, cb.cycles, "cpu.cycles diverged {ctx}");
    assert_eq!(ca.pc, cb.pc, "pc diverged {ctx}");
    assert_eq!(ca.regs, cb.regs, "regs diverged {ctx}");
    assert_eq!(ca.cop0, cb.cop0, "cop0 diverged {ctx}");
    assert_eq!((ca.hi, ca.lo), (cb.hi, cb.lo), "hi/lo diverged {ctx}");
    assert_eq!(sa, sb, "I_STAT diverged {ctx} (cycle {})", ca.cycles);
    assert_eq!(ma, mb, "I_MASK diverged {ctx}");
    assert_eq!(aa, ab, "audio stream diverged {ctx} (cycle {})", ca.cycles);
}

/// Full snapshot equality (RAM + every device's serialized internal state).
fn assert_snapshots_equal(a: &mut PsxCore, b: &mut PsxCore, ctx: &str) {
    let sa = a.save_state();
    let sb = b.save_state();
    assert!(sa == sb, "full save-state snapshot diverged {ctx}");
}

fn op_strategy() -> impl Strategy<Value = Op> {
    prop_oneof![
        // Weight stepping heavily so events actually elapse between pokes.
        6 => (1u16..48).prop_map(Op::Step),
        1 => (1u16..=0x0FFF).prop_map(Op::ArmTimer0),
        1 => Just(Op::ArmTimer1Overflow),
        1 => (1u16..=0x0FFF).prop_map(Op::ArmTimer2),
        1 => Just(Op::SpuEnable),
        1 => (0u16..0x0FFF).prop_map(Op::SpuTransferIrq),
        1 => any::<u8>().prop_map(Op::SioTx),
        1 => Just(Op::CdGetstat),
        1 => any::<u16>().prop_map(Op::UnmaskIrq),
        1 => any::<u16>().prop_map(Op::SetButtons),
    ]
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 96, ..ProptestConfig::default() })]

    /// The scheduled and naive strategies stay bit-identical across a fuzzed
    /// stream of device pokes and instruction batches. Interrupts are left
    /// CPU-disabled (reset `SR`), so device IRQs accumulate in `I_STAT` without
    /// being taken — this isolates and stresses the device catch-up / deadline
    /// timing that the scheduler must reproduce exactly.
    #[test]
    fn scheduled_matches_naive(ops in prop::collection::vec(op_strategy(), 1..40)) {
        let mut sched = staged(true);
        let mut naive = staged(false);
        prop_assert!(sched.core_mut().is_scheduler_enabled());
        prop_assert!(!naive.core_mut().is_scheduler_enabled());

        for (i, op) in ops.iter().enumerate() {
            apply(&mut sched, op);
            apply(&mut naive, op);
            assert_observations_equal(sched.core_mut(), naive.core_mut(), &format!("op[{i}]={op:?}"));
        }
        assert_snapshots_equal(sched.core_mut(), naive.core_mut(), "final");
    }
}

/// A hand-picked sequence that arms every device to fire, then advances a full
/// frame, checking equivalence across the `StepFrame` cycle budget and the
/// once-per-frame VBlank raise.
#[test]
fn scheduled_matches_naive_over_a_frame() {
    let mut sched = staged(true);
    let mut naive = staged(false);

    let setup = [
        Op::ArmTimer0(0x0040),
        Op::ArmTimer1Overflow,
        Op::ArmTimer2(0x0010),
        Op::SpuEnable,
        Op::SioTx(0x42),
        Op::CdGetstat,
        Op::UnmaskIrq(0xFFFF),
        Op::SpuTransferIrq(0x0080),
    ];
    for op in &setup {
        apply(&mut sched, op);
        apply(&mut naive, op);
    }
    assert_observations_equal(sched.core_mut(), naive.core_mut(), "after setup");

    for f in 0..3 {
        apply(&mut sched, &Op::Frame);
        apply(&mut naive, &Op::Frame);
        assert_observations_equal(
            sched.core_mut(),
            naive.core_mut(),
            &format!("after frame {f}"),
        );
    }
    assert_snapshots_equal(sched.core_mut(), naive.core_mut(), "final");
}

/// Taken-interrupt equivalence: with CPU interrupts enabled and a tiny RAM
/// exception handler that acknowledges `I_STAT` and returns, both strategies
/// must vector, save `EPC`, run the handler, and `rfe` at identical cycles.
/// (Correctness of the handler itself is irrelevant — the two cores execute the
/// same code, so any handler keeps them lock-step; the test asserts they stay
/// equal.)
#[test]
fn scheduled_matches_naive_with_taken_irqs() {
    fn build() -> Harness {
        let mut h = Harness::new();
        h.load_program(&device_loop_program());
        // General exception vector (BEV=0) is 0x8000_0080 => physical RAM 0x80.
        let handler = [
            0x3C1B_1F80u32, // lui  $27, 0x1F80
            0xAF60_1070,    // sw   $0, 0x1070($27)   ; ack all I_STAT
            0x401A_7000,    // mfc0 $26, $14          ; EPC
            0x0000_0000,    // nop
            0x4200_0010,    // rfe
            0x0340_0008,    // jr   $26
            0x0000_0000,    // nop  (delay slot)
        ];
        {
            let mem = h.core_mut().memory_mut();
            for (i, &w) in handler.iter().enumerate() {
                let addr = 0x80 + (i as u32) * 4;
                for (b, byte) in w.to_le_bytes().iter().enumerate() {
                    mem.write8(addr + b as u32, *byte);
                }
            }
        }
        // SR: IEc (bit0) | IM2 (bit10, hardware IP2), BEV clear => vector 0x80000080.
        h.core_mut().set_cop0(12, (1 << 0) | (1 << 10));
        h
    }

    let mut sched = build();
    let mut naive = build();
    naive.core_mut().set_scheduler_enabled(false);

    let setup = [
        Op::ArmTimer0(0x0080),
        Op::ArmTimer2(0x0020),
        Op::SioTx(0x7E),
        Op::CdGetstat,
        Op::UnmaskIrq(0xFFFF),
    ];
    for op in &setup {
        apply(&mut sched, op);
        apply(&mut naive, op);
    }

    for i in 0..400 {
        apply(&mut sched, &Op::Step(37));
        apply(&mut naive, &Op::Step(37));
        assert_observations_equal(sched.core_mut(), naive.core_mut(), &format!("chunk {i}"));
    }
    assert_snapshots_equal(sched.core_mut(), naive.core_mut(), "final");
}

/// The subtlest ordering point: while the CD-ROM is producing audio, decoded
/// CD-DA frames must reach the SPU (and be clocked into the output at the same
/// sample boundaries) exactly as the naive per-cycle bridge delivered them. The
/// scheduler forces a per-cycle catch-up whenever CD audio is active; this test
/// mounts a synthetic CD-DA disc, plays it on both strategies, and asserts the
/// drained audio streams — and the full machine state — are byte-identical.
#[test]
fn scheduled_matches_naive_cd_audio() {
    fn write_audio_disc(dir: &std::path::Path, n_sectors: usize) -> std::path::PathBuf {
        let mut data = vec![0u8; n_sectors * SECTOR_RAW];
        // A time-varying waveform so an ordering slip changes the output.
        for f in 0..(n_sectors * 588) {
            let b = f * 4;
            let l = ((f as i32 * 37) % 20000 - 10000) as i16;
            let r = ((f as i32 * 53) % 16000 - 8000) as i16;
            data[b..b + 2].copy_from_slice(&l.to_le_bytes());
            data[b + 2..b + 4].copy_from_slice(&r.to_le_bytes());
        }
        let bin = dir.join("audio.bin");
        let cue = dir.join("audio.cue");
        std::fs::write(&bin, &data).expect("write bin");
        std::fs::write(
            &cue,
            "FILE \"audio.bin\" BINARY\r\n  TRACK 01 AUDIO\r\n    INDEX 01 00:00:00\r\n",
        )
        .expect("write cue");
        cue
    }

    let dir = std::env::temp_dir().join(format!("psoxide-sched-cd-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mkdir");
    let cue = write_audio_disc(&dir, 3);

    fn run(cue: &std::path::Path, scheduler: bool, steps: usize) -> Vec<i16> {
        let disc = parse_cue(cue).expect("parse cue");
        let mut h = Harness::new();
        // Tight self-loop at PC 0 so each StepCpu ticks the CD-ROM and SPU.
        h.load_program(&[0x0800_0000, 0x0000_0000]);
        h.core_mut().set_scheduler_enabled(scheduler);
        h.load_disc(disc);
        // Enable SPU + CD dry mix, open main output and CD input volume.
        h.core_mut().store16(SPUCNT, 0x8000 | 0x0001);
        h.core_mut().store16(0x1F80_1D80, 0x3FFF); // MAIN_VOL_L
        h.core_mut().store16(0x1F80_1D82, 0x3FFF); // MAIN_VOL_R
        h.core_mut().store16(0x1F80_1DB0, 0x3FFF); // CD input vol L
        h.core_mut().store16(0x1F80_1DB2, 0x3FFF); // CD input vol R
        let _ = h.core_mut().drain_audio();
        // Setmode(CDDA), then Play(track 1).
        h.core_mut().store8(CD_INDEX, 0);
        h.core_mut().store8(0x1F80_1802, 0x01);
        h.core_mut().store8(CD_CMD, 0x0E);
        h.core_mut().store8(CD_INDEX, 0);
        h.core_mut().store8(0x1F80_1802, 0x01);
        h.core_mut().store8(CD_CMD, 0x03);
        h.run(steps);
        h.core_mut().sync_devices();
        h.core_mut().drain_audio()
    }

    // Enough instructions to cross a CD-DA sector boundary (451_584 cycles) and
    // clock decoded frames through the SPU: the spin loop runs ~1 cycle per
    // instruction, so > 900k steps clears the first boundary with margin.
    let steps = 1_000_000;
    let sched_audio = run(&cue, true, steps);
    let naive_audio = run(&cue, false, steps);

    let _ = std::fs::remove_dir_all(&dir);

    assert_eq!(
        sched_audio.len(),
        naive_audio.len(),
        "CD-DA drained sample count diverged"
    );
    assert!(
        sched_audio == naive_audio,
        "CD-DA → SPU audio stream diverged between scheduler and naive"
    );
    assert!(
        sched_audio.iter().any(|&s| s != 0),
        "expected non-silent CD-DA output (test would be vacuous otherwise)"
    );
}
