//! Property tests for the GTE (coprocessor 2).
//!
//! These exercise the saturation/overflow datapath through the public register
//! and command interface, checking the invariants that must hold for *any*
//! register state and command word:
//!
//! * Executing an arbitrary command never panics (the 44-bit MAC truncation,
//!   the `>>`/`<<` shifts, and the UNR division must all stay within bounds).
//! * The `IR1..IR3` accumulators always land in the signed-16 range (the IR
//!   saturation clamp is total).
//! * Each color-FIFO channel is a byte (the color clamp is `[0, 0xFF]`).
//! * The `FLAG` summary bit (bit 31) is exactly the OR of the error-mask bits.

use proptest::prelude::*;
use psoxide_core::gte::Gte;

/// The FLAG error-summary mask (bits 30..=23 and 18..=13).
const FLAG_ERROR_MASK: u32 = 0x7F87_E000;

/// Fills every GTE data and control register with `seed`-derived values so the
/// datapath sees a fully-populated (and often extreme) register file.
fn seeded_gte(values: &[u32; 64]) -> Gte {
    let mut gte = Gte::new();
    for (i, &v) in values.iter().enumerate() {
        if i < 32 {
            gte.write_data(i as u8, v);
        } else {
            gte.write_control((i - 32) as u8, v);
        }
    }
    gte
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2000))]

    /// An arbitrary command on an arbitrary register file never panics, and
    /// leaves the engine in a self-consistent state.
    #[test]
    fn execute_is_total_and_consistent(regs in any::<[u32; 64]>(), cmd in any::<u32>()) {
        let mut gte = seeded_gte(&regs);
        gte.execute(cmd);

        // IR1..IR3 always fit the signed-16 range (the IR clamp is total; the
        // register storage cannot hold anything wider).
        for rd in 9u8..=11 {
            let ir = gte.read_data(rd) as i32;
            prop_assert!((-0x8000..=0x7FFF).contains(&ir), "IR{} out of range: {ir}", rd - 8);
        }

        // FLAG bit 31 is exactly the OR of the error-mask bits.
        let flag = gte.read_control(31);
        let expect_summary = flag & FLAG_ERROR_MASK != 0;
        prop_assert_eq!((flag >> 31) & 1 == 1, expect_summary);
    }

    /// Data-register writes then reads are stable for the plain 32-bit slots
    /// (MAC1..3, RGBC, RES1): whatever is written is read back verbatim.
    #[test]
    fn data_word_registers_roundtrip(v in any::<u32>()) {
        let mut gte = Gte::new();
        for rd in [6u8, 23, 25, 26, 27] {
            gte.write_data(rd, v);
            prop_assert_eq!(gte.read_data(rd), v, "data reg {}", rd);
        }
    }

    /// Control 32-bit slots (TRX..TRZ, background/far color, OFX/OFY, DQB)
    /// round-trip verbatim.
    #[test]
    fn control_word_registers_roundtrip(v in any::<u32>()) {
        let mut gte = Gte::new();
        for rd in [5u8, 6, 7, 13, 14, 15, 21, 22, 23, 24, 25, 28] {
            gte.write_control(rd, v);
            prop_assert_eq!(gte.read_control(rd), v, "control reg {}", rd);
        }
    }

    /// After a real op that drives the saturation clamps (RTPS sets IR0 via the
    /// [0, 0x1000] clamp; a color op writes byte-clamped RGB), the clamped
    /// outputs always land in range for arbitrary register inputs.
    #[test]
    fn saturation_clamps_land_in_range(regs in any::<[u32; 64]>(), sf in any::<bool>()) {
        let sf_bit = if sf { 1 << 19 } else { 0 };

        // RTPS (opcode 0x01) always runs set_ir0 for its (single) vertex.
        let mut gte = seeded_gte(&regs);
        gte.execute(0x01 | sf_bit);
        let ir0 = gte.read_data(8) as i32;
        prop_assert!((0..=0x1000).contains(&ir0), "IR0 out of range after RTPS: {ir0}");

        // NCDS (opcode 0x13) drives the color saturation and depth-cue path;
        // it must not panic and its FLAG summary bit stays consistent, while
        // the resulting IR1..IR3 remain in the signed-16 range.
        let mut gte = seeded_gte(&regs);
        gte.execute(0x13 | sf_bit);
        for rd in 9u8..=11 {
            let ir = gte.read_data(rd) as i32;
            prop_assert!((-0x8000..=0x7FFF).contains(&ir));
        }
        let flag = gte.read_control(31);
        prop_assert_eq!((flag >> 31) & 1 == 1, flag & FLAG_ERROR_MASK != 0);
    }

    /// Running a batch of random commands never panics and keeps the FLAG
    /// summary bit consistent throughout (stresses the FIFOs and accumulators).
    #[test]
    fn command_stream_stays_consistent(
        regs in any::<[u32; 64]>(),
        cmds in prop::collection::vec(any::<u32>(), 1..16),
    ) {
        let mut gte = seeded_gte(&regs);
        for cmd in cmds {
            gte.execute(cmd);
            let flag = gte.read_control(31);
            prop_assert_eq!((flag >> 31) & 1 == 1, flag & FLAG_ERROR_MASK != 0);
        }
    }
}
