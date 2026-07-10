//! End-to-end test of the DualShock / analog-pad handshake driven by a
//! hand-assembled MIPS program running on the real [`PsxCore`] bus.
//!
//! The program performs the full config-mode dance a game's analog-pad driver
//! runs to switch a DualShock into analog mode, then polls it:
//!
//!   1. enter config   : 01 43 00 01 00 00 00 00 00
//!   2. set analog on   : 01 44 00 01 00 00 00 00 00   (AA=01 => analog mode)
//!   3. exit config     : 01 43 00 00 00 00 00 00 00
//!   4. poll            : 01 42 00 00 00 00 00 00 00
//!
//! Only the poll's nine full-duplex response bytes are captured (each stored to
//! a distinct scratch RAM word). In analog mode the pad answers
//!
//!   [0xFF, 0x73, 0x5A, buttons_lo, buttons_hi, RX, RY, LX, LY]
//!
//! where 0x73/0x5A are the analog-ID word and RX/RY/LX/LY are the right/left
//! stick axes the harness set via [`Command::SetControllerSticks`]. Proving the
//! stick values travel through the SIO0 transfer state machine — after the
//! guest itself flipped the pad into analog mode — is the point of the test.
//!
//! JOY_CTRL is 0x0003 (TXEN | /DTR); the RX byte is pushed synchronously on the
//! TX write, and phase advances on the returned ACK, so no ACK/IRQ wait is
//! needed. Setup-transaction responses are simply not read back (the 8-byte RX
//! FIFO harmlessly drops them); only the poll reads each response immediately.

use psoxide_core::{Command, ControllerKind};
use psoxide_test_harness::Harness;

// --- minimal raw-u32 MIPS encoders (same style as tests/sio0_joypad.rs) ------

/// Assemble a "load a 32-bit address into `$reg`" preamble via LUI/ORI.
fn build_addr(reg: u32, addr: u32) -> [u32; 2] {
    let hi = (addr >> 16) as u16;
    let lo = addr as u16;
    let lui = (0x0F << 26) | (reg << 16) | u32::from(hi); // LUI $reg, hi
    let ori = (0x0D << 26) | (reg << 21) | (reg << 16) | u32::from(lo); // ORI $reg, $reg, lo
    [lui, ori]
}

/// Encode `SH $rt, offset($base)`.
fn sh(rt: u32, base: u32, offset: i16) -> u32 {
    (0x29 << 26) | (base << 21) | (rt << 16) | (offset as u16 as u32)
}

/// Encode `SB $rt, offset($base)`.
fn sb(rt: u32, base: u32, offset: i16) -> u32 {
    (0x28 << 26) | (base << 21) | (rt << 16) | (offset as u16 as u32)
}

/// Encode `LBU $rt, offset($base)`.
fn lbu(rt: u32, base: u32, offset: i16) -> u32 {
    (0x24 << 26) | (base << 21) | (rt << 16) | (offset as u16 as u32)
}

/// Encode `ORI $rt, $rs, imm`.
fn ori(rt: u32, rs: u32, imm: u16) -> u32 {
    (0x0D << 26) | (rs << 21) | (rt << 16) | u32::from(imm)
}

/// `NOP` (SLL $0, $0, 0) — used as a load-delay slot after each `LBU`.
const NOP: u32 = 0;

// Register indices.
const ZERO: u32 = 0;
const T0: u32 = 8; // JOY I/O base (0xBF80_1040)
const T1: u32 = 9; // TX byte / control scratch
const T2: u32 = 10; // scratch RAM base
const T3: u32 = 11; // RX byte

/// KSEG1 uncached alias of the SIO0 register window base (JOY_RX/TX_DATA).
const JOY_BASE: u32 = 0xBF80_1040;
/// JOY_CTRL offset from `JOY_BASE` (0xBF80_104A).
const JOY_CTRL_OFF: i16 = 0x0A;
/// Scratch RAM base where the nine poll response bytes are stored (word-spaced).
const SCRATCH: u32 = 0x0000_1000;

/// Emits the instructions to shift `bytes` out of JOY_TX_DATA, discarding each
/// full-duplex response (used for the config-mode setup transactions). The read
/// still happens so the RX FIFO stays drained — exactly what a real pad driver
/// does — otherwise stale responses would offset the later poll reads.
fn emit_shift_noread(prog: &mut Vec<u32>, bytes: &[u8]) {
    for &tx in bytes {
        prog.push(ori(T1, ZERO, u16::from(tx)));
        prog.push(sb(T1, T0, 0)); // JOY_TX_DATA <- tx
        prog.push(lbu(T3, T0, 0)); // drain JOY_RX_DATA (discarded)
        prog.push(NOP); // load-delay slot
    }
}

/// Emits the instructions to shift `bytes` out of JOY_TX_DATA and store each
/// full-duplex response into `SCRATCH + i*4` (used for the poll transaction).
fn emit_poll(prog: &mut Vec<u32>, bytes: &[u8]) {
    for (i, &tx) in bytes.iter().enumerate() {
        let off = (i as i16) * 4;
        prog.push(ori(T1, ZERO, u16::from(tx)));
        prog.push(sb(T1, T0, 0)); // JOY_TX_DATA <- tx (one exchange)
        prog.push(lbu(T3, T0, 0)); // T3 <- JOY_RX_DATA (load-delayed)
        prog.push(NOP); // load-delay slot before T3 is used
        prog.push(sb(T3, T2, off)); // scratch[i] <- response byte
    }
}

/// Builds the config-mode → analog → poll program.
fn build_program() -> Vec<u32> {
    let [lui_io, ori_io] = build_addr(T0, JOY_BASE);
    let mut prog = vec![
        lui_io,
        ori_io,
        // T2 = SCRATCH (fits in 16 bits).
        ori(T2, ZERO, SCRATCH as u16),
        // JOY_CTRL = TXEN | /DTR (Select).
        ori(T1, ZERO, 0x0003),
        sh(T1, T0, JOY_CTRL_OFF),
    ];

    // 1. enter config, 2. set analog on, 3. exit config.
    emit_shift_noread(&mut prog, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]);
    emit_shift_noread(&mut prog, &[0x01, 0x44, 0x00, 0x01, 0, 0, 0, 0, 0]);
    emit_shift_noread(&mut prog, &[0x01, 0x43, 0x00, 0x00, 0, 0, 0, 0, 0]);
    // 4. poll (9-byte analog poll) and capture the responses.
    emit_poll(&mut prog, &[0x01, 0x42, 0, 0, 0, 0, 0, 0, 0]);

    prog.push(NOP); // trailing padding
    prog
}

/// Loads and runs the program with an analog pad attached to port 0 and the
/// given stick axes, returning the nine poll response bytes.
fn run_analog_poll(right: (u8, u8), left: (u8, u8)) -> [u8; 9] {
    let mut h = Harness::new();
    h.core_mut()
        .execute(Command::SetControllerType {
            port: 0,
            kind: ControllerKind::Analog,
        })
        .expect("set controller type");
    h.core_mut()
        .execute(Command::SetControllerSticks {
            port: 0,
            right,
            left,
        })
        .expect("set controller sticks");

    let prog = build_program();
    h.load_program(&prog);
    h.run(prog.len() + 8);

    let mut out = [0u8; 9];
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = (h.read_word(SCRATCH + (i as u32) * 4) & 0xFF) as u8;
    }
    out
}

#[test]
fn analog_pad_config_then_poll() {
    // Distinct stick values so a swapped/duplicated axis would fail.
    let resp = run_analog_poll((0x12, 0x34), (0x56, 0x78));
    assert_eq!(
        resp,
        [0xFF, 0x73, 0x5A, 0xFF, 0xFF, 0x12, 0x34, 0x56, 0x78],
        "analog poll after guest config-mode enable mismatch"
    );
}

#[test]
fn analog_pad_poll_centered_sticks() {
    // Centre (0x80) on every axis — the power-on / released stick position.
    let resp = run_analog_poll((0x80, 0x80), (0x80, 0x80));
    assert_eq!(
        resp,
        [0xFF, 0x73, 0x5A, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80],
        "centred-stick analog poll mismatch"
    );
}
