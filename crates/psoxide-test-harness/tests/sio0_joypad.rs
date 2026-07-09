//! End-to-end test of the SIO0 digital-pad handshake driven by a hand-assembled
//! MIPS program running on the real [`PsxCore`] bus.
//!
//! The program performs a full controller poll exactly the way a game's pad
//! driver does: it asserts JOY_CTRL (TXEN | /DTR-Select), then shifts the five
//! standard digital-pad exchange bytes (0x01 address, 0x42 "read command", and
//! three padding bytes) out of JOY_TX_DATA, reading the device's full-duplex
//! response back from JOY_RX_DATA after each write. The five responses are
//!
//!   [0xFF, 0x41, 0x5A, buttons_lo, buttons_hi]
//!
//! where 0x41/0x5A are the low/high bytes of the digital-pad ID word 0x5A41 and
//! the two button bytes are the *active-low* (inverted) pressed-button bitfield.
//! Each response byte is stored into a distinct scratch RAM word so the harness
//! can read the whole sequence back and assert it exactly — proving the button
//! state fed in via `Command::SetControllerState` travels all the way through
//! the SIO0 transfer state machine and comes back on the bus.
//!
//! The RX byte is pushed into the FIFO synchronously on the TX write, so the
//! program reads JOY_RX_DATA immediately after each JOY_TX_DATA write (no ACK/
//! IRQ wait). JOY_CTRL is 0x0003 (TXEN | /DTR) — the FIFO-read path does not
//! need the ack-interrupt-enable bit.

use psoxide_core::Command;
use psoxide_test_harness::Harness;

// --- minimal raw-u32 MIPS encoders (same style as tests/io_stubs.rs) ---------

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
/// Scratch RAM base where the five response bytes are stored (word-spaced).
const SCRATCH: u32 = 0x0000_1000;

/// Builds the digital-pad-read program: writes JOY_CTRL, then for each of the
/// five exchange bytes stores the TX byte and reads the RX response into a
/// distinct scratch RAM word (`SCRATCH + i*4`, low byte = response).
fn build_program() -> Vec<u32> {
    let [lui_io, ori_io] = build_addr(T0, JOY_BASE);
    let mut prog = vec![
        lui_io,
        ori_io,
        // T2 = SCRATCH (fits in 16 bits, so a single ORI from $zero).
        ori(T2, ZERO, SCRATCH as u16),
        // JOY_CTRL = TXEN | /DTR (Select). Halfword store to 0xBF80_104A.
        ori(T1, ZERO, 0x0003),
        sh(T1, T0, JOY_CTRL_OFF),
    ];

    // The five bytes the CPU shifts out for a standard digital-pad read.
    let tx_bytes = [0x01u16, 0x42, 0x00, 0x00, 0x00];
    for (i, &tx) in tx_bytes.iter().enumerate() {
        let off = (i as i16) * 4;
        prog.push(ori(T1, ZERO, tx)); // T1 = tx byte
        prog.push(sb(T1, T0, 0)); // JOY_TX_DATA <- tx (one full-duplex exchange)
        prog.push(lbu(T3, T0, 0)); // T3 <- JOY_RX_DATA (load-delayed)
        prog.push(NOP); // load-delay slot before T3 is used
        prog.push(sb(T3, T2, off)); // scratch[i] <- response byte
    }
    prog.push(NOP); // trailing padding
    prog
}

/// Loads and runs the pad-read program with `buttons` held on port 0, then
/// returns the five response bytes read back from scratch RAM.
fn run_pad_read(buttons: u16) -> [u8; 5] {
    let mut h = Harness::new();
    h.core_mut()
        .execute(Command::SetControllerState { port: 0, buttons })
        .expect("set controller state");

    let prog = build_program();
    h.load_program(&prog);
    // One StepCpu per instruction; a comfortable margin past the program length
    // (trailing RAM is zeroed = NOPs, so over-running is harmless).
    h.run(prog.len() + 8);

    let mut out = [0u8; 5];
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = (h.read_word(SCRATCH + (i as u32) * 4) & 0xFF) as u8;
    }
    out
}

#[test]
fn digital_pad_read_no_buttons() {
    // No buttons held: both button bytes are the inverted (all-released) value
    // 0xFF. The full handshake sequence is [0xFF, 0x41, 0x5A, 0xFF, 0xFF].
    let resp = run_pad_read(0x0000);
    assert_eq!(
        resp,
        [0xFF, 0x41, 0x5A, 0xFF, 0xFF],
        "idle digital-pad handshake mismatch"
    );
}

#[test]
fn digital_pad_read_with_buttons() {
    // Cross (bit 14) + Start (bit 3) + Up (bit 4) held.
    let buttons: u16 = (1 << 14) | (1 << 3) | (1 << 4);

    // Button bytes are active-low (inverted). Verify the expected math against
    // the layout independently of the core:
    //   low byte  = !(buttons        & 0xFF) = !((1<<3)|(1<<4)) = !0x18 = 0xE7
    //   high byte = !((buttons >> 8)  & 0xFF) = !(1<<6)          = !0x40 = 0xBF
    // (Cross is bit 14 -> bit 6 of the high byte.)
    let expected_lo = !(buttons & 0xFF) as u8;
    let expected_hi = !((buttons >> 8) & 0xFF) as u8;
    assert_eq!(expected_lo, 0xE7, "low-byte math");
    assert_eq!(expected_hi, 0xBF, "high-byte math");

    let resp = run_pad_read(buttons);
    assert_eq!(
        resp,
        [0xFF, 0x41, 0x5A, 0xE7, 0xBF],
        "held-button digital-pad handshake mismatch"
    );
}
