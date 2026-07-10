//! End-to-end test of the SIO0 memory-card protocol driven by a hand-assembled
//! MIPS program running on the real [`PsxCore`] bus.
//!
//! A card is inserted into slot 0 via `Command::InsertMemoryCard`. A single MIPS
//! program then drives the full serial exchange the BIOS card driver performs:
//! it asserts JOY_CTRL (TXEN | /DTR-Select), shifts a stream of TX bytes out of
//! JOY_TX_DATA one at a time, and reads the device's full-duplex response back
//! from JOY_RX_DATA after each write, storing every response byte into a scratch
//! RAM buffer (one byte per word). The test issues a **write** sector command
//! followed by a **read** of the same sector, then asserts the read-back data
//! matches what was written, the checksum is correct, and the command ended with
//! the good-sector marker `'G'` (0x47) — proving a sector round-trips through the
//! real SIO0 transfer state machine and the [`MemoryCard`] device.
//!
//! Like the pad test, the RX byte is pushed into the FIFO synchronously on the
//! TX write, so the program reads JOY_RX_DATA immediately after each write with
//! no ACK/IRQ wait; JOY_CTRL is 0x0003 (TXEN | /DTR).

use psoxide_core::Command;
use psoxide_test_harness::Harness;

// --- minimal raw-u32 MIPS encoders (same style as tests/sio0_joypad.rs) -------

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
/// Scratch RAM base where the response bytes are stored (word-spaced).
const SCRATCH: u32 = 0x0000_2000;

/// Builds a program that asserts JOY_CTRL then, for each byte in `tx_bytes`,
/// stores it to JOY_TX_DATA and reads the RX response into `SCRATCH + i*4`.
fn build_program(tx_bytes: &[u8]) -> Vec<u32> {
    let [lui_io, ori_io] = build_addr(T0, JOY_BASE);
    let mut prog = vec![
        lui_io,
        ori_io,
        // T2 = SCRATCH (0x2000 fits in 16 bits, single ORI from $zero).
        ori(T2, ZERO, SCRATCH as u16),
        // JOY_CTRL = TXEN | /DTR (Select). Halfword store to 0xBF80_104A.
        ori(T1, ZERO, 0x0003),
        sh(T1, T0, JOY_CTRL_OFF),
    ];

    for (i, &tx) in tx_bytes.iter().enumerate() {
        let off = (i as i16) * 4;
        prog.push(ori(T1, ZERO, u16::from(tx))); // T1 = tx byte
        prog.push(sb(T1, T0, 0)); // JOY_TX_DATA <- tx (one full-duplex exchange)
        prog.push(lbu(T3, T0, 0)); // T3 <- JOY_RX_DATA (load-delayed)
        prog.push(NOP); // load-delay slot before T3 is used
        prog.push(sb(T3, T2, off)); // scratch[i] <- response byte
    }
    prog.push(NOP); // trailing padding
    prog
}

/// Runs `tx_bytes` through the real SIO0 bus with a card in slot 0, returning
/// the per-byte responses read back from scratch RAM.
fn run_card_exchange(h: &mut Harness, tx_bytes: &[u8]) -> Vec<u8> {
    let prog = build_program(tx_bytes);
    h.load_program(&prog);
    // Generous instruction budget; trailing zeroed RAM runs as NOPs.
    h.run(prog.len() + 8);

    (0..tx_bytes.len())
        .map(|i| (h.read_word(SCRATCH + (i as u32) * 4) & 0xFF) as u8)
        .collect()
}

/// Protocol checksum: address MSB xor LSB xor all 128 data bytes.
fn checksum(addr: u16, sector: &[u8; 128]) -> u8 {
    let mut c = (addr >> 8) as u8 ^ (addr & 0xFF) as u8;
    for &b in sector.iter() {
        c ^= b;
    }
    c
}

/// The 138-byte write-command TX stream for `sector` at `addr`.
fn write_tx(addr: u16, sector: &[u8; 128], chk: u8) -> Vec<u8> {
    let mut tx = vec![
        0x81u8,
        0x57,
        0x00,
        0x00,
        (addr >> 8) as u8,
        (addr & 0xFF) as u8,
    ];
    tx.extend_from_slice(sector);
    tx.push(chk);
    tx.extend_from_slice(&[0x00, 0x00, 0x00]); // ack1, ack2, end
    tx
}

/// The 140-byte read-command TX stream for `addr`.
fn read_tx(addr: u16) -> Vec<u8> {
    let mut tx = vec![
        0x81u8,
        0x52,
        0x00,
        0x00,
        (addr >> 8) as u8,
        (addr & 0xFF) as u8,
    ];
    tx.extend(std::iter::repeat_n(0x00u8, 134));
    tx
}

#[test]
fn memory_card_write_then_read_round_trips_through_sio0() {
    let mut h = Harness::new();
    h.insert_memory_card(0, vec![0u8; 128 * 1024]);

    let addr = 0x0007u16;
    let mut sector = [0u8; 128];
    for (i, b) in sector.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(3) ^ 0x5C;
    }
    let chk = checksum(addr, &sector);

    // Drive the write command over the real SIO0 bus.
    let wresp = run_card_exchange(&mut h, &write_tx(addr, &sector, chk));
    assert_eq!(
        *wresp.last().unwrap(),
        0x47,
        "write should end with the good-sector marker 'G'"
    );

    // Drive the read command; assert the full-duplex responses.
    let rresp = run_card_exchange(&mut h, &read_tx(addr));
    // Layout: [8]=confirm-hi [9]=confirm-lo [10..138]=data [138]=checksum [139]=end.
    assert_eq!(rresp[8], (addr >> 8) as u8, "confirmed address MSB");
    assert_eq!(rresp[9], (addr & 0xFF) as u8, "confirmed address LSB");
    assert_eq!(
        &rresp[10..138],
        &sector[..],
        "read-back sector data must match what was written"
    );
    assert_eq!(rresp[138], chk, "read-back checksum must match");
    assert_eq!(rresp[139], 0x47, "read should end with 'G'");
}

#[test]
fn memory_card_get_id_over_sio0() {
    let mut h = Harness::new();
    h.insert_memory_card(0, vec![0u8; 128 * 1024]);

    let resp = run_card_exchange(&mut h, &[0x81, 0x53, 0, 0, 0, 0, 0, 0, 0, 0]);
    // FLAG(0x08 fresh), 5A 5D 5C 5D 04 00 00 80.
    assert_eq!(
        resp,
        vec![0xFF, 0x08, 0x5A, 0x5D, 0x5C, 0x5D, 0x04, 0x00, 0x00, 0x80],
        "Get-ID handshake mismatch"
    );
}

#[test]
fn memory_card_absent_slot_probe_gets_no_response() {
    // With no card inserted, address 0x81 gets open-bus and no ACK.
    let mut h = Harness::new();
    let resp = run_card_exchange(&mut h, &[0x81, 0x52, 0x00]);
    assert_eq!(resp[0], 0xFF, "absent card probe reads open-bus");
    // Because the first byte does not ACK, the transfer never advances; the
    // later bytes restart from the address phase and also read open-bus.
    assert!(
        resp.iter().all(|&b| b == 0xFF),
        "no card -> all open-bus responses"
    );
}

#[test]
fn memory_card_eject_removes_card() {
    let mut h = Harness::new();
    h.insert_memory_card(0, vec![0u8; 128 * 1024]);
    // Sanity: Get-ID answers while the card is present.
    let present = run_card_exchange(&mut h, &[0x81, 0x53, 0, 0, 0, 0, 0, 0, 0, 0]);
    assert_eq!(present[9], 0x80, "card answers Get-ID before eject");

    h.eject_memory_card(0);
    let gone = run_card_exchange(&mut h, &[0x81, 0x53, 0, 0, 0, 0, 0, 0, 0, 0]);
    assert!(
        gone.iter().all(|&b| b == 0xFF),
        "ejected card gives open-bus for every byte"
    );
}

#[test]
fn memory_card_persists_across_command_query_via_core() {
    // A Rust-level round trip through the core Command/Query API: write a sector
    // over the bus, then read the whole image back via CoreQuery::MemoryCard and
    // confirm the bytes landed and the dirty flag is set.
    use psoxide_core::{CoreQuery, QueryResult};

    let mut h = Harness::new();
    h.insert_memory_card(0, vec![0u8; 128 * 1024]);

    let addr = 0x0002u16;
    let sector = [0xA7u8; 128];
    let chk = checksum(addr, &sector);
    let wresp = run_card_exchange(&mut h, &write_tx(addr, &sector, chk));
    assert_eq!(*wresp.last().unwrap(), 0x47);

    match h.core_mut().query(CoreQuery::MemoryCard { slot: 0 }) {
        QueryResult::MemoryCard {
            present,
            data,
            dirty,
        } => {
            assert!(present, "card present");
            assert!(dirty, "card marked dirty after write");
            let base = addr as usize * 128;
            assert_eq!(&data[base..base + 128], &sector[..], "image reflects write");
        }
        other => panic!("unexpected query result: {other:?}"),
    }

    // Clearing the dirty flag then re-querying reports clean.
    h.core_mut()
        .execute(Command::ClearMemoryCardDirty { slot: 0 })
        .unwrap();
    match h.core_mut().query(CoreQuery::MemoryCard { slot: 0 }) {
        QueryResult::MemoryCard { dirty, .. } => assert!(!dirty, "dirty cleared"),
        other => panic!("unexpected query result: {other:?}"),
    }
}
