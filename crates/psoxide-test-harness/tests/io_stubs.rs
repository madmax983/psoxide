//! End-to-end tests that exercise the memory-control / cache-control / SIO0
//! stubs (plus the register-facing surface of the real CD-ROM and SPU
//! controllers) via a hand-assembled MIPS program running on the real
//! [`PsxCore`] bus.
//!
//! These verify that the stubs are correctly wired into
//! [`psoxide_core::api::PsxCore`] (i.e. the `CoreBus` dispatch reaches them
//! from CPU loads/stores), not just that they read back in isolation — those
//! isolation tests live inside `psoxide-core` itself.
//!
//! Each program uses `LUI/ORI` to build a base address in `$t0`, does a store
//! (`SW` / `SH` / `SB`), a load (`LW`), and moves the loaded value into `$v0`
//! for the harness to observe. Programs are staged into KUSEG main RAM at
//! address 0 and stopped once the load is retired.

use psoxide_test_harness::Harness;

/// Assemble a simple two-instruction "load an address into $reg" preamble via
/// LUI/ORI. Returns the two encoded words.
fn build_addr(reg: u32, addr: u32) -> [u32; 2] {
    let hi = (addr >> 16) as u16;
    // If the low half's top bit is set, adding it via ORI is fine (unsigned).
    let lo = addr as u16;
    let lui = (0x0F << 26) | (reg << 16) | u32::from(hi); // LUI $reg, hi
    let ori = (0x0D << 26) | (reg << 21) | (reg << 16) | u32::from(lo); // ORI $reg, $reg, lo
    [lui, ori]
}

/// Encode `SW $rt, offset($base)`.
fn sw(rt: u32, base: u32, offset: i16) -> u32 {
    (0x2B << 26) | (base << 21) | (rt << 16) | (offset as u16 as u32)
}

/// Encode `SH $rt, offset($base)`.
fn sh(rt: u32, base: u32, offset: i16) -> u32 {
    (0x29 << 26) | (base << 21) | (rt << 16) | (offset as u16 as u32)
}

/// Encode `LW $rt, offset($base)`.
fn lw(rt: u32, base: u32, offset: i16) -> u32 {
    (0x23 << 26) | (base << 21) | (rt << 16) | (offset as u16 as u32)
}

/// Encode `LHU $rt, offset($base)`.
fn lhu(rt: u32, base: u32, offset: i16) -> u32 {
    (0x25 << 26) | (base << 21) | (rt << 16) | (offset as u16 as u32)
}

/// Encode `LBU $rt, offset($base)`.
fn lbu(rt: u32, base: u32, offset: i16) -> u32 {
    (0x24 << 26) | (base << 21) | (rt << 16) | (offset as u16 as u32)
}

/// Encode `ORI $rt, $rs, imm`.
fn ori(rt: u32, rs: u32, imm: u16) -> u32 {
    (0x0D << 26) | (rs << 21) | (rt << 16) | u32::from(imm)
}

/// Encode `LUI $rt, imm`.
fn lui(rt: u32, imm: u16) -> u32 {
    (0x0F << 26) | (rt << 16) | u32::from(imm)
}

/// $t0..$t9 register indices.
const T0: u32 = 8;
const T1: u32 = 9;
const V0: u32 = 2;

/// Runs `n` instructions and returns `$v0`.
fn drive(program: &[u32], n: usize) -> u32 {
    let mut h = Harness::new();
    h.load_program(program);
    h.run(n);
    h.reg(V0 as usize)
}

#[test]
fn memory_control_write_readback_via_bus() {
    // t0 = 0xBFC0_1000... wait, memctrl is at 0x1F80_1000. Use KSEG1 alias
    // 0xBF80_1000 so the CPU can touch I/O without cache-mapping concerns.
    let [lui0, ori0] = build_addr(T0, 0xBF80_1000);
    // t1 = 0xDEAD_BEEF.
    let lui1 = lui(T1, 0xDEAD);
    let ori1 = ori(T1, T1, 0xBEEF);
    // sw t1, 8(t0)      ; write memctrl[+8]
    // lw v0, 8(t0)      ; read back
    let prog = [lui0, ori0, lui1, ori1, sw(T1, T0, 8), lw(V0, T0, 8), 0];
    let v = drive(&prog, prog.len());
    assert_eq!(v, 0xDEAD_BEEF);
}

#[test]
fn ram_size_readback_via_bus() {
    // t0 = 0xBF80_1060 (RAM_SIZE).
    let [lui0, ori0] = build_addr(T0, 0xBF80_1060);
    // lw v0, 0(t0)      ; default is 0x0B88.
    let prog = [lui0, ori0, lw(V0, T0, 0), 0];
    let v = drive(&prog, prog.len());
    assert_eq!(v, 0x0000_0B88);
}

#[test]
fn cache_control_write_readback_via_bus() {
    // t0 = 0xFFFE_0130.
    let [lui0, ori0] = build_addr(T0, 0xFFFE_0130);
    // t1 = 0x0001_E988 (BIOS-programmed cache-enable value).
    let lui1 = lui(T1, 0x0001);
    let ori1 = ori(T1, T1, 0xE988);
    let prog = [lui0, ori0, lui1, ori1, sw(T1, T0, 0), lw(V0, T0, 0), 0];
    let v = drive(&prog, prog.len());
    assert_eq!(v, 0x0001_E988);
}

#[test]
fn sio0_status_via_bus_reports_tx_ready() {
    // t0 = 0xBF80_1044 (SIO_STAT).
    let [lui0, ori0] = build_addr(T0, 0xBF80_1044);
    let prog = [lui0, ori0, lhu(V0, T0, 0), 0];
    let v = drive(&prog, prog.len());
    // Bit 0 (TX ready) and bit 2 (TX empty) must be set; bit 1 (RX not empty)
    // must be clear (no controller attached).
    assert_ne!(v & 0x1, 0);
    assert_ne!(v & 0x4, 0);
    assert_eq!(v & 0x2, 0);
}

#[test]
fn sio0_rx_via_bus_reads_bus_idle() {
    // t0 = 0xBF80_1040 (SIO_RX).
    let [lui0, ori0] = build_addr(T0, 0xBF80_1040);
    let prog = [lui0, ori0, lbu(V0, T0, 0), 0];
    let v = drive(&prog, prog.len());
    assert_eq!(v, 0xFF);
}

#[test]
fn cdrom_status_via_bus_not_busy() {
    // t0 = 0xBF80_1800 (CDROM status/index).
    let [lui0, ori0] = build_addr(T0, 0xBF80_1800);
    let prog = [lui0, ori0, lbu(V0, T0, 0), 0];
    let v = drive(&prog, prog.len());
    assert_eq!(v & 0x80, 0, "CD-ROM must not report busy");
}

#[test]
fn spu_register_write_readback_via_bus() {
    // t0 = 0xBF80_1C00 (SPU base).
    let [lui0, ori0] = build_addr(T0, 0xBF80_1C00);
    let lui1 = lui(T1, 0x1234);
    let ori1 = ori(T1, T1, 0x5678);
    let prog = [lui0, ori0, lui1, ori1, sw(T1, T0, 0), lw(V0, T0, 0), 0];
    let v = drive(&prog, prog.len());
    assert_eq!(v, 0x1234_5678);
}

#[test]
fn spu_status_mirrors_control_via_bus() {
    // SPUCNT (0x1F80_1DAA) is a 16-bit register. Write a value with the low
    // 6 bits distinctive, then read SPUSTAT (0x1F80_1DAE) — it mirrors those
    // bits.
    let [lui0, ori0] = build_addr(T0, 0xBF80_1DAA);
    let ori1 = ori(T1, 0, 0x8035); // t1 = 0x8035; low 6 bits = 0x35
    let prog = [
        lui0,
        ori0,
        ori1,
        sh(T1, T0, 0),  // SPUCNT = 0x8035
        lhu(V0, T0, 4), // SPUSTAT (0x1F80_1DAE = SPUCNT + 4)
        0,
    ];
    let v = drive(&prog, prog.len());
    assert_eq!(v, 0x35);
}
