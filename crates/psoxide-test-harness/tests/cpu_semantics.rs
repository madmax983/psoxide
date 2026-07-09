//! Self-contained MIPS R3000A (MIPS I) instruction-semantics oracle.
//!
//! This file builds hand-assembled MIPS programs, drives them through the
//! harness (either the PS-EXE `load_exe` + `run_hle` + TTY-HLE path, or the
//! simpler `load_program` + `run` path), and checks the computed results
//! against values derived **from the MIPS R3000A / MIPS I specification** — not
//! from this emulator. If the emulator disagrees, that indicates an interpreter
//! bug, not a wrong expectation.
//!
//! Focus is the classic bug-prone corners: load-delay hazard, branch-delay
//! slot, arithmetic overflow traps, DIV/DIVU edge cases, MULT/MULTU high word,
//! load sign/zero extension, unaligned LWL/LWR, and SLT vs SLTU signedness.

use psoxide_test_harness::Harness;

// ── Instruction assembler helpers ────────────────────────────────────────

fn rtype(rs: u32, rt: u32, rd: u32, shamt: u32, funct: u32) -> u32 {
    (rs << 21) | (rt << 16) | (rd << 11) | (shamt << 6) | funct
}
fn itype(op: u32, rs: u32, rt: u32, imm: u32) -> u32 {
    (op << 26) | (rs << 21) | (rt << 16) | (imm & 0xFFFF)
}

fn lui(rt: u32, imm: u32) -> u32 {
    itype(0x0F, 0, rt, imm)
}
fn ori(rt: u32, rs: u32, imm: u32) -> u32 {
    itype(0x0D, rs, rt, imm)
}
fn addiu(rt: u32, rs: u32, imm: u32) -> u32 {
    itype(0x09, rs, rt, imm)
}
fn addi(rt: u32, rs: u32, imm: u32) -> u32 {
    itype(0x08, rs, rt, imm)
}
fn lw(rt: u32, base: u32, off: u32) -> u32 {
    itype(0x23, base, rt, off)
}
fn sw(rt: u32, base: u32, off: u32) -> u32 {
    itype(0x2B, base, rt, off)
}
fn lb(rt: u32, base: u32, off: u32) -> u32 {
    itype(0x20, base, rt, off)
}
fn lbu(rt: u32, base: u32, off: u32) -> u32 {
    itype(0x24, base, rt, off)
}
fn lh(rt: u32, base: u32, off: u32) -> u32 {
    itype(0x21, base, rt, off)
}
fn lhu(rt: u32, base: u32, off: u32) -> u32 {
    itype(0x25, base, rt, off)
}
fn lwl(rt: u32, base: u32, off: u32) -> u32 {
    itype(0x22, base, rt, off)
}
fn lwr(rt: u32, base: u32, off: u32) -> u32 {
    itype(0x26, base, rt, off)
}
fn sb(rt: u32, base: u32, off: u32) -> u32 {
    itype(0x28, base, rt, off)
}
fn beq(rs: u32, rt: u32, off: u32) -> u32 {
    itype(0x04, rs, rt, off)
}
fn addu(rd: u32, rs: u32, rt: u32) -> u32 {
    rtype(rs, rt, rd, 0, 0x21)
}
fn subu(rd: u32, rs: u32, rt: u32) -> u32 {
    rtype(rs, rt, rd, 0, 0x23)
}
fn add(rd: u32, rs: u32, rt: u32) -> u32 {
    rtype(rs, rt, rd, 0, 0x20)
}
fn sub(rd: u32, rs: u32, rt: u32) -> u32 {
    rtype(rs, rt, rd, 0, 0x22)
}
fn slt(rd: u32, rs: u32, rt: u32) -> u32 {
    rtype(rs, rt, rd, 0, 0x2A)
}
fn sltu(rd: u32, rs: u32, rt: u32) -> u32 {
    rtype(rs, rt, rd, 0, 0x2B)
}
fn mult(rs: u32, rt: u32) -> u32 {
    rtype(rs, rt, 0, 0, 0x18)
}
fn multu(rs: u32, rt: u32) -> u32 {
    rtype(rs, rt, 0, 0, 0x19)
}
fn div(rs: u32, rt: u32) -> u32 {
    rtype(rs, rt, 0, 0, 0x1A)
}
fn divu(rs: u32, rt: u32) -> u32 {
    rtype(rs, rt, 0, 0, 0x1B)
}
fn mfhi(rd: u32) -> u32 {
    rtype(0, 0, rd, 0, 0x10)
}
fn mflo(rd: u32) -> u32 {
    rtype(0, 0, rd, 0, 0x12)
}
fn jal(target_word_index_from_base: u32) -> u32 {
    // With a base whose top nibble is 0, the absolute target address is
    // (index * 4); the encoded field is that address >> 2 == index.
    (0x03 << 26) | (target_word_index_from_base & 0x03FF_FFFF)
}
fn jr(rs: u32) -> u32 {
    rtype(rs, 0, 0, 0, 0x08)
}
const NOP: u32 = 0;

// Register name aliases (indices).
const ZERO: u32 = 0;
const A0: u32 = 4;
const S0: u32 = 16; // result base pointer
const RA: u32 = 31;

// ── PS-EXE header builder (mirrors tests/exe_loader.rs) ──────────────────

fn build_exe(pc: u32, t_addr: u32, s_addr: u32, body_words: &[u32]) -> Vec<u8> {
    let mut body: Vec<u8> = Vec::new();
    for &w in body_words {
        body.extend_from_slice(&w.to_le_bytes());
    }
    let padded = body.len().div_ceil(0x800) * 0x800;
    body.resize(padded, 0);

    let mut exe = vec![0u8; 0x800];
    exe[0..8].copy_from_slice(b"PS-X EXE");
    let put = |exe: &mut Vec<u8>, off: usize, val: u32| {
        exe[off..off + 4].copy_from_slice(&val.to_le_bytes());
    };
    put(&mut exe, 0x10, pc);
    put(&mut exe, 0x14, 0);
    put(&mut exe, 0x18, t_addr);
    put(&mut exe, 0x1C, body.len() as u32);
    put(&mut exe, 0x30, s_addr);
    put(&mut exe, 0x34, 0);

    exe.extend_from_slice(&body);
    exe
}

/// Result-region base as a KUSEG virtual address. Stores from the program to
/// this address and `Harness::read_word` from the Rust side alias the same
/// physical main-RAM bytes (`mask_region(0x2000) == 0x2000`, MainRam region),
/// which is far below the program image loaded at physical 0x0001_0000.
const RES_BASE: u32 = 0x0000_2000;
/// Scratch data word address for the load/store corner cases (KUSEG).
const DATA: u32 = 0x0000_2400;

/// Main store-to-RAM oracle: exercises the straight-line CPU corners inside a
/// side-loaded PS-EXE running through `run_hle`, and emits one TTY char so the
/// BIOS TTY-HLE path is exercised too. Each `T<slot>` result is checked against
/// a spec-derived constant on the Rust side.
#[test]
fn cpu_semantics_store_readback() {
    // Slot offsets (bytes) from $s0 == RES_BASE.
    // 0x00 load-delay: old value of dest visible to the next instruction
    // 0x04 load committed value (two instructions later)
    // 0x08 branch-delay slot executed (skipped instr did not)
    // 0x0C ADDU wrap: 0x7FFFFFFF + 1
    // 0x10 SUBU wrap: 0 - 1
    // 0x14 DIV INT_MIN / -1 -> LO
    // 0x18 DIV INT_MIN / -1 -> HI
    // 0x1C DIV 5 / 0 -> LO   (signed, dividend >= 0)
    // 0x20 DIV 5 / 0 -> HI
    // 0x24 DIV -5 / 0 -> LO  (signed, dividend < 0)
    // 0x28 DIVU 100 / 0 -> LO
    // 0x2C DIVU 100 / 0 -> HI
    // 0x30 MULT (-1)*(-1) -> HI
    // 0x34 MULT (-1)*(-1) -> LO
    // 0x38 MULTU 0xFFFFFFFF*0xFFFFFFFF -> HI
    // 0x3C MULTU 0xFFFFFFFF*0xFFFFFFFF -> LO
    // 0x40 LB  of byte 0x80 (sign-extended)
    // 0x44 LBU of byte 0x80 (zero-extended)
    // 0x48 LH  of half 0x8080 (sign-extended)
    // 0x4C LHU of half 0x8080 (zero-extended)
    // 0x50 SLT(-1, 1) signed
    // 0x54 SLTU(0xFFFFFFFF, 1) unsigned
    // 0x58 LWL/LWR unaligned assembled word
    // $s0 = RES_BASE
    let mut p: Vec<u32> = vec![ori(S0, ZERO, RES_BASE)];

    // ---- Load-delay hazard --------------------------------------------------
    // Store 0xDEADBEEF at DATA, preset dest = 0x1111, lw dest, then read dest
    // in the very next (load-delay) instruction: it must see the OLD 0x1111.
    p.push(lui(9, 0xDEAD));
    p.push(ori(9, 9, 0xBEEF)); // $t1 = 0xDEADBEEF
    p.push(ori(1, ZERO, DATA)); // $at = DATA pointer
    p.push(sw(9, 1, 0)); // mem[DATA] = 0xDEADBEEF
    p.push(ori(8, ZERO, 0x1111)); // $t0 = 0x1111 (OLD)
    p.push(lw(8, 1, 0)); // lw $t0, 0($at)  (delayed)
    p.push(addu(10, 8, ZERO)); // LOAD-DELAY SLOT: $t2 = OLD $t0 = 0x1111
    p.push(sw(10, S0, 0x00)); // slot 0x00 <- 0x1111
    p.push(sw(8, S0, 0x04)); // slot 0x04 <- committed $t0 = 0xDEADBEEF

    // ---- Branch-delay slot --------------------------------------------------
    // beq taken; delay slot sets $t3=0xAA and MUST run; the following instr is
    // skipped (would set 0xBB); branch target stores $t3.
    p.push(ori(11, ZERO, 0)); // $t3 = 0
    p.push(beq(ZERO, ZERO, 2)); // taken -> skip the 0xBB writer
    p.push(ori(11, ZERO, 0xAA)); // DELAY SLOT (runs): $t3 = 0xAA
    p.push(ori(11, ZERO, 0xBB)); // SKIPPED
    p.push(sw(11, S0, 0x08)); // branch target: slot 0x08 <- 0xAA

    // ---- ADDU / SUBU wrap (no trap) -----------------------------------------
    p.push(lui(8, 0x7FFF));
    p.push(ori(8, 8, 0xFFFF)); // $t0 = 0x7FFFFFFF
    p.push(ori(9, ZERO, 1)); // $t1 = 1
    p.push(addu(10, 8, 9)); // 0x7FFFFFFF + 1 wraps to 0x80000000
    p.push(sw(10, S0, 0x0C));
    p.push(subu(10, ZERO, 9)); // 0 - 1 wraps to 0xFFFFFFFF
    p.push(sw(10, S0, 0x10));

    // ---- DIV edge cases -----------------------------------------------------
    // INT_MIN / -1 -> LO = 0x80000000, HI = 0.
    p.push(lui(8, 0x8000)); // $t0 = 0x80000000 = INT_MIN
    p.push(addiu(9, ZERO, 0xFFFF)); // $t1 = -1 (sign-extended)
    p.push(div(8, 9));
    p.push(mflo(10));
    p.push(sw(10, S0, 0x14));
    p.push(mfhi(10));
    p.push(sw(10, S0, 0x18));
    // 5 / 0 (signed, dividend >= 0) -> LO = 0xFFFFFFFF, HI = 5.
    p.push(ori(8, ZERO, 5));
    p.push(div(8, ZERO));
    p.push(mflo(10));
    p.push(sw(10, S0, 0x1C));
    p.push(mfhi(10));
    p.push(sw(10, S0, 0x20));
    // -5 / 0 (signed, dividend < 0) -> LO = 1, HI = -5.
    p.push(addiu(8, ZERO, 0xFFFB)); // -5
    p.push(div(8, ZERO));
    p.push(mflo(10));
    p.push(sw(10, S0, 0x24));
    // DIVU 100 / 0 -> LO = 0xFFFFFFFF, HI = 100.
    p.push(ori(8, ZERO, 100));
    p.push(divu(8, ZERO));
    p.push(mflo(10));
    p.push(sw(10, S0, 0x28));
    p.push(mfhi(10));
    p.push(sw(10, S0, 0x2C));

    // ---- MULT / MULTU high word ---------------------------------------------
    // MULT (-1)*(-1) -> HI = 0, LO = 1.
    p.push(addiu(8, ZERO, 0xFFFF)); // -1
    p.push(addiu(9, ZERO, 0xFFFF)); // -1
    p.push(mult(8, 9));
    p.push(mfhi(10));
    p.push(sw(10, S0, 0x30));
    p.push(mflo(10));
    p.push(sw(10, S0, 0x34));
    // MULTU 0xFFFFFFFF * 0xFFFFFFFF -> HI = 0xFFFFFFFE, LO = 1.
    p.push(multu(8, 9)); // $t0 == $t1 == 0xFFFFFFFF from above
    p.push(mfhi(10));
    p.push(sw(10, S0, 0x38));
    p.push(mflo(10));
    p.push(sw(10, S0, 0x3C));

    // ---- Load sign/zero extension -------------------------------------------
    // Store byte 0x80 at DATA+0, half 0x8080 at DATA+4.
    p.push(ori(1, ZERO, DATA));
    p.push(ori(9, ZERO, 0x80));
    p.push(sb(9, 1, 0)); // mem[DATA] = 0x80
    p.push(lui(9, 0x8080));
    p.push(sw(9, 1, 4)); // mem[DATA+4] = 0x80800000; low half at +4 is 0x0000
    // Put 0x8080 into the low halfword at DATA+8 for LH/LHU.
    p.push(ori(9, ZERO, 0x8080));
    p.push(sw(9, 1, 8)); // mem[DATA+8] low half = 0x8080
    p.push(lb(10, 1, 0)); // LB 0x80 -> 0xFFFFFF80
    p.push(NOP); // load-delay
    p.push(sw(10, S0, 0x40));
    p.push(lbu(10, 1, 0)); // LBU 0x80 -> 0x00000080
    p.push(NOP);
    p.push(sw(10, S0, 0x44));
    p.push(lh(10, 1, 8)); // LH 0x8080 -> 0xFFFF8080
    p.push(NOP);
    p.push(sw(10, S0, 0x48));
    p.push(lhu(10, 1, 8)); // LHU 0x8080 -> 0x00008080
    p.push(NOP);
    p.push(sw(10, S0, 0x4C));

    // ---- SLT vs SLTU --------------------------------------------------------
    p.push(addiu(8, ZERO, 0xFFFF)); // -1 / 0xFFFFFFFF
    p.push(ori(9, ZERO, 1)); // 1
    p.push(slt(10, 8, 9)); // signed: -1 < 1 -> 1
    p.push(sw(10, S0, 0x50));
    p.push(sltu(10, 8, 9)); // unsigned: 0xFFFFFFFF < 1 -> 0
    p.push(sw(10, S0, 0x54));

    // ---- Unaligned LWL/LWR --------------------------------------------------
    // Store aligned word 0x11223344 at DATA+16, then read it byte-misaligned:
    // LWR at DATA+16 then LWL at DATA+19 reconstruct the full word 0x11223344.
    p.push(ori(1, ZERO, DATA));
    p.push(lui(9, 0x1122));
    p.push(ori(9, 9, 0x3344)); // $t1 = 0x11223344
    p.push(sw(9, 1, 16)); // mem[DATA+16] = 0x11223344 (aligned)
    p.push(ori(10, ZERO, 0)); // clear dest
    p.push(lwr(10, 1, 16)); // LWR at offset 0 -> dest = whole word
    p.push(lwl(10, 1, 19)); // LWL at offset 3 -> merges high bytes (chains)
    p.push(NOP); // commit load delay
    p.push(sw(10, S0, 0x58));

    // ---- Emit one TTY char via BIOS B0 std_out_putchar (func 0x3D) ----------
    p.push(ori(10, ZERO, 0xB0)); // $t2 = B-table
    p.push(ori(9, ZERO, 0x3D)); // $t1 = std_out_putchar
    p.push(ori(A0, ZERO, 0x4B)); // $a0 = 'K'
    p.push(rtype(10, 0, RA, 0, 0x09)); // jalr $ra, $t2
    p.push(NOP); // delay slot

    // ---- Return to sentinel -------------------------------------------------
    p.push(ori(RA, ZERO, 0)); // restore $ra = sentinel 0
    p.push(jr(RA)); // jr $ra -> stop
    p.push(NOP); // delay slot

    let mut h = Harness::new();
    let exe = build_exe(0x8001_0000, 0x8001_0000, 0x801F_FFF0, &p);
    h.load_exe(&exe).expect("load_exe");
    let steps = h.run_hle(5000);
    assert!(
        steps < 5000,
        "program should return via sentinel (ran {steps})"
    );

    // Spec-derived expectations. Each cites the governing MIPS I / R3000A rule.
    let r = |off: u32| h.read_word(RES_BASE.wrapping_add(off));

    // Load-delay: MIPS I load delay slot — the instruction immediately
    // following a load must not see the loaded value; it reads the OLD reg.
    assert_eq!(r(0x00), 0x1111, "load-delay: next instr must read OLD dest");
    assert_eq!(r(0x04), 0xDEAD_BEEF, "load committed value visible later");

    // Branch-delay: the instruction after a taken branch always executes;
    // the branch target = (delay-slot PC) + (signext(imm) << 2).
    assert_eq!(
        r(0x08),
        0xAA,
        "branch-delay slot ran; skipped instr did not"
    );

    // Unsigned add/sub wrap silently (no overflow trap).
    assert_eq!(r(0x0C), 0x8000_0000, "ADDU 0x7FFFFFFF+1 wraps");
    assert_eq!(r(0x10), 0xFFFF_FFFF, "SUBU 0-1 wraps");

    // DIV: R3000-defined results (no trap). INT_MIN / -1 -> LO=INT_MIN, HI=0.
    assert_eq!(r(0x14), 0x8000_0000, "DIV INT_MIN/-1 LO=INT_MIN");
    assert_eq!(r(0x18), 0x0000_0000, "DIV INT_MIN/-1 HI=0");
    // x/0 signed, x>=0 -> LO=-1, HI=x.
    assert_eq!(r(0x1C), 0xFFFF_FFFF, "DIV 5/0 LO=-1");
    assert_eq!(r(0x20), 5, "DIV 5/0 HI=dividend");
    // x/0 signed, x<0 -> LO=1, HI=x.
    assert_eq!(r(0x24), 1, "DIV -5/0 LO=1");
    // DIVU x/0 -> LO=0xFFFFFFFF, HI=x.
    assert_eq!(r(0x28), 0xFFFF_FFFF, "DIVU 100/0 LO=0xFFFFFFFF");
    assert_eq!(r(0x2C), 100, "DIVU 100/0 HI=dividend");

    // MULT signed: (-1)*(-1) = 1 -> HI=0, LO=1.
    assert_eq!(r(0x30), 0, "MULT (-1)*(-1) HI=0");
    assert_eq!(r(0x34), 1, "MULT (-1)*(-1) LO=1");
    // MULTU unsigned: 0xFFFFFFFF^2 = 0xFFFFFFFE00000001.
    assert_eq!(r(0x38), 0xFFFF_FFFE, "MULTU HI=0xFFFFFFFE");
    assert_eq!(r(0x3C), 1, "MULTU LO=1");

    // Load extension: LB/LH sign-extend, LBU/LHU zero-extend.
    assert_eq!(r(0x40), 0xFFFF_FF80, "LB 0x80 sign-extends");
    assert_eq!(r(0x44), 0x0000_0080, "LBU 0x80 zero-extends");
    assert_eq!(r(0x48), 0xFFFF_8080, "LH 0x8080 sign-extends");
    assert_eq!(r(0x4C), 0x0000_8080, "LHU 0x8080 zero-extends");

    // SLT signed vs SLTU unsigned.
    assert_eq!(r(0x50), 1, "SLT(-1,1) signed = 1");
    assert_eq!(r(0x54), 0, "SLTU(0xFFFFFFFF,1) unsigned = 0");

    // Unaligned LWL/LWR reconstruct the stored aligned word.
    assert_eq!(r(0x58), 0x1122_3344, "LWR+LWL reconstruct 0x11223344");

    // TTY-HLE path produced the emitted char.
    assert_eq!(h.tty(), "K", "std_out_putchar HLE captured 'K'");
}

/// Overflow-trap semantics for the trapping arithmetic ops. Uses the bounded
/// `load_program` + `run` path so the exception (which vectors away) does not
/// derail the checks; we inspect the CAUSE ExcCode and the destination
/// register directly. Spec: ADD/ADDI/SUB signal Integer Overflow (ExcCode
/// 0x0C) and leave the destination UNCHANGED.
#[test]
fn overflow_traps_preserve_dest() {
    const EXC_OVERFLOW: u32 = 0x0C;
    let cause_code = |h: &Harness| (h.registers().cop0[13] >> 2) & 0x1F;

    // ADD 0x7FFFFFFF + 1 traps; dest ($t2) must stay at its preset sentinel.
    // lui $t0,0x7FFF; ori $t0,$t0,0xFFFF; addiu $t1,$0,1;
    // ori $t2,$0,0x5555 (sentinel); add $t2,$t0,$t1 (traps)
    let mut h = Harness::new();
    h.load_program(&[
        lui(8, 0x7FFF),
        ori(8, 8, 0xFFFF),
        addiu(9, ZERO, 1),
        ori(10, ZERO, 0x5555),
        add(10, 8, 9),
    ]);
    h.run(5);
    assert_eq!(h.reg(10), 0x5555, "ADD overflow must not write dest");
    assert_eq!(cause_code(&h), EXC_OVERFLOW, "ADD overflow -> ExcCode 0x0C");

    // ADDU of the same operands wraps and DOES write.
    let mut h = Harness::new();
    h.load_program(&[
        lui(8, 0x7FFF),
        ori(8, 8, 0xFFFF),
        addiu(9, ZERO, 1),
        ori(10, ZERO, 0x5555),
        addu(10, 8, 9),
    ]);
    h.run(5);
    assert_eq!(h.reg(10), 0x8000_0000, "ADDU wraps to 0x80000000");

    // ADDI 0x7FFFFFFF + 1 traps; dest unchanged.
    let mut h = Harness::new();
    h.load_program(&[
        lui(8, 0x7FFF),
        ori(8, 8, 0xFFFF),
        ori(10, ZERO, 0x5555),
        addi(10, 8, 1),
    ]);
    h.run(4);
    assert_eq!(h.reg(10), 0x5555, "ADDI overflow must not write dest");
    assert_eq!(
        cause_code(&h),
        EXC_OVERFLOW,
        "ADDI overflow -> ExcCode 0x0C"
    );

    // SUB INT_MIN - 1 traps; dest unchanged.
    // lui $t0,0x8000 ($t0=INT_MIN); addiu $t1,$0,1; ori $t2,$0,0x5555; sub $t2,$t0,$t1
    let mut h = Harness::new();
    h.load_program(&[
        lui(8, 0x8000),
        addiu(9, ZERO, 1),
        ori(10, ZERO, 0x5555),
        sub(10, 8, 9),
    ]);
    h.run(4);
    assert_eq!(h.reg(10), 0x5555, "SUB overflow must not write dest");
    assert_eq!(cause_code(&h), EXC_OVERFLOW, "SUB overflow -> ExcCode 0x0C");
}

/// JAL / JALR link address = address of the jump + 8 (the instruction AFTER
/// the delay slot). Uses base-0 `load_program` so JAL's absolute target field
/// equals the word index.
#[test]
fn jal_jalr_link_is_jump_plus_8() {
    // 0: jal 3 (target index 3)   link $ra should be 0x0 + 8 = 0x8
    // 1: nop  (delay slot, runs)
    // 2: ori $t0,$0,0xBB          (would run only if we fell through; skipped)
    // 3: jr $ra ; 4: nop
    let mut h = Harness::new();
    h.load_program(&[jal(3), NOP, ori(8, ZERO, 0xBB), jr(RA), NOP]);
    h.run(2); // jal + delay slot commits $ra
    assert_eq!(h.reg(RA as usize), 0x8, "JAL link = jal_addr + 8");

    // JALR: $t3 = 0x14 (index 5); jalr $ra,$t3 at index 0 -> $ra = 0x8.
    // 0: jalr $ra,$t3 ; 1: nop ; ... ; 5: jr $ra ; 6: nop
    // Set up $t3 first, so the jalr is at index 2.
    // 0: lui $t3,0 ; 1: ori $t3,$t3,0x18 (index 6) ; 2: jalr $ra,$t3 ; 3: nop
    let mut h = Harness::new();
    h.load_program(&[
        lui(11, 0),
        ori(11, 11, 0x18),         // $t3 = 0x18 = index 6
        rtype(11, 0, RA, 0, 0x09), // jalr $ra,$t3  (at addr 0x8)
        NOP,
        NOP,
        NOP,
        jr(RA), // index 6 (addr 0x18)
        NOP,
    ]);
    h.run(4); // lui, ori, jalr, delay
    // jalr sits at address 0x8, so link = 0x8 + 8 = 0x10.
    assert_eq!(h.reg(RA as usize), 0x10, "JALR link = jalr_addr + 8");
}
