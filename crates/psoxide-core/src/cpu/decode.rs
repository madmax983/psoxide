//! Pure MIPS R3000A (MIPS I) instruction decoder.
//!
//! [`decode`] is a total function from a raw 32-bit instruction word to an
//! [`Instruction`]. Every encoding maps to some variant; unrecognized
//! encodings become [`Instruction::Illegal`] rather than panicking. The
//! decoder is side-effect free and is a Verus proof target.

/// A decoded register index (0-31). Register 0 is hardwired to zero.
pub type Reg = u8;

/// A fully decoded MIPS I instruction.
///
/// Immediate fields are stored raw (`u16`); sign- or zero-extension is applied
/// by the interpreter according to each instruction's semantics. Branch and
/// load/store `imm` fields hold the raw 16-bit immediate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Instruction {
    // ── SPECIAL (shifts) ────────────────────────────────────────────────
    /// Shift left logical: `rd = rt << shamt`.
    Sll { rd: Reg, rt: Reg, shamt: u8 },
    /// Shift right logical: `rd = rt >> shamt`.
    Srl { rd: Reg, rt: Reg, shamt: u8 },
    /// Shift right arithmetic: `rd = rt >> shamt` (sign-extended).
    Sra { rd: Reg, rt: Reg, shamt: u8 },
    /// Shift left logical variable: `rd = rt << (rs & 31)`.
    Sllv { rd: Reg, rt: Reg, rs: Reg },
    /// Shift right logical variable: `rd = rt >> (rs & 31)`.
    Srlv { rd: Reg, rt: Reg, rs: Reg },
    /// Shift right arithmetic variable.
    Srav { rd: Reg, rt: Reg, rs: Reg },

    // ── SPECIAL (jumps / system) ────────────────────────────────────────
    /// Jump register: `pc = rs`.
    Jr { rs: Reg },
    /// Jump and link register: `rd = return; pc = rs`.
    Jalr { rd: Reg, rs: Reg },
    /// System call trap.
    Syscall,
    /// Breakpoint trap.
    Break,

    // ── SPECIAL (hi/lo) ─────────────────────────────────────────────────
    /// Move from HI: `rd = hi`.
    Mfhi { rd: Reg },
    /// Move to HI: `hi = rs`.
    Mthi { rs: Reg },
    /// Move from LO: `rd = lo`.
    Mflo { rd: Reg },
    /// Move to LO: `lo = rs`.
    Mtlo { rs: Reg },
    /// Signed multiply: `hi:lo = rs * rt`.
    Mult { rs: Reg, rt: Reg },
    /// Unsigned multiply.
    Multu { rs: Reg, rt: Reg },
    /// Signed divide: `lo = rs / rt`, `hi = rs % rt`.
    Div { rs: Reg, rt: Reg },
    /// Unsigned divide.
    Divu { rs: Reg, rt: Reg },

    // ── SPECIAL (ALU register) ──────────────────────────────────────────
    /// Add with overflow trap: `rd = rs + rt`.
    Add { rd: Reg, rs: Reg, rt: Reg },
    /// Add unsigned (no trap): `rd = rs + rt`.
    Addu { rd: Reg, rs: Reg, rt: Reg },
    /// Subtract with overflow trap: `rd = rs - rt`.
    Sub { rd: Reg, rs: Reg, rt: Reg },
    /// Subtract unsigned (no trap).
    Subu { rd: Reg, rs: Reg, rt: Reg },
    /// Bitwise AND: `rd = rs & rt`.
    And { rd: Reg, rs: Reg, rt: Reg },
    /// Bitwise OR: `rd = rs | rt`.
    Or { rd: Reg, rs: Reg, rt: Reg },
    /// Bitwise XOR: `rd = rs ^ rt`.
    Xor { rd: Reg, rs: Reg, rt: Reg },
    /// Bitwise NOR: `rd = !(rs | rt)`.
    Nor { rd: Reg, rs: Reg, rt: Reg },
    /// Set on less than (signed): `rd = (rs < rt) as u32`.
    Slt { rd: Reg, rs: Reg, rt: Reg },
    /// Set on less than unsigned.
    Sltu { rd: Reg, rs: Reg, rt: Reg },

    // ── BCOND (opcode 0x01) ─────────────────────────────────────────────
    /// Branch if `rs < 0`.
    Bltz { rs: Reg, imm: u16 },
    /// Branch if `rs >= 0`.
    Bgez { rs: Reg, imm: u16 },
    /// Branch if `rs < 0`, and link.
    Bltzal { rs: Reg, imm: u16 },
    /// Branch if `rs >= 0`, and link.
    Bgezal { rs: Reg, imm: u16 },

    // ── Jumps ───────────────────────────────────────────────────────────
    /// Jump to a 26-bit target within the current 256MB region.
    J { target: u32 },
    /// Jump and link (return address in `$ra`).
    Jal { target: u32 },

    // ── Branches ────────────────────────────────────────────────────────
    /// Branch if equal.
    Beq { rs: Reg, rt: Reg, imm: u16 },
    /// Branch if not equal.
    Bne { rs: Reg, rt: Reg, imm: u16 },
    /// Branch if `rs <= 0`.
    Blez { rs: Reg, imm: u16 },
    /// Branch if `rs > 0`.
    Bgtz { rs: Reg, imm: u16 },

    // ── ALU immediate ───────────────────────────────────────────────────
    /// Add immediate with overflow trap.
    Addi { rt: Reg, rs: Reg, imm: u16 },
    /// Add immediate unsigned (no trap).
    Addiu { rt: Reg, rs: Reg, imm: u16 },
    /// Set on less than immediate (signed).
    Slti { rt: Reg, rs: Reg, imm: u16 },
    /// Set on less than immediate unsigned.
    Sltiu { rt: Reg, rs: Reg, imm: u16 },
    /// AND immediate (zero-extended).
    Andi { rt: Reg, rs: Reg, imm: u16 },
    /// OR immediate (zero-extended).
    Ori { rt: Reg, rs: Reg, imm: u16 },
    /// XOR immediate (zero-extended).
    Xori { rt: Reg, rs: Reg, imm: u16 },
    /// Load upper immediate: `rt = imm << 16`.
    Lui { rt: Reg, imm: u16 },

    // ── COP0 ────────────────────────────────────────────────────────────
    /// Move from coprocessor 0: `rt = cop0[rd]`.
    Mfc0 { rt: Reg, rd: Reg },
    /// Move to coprocessor 0: `cop0[rd] = rt`.
    Mtc0 { rt: Reg, rd: Reg },
    /// Restore from exception (pops the SR mode stack).
    Rfe,

    // ── COP2 (GTE) ──────────────────────────────────────────────────────
    /// Any coprocessor-2 (GTE) operation. Decoded but not executed; the raw
    /// instruction word is retained for future implementation.
    Cop2 { raw: u32 },

    // ── Loads ───────────────────────────────────────────────────────────
    /// Load byte (sign-extended).
    Lb { rt: Reg, rs: Reg, imm: u16 },
    /// Load halfword (sign-extended).
    Lh { rt: Reg, rs: Reg, imm: u16 },
    /// Load word left (unaligned).
    Lwl { rt: Reg, rs: Reg, imm: u16 },
    /// Load word.
    Lw { rt: Reg, rs: Reg, imm: u16 },
    /// Load byte unsigned (zero-extended).
    Lbu { rt: Reg, rs: Reg, imm: u16 },
    /// Load halfword unsigned (zero-extended).
    Lhu { rt: Reg, rs: Reg, imm: u16 },
    /// Load word right (unaligned).
    Lwr { rt: Reg, rs: Reg, imm: u16 },

    // ── Stores ──────────────────────────────────────────────────────────
    /// Store byte.
    Sb { rt: Reg, rs: Reg, imm: u16 },
    /// Store halfword.
    Sh { rt: Reg, rs: Reg, imm: u16 },
    /// Store word left (unaligned).
    Swl { rt: Reg, rs: Reg, imm: u16 },
    /// Store word.
    Sw { rt: Reg, rs: Reg, imm: u16 },
    /// Store word right (unaligned).
    Swr { rt: Reg, rs: Reg, imm: u16 },

    /// An unrecognized or reserved encoding. Executing it raises a
    /// reserved-instruction exception.
    Illegal { raw: u32 },
}

// ── Field extraction helpers ────────────────────────────────────────────

#[inline]
const fn opcode(instr: u32) -> u32 {
    instr >> 26
}
#[inline]
const fn rs(instr: u32) -> Reg {
    ((instr >> 21) & 0x1F) as Reg
}
#[inline]
const fn rt(instr: u32) -> Reg {
    ((instr >> 16) & 0x1F) as Reg
}
#[inline]
const fn rd(instr: u32) -> Reg {
    ((instr >> 11) & 0x1F) as Reg
}
#[inline]
const fn shamt(instr: u32) -> u8 {
    ((instr >> 6) & 0x1F) as u8
}
#[inline]
const fn funct(instr: u32) -> u32 {
    instr & 0x3F
}
#[inline]
const fn imm(instr: u32) -> u16 {
    (instr & 0xFFFF) as u16
}
#[inline]
const fn target(instr: u32) -> u32 {
    instr & 0x03FF_FFFF
}

/// Decodes a raw 32-bit instruction word into an [`Instruction`].
///
/// This is total: every 32-bit input yields a variant, with unrecognized
/// encodings mapping to [`Instruction::Illegal`].
///
/// # Examples
///
/// ```
/// use psoxide_core::cpu::decode::{decode, Instruction};
///
/// // ADDIU $t0, $t1, 0x1234  →  opcode 0x09
/// let insn = decode(0x2528_1234);
/// assert_eq!(insn, Instruction::Addiu { rt: 8, rs: 9, imm: 0x1234 });
///
/// // LUI $t0, 0x8000  →  opcode 0x0F
/// assert_eq!(decode(0x3C08_8000), Instruction::Lui { rt: 8, imm: 0x8000 });
/// ```
#[must_use]
pub fn decode(instr: u32) -> Instruction {
    match opcode(instr) {
        0x00 => decode_special(instr),
        0x01 => decode_bcond(instr),
        0x02 => Instruction::J {
            target: target(instr),
        },
        0x03 => Instruction::Jal {
            target: target(instr),
        },
        0x04 => Instruction::Beq {
            rs: rs(instr),
            rt: rt(instr),
            imm: imm(instr),
        },
        0x05 => Instruction::Bne {
            rs: rs(instr),
            rt: rt(instr),
            imm: imm(instr),
        },
        0x06 => Instruction::Blez {
            rs: rs(instr),
            imm: imm(instr),
        },
        0x07 => Instruction::Bgtz {
            rs: rs(instr),
            imm: imm(instr),
        },
        0x08 => Instruction::Addi {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x09 => Instruction::Addiu {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x0A => Instruction::Slti {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x0B => Instruction::Sltiu {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x0C => Instruction::Andi {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x0D => Instruction::Ori {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x0E => Instruction::Xori {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x0F => Instruction::Lui {
            rt: rt(instr),
            imm: imm(instr),
        },
        0x10 => decode_cop0(instr),
        0x12 => Instruction::Cop2 { raw: instr },
        0x20 => Instruction::Lb {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x21 => Instruction::Lh {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x22 => Instruction::Lwl {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x23 => Instruction::Lw {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x24 => Instruction::Lbu {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x25 => Instruction::Lhu {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x26 => Instruction::Lwr {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x28 => Instruction::Sb {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x29 => Instruction::Sh {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x2A => Instruction::Swl {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x2B => Instruction::Sw {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        0x2E => Instruction::Swr {
            rt: rt(instr),
            rs: rs(instr),
            imm: imm(instr),
        },
        _ => Instruction::Illegal { raw: instr },
    }
}

fn decode_special(instr: u32) -> Instruction {
    match funct(instr) {
        0x00 => Instruction::Sll {
            rd: rd(instr),
            rt: rt(instr),
            shamt: shamt(instr),
        },
        0x02 => Instruction::Srl {
            rd: rd(instr),
            rt: rt(instr),
            shamt: shamt(instr),
        },
        0x03 => Instruction::Sra {
            rd: rd(instr),
            rt: rt(instr),
            shamt: shamt(instr),
        },
        0x04 => Instruction::Sllv {
            rd: rd(instr),
            rt: rt(instr),
            rs: rs(instr),
        },
        0x06 => Instruction::Srlv {
            rd: rd(instr),
            rt: rt(instr),
            rs: rs(instr),
        },
        0x07 => Instruction::Srav {
            rd: rd(instr),
            rt: rt(instr),
            rs: rs(instr),
        },
        0x08 => Instruction::Jr { rs: rs(instr) },
        0x09 => Instruction::Jalr {
            rd: rd(instr),
            rs: rs(instr),
        },
        0x0C => Instruction::Syscall,
        0x0D => Instruction::Break,
        0x10 => Instruction::Mfhi { rd: rd(instr) },
        0x11 => Instruction::Mthi { rs: rs(instr) },
        0x12 => Instruction::Mflo { rd: rd(instr) },
        0x13 => Instruction::Mtlo { rs: rs(instr) },
        0x18 => Instruction::Mult {
            rs: rs(instr),
            rt: rt(instr),
        },
        0x19 => Instruction::Multu {
            rs: rs(instr),
            rt: rt(instr),
        },
        0x1A => Instruction::Div {
            rs: rs(instr),
            rt: rt(instr),
        },
        0x1B => Instruction::Divu {
            rs: rs(instr),
            rt: rt(instr),
        },
        0x20 => Instruction::Add {
            rd: rd(instr),
            rs: rs(instr),
            rt: rt(instr),
        },
        0x21 => Instruction::Addu {
            rd: rd(instr),
            rs: rs(instr),
            rt: rt(instr),
        },
        0x22 => Instruction::Sub {
            rd: rd(instr),
            rs: rs(instr),
            rt: rt(instr),
        },
        0x23 => Instruction::Subu {
            rd: rd(instr),
            rs: rs(instr),
            rt: rt(instr),
        },
        0x24 => Instruction::And {
            rd: rd(instr),
            rs: rs(instr),
            rt: rt(instr),
        },
        0x25 => Instruction::Or {
            rd: rd(instr),
            rs: rs(instr),
            rt: rt(instr),
        },
        0x26 => Instruction::Xor {
            rd: rd(instr),
            rs: rs(instr),
            rt: rt(instr),
        },
        0x27 => Instruction::Nor {
            rd: rd(instr),
            rs: rs(instr),
            rt: rt(instr),
        },
        0x2A => Instruction::Slt {
            rd: rd(instr),
            rs: rs(instr),
            rt: rt(instr),
        },
        0x2B => Instruction::Sltu {
            rd: rd(instr),
            rs: rs(instr),
            rt: rt(instr),
        },
        _ => Instruction::Illegal { raw: instr },
    }
}

fn decode_bcond(instr: u32) -> Instruction {
    // The rt field selects the branch condition. Bit 0 = ge/lt, bit 4 = link.
    let rs = rs(instr);
    let imm = imm(instr);
    match rt(instr) {
        0x00 => Instruction::Bltz { rs, imm },
        0x01 => Instruction::Bgez { rs, imm },
        0x10 => Instruction::Bltzal { rs, imm },
        0x11 => Instruction::Bgezal { rs, imm },
        // Real hardware treats any rt with bit 0 clear as BLTZ and bit 0 set as
        // BGEZ, linking only when bits 4..1 == 0b1000. Mirror the common cases;
        // fall back to the primary decode for the rest.
        other if other & 1 == 0 => Instruction::Bltz { rs, imm },
        _ => Instruction::Bgez { rs, imm },
    }
}

fn decode_cop0(instr: u32) -> Instruction {
    // The rs field selects the COP0 operation class.
    match rs(instr) {
        0x00 => Instruction::Mfc0 {
            rt: rt(instr),
            rd: rd(instr),
        },
        0x04 => Instruction::Mtc0 {
            rt: rt(instr),
            rd: rd(instr),
        },
        // CO=1 (rs bit 4 set): coprocessor operation selected by funct.
        rs if rs & 0x10 != 0 => match funct(instr) {
            0x10 => Instruction::Rfe,
            _ => Instruction::Illegal { raw: instr },
        },
        _ => Instruction::Illegal { raw: instr },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Assembles an I-type instruction word.
    fn i_type(op: u32, rs: u32, rt: u32, imm: u16) -> u32 {
        (op << 26) | (rs << 21) | (rt << 16) | u32::from(imm)
    }
    /// Assembles an R-type (SPECIAL) instruction word.
    fn r_type(rs: u32, rt: u32, rd: u32, shamt: u32, funct: u32) -> u32 {
        (rs << 21) | (rt << 16) | (rd << 11) | (shamt << 6) | funct
    }

    #[test]
    fn decode_immediate_alu() {
        assert_eq!(
            decode(i_type(0x08, 9, 8, 0x1234)),
            Instruction::Addi {
                rt: 8,
                rs: 9,
                imm: 0x1234
            }
        );
        assert_eq!(
            decode(i_type(0x09, 9, 8, 0x1234)),
            Instruction::Addiu {
                rt: 8,
                rs: 9,
                imm: 0x1234
            }
        );
        assert_eq!(
            decode(i_type(0x0A, 9, 8, 0xFFFF)),
            Instruction::Slti {
                rt: 8,
                rs: 9,
                imm: 0xFFFF
            }
        );
        assert_eq!(
            decode(i_type(0x0B, 9, 8, 1)),
            Instruction::Sltiu {
                rt: 8,
                rs: 9,
                imm: 1
            }
        );
        assert_eq!(
            decode(i_type(0x0C, 9, 8, 0xFF)),
            Instruction::Andi {
                rt: 8,
                rs: 9,
                imm: 0xFF
            }
        );
        assert_eq!(
            decode(i_type(0x0D, 9, 8, 0xFF)),
            Instruction::Ori {
                rt: 8,
                rs: 9,
                imm: 0xFF
            }
        );
        assert_eq!(
            decode(i_type(0x0E, 9, 8, 0xFF)),
            Instruction::Xori {
                rt: 8,
                rs: 9,
                imm: 0xFF
            }
        );
        assert_eq!(
            decode(i_type(0x0F, 0, 8, 0x8000)),
            Instruction::Lui { rt: 8, imm: 0x8000 }
        );
    }

    #[test]
    fn decode_jumps_and_branches() {
        assert_eq!(
            decode(0x0800_0000 | 0x1234),
            Instruction::J { target: 0x1234 }
        );
        assert_eq!(
            decode(0x0C00_0000 | 0x1234),
            Instruction::Jal { target: 0x1234 }
        );
        assert_eq!(
            decode(i_type(0x04, 8, 9, 0x10)),
            Instruction::Beq {
                rs: 8,
                rt: 9,
                imm: 0x10
            }
        );
        assert_eq!(
            decode(i_type(0x05, 8, 9, 0x10)),
            Instruction::Bne {
                rs: 8,
                rt: 9,
                imm: 0x10
            }
        );
        assert_eq!(
            decode(i_type(0x06, 8, 0, 0x10)),
            Instruction::Blez { rs: 8, imm: 0x10 }
        );
        assert_eq!(
            decode(i_type(0x07, 8, 0, 0x10)),
            Instruction::Bgtz { rs: 8, imm: 0x10 }
        );
    }

    #[test]
    fn decode_bcond_variants() {
        assert_eq!(
            decode(i_type(0x01, 8, 0x00, 4)),
            Instruction::Bltz { rs: 8, imm: 4 }
        );
        assert_eq!(
            decode(i_type(0x01, 8, 0x01, 4)),
            Instruction::Bgez { rs: 8, imm: 4 }
        );
        assert_eq!(
            decode(i_type(0x01, 8, 0x10, 4)),
            Instruction::Bltzal { rs: 8, imm: 4 }
        );
        assert_eq!(
            decode(i_type(0x01, 8, 0x11, 4)),
            Instruction::Bgezal { rs: 8, imm: 4 }
        );
    }

    #[test]
    fn decode_special_shifts() {
        assert_eq!(
            decode(r_type(0, 9, 8, 4, 0x00)),
            Instruction::Sll {
                rd: 8,
                rt: 9,
                shamt: 4
            }
        );
        assert_eq!(
            decode(r_type(0, 9, 8, 4, 0x02)),
            Instruction::Srl {
                rd: 8,
                rt: 9,
                shamt: 4
            }
        );
        assert_eq!(
            decode(r_type(0, 9, 8, 4, 0x03)),
            Instruction::Sra {
                rd: 8,
                rt: 9,
                shamt: 4
            }
        );
        assert_eq!(
            decode(r_type(10, 9, 8, 0, 0x04)),
            Instruction::Sllv {
                rd: 8,
                rt: 9,
                rs: 10
            }
        );
        assert_eq!(
            decode(r_type(10, 9, 8, 0, 0x06)),
            Instruction::Srlv {
                rd: 8,
                rt: 9,
                rs: 10
            }
        );
        assert_eq!(
            decode(r_type(10, 9, 8, 0, 0x07)),
            Instruction::Srav {
                rd: 8,
                rt: 9,
                rs: 10
            }
        );
    }

    #[test]
    fn decode_special_alu() {
        type Ctor = fn(Reg, Reg, Reg) -> Instruction;
        let cases: [(u32, Ctor); 10] = [
            (0x20, |rd, rs, rt| Instruction::Add { rd, rs, rt }),
            (0x21, |rd, rs, rt| Instruction::Addu { rd, rs, rt }),
            (0x22, |rd, rs, rt| Instruction::Sub { rd, rs, rt }),
            (0x23, |rd, rs, rt| Instruction::Subu { rd, rs, rt }),
            (0x24, |rd, rs, rt| Instruction::And { rd, rs, rt }),
            (0x25, |rd, rs, rt| Instruction::Or { rd, rs, rt }),
            (0x26, |rd, rs, rt| Instruction::Xor { rd, rs, rt }),
            (0x27, |rd, rs, rt| Instruction::Nor { rd, rs, rt }),
            (0x2A, |rd, rs, rt| Instruction::Slt { rd, rs, rt }),
            (0x2B, |rd, rs, rt| Instruction::Sltu { rd, rs, rt }),
        ];
        for (fun, ctor) in cases {
            assert_eq!(decode(r_type(10, 11, 8, 0, fun)), ctor(8, 10, 11));
        }
    }

    #[test]
    fn decode_special_muldiv_hilo() {
        assert_eq!(
            decode(r_type(10, 11, 0, 0, 0x18)),
            Instruction::Mult { rs: 10, rt: 11 }
        );
        assert_eq!(
            decode(r_type(10, 11, 0, 0, 0x19)),
            Instruction::Multu { rs: 10, rt: 11 }
        );
        assert_eq!(
            decode(r_type(10, 11, 0, 0, 0x1A)),
            Instruction::Div { rs: 10, rt: 11 }
        );
        assert_eq!(
            decode(r_type(10, 11, 0, 0, 0x1B)),
            Instruction::Divu { rs: 10, rt: 11 }
        );
        assert_eq!(
            decode(r_type(0, 0, 8, 0, 0x10)),
            Instruction::Mfhi { rd: 8 }
        );
        assert_eq!(
            decode(r_type(10, 0, 0, 0, 0x11)),
            Instruction::Mthi { rs: 10 }
        );
        assert_eq!(
            decode(r_type(0, 0, 8, 0, 0x12)),
            Instruction::Mflo { rd: 8 }
        );
        assert_eq!(
            decode(r_type(10, 0, 0, 0, 0x13)),
            Instruction::Mtlo { rs: 10 }
        );
    }

    #[test]
    fn decode_special_jumps_and_system() {
        assert_eq!(
            decode(r_type(10, 0, 0, 0, 0x08)),
            Instruction::Jr { rs: 10 }
        );
        assert_eq!(
            decode(r_type(10, 0, 8, 0, 0x09)),
            Instruction::Jalr { rd: 8, rs: 10 }
        );
        assert_eq!(decode(r_type(0, 0, 0, 0, 0x0C)), Instruction::Syscall);
        assert_eq!(decode(r_type(0, 0, 0, 0, 0x0D)), Instruction::Break);
    }

    #[test]
    fn decode_loads_and_stores() {
        assert_eq!(
            decode(i_type(0x20, 9, 8, 4)),
            Instruction::Lb {
                rt: 8,
                rs: 9,
                imm: 4
            }
        );
        assert_eq!(
            decode(i_type(0x23, 9, 8, 4)),
            Instruction::Lw {
                rt: 8,
                rs: 9,
                imm: 4
            }
        );
        assert_eq!(
            decode(i_type(0x24, 9, 8, 4)),
            Instruction::Lbu {
                rt: 8,
                rs: 9,
                imm: 4
            }
        );
        assert_eq!(
            decode(i_type(0x25, 9, 8, 4)),
            Instruction::Lhu {
                rt: 8,
                rs: 9,
                imm: 4
            }
        );
        assert_eq!(
            decode(i_type(0x22, 9, 8, 4)),
            Instruction::Lwl {
                rt: 8,
                rs: 9,
                imm: 4
            }
        );
        assert_eq!(
            decode(i_type(0x26, 9, 8, 4)),
            Instruction::Lwr {
                rt: 8,
                rs: 9,
                imm: 4
            }
        );
        assert_eq!(
            decode(i_type(0x28, 9, 8, 4)),
            Instruction::Sb {
                rt: 8,
                rs: 9,
                imm: 4
            }
        );
        assert_eq!(
            decode(i_type(0x2B, 9, 8, 4)),
            Instruction::Sw {
                rt: 8,
                rs: 9,
                imm: 4
            }
        );
        assert_eq!(
            decode(i_type(0x2A, 9, 8, 4)),
            Instruction::Swl {
                rt: 8,
                rs: 9,
                imm: 4
            }
        );
        assert_eq!(
            decode(i_type(0x2E, 9, 8, 4)),
            Instruction::Swr {
                rt: 8,
                rs: 9,
                imm: 4
            }
        );
    }

    #[test]
    fn decode_cop0_ops() {
        // MFC0 $t0, $12 (SR): rs=0, rt=8, rd=12
        assert_eq!(
            decode((0x10 << 26) | (8 << 16) | (12 << 11)),
            Instruction::Mfc0 { rt: 8, rd: 12 }
        );
        // MTC0 $t0, $12: rs=4
        assert_eq!(
            decode((0x10 << 26) | (0x04 << 21) | (8 << 16) | (12 << 11)),
            Instruction::Mtc0 { rt: 8, rd: 12 }
        );
        // RFE: 0x4200_0010
        assert_eq!(decode(0x4200_0010), Instruction::Rfe);
    }

    #[test]
    fn decode_cop2_is_stubbed() {
        let raw = (0x12 << 26) | 0x0048_0012;
        assert_eq!(decode(raw), Instruction::Cop2 { raw });
    }

    #[test]
    fn decode_illegal() {
        // Opcode 0x3F is not assigned.
        assert!(matches!(decode(0xFC00_0000), Instruction::Illegal { .. }));
        // SPECIAL with unassigned funct 0x3F.
        assert!(matches!(decode(0x0000_003F), Instruction::Illegal { .. }));
        // All-ones is illegal.
        assert!(matches!(decode(0xFFFF_FFFF), Instruction::Illegal { .. }));
    }

    #[test]
    fn decode_all_zero_is_sll_nop() {
        assert_eq!(
            decode(0),
            Instruction::Sll {
                rd: 0,
                rt: 0,
                shamt: 0
            }
        );
    }

    #[test]
    fn decode_never_panics_smoke() {
        // Deterministic coverage across the opcode space.
        for op in 0u32..64 {
            for sub in 0u32..64 {
                let _ = decode((op << 26) | sub);
            }
        }
    }
}
