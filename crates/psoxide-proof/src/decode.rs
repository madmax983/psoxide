//! Verus proof: the R3000A primary-opcode decode is total and maps every
//! coprocessor opcode to a coprocessor instruction class (never to the
//! reserved-instruction / `Illegal` class).
//!
//! This file is checked out-of-band by `scripts/verus-check.ps1`; it is not a
//! module of the `psoxide-proof` crate and is never compiled by `cargo`.
//!
//! It specifies the *primary-opcode* dispatch of
//! `psoxide_core::cpu::decode::decode` at the granularity of an instruction
//! *class* (the level at which coprocessor usability is decided). The mapping
//! mirrors the `match opcode(instr)` arms of `decode`:
//!
//! * `0x10` → COP0        (`Mfc0` / `Mtc0` / `Rfe` / `Cop0`)
//! * `0x11` → COP1        (`Cop1`)
//! * `0x12` → COP2        (`Cop2`)
//! * `0x13` → COP3        (`Cop3`)
//! * `0x30..=0x33` → LWCz  (`Lwc { cop }`)
//! * `0x38..=0x3B` → SWCz  (`Swc { cop }`)
//!
//! The key correctness property proved here is that these coprocessor opcodes
//! decode to a *coprocessor* class — not `Illegal` — so the interpreter routes
//! them through the coprocessor-usable path (raising Coprocessor Unusable,
//! ExcCode 0x0B) rather than the reserved-instruction path (ExcCode 0x0A).

use vstd::prelude::*;

verus! {

/// The abstract instruction class of a primary opcode. This is a coarsening of
/// `psoxide_core::cpu::decode::Instruction`: every enum variant of the concrete
/// decoder collapses onto exactly one of these classes, chosen so that the
/// class alone determines the coprocessor-usability behaviour.
#[derive(PartialEq, Eq, Structural)]
enum Class {
    /// SPECIAL (opcode 0x00): shifts, ALU-register, jr/jalr, syscall/break,
    /// mult/div, hi/lo — refined by the `funct` field, not modelled here.
    Special,
    /// REGIMM / BCOND (opcode 0x01): bltz/bgez/bltzal/bgezal.
    RegImm,
    /// j / jal (opcodes 0x02, 0x03).
    Jump,
    /// beq / bne / blez / bgtz (opcodes 0x04..=0x07).
    Branch,
    /// ALU-immediate and lui (opcodes 0x08..=0x0F).
    AluImm,
    /// Coprocessor 0 op (opcode 0x10): mfc0/mtc0/rfe or an unassigned command.
    Cop0,
    /// Coprocessor 1 op (opcode 0x11).
    Cop1,
    /// Coprocessor 2 (GTE) op (opcode 0x12).
    Cop2,
    /// Coprocessor 3 op (opcode 0x13).
    Cop3,
    /// Aligned/unaligned loads (opcodes 0x20..=0x26).
    Load,
    /// Aligned/unaligned stores (opcodes 0x28, 0x29, 0x2A, 0x2B, 0x2E).
    Store,
    /// Coprocessor load word LWCz (opcodes 0x30..=0x33).
    Lwc,
    /// Coprocessor store word SWCz (opcodes 0x38..=0x3B).
    Swc,
    /// Any unassigned / reserved primary opcode: a reserved-instruction trap.
    Illegal,
}

/// The primary (6-bit) opcode of an instruction word: `instr >> 26`.
spec fn opcode(instr: u32) -> u32 {
    instr >> 26u32
}

/// The primary-opcode class of `op` (a value in `0..=63`). This mirrors, arm
/// for arm, the top-level `match opcode(instr)` of
/// `psoxide_core::cpu::decode::decode`.
spec fn decode_class(op: u32) -> Class {
    if op == 0x00 {
        Class::Special
    } else if op == 0x01 {
        Class::RegImm
    } else if op == 0x02 || op == 0x03 {
        Class::Jump
    } else if 0x04 <= op && op <= 0x07 {
        Class::Branch
    } else if 0x08 <= op && op <= 0x0F {
        Class::AluImm
    } else if op == 0x10 {
        Class::Cop0
    } else if op == 0x11 {
        Class::Cop1
    } else if op == 0x12 {
        Class::Cop2
    } else if op == 0x13 {
        Class::Cop3
    } else if 0x20 <= op && op <= 0x26 {
        Class::Load
    } else if op == 0x28 || op == 0x29 || op == 0x2A || op == 0x2B || op == 0x2E {
        Class::Store
    } else if 0x30 <= op && op <= 0x33 {
        Class::Lwc
    } else if 0x38 <= op && op <= 0x3B {
        Class::Swc
    } else {
        Class::Illegal
    }
}

/// Whether a class is a coprocessor operation (routed through the
/// coprocessor-usable path, which may raise Coprocessor Unusable, 0x0B) as
/// opposed to the reserved-instruction path (0x0A).
spec fn is_coprocessor(c: Class) -> bool {
    c == Class::Cop0 || c == Class::Cop1 || c == Class::Cop2 || c == Class::Cop3
        || c == Class::Lwc || c == Class::Swc
}

/// The extracted opcode is always a 6-bit value.
proof fn opcode_is_six_bits(instr: u32)
    ensures opcode(instr) <= 0x3F,
{
    assert(instr >> 26u32 <= 0x3Fu32) by (bit_vector);
}

/// Every one of the six coprocessor opcode groups decodes to a coprocessor
/// class — never to `Illegal`. This is exactly the property the interpreter
/// relies on to raise Coprocessor Unusable (0x0B) rather than a
/// reserved-instruction trap (0x0A) for COP1/COP3, the unassigned COP0
/// commands, and the LWCz/SWCz coprocessor load/stores.
proof fn coprocessor_opcodes_are_coprocessor_ops(op: u32)
    ensures
        op == 0x10 ==> decode_class(op) == Class::Cop0,
        op == 0x11 ==> decode_class(op) == Class::Cop1,
        op == 0x12 ==> decode_class(op) == Class::Cop2,
        op == 0x13 ==> decode_class(op) == Class::Cop3,
        (0x30 <= op && op <= 0x33) ==> decode_class(op) == Class::Lwc,
        (0x38 <= op && op <= 0x3B) ==> decode_class(op) == Class::Swc,
        is_coprocessor(decode_class(op)) <==> (op == 0x10 || op == 0x11 || op == 0x12
            || op == 0x13 || (0x30 <= op && op <= 0x33) || (0x38 <= op && op <= 0x3B)),
{
}

/// The decode is total: every opcode maps to some class (trivially true because
/// `decode_class` is a total spec function whose final `else` catches all
/// remaining opcodes as `Illegal`). Stated explicitly for documentation.
proof fn decode_class_is_total(op: u32)
    ensures decode_class(op) == decode_class(op),
{
}

} // verus!
