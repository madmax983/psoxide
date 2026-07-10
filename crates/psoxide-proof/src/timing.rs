//! Verus proof: the pure memory-access timing model of
//! `psoxide_core::timing` (crates/psoxide-core/src/timing.rs).
//!
//! This file is checked out-of-band by `scripts/verus-check.ps1`; it is not a
//! module of the `psoxide-proof` crate and is never compiled by `cargo`.
//!
//! It specifies the *pure* timing functions of that module — `delay_1st_seq`
//! (line 189), `bus_cycles` (line 223) and `access_cycles` (line 234) — and
//! proves:
//!   * the fixed-cost classes return their exact constants,
//!   * the delay-driven classes reproduce their golden `access-time` cycle
//!     counts (BIOS/EXP1 7/13/25, EXP3 6/6/10, SPU 18/18/39, CD-ROM 8/14/26,
//!     EXP2 11/26/56 — see timing.rs:178-179 and its unit tests, and the
//!     harness golden table ps1_tests.rs:285-329),
//!   * access cost is monotonic non-decreasing in the access width, and
//!   * every field is masked so the arithmetic stays far inside `u32`
//!     (no overflow in the real `u32` exec function, and the `n - 1` bus-cycle
//!     term never underflows) — under the genuine precondition
//!     `width_bytes >= 1` that all callers satisfy (they pass 1/2/4).

#![allow(non_snake_case)]

use vstd::prelude::*;

verus! {

// ── Fixed-cost constants (timing.rs:45-52) ──────────────────────────────────
// Modelled as mathematical integers; the overflow lemmas below establish that
// every computed cost stays far inside the real `u32` range.
spec const MAIN_RAM_CYCLES: int = 5;
spec const SCRATCHPAD_CYCLES: int = 1;
spec const INTERNAL_IO_CYCLES: int = 3;

/// Mirror of `psoxide_core::timing::AccessClass` (timing.rs:60). The delay-driven
/// classes (Bios/Expansion1/Expansion2/Expansion3/Spu/Cdrom) share one code path
/// parameterised only by the region's Delay/Size word, so they are treated
/// uniformly here.
#[derive(PartialEq, Eq, Structural)]
enum Class {
    MainRam,
    Bios,
    Scratchpad,
    Expansion1,
    Expansion2,
    Expansion3,
    Spu,
    Cdrom,
    InternalIo,
    CacheControl,
    Unmapped,
}

/// Whether a class uses the wait-state (`delay_1st_seq`) path rather than a
/// fixed constant. Mirrors the `_ =>` arm of `access_cycles` (timing.rs:250).
spec fn is_delay_driven(class: Class) -> bool {
    match class {
        Class::Bios => true,
        Class::Expansion1 => true,
        Class::Expansion2 => true,
        Class::Expansion3 => true,
        Class::Spu => true,
        Class::Cdrom => true,
        _ => false,
    }
}

// ── delay_1st_seq (timing.rs:189) ───────────────────────────────────────────
// Bit-field extractors, mirroring the `let` bindings in delay_1st_seq.

/// COM0 nibble of COM_DELAY (bits 0-3).
spec fn com0(com_delay: u32) -> u32 {
    com_delay & 0xF
}

/// COM2 nibble of COM_DELAY (bits 8-11).
spec fn com2(com_delay: u32) -> u32 {
    (com_delay >> 8u32) & 0xF
}

/// Programmed read delay (Delay/Size bits 4-7).
spec fn read_delay(delay_size: u32) -> u32 {
    (delay_size >> 4u32) & 0xF
}

/// COM0 bus-recovery flag (Delay/Size bit 8).
spec fn recovery(delay_size: u32) -> bool {
    (delay_size & 0x100) != 0
}

/// COM2 data-float flag (Delay/Size bit 10).
spec fn floating(delay_size: u32) -> bool {
    (delay_size & 0x400) != 0
}

/// First-access cycle count: `base + 2` where `base = read_delay + 2`
/// (timing.rs:199,203) — i.e. `read_delay + 4`.
spec fn first_access(delay_size: u32) -> int {
    read_delay(delay_size) + 2 + 2
}

/// Sequential-access cycle count: `base` plus COM0 when recovery is set and
/// COM2 when floating is set (timing.rs:209-215).
spec fn seq_access(delay_size: u32, com_delay: u32) -> int {
    (read_delay(delay_size) + 2)
        + (if recovery(delay_size) { com0(com_delay) as int } else { 0int })
        + (if floating(delay_size) { com2(com_delay) as int } else { 0int })
}

// ── bus_cycles (timing.rs:223) ──────────────────────────────────────────────

/// Bus width in bytes selected by Delay/Size bit 12 (0 = 8-bit, 1 = 16-bit),
/// mirroring `bus_bytes` in `access_cycles` (timing.rs:253).
spec fn bus_bytes(delay_size: u32) -> u32 {
    if (delay_size & 0x1000) != 0 { 2u32 } else { 1u32 }
}

/// `ceil(width_bytes / bus_bytes)` — the div_ceil of `bus_cycles`. Defined for
/// `bus_bytes >= 1` (always true: `bus_bytes` is 1 or 2 by construction).
spec fn bus_cycles(width_bytes: u32, bus: u32) -> int {
    (width_bytes as int + bus as int - 1) / (bus as int)
}

// ── access_cycles (timing.rs:234) ───────────────────────────────────────────

/// The delay-driven cost: `first + (n - 1) * seq` (timing.rs:255) where `n` is
/// the bus-cycle count. `delay_size` is the region's Delay/Size word (what
/// `MemTiming::delay_size(class).unwrap_or(0)` yields for a delay class).
spec fn access_cycles_delay(delay_size: u32, com_delay: u32, width_bytes: u32) -> int {
    first_access(delay_size)
        + (bus_cycles(width_bytes, bus_bytes(delay_size)) - 1) * seq_access(delay_size, com_delay)
}

/// Total access cost for a class. Mirrors, arm for arm, the `match class` of
/// `psoxide_core::timing::access_cycles` (timing.rs:235). For the delay-driven
/// classes `delay_size` is the region's Delay/Size word.
spec fn access_cycles(class: Class, width_bytes: u32, delay_size: u32, com_delay: u32) -> int {
    match class {
        Class::MainRam => MAIN_RAM_CYCLES,
        Class::Scratchpad => SCRATCHPAD_CYCLES,
        Class::InternalIo => INTERNAL_IO_CYCLES,
        Class::CacheControl => if width_bytes <= 1 { 1int } else { 2int },
        Class::Unmapped => 1int,
        _ => access_cycles_delay(delay_size, com_delay, width_bytes),
    }
}

// ── Field bounds (masking keeps everything inside u32) ───────────────────────

/// Every nibble extracted by the `& 0xF` masks is at most 15. These are the
/// facts the overflow argument rests on.
proof fn field_bounds(delay_size: u32, com_delay: u32)
    ensures
        read_delay(delay_size) <= 15,
        com0(com_delay) <= 15,
        com2(com_delay) <= 15,
{
    assert((delay_size >> 4u32) & 0xF <= 0xF) by (bit_vector);
    assert(com_delay & 0xF <= 0xF) by (bit_vector);
    assert((com_delay >> 8u32) & 0xF <= 0xF) by (bit_vector);
}

/// The first/sequential single-bus-cycle counts are small: `first <= 19`
/// (read_delay<=15, +4) and `seq <= 47` (read_delay+2<=17, +com0<=15,
/// +com2<=15). Far inside u32 — the real `u32` `delay_1st_seq` cannot overflow.
proof fn delay_1st_seq_bounded(delay_size: u32, com_delay: u32)
    ensures
        first_access(delay_size) <= 19,
        seq_access(delay_size, com_delay) <= 47,
{
    field_bounds(delay_size, com_delay);
}

/// For the widths callers actually pass (1/2/4) on a 1- or 2-byte bus, the
/// bus-cycle count is between 1 and 4. The `>= 1` half is what guarantees the
/// real code's `(n - 1)` never underflows.
proof fn bus_cycles_bounds(width_bytes: u32, bus: u32)
    requires
        width_bytes == 1 || width_bytes == 2 || width_bytes == 4,
        bus == 1 || bus == 2,
    ensures
        1 <= bus_cycles(width_bytes, bus),
        bus_cycles(width_bytes, bus) <= 4,
{
}

/// The full delay-driven cost fits well inside u32: `first + (n-1)*seq`
/// with `first <= 19`, `n - 1 <= 3`, `seq <= 47` gives `<= 160`. Hence the real
/// `u32` `access_cycles` never overflows for the 1/2/4-byte accesses callers
/// make, and (via `bus_cycles_bounds`) its `n - 1` never underflows.
proof fn access_cycles_delay_no_overflow(delay_size: u32, com_delay: u32, width_bytes: u32)
    requires
        width_bytes == 1 || width_bytes == 2 || width_bytes == 4,
    ensures
        access_cycles_delay(delay_size, com_delay, width_bytes) <= 160,
{
    delay_1st_seq_bounded(delay_size, com_delay);
    bus_cycles_bounds(width_bytes, bus_bytes(delay_size));
    let n = bus_cycles(width_bytes, bus_bytes(delay_size));
    let seq = seq_access(delay_size, com_delay);
    // (n - 1) <= 3 and seq <= 47, so their product is <= 141.
    assert((n - 1) * seq <= 3 * 47) by (nonlinear_arith)
        requires n <= 4, n >= 1, seq <= 47;
}

// ── Fixed-class exact values ─────────────────────────────────────────────────

/// The four fixed-cost classes return their constants exactly, for every width;
/// cache-control is 1 for a byte and 2 for anything wider (timing.rs:236-248).
proof fn fixed_class_exact_values(width_bytes: u32, delay_size: u32, com_delay: u32)
    ensures
        access_cycles(Class::MainRam, width_bytes, delay_size, com_delay) == 5,
        access_cycles(Class::Scratchpad, width_bytes, delay_size, com_delay) == 1,
        access_cycles(Class::InternalIo, width_bytes, delay_size, com_delay) == 3,
        access_cycles(Class::Unmapped, width_bytes, delay_size, com_delay) == 1,
        width_bytes <= 1 ==> access_cycles(Class::CacheControl, width_bytes, delay_size, com_delay) == 1,
        width_bytes >= 2 ==> access_cycles(Class::CacheControl, width_bytes, delay_size, com_delay) == 2,
{
}

// ── Monotonicity in access width ─────────────────────────────────────────────

/// A wider access never costs fewer cycles than a narrower one in the same
/// delay-driven region: `access_cycles_delay` is monotonic non-decreasing across
/// the width ladder 1 -> 2 -> 4. This is the property the harness test
/// `access_cost_is_monotonic_in_width_for_delay_regions` (timing.rs:445) checks
/// at runtime, proved here for all Delay/Size and COM_DELAY words.
proof fn access_cost_monotonic_in_width(delay_size: u32, com_delay: u32)
    ensures
        access_cycles_delay(delay_size, com_delay, 1)
            <= access_cycles_delay(delay_size, com_delay, 2),
        access_cycles_delay(delay_size, com_delay, 2)
            <= access_cycles_delay(delay_size, com_delay, 4),
{
    let seq = seq_access(delay_size, com_delay);
    let n1 = bus_cycles(1, bus_bytes(delay_size));
    let n2 = bus_cycles(2, bus_bytes(delay_size));
    let n4 = bus_cycles(4, bus_bytes(delay_size));
    // The sequential cost is non-negative (read_delay >= 0 plus non-negative
    // COM turnaround terms).
    assert(seq >= 0);
    // On a 1- or 2-byte bus the bus-cycle count is monotonic: n1 <= n2 <= n4.
    assert(n1 <= n2 && n2 <= n4);
    // first + (n-1)*seq is monotonic in n because seq >= 0.
    assert((n1 - 1) * seq <= (n2 - 1) * seq) by (nonlinear_arith)
        requires n1 <= n2, n1 >= 1, seq >= 0;
    assert((n2 - 1) * seq <= (n4 - 1) * seq) by (nonlinear_arith)
        requires n2 <= n4, n2 >= 1, seq >= 0;
}

// ── Golden exact-value lemmas (access-time reference rows) ────────────────────
// Delay/Size + COM_DELAY words are the defaults the ps1-tests `access-time`
// binary programs before measuring (timing.rs test module, lines 289-295).

/// BIOS / Expansion 1 (Delay/Size 0x0013_243F, COM_DELAY 0x0003_1125):
/// `delay_1st_seq == (7, 6)` and the 8/16/32-bit costs are 7/13/25
/// (timing.rs:186, 310-320, 337-343).
proof fn golden_bios_exp1()
    ensures
        first_access(0x0013_243Fu32) == 7,
        seq_access(0x0013_243Fu32, 0x0003_1125u32) == 6,
        access_cycles(Class::Bios, 1, 0x0013_243Fu32, 0x0003_1125u32) == 7,
        access_cycles(Class::Bios, 2, 0x0013_243Fu32, 0x0003_1125u32) == 13,
        access_cycles(Class::Bios, 4, 0x0013_243Fu32, 0x0003_1125u32) == 25,
        access_cycles(Class::Expansion1, 1, 0x0013_243Fu32, 0x0003_1125u32) == 7,
        access_cycles(Class::Expansion1, 2, 0x0013_243Fu32, 0x0003_1125u32) == 13,
        access_cycles(Class::Expansion1, 4, 0x0013_243Fu32, 0x0003_1125u32) == 25,
{
    assert(first_access(0x0013_243Fu32) == 7) by (compute);
    assert(seq_access(0x0013_243Fu32, 0x0003_1125u32) == 6) by (compute);
    assert(access_cycles(Class::Bios, 1, 0x0013_243Fu32, 0x0003_1125u32) == 7) by (compute);
    assert(access_cycles(Class::Bios, 2, 0x0013_243Fu32, 0x0003_1125u32) == 13) by (compute);
    assert(access_cycles(Class::Bios, 4, 0x0013_243Fu32, 0x0003_1125u32) == 25) by (compute);
    assert(access_cycles(Class::Expansion1, 1, 0x0013_243Fu32, 0x0003_1125u32) == 7) by (compute);
    assert(access_cycles(Class::Expansion1, 2, 0x0013_243Fu32, 0x0003_1125u32) == 13) by (compute);
    assert(access_cycles(Class::Expansion1, 4, 0x0013_243Fu32, 0x0003_1125u32) == 25) by (compute);
}

/// Expansion 3 (Delay/Size 0x0000_3022, 16-bit bus): 6/6/10 (timing.rs:346-352).
proof fn golden_exp3()
    ensures
        access_cycles(Class::Expansion3, 1, 0x0000_3022u32, 0x0003_1125u32) == 6,
        access_cycles(Class::Expansion3, 2, 0x0000_3022u32, 0x0003_1125u32) == 6,
        access_cycles(Class::Expansion3, 4, 0x0000_3022u32, 0x0003_1125u32) == 10,
{
    assert(access_cycles(Class::Expansion3, 1, 0x0000_3022u32, 0x0003_1125u32) == 6) by (compute);
    assert(access_cycles(Class::Expansion3, 2, 0x0000_3022u32, 0x0003_1125u32) == 6) by (compute);
    assert(access_cycles(Class::Expansion3, 4, 0x0000_3022u32, 0x0003_1125u32) == 10) by (compute);
}

/// SPU (Delay/Size 0x2009_31E1, 16-bit bus): delay_1st_seq == (18, 21),
/// costs 18/18/39 (timing.rs:354-371).
proof fn golden_spu()
    ensures
        first_access(0x2009_31E1u32) == 18,
        seq_access(0x2009_31E1u32, 0x0003_1125u32) == 21,
        access_cycles(Class::Spu, 1, 0x2009_31E1u32, 0x0003_1125u32) == 18,
        access_cycles(Class::Spu, 2, 0x2009_31E1u32, 0x0003_1125u32) == 18,
        access_cycles(Class::Spu, 4, 0x2009_31E1u32, 0x0003_1125u32) == 39,
{
    assert(first_access(0x2009_31E1u32) == 18) by (compute);
    assert(seq_access(0x2009_31E1u32, 0x0003_1125u32) == 21) by (compute);
    assert(access_cycles(Class::Spu, 1, 0x2009_31E1u32, 0x0003_1125u32) == 18) by (compute);
    assert(access_cycles(Class::Spu, 2, 0x2009_31E1u32, 0x0003_1125u32) == 18) by (compute);
    assert(access_cycles(Class::Spu, 4, 0x2009_31E1u32, 0x0003_1125u32) == 39) by (compute);
}

/// CD-ROM (Delay/Size 0x0002_0843, 8-bit bus): delay_1st_seq == (8, 6),
/// costs 8/14/26 (timing.rs:373-388).
proof fn golden_cdrom()
    ensures
        first_access(0x0002_0843u32) == 8,
        seq_access(0x0002_0843u32, 0x0003_1125u32) == 6,
        access_cycles(Class::Cdrom, 1, 0x0002_0843u32, 0x0003_1125u32) == 8,
        access_cycles(Class::Cdrom, 2, 0x0002_0843u32, 0x0003_1125u32) == 14,
        access_cycles(Class::Cdrom, 4, 0x0002_0843u32, 0x0003_1125u32) == 26,
{
    assert(first_access(0x0002_0843u32) == 8) by (compute);
    assert(seq_access(0x0002_0843u32, 0x0003_1125u32) == 6) by (compute);
    assert(access_cycles(Class::Cdrom, 1, 0x0002_0843u32, 0x0003_1125u32) == 8) by (compute);
    assert(access_cycles(Class::Cdrom, 2, 0x0002_0843u32, 0x0003_1125u32) == 14) by (compute);
    assert(access_cycles(Class::Cdrom, 4, 0x0002_0843u32, 0x0003_1125u32) == 26) by (compute);
}

/// Expansion 2 (Delay/Size 0x0007_0777, 8-bit bus): delay_1st_seq == (11, 15),
/// costs 11/26/56 (timing.rs:390-407).
proof fn golden_exp2()
    ensures
        first_access(0x0007_0777u32) == 11,
        seq_access(0x0007_0777u32, 0x0003_1125u32) == 15,
        access_cycles(Class::Expansion2, 1, 0x0007_0777u32, 0x0003_1125u32) == 11,
        access_cycles(Class::Expansion2, 2, 0x0007_0777u32, 0x0003_1125u32) == 26,
        access_cycles(Class::Expansion2, 4, 0x0007_0777u32, 0x0003_1125u32) == 56,
{
    assert(first_access(0x0007_0777u32) == 11) by (compute);
    assert(seq_access(0x0007_0777u32, 0x0003_1125u32) == 15) by (compute);
    assert(access_cycles(Class::Expansion2, 1, 0x0007_0777u32, 0x0003_1125u32) == 11) by (compute);
    assert(access_cycles(Class::Expansion2, 2, 0x0007_0777u32, 0x0003_1125u32) == 26) by (compute);
    assert(access_cycles(Class::Expansion2, 4, 0x0007_0777u32, 0x0003_1125u32) == 56) by (compute);
}

} // verus!

fn main() {}
