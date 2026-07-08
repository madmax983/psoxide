//! Verus proof: the R3000A segment mask (`mask_region`) is total, bounded, and
//! region-correct.
//!
//! This file is checked out-of-band by `scripts/verus-check.ps1`; it is not a
//! module of the `psoxide-proof` crate and is never compiled by `cargo`.

use vstd::prelude::*;

verus! {

/// The MIPS segment selector: the top three bits of a virtual address.
spec fn seg(addr: u32) -> u32 {
    addr >> 29u32
}

/// The per-segment mask, mirroring `psoxide_core::bus::REGION_MASK`.
///
/// KUSEG (segments 0..=3) and KSEG2 (segments 6..=7) pass addresses through
/// unchanged; KSEG0 (segment 4) strips the top bit; KSEG1 (segment 5) strips
/// the top three bits.
spec fn region_mask(s: u32) -> u32 {
    if s == 4 {
        0x7FFF_FFFFu32
    } else if s == 5 {
        0x1FFF_FFFFu32
    } else {
        0xFFFF_FFFFu32
    }
}

/// The physical address produced by stripping segment bits from `addr`.
spec fn mask_region(addr: u32) -> u32 {
    addr & region_mask(seg(addr))
}

/// The segment index is always within `0..=7`.
proof fn seg_is_bounded(addr: u32)
    ensures seg(addr) <= 7,
{
    assert(addr >> 29u32 <= 7u32) by (bit_vector);
}

/// Masking never increases an address (it only clears bits).
proof fn mask_region_is_bounded(addr: u32)
    ensures mask_region(addr) <= addr,
{
    let m = region_mask(seg(addr));
    assert(addr & m <= addr) by (bit_vector);
}

/// The mask is total, bounded, and decodes each segment correctly.
proof fn mask_region_total_and_correct(addr: u32)
    ensures
        seg(addr) <= 7,
        mask_region(addr) <= addr,
        // KUSEG / KSEG2 pass through unchanged.
        (seg(addr) <= 3) ==> mask_region(addr) == addr,
        (seg(addr) >= 6) ==> mask_region(addr) == addr,
        // KSEG0 strips the top bit.
        (seg(addr) == 4) ==> mask_region(addr) == (addr & 0x7FFF_FFFFu32),
        // KSEG1 strips the top three bits (yielding the physical address).
        (seg(addr) == 5) ==> mask_region(addr) == (addr & 0x1FFF_FFFFu32),
{
    seg_is_bounded(addr);
    mask_region_is_bounded(addr);
    // The KUSEG/KSEG2 pass-through follows from `addr & 0xFFFF_FFFF == addr`.
    assert(addr & 0xFFFF_FFFFu32 == addr) by (bit_vector);
}

} // verus!

fn main() {}
