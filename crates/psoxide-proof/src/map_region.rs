//! Verus proof: the physical-address bus decode `map_region` is total and its
//! regions are pairwise disjoint, and it pins the boundary addresses of every
//! region to the expected `BusRegion`.
//!
//! This file is checked out-of-band by `scripts/verus-check.ps1`; it is not a
//! module of the `psoxide-proof` crate and is never compiled by `cargo`.
//!
//! It specifies `psoxide_core::bus::map_region(phys: u32) -> BusRegion`
//! (crates/psoxide-core/src/bus.rs, around line 112), whose `match` arms are:
//!
//! * `0x0000_0000..=0x007F_FFFF` → MainRam    (2MB RAM, mirrored ×4)
//! * `0x1F00_0000..=0x1F7F_FFFF` → Expansion1
//! * `0x1F80_0000..=0x1F80_03FF` → Scratchpad (1KB)
//! * `0x1F80_1000..=0x1F80_2FFF` → IoPorts
//! * `0x1FA0_0000..=0x1FBF_FFFF` → Expansion2
//! * `0x1FC0_0000..=0x1FC7_FFFF` → Bios       (512KB)
//! * `0xFFFE_0000..=0xFFFE_01FF` → CacheControl
//! * everything else             → Unmapped

use vstd::prelude::*;

verus! {

/// Mirror of `psoxide_core::bus::BusRegion`.
#[derive(PartialEq, Eq, Structural)]
enum Region {
    MainRam,
    Expansion1,
    Scratchpad,
    IoPorts,
    Expansion2,
    Bios,
    CacheControl,
    Unmapped,
}

/// The bus region a physical address decodes to. Mirrors, arm for arm, the
/// `match phys { .. }` of `psoxide_core::bus::map_region`. The Rust `..=` ranges
/// are inclusive; the guards below reproduce them exactly.
spec fn map_region(phys: u32) -> Region {
    if phys <= 0x007F_FFFFu32 {
        Region::MainRam
    } else if 0x1F00_0000u32 <= phys && phys <= 0x1F7F_FFFFu32 {
        Region::Expansion1
    } else if 0x1F80_0000u32 <= phys && phys <= 0x1F80_03FFu32 {
        Region::Scratchpad
    } else if 0x1F80_1000u32 <= phys && phys <= 0x1F80_2FFFu32 {
        Region::IoPorts
    } else if 0x1FA0_0000u32 <= phys && phys <= 0x1FBF_FFFFu32 {
        Region::Expansion2
    } else if 0x1FC0_0000u32 <= phys && phys <= 0x1FC7_FFFFu32 {
        Region::Bios
    } else if 0xFFFE_0000u32 <= phys && phys <= 0xFFFE_01FFu32 {
        Region::CacheControl
    } else {
        Region::Unmapped
    }
}

/// Totality: every `u32` decodes to exactly one region. Trivially true because
/// `map_region` is a total spec function whose final `else` catches every
/// address not claimed by a named range. Stated explicitly for documentation:
/// the classification is one of the eight variants for every input.
proof fn map_region_is_total(phys: u32)
    ensures
        map_region(phys) == Region::MainRam
        || map_region(phys) == Region::Expansion1
        || map_region(phys) == Region::Scratchpad
        || map_region(phys) == Region::IoPorts
        || map_region(phys) == Region::Expansion2
        || map_region(phys) == Region::Bios
        || map_region(phys) == Region::CacheControl
        || map_region(phys) == Region::Unmapped,
{
}

/// Characterization / disjointness: the region a `phys` maps to holds **iff**
/// `phys` lies in that region's address range (and no other). Because the guards
/// are checked in order, a named region's characterization carries the implicit
/// "not claimed by an earlier arm" — but the ranges are already pairwise
/// disjoint (each named range sits below the next, with gaps between them), so
/// membership in a range is equivalent to decoding to that region. This lemma
/// proves both directions for each of the seven named regions; disjointness is
/// the fact that these seven range-predicates are mutually exclusive, which
/// `map_region` returning a single variant witnesses.
proof fn map_region_characterization(phys: u32)
    ensures
        map_region(phys) == Region::MainRam <==> (phys <= 0x007F_FFFFu32),
        map_region(phys) == Region::Expansion1 <==>
            (0x1F00_0000u32 <= phys && phys <= 0x1F7F_FFFFu32),
        map_region(phys) == Region::Scratchpad <==>
            (0x1F80_0000u32 <= phys && phys <= 0x1F80_03FFu32),
        map_region(phys) == Region::IoPorts <==>
            (0x1F80_1000u32 <= phys && phys <= 0x1F80_2FFFu32),
        map_region(phys) == Region::Expansion2 <==>
            (0x1FA0_0000u32 <= phys && phys <= 0x1FBF_FFFFu32),
        map_region(phys) == Region::Bios <==>
            (0x1FC0_0000u32 <= phys && phys <= 0x1FC7_FFFFu32),
        map_region(phys) == Region::CacheControl <==>
            (0xFFFE_0000u32 <= phys && phys <= 0xFFFE_01FFu32),
{
}

/// Concrete boundary addresses, pinning each region's first/last address and the
/// gap addresses just past a region's end (which fall through to `Unmapped`).
proof fn map_region_boundaries()
    ensures
        // Main RAM: base and last mirrored byte.
        map_region(0x0000_0000u32) == Region::MainRam,
        map_region(0x007F_FFFFu32) == Region::MainRam,
        // Just past main RAM's mirror window is unmapped.
        map_region(0x0080_0000u32) == Region::Unmapped,
        // Expansion 1.
        map_region(0x1F00_0000u32) == Region::Expansion1,
        map_region(0x1F7F_FFFFu32) == Region::Expansion1,
        // Scratchpad (1KB): base and last byte; the byte past its top is a gap.
        map_region(0x1F80_0000u32) == Region::Scratchpad,
        map_region(0x1F80_03FFu32) == Region::Scratchpad,
        map_region(0x1F80_0400u32) == Region::Unmapped,
        // I/O ports: base and last byte.
        map_region(0x1F80_1000u32) == Region::IoPorts,
        map_region(0x1F80_2FFFu32) == Region::IoPorts,
        // Expansion 2.
        map_region(0x1FA0_0000u32) == Region::Expansion2,
        map_region(0x1FBF_FFFFu32) == Region::Expansion2,
        // BIOS (512KB): base and last byte; one past the top is a gap.
        map_region(0x1FC0_0000u32) == Region::Bios,
        map_region(0x1FC7_FFFFu32) == Region::Bios,
        map_region(0x1FC8_0000u32) == Region::Unmapped,
        // Cache-control register window: base and last byte.
        map_region(0xFFFE_0000u32) == Region::CacheControl,
        map_region(0xFFFE_01FFu32) == Region::CacheControl,
        // Top of the address space is unmapped.
        map_region(0xFFFF_FFFFu32) == Region::Unmapped,
{
}

} // verus!

fn main() {}
