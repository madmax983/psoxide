//! Memory bus routing for the R3000A's 32-bit virtual address space.
//!
//! The PlayStation's MIPS R3000A uses the standard MIPS segmented virtual
//! address map. The top three bits of a virtual address select a segment:
//!
//! | Segment | Virtual range              | Cached | Mapped | Mask         |
//! |---------|----------------------------|--------|--------|--------------|
//! | KUSEG   | `0x0000_0000..0x8000_0000` | yes    | (n/a)  | `0xFFFF_FFFF`|
//! | KSEG0   | `0x8000_0000..0xA000_0000` | yes    | no     | `0x7FFF_FFFF`|
//! | KSEG1   | `0xA000_0000..0xC000_0000` | no     | no     | `0x1FFF_FFFF`|
//! | KSEG2   | `0xC000_0000..0xFFFF_FFFF` | yes    | (n/a)  | `0xFFFF_FFFF`|
//!
//! The PSX has no MMU/TLB in practice, so [`mask_region`] simply strips the
//! segment bits to yield a physical address, and [`map_region`] decodes that
//! physical address to a [`BusRegion`].

/// Per-segment address masks indexed by the top three bits of a virtual
/// address (`addr >> 29`).
///
/// KUSEG (indices 0-3) and KSEG2 (indices 6-7) pass through unchanged; KSEG0
/// (index 4) strips the top bit; KSEG1 (index 5) strips the top three bits.
pub const REGION_MASK: [u32; 8] = [
    // KUSEG: 0x0000_0000..0x8000_0000
    0xFFFF_FFFF,
    0xFFFF_FFFF,
    0xFFFF_FFFF,
    0xFFFF_FFFF,
    // KSEG0: 0x8000_0000..0xA000_0000
    0x7FFF_FFFF,
    // KSEG1: 0xA000_0000..0xC000_0000
    0x1FFF_FFFF,
    // KSEG2: 0xC000_0000..0xFFFF_FFFF
    0xFFFF_FFFF,
    0xFFFF_FFFF,
];

/// Strips the MIPS segment bits from a virtual `addr`, yielding the physical
/// address used to index hardware.
///
/// This is a total, branch-free function: the top three bits pick a mask from
/// [`REGION_MASK`] and the remaining bits are preserved.
///
/// # Examples
///
/// ```
/// use psoxide_core::bus::mask_region;
///
/// // KUSEG passes through unchanged.
/// assert_eq!(mask_region(0x0000_1234), 0x0000_1234);
/// // KSEG0 (0x8000_0000) maps the BIOS mirror down to its physical address.
/// assert_eq!(mask_region(0x9FC0_0000), 0x1FC0_0000);
/// // KSEG1 (0xA000_0000) is the uncached BIOS window; reset vector lives here.
/// assert_eq!(mask_region(0xBFC0_0000), 0x1FC0_0000);
/// ```
#[must_use]
pub fn mask_region(addr: u32) -> u32 {
    let index = (addr >> 29) as usize;
    addr & REGION_MASK[index]
}

/// A distinct region of the PlayStation physical memory map.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusRegion {
    /// 2MB main RAM (`0x0000_0000..0x0020_0000`), mirrored up to `0x0080_0000`.
    MainRam,
    /// Expansion region 1 (`0x1F00_0000..0x1F80_0000`). Parallel port / ROM.
    Expansion1,
    /// 1KB fast scratchpad / D-cache (`0x1F80_0000..0x1F80_0400`).
    Scratchpad,
    /// Hardware I/O registers (`0x1F80_1000..0x1F80_3000`).
    IoPorts,
    /// Expansion region 2 (`0x1FA0_0000..0x1FC0_0000`). Debug / DTL board.
    Expansion2,
    /// 512KB BIOS ROM (`0x1FC0_0000..0x1FC8_0000`). Read-only.
    Bios,
    /// Cache control register (`0xFFFE_0000`), physically outside KSEG masking.
    CacheControl,
    /// Any address not covered by a known region.
    Unmapped,
}

/// Main RAM size in bytes (2MB).
pub const MAIN_RAM_SIZE: usize = 2 * 1024 * 1024;
/// Scratchpad size in bytes (1KB).
pub const SCRATCHPAD_SIZE: usize = 1024;
/// BIOS ROM size in bytes (512KB).
pub const BIOS_SIZE: usize = 512 * 1024;

/// Mask applied to a main-RAM physical address to fold the 2MB region and its
/// mirrors down to the backing store.
pub const MAIN_RAM_MASK: u32 = 0x001F_FFFF;

/// Decodes a physical `phys` address (post-[`mask_region`]) to a [`BusRegion`].
///
/// Main RAM is mirrored four times across `0x0000_0000..0x0080_0000`; callers
/// fold the offset with [`MAIN_RAM_MASK`].
///
/// # Examples
///
/// ```
/// use psoxide_core::bus::{map_region, BusRegion};
///
/// assert_eq!(map_region(0x0000_0000), BusRegion::MainRam);
/// assert_eq!(map_region(0x0010_0000), BusRegion::MainRam);
/// assert_eq!(map_region(0x1F80_0000), BusRegion::Scratchpad);
/// assert_eq!(map_region(0x1F80_1000), BusRegion::IoPorts);
/// assert_eq!(map_region(0x1FC0_0000), BusRegion::Bios);
/// assert_eq!(map_region(0xFFFE_0000), BusRegion::CacheControl);
/// assert_eq!(map_region(0x0FFF_FFFF), BusRegion::Unmapped);
/// ```
#[must_use]
pub fn map_region(phys: u32) -> BusRegion {
    match phys {
        0x0000_0000..=0x007F_FFFF => BusRegion::MainRam,
        0x1F00_0000..=0x1F7F_FFFF => BusRegion::Expansion1,
        0x1F80_0000..=0x1F80_03FF => BusRegion::Scratchpad,
        0x1F80_1000..=0x1F80_2FFF => BusRegion::IoPorts,
        0x1FA0_0000..=0x1FBF_FFFF => BusRegion::Expansion2,
        0x1FC0_0000..=0x1FC7_FFFF => BusRegion::Bios,
        0xFFFE_0000..=0xFFFE_01FF => BusRegion::CacheControl,
        _ => BusRegion::Unmapped,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mask_region_kuseg_passthrough() {
        assert_eq!(mask_region(0x0000_0000), 0x0000_0000);
        assert_eq!(mask_region(0x0000_1234), 0x0000_1234);
        assert_eq!(mask_region(0x001F_FFFF), 0x001F_FFFF);
        assert_eq!(mask_region(0x7FFF_FFFF), 0x7FFF_FFFF);
    }

    #[test]
    fn mask_region_kseg0_strips_top_bit() {
        assert_eq!(mask_region(0x8000_0000), 0x0000_0000);
        assert_eq!(mask_region(0x9FC0_0000), 0x1FC0_0000);
        assert_eq!(mask_region(0x8020_0000), 0x0020_0000);
    }

    #[test]
    fn mask_region_kseg1_strips_top_three_bits() {
        assert_eq!(mask_region(0xA000_0000), 0x0000_0000);
        assert_eq!(mask_region(0xBFC0_0000), 0x1FC0_0000);
        assert_eq!(mask_region(0xA000_1000), 0x0000_1000);
    }

    #[test]
    fn mask_region_kseg2_passthrough() {
        assert_eq!(mask_region(0xC000_0000), 0xC000_0000);
        assert_eq!(mask_region(0xFFFE_0000), 0xFFFE_0000);
        assert_eq!(mask_region(0xFFFF_FFFF), 0xFFFF_FFFF);
    }

    #[test]
    fn mask_region_matches_table_for_all_segments() {
        for seg in 0u32..8 {
            let base = seg << 29;
            assert_eq!(
                mask_region(base | 0x0001_0000),
                (base | 0x0001_0000) & REGION_MASK[seg as usize]
            );
        }
    }

    #[test]
    fn map_region_main_ram_and_mirrors() {
        assert_eq!(map_region(0x0000_0000), BusRegion::MainRam);
        assert_eq!(map_region(0x001F_FFFF), BusRegion::MainRam);
        // Mirrors.
        assert_eq!(map_region(0x0020_0000), BusRegion::MainRam);
        assert_eq!(map_region(0x0060_0000), BusRegion::MainRam);
        assert_eq!(map_region(0x007F_FFFF), BusRegion::MainRam);
    }

    #[test]
    fn map_region_boundaries() {
        // Just below scratchpad is unmapped.
        assert_eq!(map_region(0x1F7F_FFFF), BusRegion::Expansion1);
        assert_eq!(map_region(0x1F80_0000), BusRegion::Scratchpad);
        assert_eq!(map_region(0x1F80_03FF), BusRegion::Scratchpad);
        assert_eq!(map_region(0x1F80_0400), BusRegion::Unmapped);
        assert_eq!(map_region(0x1F80_0FFF), BusRegion::Unmapped);
        assert_eq!(map_region(0x1F80_1000), BusRegion::IoPorts);
        assert_eq!(map_region(0x1F80_2FFF), BusRegion::IoPorts);
        assert_eq!(map_region(0x1F80_3000), BusRegion::Unmapped);
    }

    #[test]
    fn map_region_bios_bounds() {
        assert_eq!(map_region(0x1FBF_FFFF), BusRegion::Expansion2);
        assert_eq!(map_region(0x1FC0_0000), BusRegion::Bios);
        assert_eq!(map_region(0x1FC7_FFFF), BusRegion::Bios);
        assert_eq!(map_region(0x1FC8_0000), BusRegion::Unmapped);
    }

    #[test]
    fn map_region_expansion_regions() {
        assert_eq!(map_region(0x1F00_0000), BusRegion::Expansion1);
        assert_eq!(map_region(0x1FA0_0000), BusRegion::Expansion2);
    }

    #[test]
    fn map_region_cache_control() {
        assert_eq!(map_region(0xFFFE_0000), BusRegion::CacheControl);
        assert_eq!(map_region(0xFFFE_0130), BusRegion::CacheControl);
    }

    #[test]
    fn map_region_unmapped() {
        assert_eq!(map_region(0x0FFF_FFFF), BusRegion::Unmapped);
        assert_eq!(map_region(0x1E00_0000), BusRegion::Unmapped);
        assert_eq!(map_region(0x8000_0000), BusRegion::Unmapped);
    }

    #[test]
    fn reset_vector_masks_into_bios() {
        // CPU reset PC (KSEG1 uncached BIOS entry) must resolve to BIOS.
        let phys = mask_region(0xBFC0_0000);
        assert_eq!(phys, 0x1FC0_0000);
        assert_eq!(map_region(phys), BusRegion::Bios);
    }
}
