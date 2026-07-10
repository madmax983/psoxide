//! Cycle-accurate memory-access timing model.
//!
//! Every PlayStation bus access costs a region-dependent number of CPU cycles.
//! Fast internal memory (scratchpad, main RAM) answers in a handful of cycles;
//! the external ROM/expansion/peripheral regions add programmable wait states
//! driven by the memory-control registers (`0x1F80_1008..0x1F80_1020`). This
//! module turns those registers into a per-access cycle cost that the CPU step
//! loop charges to the hardware timers, so a guest that measures access latency
//! (JaCzekanski `ps1-tests` `cpu/access-time`) sees region-aware numbers instead
//! of a flat one-cycle-per-instruction model.
//!
//! ## The wait-state formula (Nocash PSX-SPX)
//!
//! Each delay-driven region owns a *Delay/Size* register plus a shared
//! *COM_DELAY* register (`0x1F80_1020`). [`delay_1st_seq`] reproduces the
//! PSX-SPX derivation of the *first-access* and *sequential-access* cycle counts
//! from those two words. A word (32-bit) access on an 8-bit bus decomposes into
//! four bus cycles — one first + three sequential — while a 16-bit bus halves
//! that; [`access_cycles`] applies the decomposition.
//!
//! ## Fixed-cost regions
//!
//! Scratchpad, main RAM, the internal I/O register block, and the cache-control
//! register do not use the wait-state formula — they answer in a fixed number of
//! cycles (see the constants below). These reproduce their `access-time` golden
//! rows directly.
//!
//! ## Golden-match
//!
//! [`delay_1st_seq`] splits the wait-state cost into a shared base
//! (`read_delay + 2`), a fixed first-access setup (`+2`), and a per-*sequential*
//! bus-turnaround (COM0 recovery + COM2 floating). With that split every
//! delay-driven region reproduces its `access-time` reference row exactly at the
//! default Delay/Size words — including the SPU (18/18/39), CD-ROM (8/14/26), and
//! Expansion 2 (11/26/56) rows that a single first-access-loaded formula
//! previously missed by 1-4 cycles. The fixed-cost regions (RAM, scratchpad,
//! internal I/O, cache-control) return their constants directly.

use crate::bus::mask_region;

// ── Fixed-cost regions ──────────────────────────────────────────────────────

/// Main-RAM access cost in CPU cycles (width-independent). The reference log
/// reports ~5.2/5.3/5.1 for 8/16/32-bit RAM reads.
pub const MAIN_RAM_CYCLES: u32 = 5;
/// Scratchpad (D-cache-as-RAM) access cost — the fastest region, no wait states
/// and width-independent (log: ~1.5/1.1/0.94).
pub const SCRATCHPAD_CYCLES: u32 = 1;
/// Internal I/O register access cost (DMA / JOY / SIO / IRQ / timers / GPU /
/// MDEC / RAM_SIZE). These are on-chip registers that answer in ~3 cycles
/// regardless of width (log rows cluster around 3.0-3.8).
pub const INTERNAL_IO_CYCLES: u32 = 3;

/// The timing class of a physical address. Finer-grained than
/// [`crate::bus::BusRegion`] because the I/O window mixes fast internal
/// registers with wait-stated external devices (SPU, CD-ROM) and the
/// Expansion 2 port, and because the reference test's "EXPANSION2" /
/// "EXPANSION3" labels do not line up with `BusRegion`'s coarser split.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccessClass {
    /// 2MB main RAM and its mirrors. Fixed [`MAIN_RAM_CYCLES`].
    MainRam,
    /// 512KB BIOS ROM. Wait-stated via the BIOS Delay/Size register (`0x1010`).
    Bios,
    /// 1KB scratchpad. Fixed [`SCRATCHPAD_CYCLES`].
    Scratchpad,
    /// Expansion region 1 (`0x1F00_0000`). Delay/Size register `0x1008`.
    Expansion1,
    /// Expansion region 2 (`0x1F80_2000`). Delay/Size register `0x101C`.
    Expansion2,
    /// Expansion region 3 (`0x1FA0_0000`). Delay/Size register `0x100C`.
    Expansion3,
    /// SPU register window (`0x1F80_1C00`). Delay/Size register `0x1014`.
    Spu,
    /// CD-ROM registers (`0x1F80_1800`). Delay/Size register `0x1018`.
    Cdrom,
    /// On-chip I/O registers. Fixed [`INTERNAL_IO_CYCLES`].
    InternalIo,
    /// Cache-control register (`0xFFFE_0130`). 1 cycle for 8-bit, 2 otherwise.
    CacheControl,
    /// Any address not covered by a known region (open bus).
    Unmapped,
}

/// Classifies a *physical* address (post-[`mask_region`]) into an
/// [`AccessClass`]. The SPU and CD-ROM sub-windows are carved out of the I/O
/// block before the catch-all internal-I/O range, and Expansion 2/3 are
/// distinguished from the internal registers.
#[must_use]
pub fn access_class(phys: u32) -> AccessClass {
    match phys {
        0x0000_0000..=0x007F_FFFF => AccessClass::MainRam,
        0x1F00_0000..=0x1F7F_FFFF => AccessClass::Expansion1,
        0x1F80_0000..=0x1F80_03FF => AccessClass::Scratchpad,
        // CD-ROM and SPU are wait-stated external devices inside the I/O window;
        // match them before the internal-I/O catch-all.
        0x1F80_1800..=0x1F80_1803 => AccessClass::Cdrom,
        0x1F80_1C00..=0x1F80_1FFF => AccessClass::Spu,
        0x1F80_1000..=0x1F80_1FFF => AccessClass::InternalIo,
        // Expansion 2 lives at 0x1F80_2000 (test label "EXPANSION2").
        0x1F80_2000..=0x1F80_2FFF => AccessClass::Expansion2,
        // Expansion 3 at 0x1FA0_0000 (test label "EXPANSION3").
        0x1FA0_0000..=0x1FBF_FFFF => AccessClass::Expansion3,
        0x1FC0_0000..=0x1FC7_FFFF => AccessClass::Bios,
        0xFFFE_0000..=0xFFFE_01FF => AccessClass::CacheControl,
        _ => AccessClass::Unmapped,
    }
}

/// The memory-control timing state: the shared COM_DELAY word plus each
/// wait-stated region's Delay/Size word. Built cheaply from
/// [`crate::iostubs::MemCtrl`] each access.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MemTiming {
    /// COM_DELAY (`0x1F80_1020`): COM0..COM3 in nibbles 0..3.
    pub com_delay: u32,
    /// BIOS Delay/Size (`0x1F80_1010`).
    pub bios: u32,
    /// Expansion 1 Delay/Size (`0x1F80_1008`).
    pub exp1: u32,
    /// Expansion 2 Delay/Size (`0x1F80_101C`).
    pub exp2: u32,
    /// Expansion 3 Delay/Size (`0x1F80_100C`).
    pub exp3: u32,
    /// SPU Delay/Size (`0x1F80_1014`).
    pub spu: u32,
    /// CD-ROM Delay/Size (`0x1F80_1018`).
    pub cdrom: u32,
}

impl MemTiming {
    /// Returns the Delay/Size word for a delay-driven class, or `None` for the
    /// fixed-cost / open-bus classes.
    #[inline]
    fn delay_size(&self, class: AccessClass) -> Option<u32> {
        match class {
            AccessClass::Bios => Some(self.bios),
            AccessClass::Expansion1 => Some(self.exp1),
            AccessClass::Expansion2 => Some(self.exp2),
            AccessClass::Expansion3 => Some(self.exp3),
            AccessClass::Spu => Some(self.spu),
            AccessClass::Cdrom => Some(self.cdrom),
            _ => None,
        }
    }
}

/// Derives the (first-access, sequential-access) cycle counts for one bus cycle
/// from a region's `delay_size` word and the shared `com_delay` word, following
/// the Nocash PSX-SPX derivation.
///
/// `delay_size` bit layout: bits0-3 write delay, bits4-7 **read** delay, bit8
/// recovery (COM0 bus-recovery), bit9 hold (COM1), bit10 floating (COM2 data
/// float/hold), bit11 pre-strobe (COM3), bit12 bus width (0 = 8-bit,
/// 1 = 16-bit). `com_delay` bit layout: COM0 in bits0-3, COM1 in 4-7, COM2 in
/// 8-11, COM3 in 12-15.
///
/// This models the **read** path (the `access-time` test times volatile loads).
///
/// ## Model
///
/// Every access — first or sequential — pays a shared base of `read_delay + 2`
/// (the programmed read delay plus a fixed 2-cycle strobe/latch). Beyond that
/// base the first and sequential accesses diverge:
///
/// * The **first** access in a run pays a fixed `+2` address-setup penalty and
///   *no* bus-turnaround: there is no previous access to recover from, so the
///   COM0/COM1/COM2 turnaround terms do not apply. Hence `first = read_delay + 4`
///   uniformly, independent of the recovery/floating/hold flags.
/// * Each **sequential** access instead pays the programmable bus-turnaround —
///   COM0 when the recovery bit (bit8) is set and COM2 when the floating bit
///   (bit10) is set. The hold bit (bit9 / COM1) and the pre-strobe bit
///   (bit11 / COM3) are decoded but do not add to the read-path cost on
///   hardware (they govern write-strobe / address-strobe shaping, which the
///   `access-time` golden's volatile *loads* do not observe).
///
/// This reproduces every delay-driven `access-time` golden row exactly at the
/// default Delay/Size words (BIOS/EXP1 7/13/25, EXP3 6/6/10, SPU 18/18/39,
/// CD-ROM 8/14/26, EXP2 11/26/56).
///
/// # Examples
///
/// ```
/// use psoxide_core::timing::delay_1st_seq;
/// // BIOS default (Delay/Size 0x0013243F, COM_DELAY 0x00031125).
/// assert_eq!(delay_1st_seq(0x0013_243F, 0x0003_1125), (7, 6));
/// ```
#[must_use]
pub fn delay_1st_seq(delay_size: u32, com_delay: u32) -> (u32, u32) {
    let com0 = com_delay & 0xF;
    let com2 = (com_delay >> 8) & 0xF;

    let read_delay = (delay_size >> 4) & 0xF;
    let recovery = delay_size & (1 << 8) != 0;
    let floating = delay_size & (1 << 10) != 0;

    // Base cost shared by every access: the programmed read delay plus a fixed
    // 2-cycle strobe/latch.
    let base = read_delay + 2;

    // The first access pays an extra fixed 2-cycle address setup but no
    // bus-turnaround (nothing precedes it to recover from).
    let first = base + 2;

    // Each sequential access instead pays the programmable bus-turnaround: COM0
    // recovery (bit8) and COM2 floating/data-hold (bit10). The COM1 hold bit
    // (bit9) and COM3 pre-strobe bit (bit11) are decoded elsewhere but do not
    // add to the read-path turnaround on hardware.
    let mut seq = base;
    if recovery {
        seq += com0;
    }
    if floating {
        seq += com2;
    }

    (first, seq)
}

/// Number of bus cycles a `width_bytes` access decomposes into on a bus of
/// `bus_bytes` width — `ceil(width_bytes / bus_bytes)`.
#[inline]
fn bus_cycles(width_bytes: u32, bus_bytes: u32) -> u32 {
    width_bytes.div_ceil(bus_bytes)
}

/// Total CPU-cycle cost of a single `width_bytes` (1/2/4) read access to a
/// region of class `class`, given the current memory-control timing `t`.
///
/// Fixed-cost regions return their constant; delay-driven regions apply
/// [`delay_1st_seq`] and the bus-width decomposition
/// `1st + (n - 1) * seq`.
#[must_use]
pub fn access_cycles(class: AccessClass, width_bytes: u32, t: &MemTiming) -> u32 {
    match class {
        AccessClass::MainRam => MAIN_RAM_CYCLES,
        AccessClass::Scratchpad => SCRATCHPAD_CYCLES,
        AccessClass::InternalIo => INTERNAL_IO_CYCLES,
        // Cache-control answers in 1 cycle for a byte, 2 for wider accesses.
        AccessClass::CacheControl => {
            if width_bytes <= 1 {
                1
            } else {
                2
            }
        }
        // Open bus: treat as a single fast cycle (no golden row exercises it).
        AccessClass::Unmapped => 1,
        // Delay-driven external regions.
        _ => {
            let delay_size = t.delay_size(class).unwrap_or(0);
            let (first, seq) = delay_1st_seq(delay_size, t.com_delay);
            let bus_bytes = if delay_size & (1 << 12) != 0 { 2 } else { 1 };
            let n = bus_cycles(width_bytes, bus_bytes);
            first + (n - 1) * seq
        }
    }
}

/// Total CPU-cycle cost of an instruction fetch at virtual address `pc`.
///
/// The R3000A's instruction cache covers the cached segments (KUSEG / KSEG0).
/// psoxide models the i-cache as a steady-state hit: a fetch from a cached
/// segment costs a single cycle. This is the common case for game/test code and
/// is what the `access-time` loop relies on — it runs cached, so its timed
/// loads report the pure *data* access cost with no fetch penalty folded in.
///
/// Fetches from the uncached KSEG1 window (and the KSEG2 tail) pay the region's
/// full word-access cost, matching hardware running code directly out of
/// uncached ROM. There are no fetch-timing golden rows, so this is deliberately
/// a simplification: the i-cache enable bit and cache-line refill mechanics are
/// not modelled.
#[must_use]
pub fn fetch_cycles(pc: u32, t: &MemTiming) -> u32 {
    // Top three bits select the segment: 0-3 = KUSEG, 4 = KSEG0 (both cached),
    // 5 = KSEG1 (uncached), 6-7 = KSEG2.
    if pc >> 29 < 5 {
        return 1;
    }
    access_cycles(access_class(mask_region(pc)), 4, t)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Default Delay/Size and COM_DELAY the `access-time` test programs before
    /// measuring, per Nocash PSX-SPX.
    const COM_DELAY: u32 = 0x0003_1125;
    const BIOS_DS: u32 = 0x0013_243F;
    const EXP1_DS: u32 = 0x0013_243F;
    const EXP3_DS: u32 = 0x0000_3022;
    const SPU_DS: u32 = 0x2009_31E1;
    const CDROM_DS: u32 = 0x0002_0843;
    const EXP2_DS: u32 = 0x0007_0777;

    fn default_timing() -> MemTiming {
        MemTiming {
            com_delay: COM_DELAY,
            bios: BIOS_DS,
            exp1: EXP1_DS,
            exp2: EXP2_DS,
            exp3: EXP3_DS,
            spu: SPU_DS,
            cdrom: CDROM_DS,
        }
    }

    #[test]
    fn delay_1st_seq_bios_default() {
        assert_eq!(delay_1st_seq(BIOS_DS, COM_DELAY), (7, 6));
    }

    #[test]
    fn access_cycles_bios_matches_golden() {
        let t = default_timing();
        assert_eq!(access_cycles(AccessClass::Bios, 1, &t), 7);
        assert_eq!(access_cycles(AccessClass::Bios, 2, &t), 13);
        assert_eq!(access_cycles(AccessClass::Bios, 4, &t), 25);
    }

    #[test]
    fn access_cycles_fixed_regions() {
        let t = default_timing();
        for w in [1, 2, 4] {
            assert_eq!(access_cycles(AccessClass::MainRam, w, &t), 5);
            assert_eq!(access_cycles(AccessClass::Scratchpad, w, &t), 1);
            assert_eq!(access_cycles(AccessClass::InternalIo, w, &t), 3);
        }
        // Cache-control: 1 for a byte, 2 for half/word.
        assert_eq!(access_cycles(AccessClass::CacheControl, 1, &t), 1);
        assert_eq!(access_cycles(AccessClass::CacheControl, 2, &t), 2);
        assert_eq!(access_cycles(AccessClass::CacheControl, 4, &t), 2);
    }

    #[test]
    fn access_cycles_expansion1_matches_golden() {
        let t = default_timing();
        // Same Delay/Size word as BIOS => same 7/13/25.
        assert_eq!(access_cycles(AccessClass::Expansion1, 1, &t), 7);
        assert_eq!(access_cycles(AccessClass::Expansion1, 2, &t), 13);
        assert_eq!(access_cycles(AccessClass::Expansion1, 4, &t), 25);
    }

    #[test]
    fn access_cycles_expansion3_matches_golden() {
        let t = default_timing();
        // 16-bit bus (bit12 set), read_delay 2 => 1st=6, seq=6.
        assert_eq!(access_cycles(AccessClass::Expansion3, 1, &t), 6);
        assert_eq!(access_cycles(AccessClass::Expansion3, 2, &t), 6);
        assert_eq!(access_cycles(AccessClass::Expansion3, 4, &t), 10);
    }

    #[test]
    fn access_cycles_spu_matches_golden() {
        // Golden SPUCNT: 17.99 / 17.99 / 38.94 (the 32-bit cell is a misaligned
        // word access on hardware; the whole-cycle bus cost is 39). The
        // sequential-turnaround split reproduces this exactly: Delay/Size
        // 0x200931E1 has read_delay=14 + recovery(COM0=5), 16-bit bus, so
        // first=18, seq=21 => 18 / 18 / 39.
        let t = default_timing();
        assert_eq!(delay_1st_seq(SPU_DS, COM_DELAY), (18, 21));
        assert_eq!(
            (
                access_cycles(AccessClass::Spu, 1, &t),
                access_cycles(AccessClass::Spu, 2, &t),
                access_cycles(AccessClass::Spu, 4, &t),
            ),
            (18, 18, 39)
        );
    }

    #[test]
    fn access_cycles_cdrom_matches_golden() {
        // Golden CDROM_STAT: 8.0 / 14.0 / 25.93. Delay/Size 0x00020843 has
        // read_delay=4 with no recovery/floating on the read path, 8-bit bus, so
        // first=8, seq=6 => 8 / 14 / 26.
        let t = default_timing();
        assert_eq!(delay_1st_seq(CDROM_DS, COM_DELAY), (8, 6));
        assert_eq!(
            (
                access_cycles(AccessClass::Cdrom, 1, &t),
                access_cycles(AccessClass::Cdrom, 2, &t),
                access_cycles(AccessClass::Cdrom, 4, &t),
            ),
            (8, 14, 26)
        );
    }

    #[test]
    fn access_cycles_expansion2_matches_golden() {
        // Golden EXPANSION2: 10.99 / 25.99 / 55.98 — the largest 32-bit cost of
        // any aligned region. Delay/Size 0x00070777 has read_delay=7 +
        // recovery(COM0=5) + floating(COM2=1) on the sequential term (the hold
        // bit is decoded but adds nothing to the read path), 8-bit bus, so
        // first=11, seq=15 => 11 / 26 / 56.
        let t = default_timing();
        assert_eq!(delay_1st_seq(EXP2_DS, COM_DELAY), (11, 15));
        assert_eq!(
            (
                access_cycles(AccessClass::Expansion2, 1, &t),
                access_cycles(AccessClass::Expansion2, 2, &t),
                access_cycles(AccessClass::Expansion2, 4, &t),
            ),
            (11, 26, 56)
        );
    }

    #[test]
    fn access_class_maps_test_addresses() {
        assert_eq!(access_class(0x0000_0000), AccessClass::MainRam);
        assert_eq!(access_class(0x1FC0_0000), AccessClass::Bios);
        assert_eq!(access_class(0x1F80_0000), AccessClass::Scratchpad);
        assert_eq!(access_class(0x1F00_0000), AccessClass::Expansion1);
        assert_eq!(access_class(0x1F80_2000), AccessClass::Expansion2);
        assert_eq!(access_class(0x1FA0_0000), AccessClass::Expansion3);
        assert_eq!(access_class(0x1F80_10F0), AccessClass::InternalIo); // DMAC
        assert_eq!(access_class(0x1F80_1044), AccessClass::InternalIo); // JOY
        assert_eq!(access_class(0x1F80_1054), AccessClass::InternalIo); // SIO
        assert_eq!(access_class(0x1F80_1060), AccessClass::InternalIo); // RAM_SIZE
        assert_eq!(access_class(0x1F80_1070), AccessClass::InternalIo); // I_STAT
        assert_eq!(access_class(0x1F80_1100), AccessClass::InternalIo); // TIMER0
        assert_eq!(access_class(0x1F80_1814), AccessClass::InternalIo); // GPUSTAT
        assert_eq!(access_class(0x1F80_1824), AccessClass::InternalIo); // MDECSTAT
        assert_eq!(access_class(0x1F80_1800), AccessClass::Cdrom);
        assert_eq!(access_class(0x1F80_1DAA), AccessClass::Spu); // SPUCNT
        assert_eq!(access_class(0xFFFE_0130), AccessClass::CacheControl);
    }

    #[test]
    fn fetch_cached_segments_are_one_cycle() {
        let t = default_timing();
        // KUSEG and KSEG0 code fetch: i-cache hit model.
        assert_eq!(fetch_cycles(0x0001_0000, &t), 1);
        assert_eq!(fetch_cycles(0x8001_5230, &t), 1);
    }

    #[test]
    fn fetch_uncached_bios_pays_word_cost() {
        let t = default_timing();
        // KSEG1 BIOS reset vector: uncached, pays the 32-bit BIOS access cost.
        assert_eq!(fetch_cycles(0xBFC0_0000, &t), 25);
    }

    #[test]
    fn access_cost_is_monotonic_in_width_for_delay_regions() {
        // A wider access can never be cheaper than a narrower one in the same
        // region (the bus decomposition only adds sequential cycles).
        let t = default_timing();
        for class in [
            AccessClass::Bios,
            AccessClass::Expansion1,
            AccessClass::Expansion2,
            AccessClass::Expansion3,
            AccessClass::Spu,
            AccessClass::Cdrom,
        ] {
            let c8 = access_cycles(class, 1, &t);
            let c16 = access_cycles(class, 2, &t);
            let c32 = access_cycles(class, 4, &t);
            assert!(c8 <= c16, "{class:?}: 8-bit {c8} > 16-bit {c16}");
            assert!(c16 <= c32, "{class:?}: 16-bit {c16} > 32-bit {c32}");
        }
    }
}
