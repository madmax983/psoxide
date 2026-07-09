//! Read-back-sane device stubs for regions the BIOS boot path touches but that
//! do not yet have real emulation.
//!
//! These modules cover the memory-mapped register regions Nocash PSX-SPX
//! documents (memory control, cache control, SIO0 joypad, CD-ROM, SPU) so a
//! real BIOS image can perform its startup register writes without triggering
//! FIFO desync, panics, or bogus reads. Each stub owns a small backing store
//! and returns the last value written; reads from unwritten offsets return
//! documented power-on defaults.
//!
//! Only the write-then-read-back contract is implemented — no side effects, no
//! DMA, no interrupts. Once real emulation lands for a device, its region can
//! be moved off the stub and onto the real controller.

use serde::{Deserialize, Serialize};

/// Physical base of the memory-control register block.
pub const MEMCTRL_BASE: u32 = 0x1F80_1000;
/// Physical end (inclusive) of the memory-control register block.
pub const MEMCTRL_END: u32 = 0x1F80_1023;
/// RAM_SIZE register at 0x1F80_1060 (technically separate but conceptually the
/// same family; the BIOS writes it during boot).
pub const RAM_SIZE_REG: u32 = 0x1F80_1060;

/// Cache-control register (`CACHE_CTRL`), lives outside KSEG masking at
/// `0xFFFE_0130`.
pub const CACHE_CTRL_REG: u32 = 0xFFFE_0130;

/// Physical base of the SIO0 / joypad register window.
pub const SIO0_BASE: u32 = 0x1F80_1040;
/// Physical end (inclusive) of the SIO0 / joypad register window.
pub const SIO0_END: u32 = 0x1F80_105F;

/// Physical base of the SPU register window.
pub const SPU_BASE: u32 = 0x1F80_1C00;
/// Physical end (inclusive) of the SPU register window.
pub const SPU_END: u32 = 0x1F80_1FFF;

/// Memory-control register stub. Backs the nine 32-bit config words the BIOS
/// programs at 0x1F80_1000..0x1F80_1023 plus the RAM_SIZE register at
/// 0x1F80_1060.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MemCtrl {
    /// 0x1F80_1000..0x1F80_1020 — Expansion base, delay/size registers, and
    /// the COMMON delay register (9 words).
    regs: [u32; 9],
    /// 0x1F80_1060 — RAM_SIZE.
    ram_size: u32,
}

impl Default for MemCtrl {
    fn default() -> Self {
        Self::new()
    }
}

impl MemCtrl {
    /// Post-reset defaults per Nocash PSX-SPX (Expansion 1 base = 0x1F00_0000,
    /// Expansion 2 base = 0x1F80_2000; the delay/size words are left zero
    /// because real BIOSes always reprogram them).
    #[must_use]
    pub fn new() -> Self {
        let mut regs = [0u32; 9];
        regs[0] = 0x1F00_0000; // Expansion 1 base
        regs[1] = 0x1F80_2000; // Expansion 2 base
        Self {
            regs,
            ram_size: 0x0000_0B88, // 2MB, per SPX default
        }
    }

    /// Returns `true` if `phys` falls in a memory-control register.
    #[must_use]
    pub fn contains(phys: u32) -> bool {
        matches!(phys, MEMCTRL_BASE..=MEMCTRL_END) || phys & !0x3 == RAM_SIZE_REG
    }

    /// Reads a 32-bit value at `phys`.
    #[must_use]
    pub fn read32(&self, phys: u32) -> u32 {
        if phys & !0x3 == RAM_SIZE_REG {
            return self.ram_size;
        }
        let idx = ((phys - MEMCTRL_BASE) / 4) as usize;
        self.regs.get(idx).copied().unwrap_or(0)
    }

    /// Writes a 32-bit value at `phys`.
    pub fn write32(&mut self, phys: u32, val: u32) {
        if phys & !0x3 == RAM_SIZE_REG {
            self.ram_size = val;
            return;
        }
        let idx = ((phys - MEMCTRL_BASE) / 4) as usize;
        if let Some(slot) = self.regs.get_mut(idx) {
            *slot = val;
        }
    }
}

/// Cache-control register stub. The BIOS programs this early with values such
/// as 0x0001_E988 to enable the scratchpad + I-cache; we just store it.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct CacheCtrl {
    /// Last value written to `CACHE_CTRL`.
    pub value: u32,
}

impl CacheCtrl {
    /// Creates a controller with the reset value.
    #[must_use]
    pub fn new() -> Self {
        Self { value: 0 }
    }

    /// Reads the cache-control register.
    #[must_use]
    pub fn read32(&self) -> u32 {
        self.value
    }

    /// Writes the cache-control register.
    pub fn write32(&mut self, val: u32) {
        self.value = val;
    }
}

/// SIO0 (controller / memory-card serial port) stub in "no controller"
/// configuration. Reads of the status register report "TX ready / TX empty"
/// and reads of the RX FIFO return 0xFF (bus-idle, no device on the port).
///
/// The register file at 0x1F80_1040..0x1F80_105F is small (five distinct
/// registers plus padding) so we back it with a plain 32-byte buffer plus a
/// status/mode/control triple that we synthesize on read.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Sio0 {
    /// 0x1F80_1048 SIO_MODE (16-bit).
    pub mode: u16,
    /// 0x1F80_104A SIO_CTRL (16-bit).
    pub ctrl: u16,
    /// 0x1F80_104E SIO_BAUD (16-bit).
    pub baud: u16,
}

impl Default for Sio0 {
    fn default() -> Self {
        Self::new()
    }
}

impl Sio0 {
    /// Creates a fresh controller in "no controller attached" state.
    #[must_use]
    pub fn new() -> Self {
        Self {
            mode: 0,
            ctrl: 0,
            baud: 0,
        }
    }

    /// Returns `true` if `phys` falls in the SIO0 register window.
    #[must_use]
    pub fn contains(phys: u32) -> bool {
        matches!(phys, SIO0_BASE..=SIO0_END)
    }

    /// SIO_STAT bits: TX ready (0), TX empty (2). No RX data, no ACK.
    const STAT_IDLE: u32 = (1 << 0) | (1 << 2);

    /// Reads a 32-bit value.
    #[must_use]
    pub fn read32(&self, phys: u32) -> u32 {
        match phys {
            0x1F80_1040 => 0xFFFF_FFFF, // RX FIFO — bus-idle
            0x1F80_1044 => Self::STAT_IDLE,
            _ => u32::from(self.read16(phys)),
        }
    }

    /// Reads a 16-bit value.
    #[must_use]
    pub fn read16(&self, phys: u32) -> u16 {
        match phys {
            0x1F80_1040 => 0xFFFF,
            0x1F80_1044 => Self::STAT_IDLE as u16,
            0x1F80_1048 => self.mode,
            0x1F80_104A => self.ctrl,
            0x1F80_104E => self.baud,
            _ => 0,
        }
    }

    /// Reads an 8-bit value.
    #[must_use]
    pub fn read8(&self, phys: u32) -> u8 {
        match phys {
            0x1F80_1040 => 0xFF,
            0x1F80_1044 => Self::STAT_IDLE as u8,
            _ => 0,
        }
    }

    /// Writes a 32-bit value. TX writes are dropped; the mode/ctrl/baud
    /// registers latch as-is.
    pub fn write32(&mut self, phys: u32, val: u32) {
        self.write16(phys, val as u16);
        self.write16(phys + 2, (val >> 16) as u16);
    }

    /// Writes a 16-bit value.
    pub fn write16(&mut self, phys: u32, val: u16) {
        match phys {
            0x1F80_1048 => self.mode = val,
            0x1F80_104A => self.ctrl = val,
            0x1F80_104E => self.baud = val,
            _ => {}
        }
    }

    /// Writes an 8-bit value. TX writes are dropped.
    pub fn write8(&mut self, _phys: u32, _val: u8) {}
}

/// SPU register stub. The SPU occupies a full 0x400-byte window (0x1F80_1C00..
/// 0x1F80_1FFF). Real audio emulation is out of scope for boot; this stub is
/// a plain byte-addressable backing store so the BIOS's SPU-reset sequence
/// (which reads back register defaults, sets voice keys, etc.) does not
/// wedge.
///
/// The SPU control register (SPUCNT, 0x1F80_1DAA) and status register (SPUSTAT,
/// 0x1F80_1DAE) get a small amount of extra care: SPUSTAT is synthesized to
/// mirror the low bits of SPUCNT the way real hardware does, which is enough
/// for BIOS SPU init to complete.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Spu {
    /// 1024-byte register file, initialized to zero.
    #[serde(with = "spu_regs_serde")]
    regs: Box<[u8; SPU_REG_BYTES]>,
}

/// Size of the SPU register window (1KB).
pub const SPU_REG_BYTES: usize = 1024;

impl Default for Spu {
    fn default() -> Self {
        Self::new()
    }
}

impl Spu {
    /// Creates a fresh SPU register file (all zero).
    #[must_use]
    pub fn new() -> Self {
        Self {
            regs: Box::new([0; SPU_REG_BYTES]),
        }
    }

    /// Returns `true` if `phys` falls in the SPU register window.
    #[must_use]
    pub fn contains(phys: u32) -> bool {
        matches!(phys, SPU_BASE..=SPU_END)
    }

    /// Reads an 8-bit value.
    #[must_use]
    pub fn read8(&self, phys: u32) -> u8 {
        let off = (phys - SPU_BASE) as usize;
        self.regs.get(off).copied().unwrap_or(0)
    }

    /// Reads a 16-bit value.
    #[must_use]
    pub fn read16(&self, phys: u32) -> u16 {
        // SPUSTAT mirrors the low six bits of SPUCNT.
        if phys == 0x1F80_1DAE {
            let cnt = self.read16(0x1F80_1DAA);
            return cnt & 0x3F;
        }
        u16::from_le_bytes([self.read8(phys), self.read8(phys.wrapping_add(1))])
    }

    /// Reads a 32-bit value.
    #[must_use]
    pub fn read32(&self, phys: u32) -> u32 {
        u32::from_le_bytes([
            self.read8(phys),
            self.read8(phys.wrapping_add(1)),
            self.read8(phys.wrapping_add(2)),
            self.read8(phys.wrapping_add(3)),
        ])
    }

    /// Writes an 8-bit value.
    pub fn write8(&mut self, phys: u32, val: u8) {
        let off = (phys - SPU_BASE) as usize;
        if let Some(slot) = self.regs.get_mut(off) {
            *slot = val;
        }
    }

    /// Writes a 16-bit value.
    pub fn write16(&mut self, phys: u32, val: u16) {
        let b = val.to_le_bytes();
        self.write8(phys, b[0]);
        self.write8(phys.wrapping_add(1), b[1]);
    }

    /// Writes a 32-bit value.
    pub fn write32(&mut self, phys: u32, val: u32) {
        let b = val.to_le_bytes();
        self.write8(phys, b[0]);
        self.write8(phys.wrapping_add(1), b[1]);
        self.write8(phys.wrapping_add(2), b[2]);
        self.write8(phys.wrapping_add(3), b[3]);
    }
}

mod spu_regs_serde {
    use super::SPU_REG_BYTES;
    use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error};

    pub fn serialize<S: Serializer>(v: &[u8; SPU_REG_BYTES], s: S) -> Result<S::Ok, S::Error> {
        v.as_slice().serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(
        d: D,
    ) -> Result<Box<[u8; SPU_REG_BYTES]>, D::Error> {
        let v: Vec<u8> = Vec::deserialize(d)?;
        if v.len() != SPU_REG_BYTES {
            return Err(D::Error::custom("spu register file has wrong length"));
        }
        let mut boxed = Box::new([0u8; SPU_REG_BYTES]);
        boxed.copy_from_slice(&v);
        Ok(boxed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn memctrl_write_readback() {
        let mut m = MemCtrl::new();
        m.write32(0x1F80_1008, 0xDEAD_BEEF);
        assert_eq!(m.read32(0x1F80_1008), 0xDEAD_BEEF);
        // Default expansion base retained until written.
        assert_eq!(m.read32(0x1F80_1000), 0x1F00_0000);
    }

    #[test]
    fn memctrl_ram_size_readback() {
        let mut m = MemCtrl::new();
        assert_eq!(m.read32(RAM_SIZE_REG), 0x0000_0B88);
        m.write32(RAM_SIZE_REG, 0x0000_0888);
        assert_eq!(m.read32(RAM_SIZE_REG), 0x0000_0888);
    }

    #[test]
    fn memctrl_contains_bounds() {
        assert!(MemCtrl::contains(0x1F80_1000));
        assert!(MemCtrl::contains(0x1F80_1020));
        assert!(!MemCtrl::contains(0x1F80_1024));
        assert!(MemCtrl::contains(RAM_SIZE_REG));
    }

    #[test]
    fn cache_ctrl_readback() {
        let mut c = CacheCtrl::new();
        assert_eq!(c.read32(), 0);
        c.write32(0x0001_E988);
        assert_eq!(c.read32(), 0x0001_E988);
    }

    #[test]
    fn sio0_status_reports_tx_ready() {
        let sio = Sio0::new();
        let stat = sio.read16(0x1F80_1044);
        // TX ready (bit 0) and TX empty (bit 2).
        assert_ne!(stat & 0x1, 0);
        assert_ne!(stat & 0x4, 0);
        // No RX (bit 1) and no ACK (bit 7).
        assert_eq!(stat & 0x2, 0);
    }

    #[test]
    fn sio0_rx_reads_bus_idle() {
        let sio = Sio0::new();
        assert_eq!(sio.read8(0x1F80_1040), 0xFF);
        assert_eq!(sio.read16(0x1F80_1040), 0xFFFF);
        assert_eq!(sio.read32(0x1F80_1040), 0xFFFF_FFFF);
    }

    #[test]
    fn sio0_mode_ctrl_baud_roundtrip() {
        let mut sio = Sio0::new();
        sio.write16(0x1F80_1048, 0x1234);
        sio.write16(0x1F80_104A, 0x5678);
        sio.write16(0x1F80_104E, 0x9ABC);
        assert_eq!(sio.read16(0x1F80_1048), 0x1234);
        assert_eq!(sio.read16(0x1F80_104A), 0x5678);
        assert_eq!(sio.read16(0x1F80_104E), 0x9ABC);
    }

    #[test]
    fn spu_write_readback() {
        let mut spu = Spu::new();
        spu.write16(0x1F80_1C00, 0x1234);
        assert_eq!(spu.read16(0x1F80_1C00), 0x1234);
        spu.write32(0x1F80_1F00, 0xDEAD_BEEF);
        assert_eq!(spu.read32(0x1F80_1F00), 0xDEAD_BEEF);
    }

    #[test]
    fn spu_status_mirrors_control() {
        let mut spu = Spu::new();
        // SPUCNT
        spu.write16(0x1F80_1DAA, 0x8032);
        // SPUSTAT should mirror low 6 bits (0x32).
        assert_eq!(spu.read16(0x1F80_1DAE), 0x0032);
    }

    #[test]
    fn spu_serde_roundtrip() {
        let mut spu = Spu::new();
        spu.write16(0x1F80_1C00, 0xABCD);
        let s = serde_json::to_string(&spu).unwrap();
        let back: Spu = serde_json::from_str(&s).unwrap();
        assert_eq!(back.read16(0x1F80_1C00), 0xABCD);
    }

    #[test]
    fn spu_serde_rejects_wrong_length() {
        // Craft a JSON blob whose "regs" array is empty.
        let bad = "{\"regs\":[]}";
        let r: Result<Spu, _> = serde_json::from_str(bad);
        assert!(r.is_err());
    }
}
