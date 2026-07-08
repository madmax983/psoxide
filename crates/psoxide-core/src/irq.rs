//! Interrupt controller (I_STAT / I_MASK).
//!
//! The PlayStation routes all device interrupts through a single controller
//! that ORs pending requests into `I_STAT` (0x1F80_1070) and gates them with
//! `I_MASK` (0x1F80_1074). When any unmasked bit is pending the controller
//! asserts the CPU's hardware interrupt line (cop0 CAUSE bit 10 / IP2).
//!
//! Only the request/mask book-keeping lives here; delivery to the CPU is wired
//! up in the core step loop.

use serde::{Deserialize, Serialize};

/// Interrupt source bit indices within `I_STAT` / `I_MASK`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum IrqLine {
    /// Vertical blank (bit 0).
    VBlank = 0,
    /// GPU (bit 1).
    Gpu = 1,
    /// CD-ROM (bit 2).
    CdRom = 2,
    /// DMA (bit 3).
    Dma = 3,
    /// Timer 0 (bit 4).
    Timer0 = 4,
    /// Timer 1 (bit 5).
    Timer1 = 5,
    /// Timer 2 (bit 6).
    Timer2 = 6,
    /// Controller / memory-card SIO (bit 7).
    Sio = 7,
    /// Serial port (bit 8).
    Serial = 8,
    /// SPU (bit 9).
    Spu = 9,
    /// Lightpen / PIO (bit 10).
    Pio = 10,
}

impl IrqLine {
    /// Returns the bit index of this interrupt line.
    #[must_use]
    pub fn bit(self) -> u32 {
        self as u32
    }
}

/// Interrupt controller state.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Irq {
    /// Pending interrupt requests (`I_STAT`).
    pub i_stat: u32,
    /// Interrupt enable mask (`I_MASK`).
    pub i_mask: u32,
}

impl Default for Irq {
    fn default() -> Self {
        Self::new()
    }
}

impl Irq {
    /// Creates a controller with no pending or enabled interrupts.
    #[must_use]
    pub fn new() -> Self {
        Self {
            i_stat: 0,
            i_mask: 0,
        }
    }

    /// Raises interrupt `line`, setting its `I_STAT` bit.
    pub fn set(&mut self, line: IrqLine) {
        self.i_stat |= 1 << line.bit();
    }

    /// Raises the interrupt at raw bit `bit`.
    pub fn set_bit(&mut self, bit: u32) {
        self.i_stat |= 1 << bit;
    }

    /// Reads `I_STAT`.
    #[must_use]
    pub fn read_stat(&self) -> u32 {
        self.i_stat
    }

    /// Reads `I_MASK`.
    #[must_use]
    pub fn read_mask(&self) -> u32 {
        self.i_mask
    }

    /// Acknowledges interrupts. Hardware clears any `I_STAT` bit that is written
    /// as 0, so the effective operation is `i_stat &= val`.
    pub fn write_stat(&mut self, val: u32) {
        self.i_stat &= val;
    }

    /// Writes `I_MASK`.
    pub fn write_mask(&mut self, val: u32) {
        self.i_mask = val;
    }

    /// Returns whether any unmasked interrupt is pending.
    #[must_use]
    pub fn pending(&self) -> bool {
        (self.i_stat & self.i_mask) != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_and_read() {
        let mut irq = Irq::new();
        assert_eq!(irq.read_stat(), 0);
        irq.set(IrqLine::VBlank);
        irq.set(IrqLine::Dma);
        assert_eq!(irq.read_stat(), 0b1001);
    }

    #[test]
    fn ack_clears_written_zero_bits() {
        let mut irq = Irq::new();
        irq.set(IrqLine::VBlank);
        irq.set(IrqLine::Gpu);
        irq.set(IrqLine::Dma);
        assert_eq!(irq.read_stat(), 0b1011);
        // Acknowledge VBlank by writing 0 to bit 0, keeping the others.
        irq.write_stat(!0b0001);
        assert_eq!(irq.read_stat(), 0b1010);
        // Writing 0 clears everything.
        irq.write_stat(0);
        assert_eq!(irq.read_stat(), 0);
    }

    #[test]
    fn mask_and_pending() {
        let mut irq = Irq::new();
        irq.set(IrqLine::VBlank);
        assert!(!irq.pending(), "masked off by default");
        irq.write_mask(1 << IrqLine::VBlank.bit());
        assert!(irq.pending());
        assert_eq!(irq.read_mask(), 1);
        // A pending-but-masked source does not signal.
        irq.write_stat(0);
        irq.set(IrqLine::Gpu);
        assert!(!irq.pending());
    }
}
