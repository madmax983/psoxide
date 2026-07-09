//! Read-back-sane device stubs for regions the BIOS boot path touches but that
//! do not yet have real emulation.
//!
//! These modules cover the memory-mapped register regions Nocash PSX-SPX
//! documents (memory control, cache control, SIO0 joypad) so a real BIOS image
//! can perform its startup register writes without triggering FIFO desync,
//! panics, or bogus reads. Each stub owns a small backing store and returns the
//! last value written; reads from unwritten offsets return documented power-on
//! defaults. (The CD-ROM and SPU windows are no longer stubs — see the real
//! `cdrom` and `spu` controllers.)
//!
//! Only the write-then-read-back contract is implemented — no side effects, no
//! DMA, no interrupts. Once real emulation lands for a device, its region can
//! be moved off the stub and onto the real controller.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use crate::irq::{Irq, IrqLine};

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

/// A digital (SCPH-1080) controller: a 16-bit active-high button bitfield laid
/// out per [`crate::api::Button::bit_mask`] (a **set** bit = pressed). On the
/// wire the controller reports buttons active-low, so the transfer inverts the
/// field before emitting the two data bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct DigitalPad {
    /// Pressed-button bitfield (active-high, PSX bit layout).
    pub buttons: u16,
}

impl DigitalPad {
    /// Digital-pad byte exchange indexed by transfer `phase`. Returns the
    /// response byte the pad shifts back and whether it asserts `/ACK` (which
    /// requests the next byte). The final data byte (phase 4) does **not** ACK,
    /// ending the transfer.
    fn exchange(&self, phase: u8) -> (u8, bool) {
        let inv = !self.buttons; // active-low on the wire
        match phase {
            // Address byte (0x01) acknowledged: pad answers idle, requests more.
            0 => (0xFF, true),
            // Command byte (expect 0x42 "read"): low byte of the digital ID 0x5A41.
            1 => (0x41, true),
            // High byte of the digital ID.
            2 => (0x5A, true),
            // Buttons, low byte (Select..Left).
            3 => ((inv & 0xFF) as u8, true),
            // Buttons, high byte (L2..Square) — last byte, no ACK.
            4 => ((inv >> 8) as u8, false),
            // Past the end of a digital exchange: nothing more to send.
            _ => (0xFF, false),
        }
    }
}

/// A device that can be attached to a controller/memory-card slot. Enum
/// dispatch (family pattern, no trait objects) so analog / DualShock variants
/// can be added later without changing the SIO0 transfer plumbing.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PadDevice {
    /// A standard digital pad.
    Digital(DigitalPad),
    /// No device in this slot.
    Disconnected,
}

impl Default for PadDevice {
    fn default() -> Self {
        PadDevice::Digital(DigitalPad::default())
    }
}

impl PadDevice {
    /// One byte of the slot device's exchange. Returns `(response, ack)`.
    fn exchange(&self, phase: u8) -> (u8, bool) {
        match self {
            PadDevice::Digital(pad) => pad.exchange(phase),
            // Nothing on the bus: pad answers open-bus and never ACKs.
            PadDevice::Disconnected => (0xFF, false),
        }
    }
}

/// The device currently addressed by an in-progress SIO0 transfer, selected by
/// the first (address) byte the CPU shifts out.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum TransferTarget {
    /// No transfer in progress / no device addressed.
    #[default]
    None,
    /// Controller addressed (address byte 0x01).
    Pad,
    /// Memory card addressed (address byte 0x81).
    MemoryCard,
}

/// SIO0 (controller / memory-card serial port) with a real digital-pad
/// transfer model.
///
/// The register file at 0x1F80_1040..0x1F80_105F exposes five registers:
///
/// | Offset | Register                    | Access                          |
/// |--------|-----------------------------|---------------------------------|
/// | 0x1040 | JOY_RX_DATA / JOY_TX_DATA   | read pops RX FIFO; write shifts |
/// | 0x1044 | JOY_STAT                    | read-only, synthesized          |
/// | 0x1048 | JOY_MODE                    | u16 latch                       |
/// | 0x104A | JOY_CTRL                    | u16, acted on (TXEN/DTR/ACK/...) |
/// | 0x104E | JOY_BAUD                    | u16 latch                       |
///
/// A CPU write to JOY_TX_DATA performs one full-duplex byte exchange with the
/// addressed device: the response is pushed into the RX FIFO and, if the device
/// ACKs, an ACK is scheduled [`ACK_DELAY_CYCLES`] cycles out. When it fires,
/// JOY_STAT bit 9 (interrupt request) latches and — if JOY_CTRL bit 12
/// (ack-interrupt-enable) is set — [`IrqLine::Sio`] is raised.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Sio0 {
    /// 0x1F80_1048 JOY_MODE (16-bit latch).
    pub mode: u16,
    /// 0x1F80_104A JOY_CTRL (16-bit control).
    pub ctrl: u16,
    /// 0x1F80_104E JOY_BAUD (16-bit latch).
    pub baud: u16,
    /// Receive FIFO (device response bytes), capped at [`Self::RX_FIFO_CAP`].
    rx_fifo: VecDeque<u8>,
    /// JOY_STAT bit 9 (interrupt request) latch.
    stat_irq: bool,
    /// Momentary `/ACK` input level (JOY_STAT bit 7).
    ack_level: bool,
    /// Countdown to the pending `/ACK`; `< 0` means no ACK is scheduled.
    ack_timer: i32,
    /// Position within the current transfer (0 = expecting the address byte).
    phase: u8,
    /// Device addressed by the in-progress transfer.
    active: TransferTarget,
    /// The two slot devices (slot 0 / slot 1), selected by JOY_CTRL bit 13.
    pads: [PadDevice; 2],
}

impl Default for Sio0 {
    fn default() -> Self {
        Self::new()
    }
}

impl Sio0 {
    /// JOY_CTRL bit 0 — TX enable.
    const CTRL_TXEN: u16 = 1 << 0;
    /// JOY_CTRL bit 1 — `/DTR` (asserts the Select line to the pad).
    const CTRL_DTR: u16 = 1 << 1;
    /// JOY_CTRL bit 4 — Acknowledge (write 1 clears STAT bits 3 and 9/IRQ).
    const CTRL_ACK: u16 = 1 << 4;
    /// JOY_CTRL bit 6 — Reset (write 1 zeroes registers and transfer state).
    const CTRL_RESET: u16 = 1 << 6;
    /// JOY_CTRL bit 12 — ACK-interrupt-enable.
    const CTRL_ACK_IEN: u16 = 1 << 12;

    /// Receive-FIFO capacity in bytes.
    const RX_FIFO_CAP: usize = 8;

    /// Approximate `/ACK` latency. Real hardware asserts `/ACK` a few hundred
    /// nanoseconds after each non-final byte; we deliver IRQ7 this many CPU
    /// cycles later, which is enough for BIOS/game busy-wait loops that poll
    /// JOY_STAT.IRQ. Not cycle-accurate.
    const ACK_DELAY_CYCLES: i32 = 100;

    /// Creates a fresh SIO0 with a digital pad in each slot (no buttons held).
    #[must_use]
    pub fn new() -> Self {
        Self {
            mode: 0,
            ctrl: 0,
            baud: 0,
            rx_fifo: VecDeque::new(),
            stat_irq: false,
            ack_level: false,
            ack_timer: -1,
            phase: 0,
            active: TransferTarget::None,
            pads: [PadDevice::default(), PadDevice::default()],
        }
    }

    /// Returns `true` if `phys` falls in the SIO0 register window.
    #[must_use]
    pub fn contains(phys: u32) -> bool {
        matches!(phys, SIO0_BASE..=SIO0_END)
    }

    /// Replaces the pressed-button bitfield for `slot` (0 or 1). A slot that was
    /// disconnected becomes a digital pad. Out-of-range slots are ignored.
    pub fn set_buttons(&mut self, slot: usize, buttons: u16) {
        if let Some(dev) = self.pads.get_mut(slot) {
            match dev {
                PadDevice::Digital(pad) => pad.buttons = buttons,
                PadDevice::Disconnected => {
                    *dev = PadDevice::Digital(DigitalPad { buttons });
                }
            }
        }
    }

    /// Advances the approximate `/ACK` timer by `cycles`. When a scheduled ACK
    /// fires, latches JOY_STAT.IRQ and — if ack-interrupt-enable is set — raises
    /// [`IrqLine::Sio`]. Called once per CPU cycle from `step_cpu`.
    pub fn tick(&mut self, cycles: u32, irq: &mut Irq) {
        if self.ack_timer < 0 {
            return;
        }
        self.ack_timer -= cycles as i32;
        if self.ack_timer <= 0 {
            self.ack_timer = -1;
            self.ack_level = true;
            // STAT.IRQ latches (and the line is raised) only when the CPU has
            // enabled ACK interrupts; the I_MASK bit still gates final delivery.
            if self.ctrl & Self::CTRL_ACK_IEN != 0 {
                self.stat_irq = true;
                irq.set(IrqLine::Sio);
            }
        }
    }

    /// Synthesizes JOY_STAT. Idle (empty RX, no pending IRQ) reads back 0x5.
    fn stat(&self) -> u32 {
        let mut s = (1 << 0) | (1 << 2); // TX ready flag 1 + TX ready flag 2
        if !self.rx_fifo.is_empty() {
            s |= 1 << 1; // RX FIFO not empty
        }
        if self.ack_level {
            s |= 1 << 7; // /ACK input level
        }
        if self.stat_irq {
            s |= 1 << 9; // interrupt request
        }
        s
    }

    /// Pops one byte off the RX FIFO (0xFF when empty — open bus).
    fn pop_rx(&mut self) -> u8 {
        self.rx_fifo.pop_front().unwrap_or(0xFF)
    }

    /// Pushes a device response byte, dropping the oldest if the FIFO is full.
    fn push_rx(&mut self, byte: u8) {
        if self.rx_fifo.len() >= Self::RX_FIFO_CAP {
            self.rx_fifo.pop_front();
        }
        self.rx_fifo.push_back(byte);
    }

    /// Performs one full-duplex byte exchange when TXEN and `/DTR` (Select) are
    /// asserted. Routes the address byte, pushes the device response, and either
    /// schedules an ACK (device requested the next byte) or ends the transfer.
    fn tx_byte(&mut self, tx: u8) {
        if self.ctrl & Self::CTRL_TXEN == 0 || self.ctrl & Self::CTRL_DTR == 0 {
            // No transfer without TX enable + Select asserted; drop the write.
            return;
        }
        if self.phase == 0 {
            // The first byte of a transfer selects the device.
            self.active = match tx {
                0x01 => TransferTarget::Pad,
                0x81 => TransferTarget::MemoryCard,
                _ => TransferTarget::None,
            };
        }
        let slot = ((self.ctrl >> 13) & 1) as usize;
        let (resp, ack) = match self.active {
            TransferTarget::Pad => self.pads[slot].exchange(self.phase),
            // No memory card is present: answer open-bus and never ACK, so a
            // BIOS/game probe (address 0x81) sees "no card" and moves on.
            TransferTarget::MemoryCard | TransferTarget::None => (0xFF, false),
        };
        self.push_rx(resp);
        self.ack_level = false;
        if ack {
            self.ack_timer = Self::ACK_DELAY_CYCLES;
            self.phase = self.phase.wrapping_add(1);
        } else {
            // Final byte (or no device): the transfer ends; the next write to
            // JOY_TX_DATA starts a fresh transfer from the address byte.
            self.ack_timer = -1;
            self.phase = 0;
            self.active = TransferTarget::None;
        }
    }

    /// Applies a JOY_CTRL write, acting on the Reset / Acknowledge / Select
    /// (`/DTR`) control bits.
    fn write_ctrl(&mut self, val: u16) {
        let prev = self.ctrl;
        self.ctrl = val;
        if val & Self::CTRL_RESET != 0 {
            // Reset: zero the registers and all transfer/IRQ state.
            self.mode = 0;
            self.ctrl = 0;
            self.baud = 0;
            self.rx_fifo.clear();
            self.phase = 0;
            self.active = TransferTarget::None;
            self.stat_irq = false;
            self.ack_level = false;
            self.ack_timer = -1;
            return;
        }
        if val & Self::CTRL_ACK != 0 {
            // Acknowledge clears STAT bits 3 and 9 (IRQ) and the /ACK level.
            self.stat_irq = false;
            self.ack_level = false;
        }
        if prev & Self::CTRL_DTR != 0 && val & Self::CTRL_DTR == 0 {
            // Select deasserted between polls: abandon any in-progress transfer
            // so the next poll re-addresses the device from scratch.
            self.phase = 0;
            self.active = TransferTarget::None;
        }
    }

    /// Reads a 32-bit value.
    pub fn read32(&mut self, phys: u32) -> u32 {
        match phys {
            0x1F80_1040 => u32::from(self.pop_rx()) | 0xFFFF_FF00, // pad hi bytes open-bus
            0x1F80_1044 => self.stat(),
            _ => u32::from(self.read16(phys)),
        }
    }

    /// Reads a 16-bit value.
    pub fn read16(&mut self, phys: u32) -> u16 {
        match phys {
            0x1F80_1040 => u16::from(self.pop_rx()) | 0xFF00, // pad high byte open-bus
            0x1F80_1044 => self.stat() as u16,
            0x1F80_1048 => self.mode,
            0x1F80_104A => self.ctrl,
            0x1F80_104E => self.baud,
            _ => 0,
        }
    }

    /// Reads an 8-bit value.
    pub fn read8(&mut self, phys: u32) -> u8 {
        match phys {
            0x1F80_1040 => self.pop_rx(),
            0x1F80_1044 => self.stat() as u8,
            _ => 0,
        }
    }

    /// Writes a 32-bit value (low half then high half of the register pair).
    pub fn write32(&mut self, phys: u32, val: u32) {
        self.write16(phys, val as u16);
        self.write16(phys + 2, (val >> 16) as u16);
    }

    /// Writes a 16-bit value.
    pub fn write16(&mut self, phys: u32, val: u16) {
        match phys {
            0x1F80_1040 => self.tx_byte(val as u8),
            0x1F80_1048 => self.mode = val,
            0x1F80_104A => self.write_ctrl(val),
            0x1F80_104E => self.baud = val,
            _ => {}
        }
    }

    /// Writes an 8-bit value. JOY_TX_DATA is a byte register; the mode/ctrl/baud
    /// latches merge the byte into their low half.
    pub fn write8(&mut self, phys: u32, val: u8) {
        match phys {
            0x1F80_1040 => self.tx_byte(val),
            0x1F80_1048 => self.mode = (self.mode & 0xFF00) | u16::from(val),
            0x1F80_104A => self.write_ctrl((self.ctrl & 0xFF00) | u16::from(val)),
            0x1F80_104E => self.baud = (self.baud & 0xFF00) | u16::from(val),
            _ => {}
        }
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
        // Reads now pop the RX FIFO, so `sio` must be mutable.
        let mut sio = Sio0::new();
        let stat = sio.read16(0x1F80_1044);
        // TX ready (bit 0) and TX ready flag 2 (bit 2).
        assert_ne!(stat & 0x1, 0);
        assert_ne!(stat & 0x4, 0);
        // No RX (bit 1) and no ACK (bit 7). Idle STAT == 0x5.
        assert_eq!(stat & 0x2, 0);
        assert_eq!(stat, 0x5);
    }

    #[test]
    fn sio0_rx_reads_bus_idle() {
        // With no transfer performed the RX FIFO is empty and reads return the
        // open-bus 0xFF (padded to the read width).
        let mut sio = Sio0::new();
        assert_eq!(sio.read8(0x1F80_1040), 0xFF);
        assert_eq!(sio.read16(0x1F80_1040), 0xFFFF);
        assert_eq!(sio.read32(0x1F80_1040), 0xFFFF_FFFF);
    }

    #[test]
    fn sio0_mode_ctrl_baud_roundtrip() {
        let mut sio = Sio0::new();
        sio.write16(0x1F80_1048, 0x1234);
        // JOY_CTRL now has side effects: use a value with no Reset (bit 6) or
        // Acknowledge (bit 4) bit set so it latches verbatim. 0x1002 = /DTR +
        // ack-interrupt-enable.
        sio.write16(0x1F80_104A, 0x1002);
        sio.write16(0x1F80_104E, 0x9ABC);
        assert_eq!(sio.read16(0x1F80_1048), 0x1234);
        assert_eq!(sio.read16(0x1F80_104A), 0x1002);
        assert_eq!(sio.read16(0x1F80_104E), 0x9ABC);
    }

    /// Drives one digital-pad byte exchange: writes `tx` to JOY_TX_DATA, ticks
    /// past the ACK delay, and returns the response popped from JOY_RX_DATA.
    fn pad_exchange(sio: &mut Sio0, irq: &mut Irq, tx: u8) -> u8 {
        sio.write8(0x1F80_1040, tx);
        // Enough cycles to cross the approximate /ACK latency.
        sio.tick(200, irq);
        sio.read8(0x1F80_1040)
    }

    #[test]
    fn digital_pad_read_sequence_no_buttons() {
        let mut sio = Sio0::new();
        let mut irq = Irq::new();
        // TXEN | /DTR(Select) | ack-interrupt-enable, slot 0.
        sio.write16(0x1F80_104A, 0x1003);

        // Address 0x01, command 0x42, then three read bytes.
        let seq = [0x01u8, 0x42, 0x00, 0x00, 0x00];
        let mut resp = Vec::new();
        for &b in &seq {
            resp.push(pad_exchange(&mut sio, &mut irq, b));
        }
        // No buttons held => both data bytes read back 0xFF.
        assert_eq!(resp, vec![0xFF, 0x41, 0x5A, 0xFF, 0xFF]);

        // After an ACKing byte with ack-interrupt-enable set, STAT.IRQ (bit 9).
        assert_ne!(sio.read32(0x1F80_1044) & (1 << 9), 0);
        // Acknowledge (JOY_CTRL bit 4) clears it while keeping Select asserted.
        sio.write16(0x1F80_104A, 0x1013);
        assert_eq!(sio.read32(0x1F80_1044) & (1 << 9), 0);
    }

    #[test]
    fn digital_pad_read_sequence_with_buttons() {
        let mut sio = Sio0::new();
        let mut irq = Irq::new();
        sio.write16(0x1F80_104A, 0x1003);

        // Cross (bit 14) and Up (bit 4) pressed.
        let mask: u16 = (1 << 14) | (1 << 4);
        sio.set_buttons(0, mask);

        let seq = [0x01u8, 0x42, 0x00, 0x00, 0x00];
        let mut resp = Vec::new();
        for &b in &seq {
            resp.push(pad_exchange(&mut sio, &mut irq, b));
        }
        let inv = !mask;
        assert_eq!(resp[3], (inv & 0xFF) as u8, "buttons low byte");
        assert_eq!(resp[4], (inv >> 8) as u8, "buttons high byte");
    }

    #[test]
    fn memory_card_probe_returns_no_ack() {
        let mut sio = Sio0::new();
        let mut irq = Irq::new();
        // TXEN | /DTR | ack-interrupt-enable.
        sio.write16(0x1F80_104A, 0x1003);

        // Address 0x81 selects the (absent) memory card.
        sio.write8(0x1F80_1040, 0x81);
        // No ACK is scheduled, so ticking raises nothing.
        sio.tick(1000, &mut irq);
        assert!(
            !irq.pending() && irq.read_stat() == 0,
            "no IRQ for absent card"
        );
        // Response is open-bus and the transfer resets.
        assert_eq!(sio.read8(0x1F80_1040), 0xFF);
        assert_eq!(sio.read32(0x1F80_1044) & (1 << 9), 0, "no STAT.IRQ");
    }

    #[test]
    fn ack_raises_irq7() {
        let mut sio = Sio0::new();
        let mut irq = Irq::new();
        // Unmask the SIO line (bit 7) so pending() reflects delivery.
        irq.write_mask(1 << 7);
        // TXEN | /DTR | ack-interrupt-enable.
        sio.write16(0x1F80_104A, 0x1003);

        sio.write8(0x1F80_1040, 0x01); // address the pad => ACKs
        assert!(!irq.pending(), "ACK is delayed, not immediate");
        sio.tick(Sio0::ACK_DELAY_CYCLES as u32, &mut irq);
        assert_ne!(irq.read_stat() & (1 << 7), 0, "I_STAT bit 7 set");
        assert!(irq.pending(), "unmasked SIO IRQ pending");
    }

    #[test]
    fn sio0_stat_idle_is_five() {
        let mut sio = Sio0::new();
        assert_eq!(sio.read32(0x1F80_1044), 0x5);
    }
}
