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

use crate::api::ControllerKind;
use crate::irq::{Irq, IrqLine};
use crate::timing::MemTiming;

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
    /// Post-boot defaults per Nocash PSX-SPX. The Expansion 1/2 base words are
    /// the hardware reset values; the Delay/Size and COM_DELAY words are seeded
    /// with the standard values the retail BIOS programs during early boot.
    ///
    /// Seeding the wait-state words (rather than leaving them zero) means the
    /// access-timing model produces realistic region latencies even when a test
    /// harness side-loads a program without booting a real BIOS — the `ps1-tests`
    /// `cpu/access-time` measurement relies on these already being configured.
    /// A running BIOS simply rewrites the same values.
    #[must_use]
    pub fn new() -> Self {
        let mut regs = [0u32; 9];
        regs[0] = 0x1F00_0000; // 0x1000 Expansion 1 base
        regs[1] = 0x1F80_2000; // 0x1004 Expansion 2 base
        regs[2] = 0x0013_243F; // 0x1008 Expansion 1 delay/size
        regs[3] = 0x0000_3022; // 0x100C Expansion 3 delay/size
        regs[4] = 0x0013_243F; // 0x1010 BIOS ROM delay/size
        regs[5] = 0x2009_31E1; // 0x1014 SPU delay/size
        regs[6] = 0x0002_0843; // 0x1018 CD-ROM delay/size
        regs[7] = 0x0007_0777; // 0x101C Expansion 2 delay/size
        regs[8] = 0x0003_1125; // 0x1020 COM_DELAY (COM0=5,COM1=2,COM2=1,COM3=1)
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

    /// Snapshots the wait-state configuration into a [`MemTiming`] for the
    /// access-cost model. The Delay/Size words live at register indices 2/3/4/
    /// 5/6/7 (Exp1 / Exp3 / BIOS / SPU / CD-ROM / Exp2) and COM_DELAY at index 8
    /// — the layout the BIOS and the `access-time` test program.
    #[must_use]
    pub fn timing(&self) -> MemTiming {
        MemTiming {
            com_delay: self.regs[8],
            bios: self.regs[4],
            exp1: self.regs[2],
            exp2: self.regs[7],
            exp3: self.regs[3],
            spu: self.regs[5],
            cdrom: self.regs[6],
        }
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
    /// ending the transfer. The outgoing `tx` byte is unused by a digital pad
    /// (it has no configurable/rumble state) but is accepted so every pad
    /// variant shares one `exchange` signature.
    fn exchange(&mut self, phase: u8, _tx: u8) -> (u8, bool) {
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

/// A Sony DualShock / DualAnalog (SCPH-1200) controller.
///
/// The pad implements the Nocash PSX-SPX serial protocol byte-for-byte through
/// [`AnalogPad::exchange`], which — like [`MemoryCard::exchange`] — is called
/// once per transfer byte with the transfer `phase` and the CPU's outgoing `tx`
/// byte and returns `(response, ack)`.
///
/// Two orthogonal mode bits shape every transaction:
/// - `analog_mode`: when clear the pad reports the digital ID `0x41` and a
///   2-byte (buttons-only) poll, identical to a [`DigitalPad`]; when set it
///   reports the analog ID `0x73` and a 6-byte poll (buttons + both sticks).
/// - `config_mode` (a.k.a. escape mode): entered/left with command `0x43`.
///   While set, the pad reports the config ID `0xF3`, every command runs the
///   9-byte config framing, and the `0x44`..`0x4D` config command set is
///   honoured.
///
/// The ID reported in the header (phases 1–2) reflects the mode at the **start**
/// of the transaction, so a `0x43`-enter shows the previous-mode ID and only
/// flips `config_mode` when the argument byte arrives at phase 3.
///
/// Rumble: while polling in analog/config mode the CPU's outgoing bytes are
/// motor actuations. `vib_map` (set by `0x4D`) routes each of the six poll
/// bytes to a motor — map value `0x00` drives the small motor, `0x01` the large
/// motor, `0xFF` (the default) ignores the byte — and the last routed value is
/// latched into `motor_small` / `motor_large` for the frontend to read.
///
/// The physical "Analog" toggle button on the real pad flips `analog_mode`
/// unless the host has locked it via a `0x44 .. BB=02` command; see
/// [`AnalogPad::press_analog_button`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct AnalogPad {
    /// Pressed-button bitfield (active-high, PSX bit layout incl. L2/R2/L3/R3).
    pub buttons: u16,
    /// Right analog stick X (0x00 left … 0x80 centre … 0xFF right).
    pub rx: u8,
    /// Right analog stick Y (0x00 up … 0x80 centre … 0xFF down).
    pub ry: u8,
    /// Left analog stick X.
    pub lx: u8,
    /// Left analog stick Y.
    pub ly: u8,
    /// Analog mode: `false` → digital ID/poll, `true` → analog ID/poll.
    pub analog_mode: bool,
    /// Analog lock: set by `0x44 .. BB=02` (physical Analog button "disabled").
    /// We have no physical button beyond [`Self::press_analog_button`]; when
    /// locked that helper is a no-op. The flag is stored and reported verbatim.
    pub analog_locked: bool,
    /// Escape / config mode (entered/left via `0x43`).
    pub config_mode: bool,
    /// `0x4D` vibration byte→motor mapping for the six poll bytes. Default all
    /// `0xFF` (every byte ignored).
    pub vib_map: [u8; 6],
    /// Last small-motor actuation routed from a poll's motor bytes.
    pub motor_small: u8,
    /// Last large-motor actuation routed from a poll's motor bytes.
    pub motor_large: u8,
    /// Command byte latched at phase 1 (transient; reset each transfer).
    #[serde(skip)]
    cmd: u8,
    /// First argument byte latched at phase 3 for commands whose later response
    /// bytes depend on it (`0x46` / `0x4C`). Transient; reset each transfer.
    #[serde(skip)]
    arg: u8,
}

impl Default for AnalogPad {
    fn default() -> Self {
        Self {
            buttons: 0,
            rx: 0x80,
            ry: 0x80,
            lx: 0x80,
            ly: 0x80,
            analog_mode: false,
            analog_locked: false,
            config_mode: false,
            vib_map: [0xFF; 6],
            motor_small: 0,
            motor_large: 0,
            cmd: 0,
            arg: 0,
        }
    }
}

impl AnalogPad {
    /// The header ID-low byte for a transaction, reflecting the mode at its
    /// start: `0xF3` in config mode, else `0x73` in analog mode, else `0x41`.
    fn id_low(&self) -> u8 {
        if self.config_mode {
            0xF3
        } else if self.analog_mode {
            0x73
        } else {
            0x41
        }
    }

    /// Number of data bytes after the `0x5A` header byte. A plain digital poll
    /// (`0x42` while neither analog nor config mode) sends two (buttons only);
    /// every other command — analog/config poll, `0x43`, and the config command
    /// set — sends six.
    fn data_len(&self) -> u8 {
        if self.cmd == 0x42 && !self.analog_mode && !self.config_mode {
            2
        } else {
            6
        }
    }

    /// Simulates the physical "Analog" toggle button: flips `analog_mode` unless
    /// the host has locked analog control via `0x44 .. BB=02`. Real pads latch a
    /// button press here; we expose it as an explicit host action since the core
    /// has no physical button.
    pub fn press_analog_button(&mut self) {
        if !self.analog_locked {
            self.analog_mode = !self.analog_mode;
        }
    }

    /// One byte of the poll response (`0x42`), data index `idx` (0-based, i.e.
    /// phase `3 + idx`). In analog/config mode the six outgoing `tx` bytes are
    /// motor actuations routed through `vib_map`.
    fn poll_byte(&mut self, idx: u8, tx: u8) -> u8 {
        // Route the outgoing motor byte for this slot (only the 6-byte analog/
        // config poll carries motor bytes; the 2-byte digital poll does not).
        if self.analog_mode || self.config_mode {
            match self.vib_map.get(idx as usize).copied().unwrap_or(0xFF) {
                0x00 => self.motor_small = tx,
                0x01 => self.motor_large = tx,
                _ => {}
            }
        }
        let inv = !self.buttons; // active-low on the wire
        match idx {
            0 => (inv & 0xFF) as u8,
            1 => (inv >> 8) as u8,
            2 => self.rx,
            3 => self.ry,
            4 => self.lx,
            5 => self.ly,
            _ => 0xFF,
        }
    }

    /// One data byte (after the `0x5A` header) for the latched command, data
    /// index `idx` (phase `3 + idx`) with the CPU's outgoing `tx` byte. This is
    /// also where command side effects (mode toggles, arg latching, motor
    /// routing, vib-map programming) are applied.
    ///
    /// Config-mode-only commands (`0x44`..`0x4D`) are honoured **only** while
    /// `config_mode` is set; issued outside config mode they fall through to the
    /// unknown-command arm, which returns all-zero data with the current-mode
    /// header (a safe no-op ACKed for the full 6-byte config framing).
    fn data_byte(&mut self, idx: u8, tx: u8) -> u8 {
        match self.cmd {
            // Poll: buttons (+ sticks in analog/config mode).
            0x42 => self.poll_byte(idx, tx),
            // Enter/exit config: arg at phase 3 toggles config mode.
            0x43 => {
                if idx == 0 {
                    match tx {
                        0x01 => self.config_mode = true,
                        0x00 => self.config_mode = false,
                        _ => {}
                    }
                }
                0x00
            }
            // Set analog mode / LED (config only).
            0x44 if self.config_mode => {
                match idx {
                    0 => match tx {
                        0x00 => self.analog_mode = false,
                        0x01 => self.analog_mode = true,
                        _ => {}
                    },
                    1 => match tx {
                        0x02 => self.analog_locked = true,
                        0x03 => self.analog_locked = false,
                        _ => {}
                    },
                    _ => {}
                }
                0x00
            }
            // Get LED status (config only): 01 02 LL 02 01 00.
            0x45 if self.config_mode => match idx {
                0 => 0x01,
                1 => 0x02,
                2 => u8::from(self.analog_mode),
                3 => 0x02,
                4 => 0x01,
                _ => 0x00,
            },
            // Constant query with arg at phase 3 (config only).
            0x46 if self.config_mode => {
                if idx == 0 {
                    self.arg = tx;
                }
                // arg 00 => 00 00 01 02 00 0A ; arg 01 => 00 00 01 01 01 14.
                match idx {
                    0 | 1 => 0x00,
                    2 => 0x01,
                    3 => {
                        if self.arg == 0x00 {
                            0x02
                        } else {
                            0x01
                        }
                    }
                    4 => {
                        if self.arg == 0x00 {
                            0x00
                        } else {
                            0x01
                        }
                    }
                    _ => {
                        if self.arg == 0x00 {
                            0x0A
                        } else {
                            0x14
                        }
                    }
                }
            }
            // Constant query (config only): 00 00 02 00 01 00.
            0x47 if self.config_mode => match idx {
                2 => 0x02,
                4 => 0x01,
                _ => 0x00,
            },
            // Constant query with arg at phase 3 (config only).
            0x4C if self.config_mode => {
                if idx == 0 {
                    self.arg = tx;
                }
                // arg 00 => 00 00 00 04 00 00 ; arg 01 => 00 00 00 07 00 00.
                if idx == 3 {
                    if self.arg == 0x00 { 0x04 } else { 0x07 }
                } else {
                    0x00
                }
            }
            // Set vibration mapping (config only): responds with the OLD map,
            // stores the new six bytes.
            0x4D if self.config_mode => {
                let i = idx as usize;
                let old = self.vib_map.get(i).copied().unwrap_or(0xFF);
                if let Some(slot) = self.vib_map.get_mut(i) {
                    *slot = tx;
                }
                old
            }
            // Unknown command (or a config command outside config mode): a safe
            // all-zero no-op ACKed for the full framing.
            _ => 0x00,
        }
    }

    /// Full-duplex byte exchange indexed by the transfer `phase` with the CPU's
    /// outgoing `tx` byte. Returns `(response, ack)`; `ack == false` ends the
    /// transfer.
    fn exchange(&mut self, phase: u8, tx: u8) -> (u8, bool) {
        match phase {
            // Address byte (0x01) selected this pad; reset per-command scratch.
            0 => {
                self.cmd = 0;
                self.arg = 0;
                (0xFF, true)
            }
            // Latch the command byte; answer the mode-dependent ID-low byte.
            1 => {
                self.cmd = tx;
                (self.id_low(), true)
            }
            // ID-high byte.
            2 => (0x5A, true),
            // Data bytes; the final one does not ACK.
            _ => {
                let idx = phase - 3;
                let last = self.data_len().saturating_sub(1);
                let resp = self.data_byte(idx, tx);
                let ack = idx < last;
                if !ack {
                    // Transaction complete: clear the transient scratch so the
                    // post-transaction pad state is canonical (matters for the
                    // `#[serde(skip)]` fields' round-trip and for a clean start
                    // of the next transfer).
                    self.cmd = 0;
                    self.arg = 0;
                }
                (resp, ack)
            }
        }
    }
}

/// Size of a PSX memory card in bytes: 1024 sectors × 128 bytes = 128 KB.
pub const MEMCARD_SIZE: usize = 1024 * 128;
/// Bytes per memory-card sector (frame).
pub const MEMCARD_SECTOR_SIZE: usize = 128;
/// Number of addressable sectors on a memory card.
pub const MEMCARD_SECTOR_COUNT: u16 = 1024;

/// The command byte a memory-card transfer is currently servicing, latched from
/// the second byte of the exchange (phase 1). Reset at the start of every
/// transfer (phase 0) so consecutive commands do not bleed into each other.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
enum McCommand {
    /// No command selected yet / unknown command.
    #[default]
    None,
    /// Read sector (0x52).
    Read,
    /// Write sector (0x57).
    Write,
    /// Get card ID (0x53).
    GetId,
}

/// A Sony PSX memory card (SCPH-1020): a 128 KB block device (1024 × 128-byte
/// sectors) driven over SIO0 with a big-endian 16-bit sector address.
///
/// The card implements the Nocash PSX-SPX serial protocol byte-for-byte through
/// [`MemoryCard::exchange`], which is called once per transfer byte with the
/// transfer `phase` (byte index within the current command, tracked by the
/// [`Sio0`] state machine) and the CPU's outgoing `tx` byte. It returns the
/// response byte the card shifts back and whether it asserts `/ACK` (requesting
/// the next byte); the final byte of each command clears ACK to end the
/// transfer.
///
/// `flag` starts at `0x08` (the "fresh/unwritten card" bit) and is cleared to
/// `0x00` after the first successful write. `dirty` tracks whether the image has
/// been modified since the frontend last flushed it to disk.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MemoryCard {
    /// The 128 KB card image (1024 × 128-byte sectors).
    data: Vec<u8>,
    /// FLAG byte: bit3 (0x08) set = card has not been written since insertion.
    flag: u8,
    /// Set when `data` has been modified since the last frontend flush.
    dirty: bool,
    /// Command currently being serviced (transient; reset each transfer).
    #[serde(skip)]
    cmd: McCommand,
    /// Sector address accumulated from the MSB/LSB address bytes (transient).
    #[serde(skip)]
    addr: u16,
    /// Per-write staging buffer for the 128 incoming data bytes (transient).
    #[serde(skip, default = "zero_sector")]
    write_buf: [u8; MEMCARD_SECTOR_SIZE],
    /// Checksum byte received during a write command (transient).
    #[serde(skip)]
    recv_checksum: u8,
}

/// A zeroed 128-byte sector buffer (`[u8; 128]` has no `Default` impl, so serde
/// needs an explicit default for the skipped `write_buf` field).
fn zero_sector() -> [u8; MEMCARD_SECTOR_SIZE] {
    [0u8; MEMCARD_SECTOR_SIZE]
}

impl Default for MemoryCard {
    fn default() -> Self {
        Self::blank()
    }
}

impl MemoryCard {
    /// FLAG bit indicating the card has not been written since insertion.
    const FLAG_FRESH: u8 = 0x08;

    /// Creates a blank (all-zero) 128 KB card with the fresh-card flag set.
    #[must_use]
    pub fn blank() -> Self {
        Self::from_data(vec![0u8; MEMCARD_SIZE])
    }

    /// Creates a card from an existing image, padding or truncating to the
    /// 128 KB card size. The fresh-card flag is set and the card starts clean.
    #[must_use]
    pub fn from_data(mut data: Vec<u8>) -> Self {
        data.resize(MEMCARD_SIZE, 0);
        Self {
            data,
            flag: Self::FLAG_FRESH,
            dirty: false,
            cmd: McCommand::None,
            addr: 0,
            write_buf: [0u8; MEMCARD_SECTOR_SIZE],
            recv_checksum: 0,
        }
    }

    /// Returns a clone of the card image plus its dirty flag.
    #[must_use]
    pub fn image(&self) -> (Vec<u8>, bool) {
        (self.data.clone(), self.dirty)
    }

    /// Clears the dirty flag (call after flushing the image to disk).
    pub fn clear_dirty(&mut self) {
        self.dirty = false;
    }

    /// Reads a sector's 128 bytes, or all zeros if `addr` is out of range.
    fn sector(&self, addr: u16) -> [u8; MEMCARD_SECTOR_SIZE] {
        let mut out = [0u8; MEMCARD_SECTOR_SIZE];
        if addr < MEMCARD_SECTOR_COUNT {
            let base = addr as usize * MEMCARD_SECTOR_SIZE;
            out.copy_from_slice(&self.data[base..base + MEMCARD_SECTOR_SIZE]);
        }
        out
    }

    /// Computes the protocol checksum of a sector: MSB xor LSB xor all 128 data
    /// bytes.
    fn checksum(addr: u16, sector: &[u8; MEMCARD_SECTOR_SIZE]) -> u8 {
        let mut chk = (addr >> 8) as u8 ^ (addr & 0xFF) as u8;
        for &b in sector.iter() {
            chk ^= b;
        }
        chk
    }

    /// Performs one full-duplex byte exchange, indexed by the transfer `phase`
    /// (byte index within the command) with the CPU's outgoing `tx` byte.
    /// Returns `(response, ack)`; `ack == false` ends the transfer.
    fn exchange(&mut self, phase: u8, tx: u8) -> (u8, bool) {
        // Phase 0 is the address byte (0x81) that selected this card; reset the
        // per-command scratch so consecutive commands start clean.
        if phase == 0 {
            self.cmd = McCommand::None;
            self.addr = 0;
            self.recv_checksum = 0;
            return (0xFF, true);
        }
        // Phase 1 latches the command byte.
        if phase == 1 {
            self.cmd = match tx {
                0x52 => McCommand::Read,
                0x57 => McCommand::Write,
                0x53 => McCommand::GetId,
                // Unknown command: no ACK, transfer ends.
                _ => return (0xFF, false),
            };
            return (self.flag, true);
        }
        match self.cmd {
            McCommand::Read => self.exchange_read(phase, tx),
            McCommand::Write => self.exchange_write(phase, tx),
            McCommand::GetId => self.exchange_get_id(phase),
            // Not reachable (phase >= 2 always has a latched command).
            McCommand::None => (0xFF, false),
        }
    }

    /// Read command (0x52) byte exchange for phases >= 2.
    fn exchange_read(&mut self, phase: u8, tx: u8) -> (u8, bool) {
        match phase {
            2 => (0x5A, true), // ID1
            3 => (0x5D, true), // ID2
            4 => {
                self.addr = u16::from(tx) << 8; // address MSB
                (0x00, true)
            }
            5 => {
                self.addr |= u16::from(tx); // address LSB
                (0x00, true)
            }
            6 => (0x5C, true),                     // ack 1
            7 => (0x5D, true),                     // ack 2
            8 => ((self.addr >> 8) as u8, true),   // confirmed address MSB
            9 => ((self.addr & 0xFF) as u8, true), // confirmed address LSB
            10..=137 => {
                // 128 data bytes (zeros when the sector is out of range).
                let idx = (phase - 10) as usize;
                (self.sector(self.addr)[idx], true)
            }
            138 => (Self::checksum(self.addr, &self.sector(self.addr)), true),
            139 => {
                // End byte: 'G' (0x47) for a good sector, 0xFF for bad address.
                let end = if self.addr < MEMCARD_SECTOR_COUNT {
                    0x47
                } else {
                    0xFF
                };
                (end, false)
            }
            _ => (0xFF, false),
        }
    }

    /// Write command (0x57) byte exchange for phases >= 2.
    fn exchange_write(&mut self, phase: u8, tx: u8) -> (u8, bool) {
        match phase {
            2 => (0x5A, true),
            3 => (0x5D, true),
            4 => {
                self.addr = u16::from(tx) << 8;
                (0x00, true)
            }
            5 => {
                self.addr |= u16::from(tx);
                (0x00, true)
            }
            6..=133 => {
                // Receive 128 data bytes into the staging buffer.
                self.write_buf[(phase - 6) as usize] = tx;
                (0x00, true)
            }
            134 => {
                self.recv_checksum = tx; // received checksum
                (0x00, true)
            }
            135 => (0x5C, true), // ack 1
            136 => (0x5D, true), // ack 2
            137 => (self.commit_write(), false),
            _ => (0xFF, false),
        }
    }

    /// Commits a staged write, returning the end status byte:
    /// - 0xFF (bad sector) if the address is out of range,
    /// - 0x4E ('N', bad checksum) if the received checksum mismatches,
    /// - 0x47 ('G', good) otherwise (writes the sector, sets dirty, clears FLAG).
    fn commit_write(&mut self) -> u8 {
        if self.addr >= MEMCARD_SECTOR_COUNT {
            return 0xFF;
        }
        let expected = Self::checksum(self.addr, &self.write_buf);
        if expected != self.recv_checksum {
            return 0x4E;
        }
        let base = self.addr as usize * MEMCARD_SECTOR_SIZE;
        self.data[base..base + MEMCARD_SECTOR_SIZE].copy_from_slice(&self.write_buf);
        self.dirty = true;
        self.flag &= !Self::FLAG_FRESH;
        0x47
    }

    /// Get-ID command (0x53) byte exchange for phases >= 2.
    fn exchange_get_id(&mut self, phase: u8) -> (u8, bool) {
        match phase {
            2 => (0x5A, true),
            3 => (0x5D, true),
            4 => (0x5C, true),
            5 => (0x5D, true),
            6 => (0x04, true),
            7 => (0x00, true),
            8 => (0x00, true),
            9 => (0x80, false),
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
    /// A DualShock / DualAnalog pad.
    Analog(AnalogPad),
    /// No device in this slot.
    Disconnected,
}

impl Default for PadDevice {
    fn default() -> Self {
        PadDevice::Digital(DigitalPad::default())
    }
}

impl PadDevice {
    /// One byte of the slot device's exchange with the CPU's outgoing `tx`
    /// byte. Returns `(response, ack)`.
    fn exchange(&mut self, phase: u8, tx: u8) -> (u8, bool) {
        match self {
            PadDevice::Digital(pad) => pad.exchange(phase, tx),
            PadDevice::Analog(pad) => pad.exchange(phase, tx),
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
    /// The memory card in each slot (0 / 1), or `None` when the slot is empty.
    #[serde(default = "no_cards")]
    cards: [Option<MemoryCard>; 2],
}

/// serde default for [`Sio0::cards`]: both slots empty. (`[None, None]` cannot
/// be spelled as a `#[serde(default)]` on the field because `Option<MemoryCard>`
/// arrays do not implement `Default` for the array itself.)
fn no_cards() -> [Option<MemoryCard>; 2] {
    [None, None]
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
            cards: [None, None],
        }
    }

    /// Inserts a memory card built from `data` (padded/truncated to 128 KB) into
    /// `slot` (0 or 1). Out-of-range slots are ignored.
    pub fn insert_card(&mut self, slot: usize, data: Vec<u8>) {
        if let Some(c) = self.cards.get_mut(slot) {
            *c = Some(MemoryCard::from_data(data));
        }
    }

    /// Ejects the memory card in `slot`, if any. Out-of-range slots are ignored.
    pub fn eject_card(&mut self, slot: usize) {
        if let Some(c) = self.cards.get_mut(slot) {
            *c = None;
        }
    }

    /// Returns the memory-card image and its dirty flag for `slot`, or `None`
    /// when no card is inserted (or the slot is out of range).
    #[must_use]
    pub fn card_image(&self, slot: usize) -> Option<(Vec<u8>, bool)> {
        self.cards
            .get(slot)
            .and_then(|c| c.as_ref())
            .map(MemoryCard::image)
    }

    /// Clears the dirty flag on the memory card in `slot` (after a frontend
    /// flush). A no-op when no card is inserted.
    pub fn clear_card_dirty(&mut self, slot: usize) {
        if let Some(Some(card)) = self.cards.get_mut(slot) {
            card.clear_dirty();
        }
    }

    /// Returns `true` if `phys` falls in the SIO0 register window.
    #[must_use]
    pub fn contains(phys: u32) -> bool {
        matches!(phys, SIO0_BASE..=SIO0_END)
    }

    /// Replaces the pressed-button bitfield for `slot` (0 or 1). A slot that was
    /// disconnected becomes a digital pad; an analog pad keeps its analog state.
    /// Out-of-range slots are ignored.
    pub fn set_buttons(&mut self, slot: usize, buttons: u16) {
        if let Some(dev) = self.pads.get_mut(slot) {
            match dev {
                PadDevice::Digital(pad) => pad.buttons = buttons,
                PadDevice::Analog(pad) => pad.buttons = buttons,
                PadDevice::Disconnected => {
                    *dev = PadDevice::Digital(DigitalPad { buttons });
                }
            }
        }
    }

    /// Returns the buttons currently held on the pad in `slot` (0 for an empty
    /// slot / out-of-range index).
    fn slot_buttons(&self, slot: usize) -> u16 {
        match self.pads.get(slot) {
            Some(PadDevice::Digital(pad)) => pad.buttons,
            Some(PadDevice::Analog(pad)) => pad.buttons,
            _ => 0,
        }
    }

    /// Sets the device kind attached to `slot` (0 or 1), preserving the held
    /// buttons across the change. Promoting to [`ControllerKind::Analog`]
    /// power-on-resets the analog state (digital mode, centred sticks).
    /// Out-of-range slots are ignored.
    pub fn set_controller_type(&mut self, slot: usize, kind: ControllerKind) {
        let buttons = self.slot_buttons(slot);
        if let Some(dev) = self.pads.get_mut(slot) {
            *dev = match kind {
                ControllerKind::Disconnected => PadDevice::Disconnected,
                ControllerKind::Digital => PadDevice::Digital(DigitalPad { buttons }),
                ControllerKind::Analog => PadDevice::Analog(AnalogPad {
                    buttons,
                    ..AnalogPad::default()
                }),
            };
        }
    }

    /// Updates the analog-stick axes for `slot` (right = `(x, y)`, left =
    /// `(x, y)`; `0x80` is centre). A digital or disconnected slot is **promoted
    /// to an analog pad** (preserving buttons) so a frontend that reports stick
    /// motion always gets analog behaviour. Out-of-range slots are ignored.
    pub fn set_sticks(&mut self, slot: usize, right: (u8, u8), left: (u8, u8)) {
        let buttons = self.slot_buttons(slot);
        if let Some(dev) = self.pads.get_mut(slot) {
            if !matches!(dev, PadDevice::Analog(_)) {
                *dev = PadDevice::Analog(AnalogPad {
                    buttons,
                    ..AnalogPad::default()
                });
            }
            if let PadDevice::Analog(pad) = dev {
                pad.rx = right.0;
                pad.ry = right.1;
                pad.lx = left.0;
                pad.ly = left.1;
            }
        }
    }

    /// Simulates a press of the pad's physical "Analog" button on `slot`,
    /// toggling analog mode unless the host has locked it. A no-op for a
    /// non-analog or out-of-range slot.
    pub fn press_analog_button(&mut self, slot: usize) {
        if let Some(PadDevice::Analog(pad)) = self.pads.get_mut(slot) {
            pad.press_analog_button();
        }
    }

    /// Returns the `(small, large)` motor actuation last latched by the analog
    /// pad in `slot`, or `None` when the slot holds no analog pad.
    #[must_use]
    pub fn motor_state(&self, slot: usize) -> Option<(u8, u8)> {
        match self.pads.get(slot) {
            Some(PadDevice::Analog(pad)) => Some((pad.motor_small, pad.motor_large)),
            _ => None,
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

    /// Number of CPU cycles from the current state until the SIO0 controller
    /// next raises its `/ACK` interrupt, or `None` when no ACK is scheduled, for
    /// the lazy device scheduler.
    ///
    /// The ACK fires when the `ack_timer` countdown reaches zero: with
    /// `ack_timer` cycles remaining, a `tick(ack_timer)` drives it to `<= 0` and
    /// fires, so the deadline is exactly `ack_timer` (clamped to a minimum of 1).
    #[must_use]
    pub fn cycles_to_next_event(&self) -> Option<u64> {
        if self.ack_timer < 0 {
            None
        } else {
            Some((self.ack_timer as u64).max(1))
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
            TransferTarget::Pad => self.pads[slot].exchange(self.phase, tx),
            TransferTarget::MemoryCard => match self.cards[slot].as_mut() {
                // A card is present: run the serial protocol byte exchange.
                Some(card) => card.exchange(self.phase, tx),
                // No card in this slot: answer open-bus and never ACK, so a
                // BIOS/game probe (address 0x81) sees "no card" and moves on.
                None => (0xFF, false),
            },
            TransferTarget::None => (0xFF, false),
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
    fn cycles_to_next_event_predicts_ack() {
        let mut sio = Sio0::new();
        let mut irq = Irq::new();
        // TXEN | /DTR | ack-interrupt-enable.
        sio.write16(0x1F80_104A, 0x1003);

        // Idle: no ACK scheduled.
        assert_eq!(sio.cycles_to_next_event(), None);

        sio.write8(0x1F80_1040, 0x01); // address the pad => schedules an ACK
        let n = sio.cycles_to_next_event().expect("ACK scheduled");
        assert_eq!(n, Sio0::ACK_DELAY_CYCLES as u64);

        // No fire strictly before the predicted cycle; fires exactly at it.
        for _ in 0..(n - 1) {
            sio.tick(1, &mut irq);
        }
        assert_eq!(irq.read_stat(), 0, "no ACK before the predicted cycle");
        sio.tick(1, &mut irq);
        assert_ne!(
            irq.read_stat() & (1 << 7),
            0,
            "ACK fires at predicted cycle"
        );
        assert_eq!(sio.cycles_to_next_event(), None, "ACK consumed");
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

    // ── Memory-card tests ───────────────────────────────────────────────────

    /// Drives one full SIO0 memory-card command through the register file,
    /// returning the device's per-byte responses. Reuses the same write-TX /
    /// tick-ACK / read-RX sequence a game's card driver performs.
    fn card_command(sio: &mut Sio0, irq: &mut Irq, tx: &[u8]) -> Vec<u8> {
        let mut resp = Vec::with_capacity(tx.len());
        for &b in tx {
            resp.push(pad_exchange(sio, irq, b));
        }
        resp
    }

    /// A blank card in slot 0 with JOY_CTRL set to TXEN | /DTR | ack-IEN.
    fn sio_with_card() -> (Sio0, Irq) {
        let mut sio = Sio0::new();
        let irq = Irq::new();
        sio.insert_card(0, vec![0u8; MEMCARD_SIZE]);
        sio.write16(0x1F80_104A, 0x1003);
        (sio, irq)
    }

    #[test]
    fn memory_card_get_id_sequence() {
        let (mut sio, mut irq) = sio_with_card();
        // 0x81 select, 0x53 Get-ID, then eight trailing bytes.
        let tx = [0x81u8, 0x53, 0, 0, 0, 0, 0, 0, 0, 0];
        let resp = card_command(&mut sio, &mut irq, &tx);
        // phase0=FF, phase1=FLAG(0x08), then 5A 5D 5C 5D 04 00 00 80.
        assert_eq!(
            resp,
            vec![0xFF, 0x08, 0x5A, 0x5D, 0x5C, 0x5D, 0x04, 0x00, 0x00, 0x80]
        );
    }

    #[test]
    fn memory_card_flag_is_fresh_before_write() {
        let (mut sio, mut irq) = sio_with_card();
        // The FLAG byte comes back at phase 1 of any command; use a full Get-ID.
        let resp = card_command(&mut sio, &mut irq, &get_id_tx());
        assert_eq!(resp[1], 0x08, "fresh card FLAG has bit3 set");
    }

    /// Builds the 138-byte write-command TX stream for `sector` at `addr` with
    /// the given trailing `checksum` byte.
    fn write_tx(addr: u16, sector: &[u8; 128], checksum: u8) -> Vec<u8> {
        let mut tx = vec![
            0x81u8,
            0x57,
            0x00,
            0x00,
            (addr >> 8) as u8,
            (addr & 0xFF) as u8,
        ];
        tx.extend_from_slice(sector);
        tx.push(checksum); // received checksum
        tx.extend_from_slice(&[0x00, 0x00, 0x00]); // ack1, ack2, end
        tx
    }

    /// Builds the 140-byte read-command TX stream for `addr` (phases 0..=139).
    fn read_tx(addr: u16) -> Vec<u8> {
        let mut tx = vec![
            0x81u8,
            0x52,
            0x00,
            0x00,
            (addr >> 8) as u8,
            (addr & 0xFF) as u8,
        ];
        // ack1, ack2, confirm-hi, confirm-lo, 128 data, checksum, end = 134 more.
        tx.extend(std::iter::repeat_n(0x00u8, 134));
        tx
    }

    /// The complete 10-byte Get-ID TX stream (phases 0..=9); returns the FLAG
    /// byte (response index 1) as its side-observable state without leaving the
    /// card mid-command.
    fn get_id_tx() -> [u8; 10] {
        [0x81, 0x53, 0, 0, 0, 0, 0, 0, 0, 0]
    }

    fn checksum(addr: u16, sector: &[u8; 128]) -> u8 {
        let mut c = (addr >> 8) as u8 ^ (addr & 0xFF) as u8;
        for &b in sector.iter() {
            c ^= b;
        }
        c
    }

    #[test]
    fn memory_card_write_then_read_round_trip() {
        let (mut sio, mut irq) = sio_with_card();
        let addr = 0x0003u16;
        let mut sector = [0u8; 128];
        for (i, b) in sector.iter_mut().enumerate() {
            *b = (i as u8) ^ 0xA5;
        }
        let chk = checksum(addr, &sector);

        // Write.
        let wresp = card_command(&mut sio, &mut irq, &write_tx(addr, &sector, chk));
        assert_eq!(*wresp.last().unwrap(), 0x47, "good write ends with 'G'");

        // FLAG is cleared after a successful write: probe it via a full Get-ID.
        let idresp = card_command(&mut sio, &mut irq, &get_id_tx());
        assert_eq!(idresp[1], 0x00, "FLAG fresh bit cleared after write");

        // Read back and check the data + checksum + end byte.
        let rresp = card_command(&mut sio, &mut irq, &read_tx(addr));
        // Layout: [0]=FF [1]=FLAG [2]=5A [3]=5D [4]=00 [5]=00 [6]=5C [7]=5D
        //         [8]=confirm-hi [9]=confirm-lo [10..138]=data [138]=checksum
        //         [139]=end.
        assert_eq!(rresp[8], (addr >> 8) as u8, "confirmed addr MSB");
        assert_eq!(rresp[9], (addr & 0xFF) as u8, "confirmed addr LSB");
        assert_eq!(&rresp[10..138], &sector[..], "read data matches written");
        assert_eq!(rresp[138], chk, "checksum matches");
        assert_eq!(rresp[139], 0x47, "good read ends with 'G'");
    }

    #[test]
    fn memory_card_bad_checksum_rejects_write() {
        let (mut sio, mut irq) = sio_with_card();
        let addr = 0x0005u16;
        let sector = [0x11u8; 128];
        let good = checksum(addr, &sector);
        // Send a deliberately wrong checksum.
        let wresp = card_command(&mut sio, &mut irq, &write_tx(addr, &sector, good ^ 0xFF));
        assert_eq!(*wresp.last().unwrap(), 0x4E, "bad checksum ends with 'N'");

        // Sector must be unchanged (still zeros): read it back.
        let rresp = card_command(&mut sio, &mut irq, &read_tx(addr));
        assert!(rresp[10..138].iter().all(|&b| b == 0), "sector untouched");
        // FLAG must still be fresh (no successful write happened).
        let idresp = card_command(&mut sio, &mut irq, &get_id_tx());
        assert_eq!(idresp[1], 0x08, "FLAG still fresh after rejected write");
    }

    #[test]
    fn memory_card_out_of_range_write_and_read() {
        let (mut sio, mut irq) = sio_with_card();
        let addr = 0x0400u16; // first out-of-range sector (== 1024).
        let sector = [0x22u8; 128];
        let chk = checksum(addr, &sector);
        let wresp = card_command(&mut sio, &mut irq, &write_tx(addr, &sector, chk));
        assert_eq!(*wresp.last().unwrap(), 0xFF, "out-of-range write ends 0xFF");

        // Read of an out-of-range sector: zeros + 0xFF end byte.
        let rresp = card_command(&mut sio, &mut irq, &read_tx(addr));
        assert!(rresp[10..138].iter().all(|&b| b == 0), "OOR read is zeros");
        assert_eq!(rresp[139], 0xFF, "out-of-range read ends 0xFF");
    }

    #[test]
    fn memory_card_dirty_tracking_and_clear() {
        let mut sio = Sio0::new();
        let mut irq = Irq::new();
        sio.insert_card(0, vec![0u8; MEMCARD_SIZE]);
        sio.write16(0x1F80_104A, 0x1003);
        // Freshly inserted: not dirty.
        assert_eq!(sio.card_image(0).map(|(_, d)| d), Some(false));

        let addr = 0x0001u16;
        let sector = [0x7Fu8; 128];
        let chk = checksum(addr, &sector);
        card_command(&mut sio, &mut irq, &write_tx(addr, &sector, chk));
        assert_eq!(sio.card_image(0).map(|(_, d)| d), Some(true), "dirty set");

        // The image should reflect the written sector.
        let (image, _) = sio.card_image(0).unwrap();
        assert_eq!(&image[128..256], &sector[..], "image reflects write");

        sio.clear_card_dirty(0);
        assert_eq!(
            sio.card_image(0).map(|(_, d)| d),
            Some(false),
            "dirty cleared"
        );
    }

    #[test]
    fn memory_card_unknown_command_no_ack() {
        let (mut sio, mut irq) = sio_with_card();
        // Address select (ACKs) then an unknown command byte (no ACK).
        let r0 = pad_exchange(&mut sio, &mut irq, 0x81);
        assert_eq!(r0, 0xFF);
        // No IRQ pending yet is fine; drive the unknown command.
        let r1 = pad_exchange(&mut sio, &mut irq, 0x99);
        assert_eq!(r1, 0xFF, "unknown command returns open-bus");
        // The transfer ended (no ACK): the next byte restarts at phase 0.
    }

    #[test]
    fn memory_card_insert_eject() {
        let mut sio = Sio0::new();
        assert!(sio.card_image(0).is_none(), "no card at power-on");
        sio.insert_card(0, vec![0u8; MEMCARD_SIZE]);
        assert!(sio.card_image(0).is_some(), "card present after insert");
        sio.eject_card(0);
        assert!(sio.card_image(0).is_none(), "card gone after eject");
        // Slot 1 stays empty throughout.
        assert!(sio.card_image(1).is_none());
    }

    #[test]
    fn memory_card_probe_absent_slot_no_ack() {
        // A card in slot 0 must not answer for a probe of empty slot 1.
        let mut sio = Sio0::new();
        let mut irq = Irq::new();
        sio.insert_card(0, vec![0u8; MEMCARD_SIZE]);
        // Slot select bit 13 -> slot 1, plus TXEN | /DTR | ack-IEN.
        sio.write16(0x1F80_104A, 0x1003 | (1 << 13));
        sio.write8(0x1F80_1040, 0x81);
        sio.tick(1000, &mut irq);
        assert!(!irq.pending(), "empty slot 1 does not ACK");
        assert_eq!(sio.read8(0x1F80_1040), 0xFF);
    }

    #[test]
    fn memory_card_from_data_pads_and_truncates() {
        let short = MemoryCard::from_data(vec![0xAB; 10]);
        let (img, _) = short.image();
        assert_eq!(img.len(), MEMCARD_SIZE, "short image padded to 128 KB");
        assert_eq!(&img[0..10], &[0xAB; 10]);
        assert!(img[10..].iter().all(|&b| b == 0), "padding is zero");

        let long = MemoryCard::from_data(vec![0xCD; MEMCARD_SIZE + 100]);
        assert_eq!(long.image().0.len(), MEMCARD_SIZE, "long image truncated");
    }

    // ── Analog (DualShock) pad tests ────────────────────────────────────────

    /// A fresh SIO0 with an analog pad in slot 0 and JOY_CTRL = TXEN | /DTR |
    /// ack-IEN.
    fn analog_sio() -> (Sio0, Irq) {
        let mut sio = Sio0::new();
        let irq = Irq::new();
        sio.set_controller_type(0, ControllerKind::Analog);
        sio.write16(0x1F80_104A, 0x1003);
        (sio, irq)
    }

    /// Drives one full analog-pad transaction through the SIO0 register file,
    /// returning the device's per-byte responses.
    fn analog_cmd(sio: &mut Sio0, irq: &mut Irq, tx: &[u8]) -> Vec<u8> {
        tx.iter().map(|&b| pad_exchange(sio, irq, b)).collect()
    }

    #[test]
    fn analog_pad_poll_digital_id() {
        // An analog pad with analog_mode == false polls exactly like a digital
        // pad: 0x41 ID and a 2-byte (buttons-only) response.
        let (mut sio, mut irq) = analog_sio();
        let resp = analog_cmd(&mut sio, &mut irq, &[0x01, 0x42, 0x00, 0x00, 0x00]);
        assert_eq!(resp, vec![0xFF, 0x41, 0x5A, 0xFF, 0xFF]);
    }

    #[test]
    fn analog_pad_poll_analog_mode() {
        let (mut sio, mut irq) = analog_sio();
        // Distinct stick values, then enable analog mode via the physical button.
        sio.set_sticks(0, (0x12, 0x34), (0x56, 0x78));
        sio.press_analog_button(0);
        // 9-byte analog poll: FF 73 5A BL BH RX RY LX LY.
        let resp = analog_cmd(&mut sio, &mut irq, &[0x01, 0x42, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            resp,
            vec![0xFF, 0x73, 0x5A, 0xFF, 0xFF, 0x12, 0x34, 0x56, 0x78]
        );
    }

    #[test]
    fn analog_pad_poll_analog_mode_with_buttons() {
        let (mut sio, mut irq) = analog_sio();
        sio.set_sticks(0, (0x10, 0x20), (0x30, 0x40));
        sio.press_analog_button(0);
        // L3 (bit1) + R2 (bit9) held — buttons only reachable with an analog pad.
        let mask: u16 = (1 << 1) | (1 << 9);
        sio.set_buttons(0, mask);
        let resp = analog_cmd(&mut sio, &mut irq, &[0x01, 0x42, 0, 0, 0, 0, 0, 0, 0]);
        let inv = !mask;
        assert_eq!(resp[3], (inv & 0xFF) as u8, "buttons low byte");
        assert_eq!(resp[4], (inv >> 8) as u8, "buttons high byte");
    }

    #[test]
    fn analog_enter_config_returns_f3() {
        let (mut sio, mut irq) = analog_sio();
        // 0x43 enter (arg 0x01). The header ID reflects the *old* mode (0x41).
        let enter = analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]);
        assert_eq!(
            enter,
            vec![0xFF, 0x41, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
        // A follow-up transaction now reports the config ID 0xF3.
        let follow = analog_cmd(&mut sio, &mut irq, &[0x01, 0x45, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(follow[1], 0xF3, "config-mode header ID");
    }

    #[test]
    fn analog_exit_config() {
        let (mut sio, mut irq) = analog_sio();
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]); // enter
        // 0x43 exit (arg 0x00). The header still shows 0xF3 (mode at start).
        let exit = analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x00, 0, 0, 0, 0, 0]);
        assert_eq!(exit[1], 0xF3, "exit header still config-mode ID");
        // Back to normal: a poll reports the digital ID 0x41 again.
        let poll = analog_cmd(&mut sio, &mut irq, &[0x01, 0x42, 0, 0, 0]);
        assert_eq!(poll, vec![0xFF, 0x41, 0x5A, 0xFF, 0xFF]);
    }

    #[test]
    fn config_0x44_set_analog() {
        let (mut sio, mut irq) = analog_sio();
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]); // enter config
        // 0x44 AA=01 (analog on), BB=02 (lock analog).
        let r = analog_cmd(
            &mut sio,
            &mut irq,
            &[0x01, 0x44, 0x00, 0x01, 0x02, 0, 0, 0, 0],
        );
        assert_eq!(
            r,
            vec![0xFF, 0xF3, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
        // Exit config: analog mode is now on (0x73 poll).
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x00, 0, 0, 0, 0, 0]);
        let poll = analog_cmd(&mut sio, &mut irq, &[0x01, 0x42, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(poll[1], 0x73, "analog on after 0x44 AA=01");
        // The analog toggle is locked: the physical button is a no-op.
        sio.press_analog_button(0);
        let poll2 = analog_cmd(&mut sio, &mut irq, &[0x01, 0x42, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(poll2[1], 0x73, "locked: analog button ignored");

        // 0x44 AA=00 turns analog back off (unlock first with BB=03).
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]);
        analog_cmd(
            &mut sio,
            &mut irq,
            &[0x01, 0x44, 0x00, 0x00, 0x03, 0, 0, 0, 0],
        );
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x00, 0, 0, 0, 0, 0]);
        let poll3 = analog_cmd(&mut sio, &mut irq, &[0x01, 0x42, 0, 0, 0]);
        assert_eq!(poll3[1], 0x41, "analog off after 0x44 AA=00");
    }

    #[test]
    fn config_0x45_led_status() {
        let (mut sio, mut irq) = analog_sio();
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]); // enter config
        // Analog off: LL == 0x00.
        let off = analog_cmd(&mut sio, &mut irq, &[0x01, 0x45, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            off,
            vec![0xFF, 0xF3, 0x5A, 0x01, 0x02, 0x00, 0x02, 0x01, 0x00]
        );
        // Turn analog on, then LL == 0x01.
        analog_cmd(
            &mut sio,
            &mut irq,
            &[0x01, 0x44, 0x00, 0x01, 0x00, 0, 0, 0, 0],
        );
        let on = analog_cmd(&mut sio, &mut irq, &[0x01, 0x45, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            on,
            vec![0xFF, 0xF3, 0x5A, 0x01, 0x02, 0x01, 0x02, 0x01, 0x00]
        );
    }

    #[test]
    fn config_0x46_constant() {
        let (mut sio, mut irq) = analog_sio();
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]); // enter config
        let a0 = analog_cmd(&mut sio, &mut irq, &[0x01, 0x46, 0x00, 0x00, 0, 0, 0, 0, 0]);
        assert_eq!(
            a0,
            vec![0xFF, 0xF3, 0x5A, 0x00, 0x00, 0x01, 0x02, 0x00, 0x0A]
        );
        let a1 = analog_cmd(&mut sio, &mut irq, &[0x01, 0x46, 0x00, 0x01, 0, 0, 0, 0, 0]);
        assert_eq!(
            a1,
            vec![0xFF, 0xF3, 0x5A, 0x00, 0x00, 0x01, 0x01, 0x01, 0x14]
        );
    }

    #[test]
    fn config_0x47_constant() {
        let (mut sio, mut irq) = analog_sio();
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]); // enter config
        let r = analog_cmd(&mut sio, &mut irq, &[0x01, 0x47, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            r,
            vec![0xFF, 0xF3, 0x5A, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00]
        );
    }

    #[test]
    fn config_0x4c_constant() {
        let (mut sio, mut irq) = analog_sio();
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]); // enter config
        let a0 = analog_cmd(&mut sio, &mut irq, &[0x01, 0x4C, 0x00, 0x00, 0, 0, 0, 0, 0]);
        assert_eq!(
            a0,
            vec![0xFF, 0xF3, 0x5A, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00]
        );
        let a1 = analog_cmd(&mut sio, &mut irq, &[0x01, 0x4C, 0x00, 0x01, 0, 0, 0, 0, 0]);
        assert_eq!(
            a1,
            vec![0xFF, 0xF3, 0x5A, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00]
        );
    }

    #[test]
    fn config_0x4d_vibration_map() {
        let (mut sio, mut irq) = analog_sio();
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]); // enter config
        // First 0x4D: response carries the DEFAULT map (all 0xFF); stores the
        // new map [00 01 FF FF FF FF]. Motor bytes are at tx phases 3..8.
        let first = analog_cmd(
            &mut sio,
            &mut irq,
            &[0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF],
        );
        assert_eq!(
            first,
            vec![0xFF, 0xF3, 0x5A, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
            "old map = default all-0xFF"
        );
        // Second 0x4D: response now carries the previously-set map; stores a new
        // one [02 03 04 05 06 07].
        let second = analog_cmd(
            &mut sio,
            &mut irq,
            &[0x01, 0x4D, 0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
        );
        assert_eq!(
            second,
            vec![0xFF, 0xF3, 0x5A, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF],
            "old map = previously-set [00 01 FF FF FF FF]"
        );
    }

    #[test]
    fn config_command_outside_config_mode_is_noop() {
        // A config command (0x45) issued while NOT in config mode responds like
        // an unknown command: current-mode header + all-zero data, no side
        // effect. (Digital ID here since analog/config are both off.)
        let (mut sio, mut irq) = analog_sio();
        let r = analog_cmd(&mut sio, &mut irq, &[0x01, 0x45, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            r,
            vec![0xFF, 0x41, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn analog_rumble_bytes_recorded() {
        let (mut sio, mut irq) = analog_sio();
        // Program a vib map routing poll byte 0 -> small motor, byte 1 -> large.
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]); // enter config
        analog_cmd(
            &mut sio,
            &mut irq,
            &[0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF],
        );
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x00, 0, 0, 0, 0, 0]); // exit config
        sio.press_analog_button(0); // analog mode on

        // No motor actuation recorded yet.
        assert_eq!(sio.motor_state(0), Some((0x00, 0x00)));

        // Poll: tx phase-3 byte (0xAA) -> small motor, phase-4 byte (0xBB) ->
        // large motor. (tx index == phase; phase 2 byte is ignored.)
        analog_cmd(
            &mut sio,
            &mut irq,
            &[0x01, 0x42, 0x00, 0xAA, 0xBB, 0x00, 0x00, 0x00, 0x00],
        );
        assert_eq!(
            sio.motor_state(0),
            Some((0xAA, 0xBB)),
            "motor bytes routed via vib_map"
        );
    }

    #[test]
    fn analog_pad_serde_round_trip() {
        // Analog pad state (incl. modes + vib_map + motors) survives a snapshot.
        let (mut sio, mut irq) = analog_sio();
        analog_cmd(&mut sio, &mut irq, &[0x01, 0x43, 0x00, 0x01, 0, 0, 0, 0, 0]);
        analog_cmd(
            &mut sio,
            &mut irq,
            &[0x01, 0x44, 0x00, 0x01, 0x00, 0, 0, 0, 0],
        );
        analog_cmd(
            &mut sio,
            &mut irq,
            &[0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF],
        );
        let json = serde_json::to_string(&sio).unwrap();
        let back: Sio0 = serde_json::from_str(&json).unwrap();
        assert_eq!(sio, back);
    }

    #[test]
    fn set_controller_type_preserves_buttons() {
        let mut sio = Sio0::new();
        sio.set_buttons(0, 0x00F0);
        sio.set_controller_type(0, ControllerKind::Analog);
        assert_eq!(sio.slot_buttons(0), 0x00F0, "buttons preserved on promote");
        sio.set_controller_type(0, ControllerKind::Disconnected);
        // A disconnected pad answers open-bus and never ACKs.
        let mut irq = Irq::new();
        sio.write16(0x1F80_104A, 0x1003);
        assert_eq!(pad_exchange(&mut sio, &mut irq, 0x01), 0xFF);
    }
}
