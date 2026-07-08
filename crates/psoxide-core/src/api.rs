//! The core emulator API surface.
//!
//! [`PsxCore`] is the I/O-free entry point host applications use to drive the
//! emulator. It owns all hardware state by value: the CPU and the system
//! memory (RAM, scratchpad, BIOS). Frontends issue [`Command`]s via
//! [`PsxCore::execute`] and inspect state via [`PsxCore::query`].

use core::fmt;

use serde::{Deserialize, Serialize};

use crate::bus::{
    self, BIOS_SIZE, BusRegion, MAIN_RAM_MASK, MAIN_RAM_SIZE, SCRATCHPAD_SIZE, map_region,
    mask_region,
};
use crate::cpu::execute::Bus;
use crate::cpu::{Cpu, CpuSnapshot, step};

/// Placeholder framebuffer width in pixels.
pub const FRAME_WIDTH: usize = 320;
/// Placeholder framebuffer height in pixels.
pub const FRAME_HEIGHT: usize = 240;
/// Placeholder framebuffer size in bytes (RGBA).
pub const FRAME_RGBA_BYTES: usize = FRAME_WIDTH * FRAME_HEIGHT * 4;

/// R3000A master clock in Hz (~33.8688 MHz).
pub const CPU_CLOCK_HZ: u64 = 33_868_800;
/// Approximate NTSC frame rate.
pub const FRAMES_PER_SECOND: u64 = 60;
/// Instructions stepped per [`Command::StepFrame`] (rough placeholder pacing).
pub const STEPS_PER_FRAME: u64 = CPU_CLOCK_HZ / FRAMES_PER_SECOND;

/// A standard digital controller button (SCPH-1080 layout).
#[derive(Debug, Clone, Copy, PartialEq, Eq, core::hash::Hash)]
pub enum Button {
    /// D-pad up.
    Up,
    /// D-pad down.
    Down,
    /// D-pad left.
    Left,
    /// D-pad right.
    Right,
    /// Cross button.
    Cross,
    /// Circle button.
    Circle,
    /// Square button.
    Square,
    /// Triangle button.
    Triangle,
    /// L1 shoulder.
    L1,
    /// R1 shoulder.
    R1,
    /// Start button.
    Start,
    /// Select button.
    Select,
}

impl Button {
    /// Returns this button's bit within a controller bitfield.
    #[must_use]
    pub fn bit_mask(self) -> u16 {
        match self {
            Self::Select => 1 << 0,
            Self::Start => 1 << 3,
            Self::Up => 1 << 4,
            Self::Right => 1 << 5,
            Self::Down => 1 << 6,
            Self::Left => 1 << 7,
            Self::L1 => 1 << 10,
            Self::R1 => 1 << 11,
            Self::Triangle => 1 << 12,
            Self::Circle => 1 << 13,
            Self::Cross => 1 << 14,
            Self::Square => 1 << 15,
        }
    }
}

/// Commands that drive [`PsxCore`] state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    /// Load a 512KB BIOS image.
    LoadBios(Vec<u8>),
    /// Side-load a PSX-EXE image (currently a stub — accepted but ignored).
    LoadExe(Vec<u8>),
    /// Reset the CPU to the BIOS entry vector.
    Reset,
    /// Execute one CPU instruction.
    StepCpu,
    /// Execute a frame's worth of instructions ([`STEPS_PER_FRAME`]).
    StepFrame,
    /// Replace a controller port's button bitfield.
    SetControllerState {
        /// Controller port index (0 or 1).
        port: u8,
        /// Pressed-button bitfield.
        buttons: u16,
    },
    /// Pause emulation stepping.
    Pause,
    /// Resume emulation stepping.
    Resume,
}

/// Read-only queries against [`PsxCore`] state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CoreQuery {
    /// Return the CPU register snapshot.
    Registers,
    /// Return `len` bytes of CPU-visible memory starting at `addr`.
    Memory {
        /// Virtual start address.
        addr: u32,
        /// Number of bytes to read.
        len: u32,
    },
    /// Return the current program counter.
    Pc,
    /// Return lightweight machine status.
    EmulatorState,
}

/// Lightweight machine status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EmulatorState {
    /// Whether stepping is paused.
    pub paused: bool,
    /// Whether a BIOS image is loaded.
    pub bios_loaded: bool,
    /// Controller-port button bitfields.
    pub controllers: [u16; 2],
    /// Retired instruction count.
    pub cycles: u64,
}

/// Result of a [`CoreQuery`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QueryResult {
    /// [`CoreQuery::Registers`] response.
    Registers(Box<CpuSnapshot>),
    /// [`CoreQuery::Memory`] response.
    Memory(Vec<u8>),
    /// [`CoreQuery::Pc`] response.
    Pc(u32),
    /// [`CoreQuery::EmulatorState`] response.
    EmulatorState(EmulatorState),
}

/// Errors returned by [`PsxCore::execute`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CoreError {
    /// The provided BIOS image was not exactly [`BIOS_SIZE`] bytes.
    BiosWrongSize {
        /// Expected size (512KB).
        expected: usize,
        /// Size actually provided.
        found: usize,
    },
}

impl fmt::Display for CoreError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BiosWrongSize { expected, found } => {
                write!(f, "bios image is {found} bytes, expected {expected}")
            }
        }
    }
}

impl std::error::Error for CoreError {}

/// PlayStation system memory: main RAM, scratchpad, and BIOS ROM.
#[derive(Clone)]
pub struct Memory {
    /// 2MB main RAM.
    pub ram: Box<[u8; MAIN_RAM_SIZE]>,
    /// 1KB scratchpad (D-cache used as fast RAM).
    pub scratchpad: Box<[u8; SCRATCHPAD_SIZE]>,
    /// BIOS ROM image (512KB when loaded; empty otherwise).
    pub bios: Vec<u8>,
}

impl fmt::Debug for Memory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Memory")
            .field("ram", &format_args!("[{} bytes]", self.ram.len()))
            .field(
                "scratchpad",
                &format_args!("[{} bytes]", self.scratchpad.len()),
            )
            .field("bios", &format_args!("[{} bytes]", self.bios.len()))
            .finish()
    }
}

impl Default for Memory {
    fn default() -> Self {
        Self::new()
    }
}

impl Memory {
    /// Creates zeroed memory with no BIOS loaded.
    #[must_use]
    pub fn new() -> Self {
        Self {
            ram: Box::new([0; MAIN_RAM_SIZE]),
            scratchpad: Box::new([0; SCRATCHPAD_SIZE]),
            bios: Vec::new(),
        }
    }

    /// Reads a byte from a physical address, logging (returning 0 for) I/O and
    /// unmapped regions. This is the primitive all sized reads decompose into.
    #[must_use]
    pub fn read8(&self, virt: u32) -> u8 {
        let phys = mask_region(virt);
        match map_region(phys) {
            BusRegion::MainRam => self.ram[(phys & MAIN_RAM_MASK) as usize],
            BusRegion::Scratchpad => self.scratchpad[(phys & 0x3FF) as usize],
            BusRegion::Bios => {
                let offset = (phys - 0x1FC0_0000) as usize;
                self.bios.get(offset).copied().unwrap_or(0)
            }
            // I/O, expansion, cache-control, and unmapped reads are stubbed.
            _ => 0,
        }
    }

    /// Writes a byte to a physical address; I/O and unmapped regions are
    /// stubbed (ignored), and the BIOS region is read-only.
    pub fn write8(&mut self, virt: u32, value: u8) {
        let phys = mask_region(virt);
        match map_region(phys) {
            BusRegion::MainRam => self.ram[(phys & MAIN_RAM_MASK) as usize] = value,
            BusRegion::Scratchpad => self.scratchpad[(phys & 0x3FF) as usize] = value,
            // BIOS is read-only; I/O and unmapped writes are ignored (stub).
            _ => {}
        }
    }
}

/// Adapter that lets the CPU drive [`Memory`] through the [`Bus`] trait.
/// Loads/stores are little-endian and decompose into byte accesses so that
/// region routing lives in one place.
struct CoreBus<'a> {
    mem: &'a mut Memory,
}

impl Bus for CoreBus<'_> {
    fn load8(&mut self, addr: u32) -> u8 {
        self.mem.read8(addr)
    }
    fn load16(&mut self, addr: u32) -> u16 {
        u16::from_le_bytes([self.mem.read8(addr), self.mem.read8(addr.wrapping_add(1))])
    }
    fn load32(&mut self, addr: u32) -> u32 {
        u32::from_le_bytes([
            self.mem.read8(addr),
            self.mem.read8(addr.wrapping_add(1)),
            self.mem.read8(addr.wrapping_add(2)),
            self.mem.read8(addr.wrapping_add(3)),
        ])
    }
    fn store8(&mut self, addr: u32, value: u8) {
        self.mem.write8(addr, value);
    }
    fn store16(&mut self, addr: u32, value: u16) {
        let b = value.to_le_bytes();
        self.mem.write8(addr, b[0]);
        self.mem.write8(addr.wrapping_add(1), b[1]);
    }
    fn store32(&mut self, addr: u32, value: u32) {
        let b = value.to_le_bytes();
        self.mem.write8(addr, b[0]);
        self.mem.write8(addr.wrapping_add(1), b[1]);
        self.mem.write8(addr.wrapping_add(2), b[2]);
        self.mem.write8(addr.wrapping_add(3), b[3]);
    }
}

/// A complete serializable machine snapshot for save states.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoreSnapshot {
    /// Pause state.
    pub paused: bool,
    /// Controller-port button bitfields.
    pub controllers: [u16; 2],
    /// CPU state.
    pub cpu: CpuSnapshot,
    /// Main RAM (2MB).
    #[serde(deserialize_with = "deserialize_ram")]
    pub ram: Vec<u8>,
    /// Scratchpad (1KB).
    pub scratchpad: Vec<u8>,
    /// BIOS image.
    pub bios: Vec<u8>,
}

/// Deserializes the RAM buffer, rejecting snapshots whose length is not the
/// expected 2MB. Serialization uses serde's default `Vec<u8>` encoding.
fn deserialize_ram<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Vec<u8>, D::Error> {
    use serde::de::Error;
    let v: Vec<u8> = Vec::deserialize(d)?;
    if v.len() != MAIN_RAM_SIZE {
        return Err(D::Error::custom("ram snapshot has wrong length"));
    }
    Ok(v)
}

/// The central PlayStation machine.
#[derive(Debug, Clone)]
pub struct PsxCore {
    cpu: Cpu,
    mem: Memory,
    paused: bool,
    controllers: [u16; 2],
}

impl Default for PsxCore {
    fn default() -> Self {
        Self::new()
    }
}

impl PsxCore {
    /// Creates a core with power-on defaults and no BIOS loaded.
    #[must_use]
    pub fn new() -> Self {
        Self {
            cpu: Cpu::new(),
            mem: Memory::new(),
            paused: false,
            controllers: [0; 2],
        }
    }

    /// Returns whether stepping is paused.
    #[must_use]
    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// Returns the current program counter.
    #[must_use]
    pub fn pc(&self) -> u32 {
        self.cpu.pc
    }

    /// Returns a CPU register snapshot.
    #[must_use]
    pub fn cpu_snapshot(&self) -> CpuSnapshot {
        self.cpu.snapshot()
    }

    /// Returns a shared reference to system memory.
    #[must_use]
    pub fn memory(&self) -> &Memory {
        &self.mem
    }

    /// Returns a mutable reference to system memory (for test harnesses that
    /// hand-load programs into RAM).
    pub fn memory_mut(&mut self) -> &mut Memory {
        &mut self.mem
    }

    /// Sets the program counter (and the delayed `next_pc`). Useful for test
    /// harnesses that stage a program at a known address.
    pub fn set_pc(&mut self, pc: u32) {
        self.cpu.pc = pc;
        self.cpu.next_pc = pc.wrapping_add(4);
        self.cpu.current_pc = pc;
    }

    /// Executes a single command.
    ///
    /// # Errors
    ///
    /// Returns [`CoreError::BiosWrongSize`] if a [`Command::LoadBios`] payload
    /// is not exactly [`BIOS_SIZE`] bytes.
    pub fn execute(&mut self, command: Command) -> Result<(), CoreError> {
        match command {
            Command::LoadBios(image) => {
                if image.len() != BIOS_SIZE {
                    return Err(CoreError::BiosWrongSize {
                        expected: BIOS_SIZE,
                        found: image.len(),
                    });
                }
                self.mem.bios = image;
                self.cpu.reset();
            }
            Command::LoadExe(_image) => {
                // PSX-EXE side-loading is not yet implemented; accepted as a no-op.
            }
            Command::Reset => self.cpu.reset(),
            Command::StepCpu => self.step_cpu(),
            Command::StepFrame => {
                for _ in 0..STEPS_PER_FRAME {
                    self.step_cpu();
                }
            }
            Command::SetControllerState { port, buttons } => {
                if let Some(slot) = self.controllers.get_mut(port as usize) {
                    *slot = buttons;
                }
            }
            Command::Pause => self.paused = true,
            Command::Resume => self.paused = false,
        }
        Ok(())
    }

    /// Executes a read-only query.
    #[must_use]
    pub fn query(&self, query: CoreQuery) -> QueryResult {
        match query {
            CoreQuery::Registers => QueryResult::Registers(Box::new(self.cpu.snapshot())),
            CoreQuery::Pc => QueryResult::Pc(self.cpu.pc),
            CoreQuery::Memory { addr, len } => {
                let mut out = Vec::with_capacity(len as usize);
                for i in 0..len {
                    out.push(self.mem.read8(addr.wrapping_add(i)));
                }
                QueryResult::Memory(out)
            }
            CoreQuery::EmulatorState => QueryResult::EmulatorState(EmulatorState {
                paused: self.paused,
                bios_loaded: self.mem.bios.len() == BIOS_SIZE,
                controllers: self.controllers,
                cycles: self.cpu.cycles,
            }),
        }
    }

    fn step_cpu(&mut self) {
        let mut bus = CoreBus { mem: &mut self.mem };
        step(&mut self.cpu, &mut bus);
    }

    /// Returns a freshly allocated placeholder RGBA framebuffer.
    ///
    /// GPU emulation is not yet implemented; this renders a deterministic
    /// gradient so the desktop frontend has something to display.
    #[must_use]
    pub fn framebuffer_rgba(&self) -> Vec<u8> {
        let mut frame = vec![0u8; FRAME_RGBA_BYTES];
        for y in 0..FRAME_HEIGHT {
            for x in 0..FRAME_WIDTH {
                let i = (y * FRAME_WIDTH + x) * 4;
                frame[i] = (x * 255 / FRAME_WIDTH) as u8;
                frame[i + 1] = (y * 255 / FRAME_HEIGHT) as u8;
                frame[i + 2] = 0x40;
                frame[i + 3] = 0xFF;
            }
        }
        frame
    }

    /// Captures a full save-state snapshot.
    #[must_use]
    pub fn save_state(&self) -> CoreSnapshot {
        CoreSnapshot {
            paused: self.paused,
            controllers: self.controllers,
            cpu: self.cpu.snapshot(),
            ram: self.mem.ram.to_vec(),
            scratchpad: self.mem.scratchpad.to_vec(),
            bios: self.mem.bios.clone(),
        }
    }

    /// Restores a previously captured save-state snapshot.
    pub fn load_state(&mut self, snap: &CoreSnapshot) {
        self.paused = snap.paused;
        self.controllers = snap.controllers;
        self.cpu.restore(&snap.cpu);
        if snap.ram.len() == MAIN_RAM_SIZE {
            self.mem.ram.copy_from_slice(&snap.ram);
        }
        if snap.scratchpad.len() == SCRATCHPAD_SIZE {
            self.mem.scratchpad.copy_from_slice(&snap.scratchpad);
        }
        self.mem.bios = snap.bios.clone();
    }
}

// Re-export a couple of bus items commonly needed alongside the API.
pub use bus::BIOS_SIZE as BIOS_IMAGE_SIZE;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_core_defaults() {
        let core = PsxCore::new();
        assert!(!core.is_paused());
        assert_eq!(core.pc(), crate::cpu::RESET_PC);
    }

    #[test]
    fn load_bios_wrong_size_errors() {
        let mut core = PsxCore::new();
        let err = core.execute(Command::LoadBios(vec![0; 100])).unwrap_err();
        assert_eq!(
            err,
            CoreError::BiosWrongSize {
                expected: BIOS_SIZE,
                found: 100
            }
        );
    }

    #[test]
    fn load_bios_and_step_reads_from_bios() {
        let mut core = PsxCore::new();
        let mut bios = vec![0u8; BIOS_SIZE];
        // Place `addiu $t0,$zero,0x1234` at the BIOS reset vector.
        let insn: u32 = (0x09 << 26) | (8 << 16) | 0x1234;
        bios[0..4].copy_from_slice(&insn.to_le_bytes());
        core.execute(Command::LoadBios(bios)).unwrap();
        core.execute(Command::StepCpu).unwrap();
        let snap = core.cpu_snapshot();
        assert_eq!(snap.regs[8], 0x1234);
    }

    #[test]
    fn pause_resume_toggles_state() {
        let mut core = PsxCore::new();
        core.execute(Command::Pause).unwrap();
        assert!(core.is_paused());
        core.execute(Command::Resume).unwrap();
        assert!(!core.is_paused());
    }

    #[test]
    fn controller_state_query() {
        let mut core = PsxCore::new();
        core.execute(Command::SetControllerState {
            port: 0,
            buttons: 0xF0,
        })
        .unwrap();
        match core.query(CoreQuery::EmulatorState) {
            QueryResult::EmulatorState(s) => assert_eq!(s.controllers[0], 0xF0),
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn memory_query_reads_ram() {
        let mut core = PsxCore::new();
        core.memory_mut().write8(0x1000, 0xAB);
        core.memory_mut().write8(0x1001, 0xCD);
        match core.query(CoreQuery::Memory {
            addr: 0x1000,
            len: 2,
        }) {
            QueryResult::Memory(bytes) => assert_eq!(bytes, vec![0xAB, 0xCD]),
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn ram_mirroring_folds_into_2mb() {
        let mut core = PsxCore::new();
        core.memory_mut().write8(0x0000_0000, 0x42);
        // Mirror at +2MB reads the same cell.
        assert_eq!(core.memory().read8(0x0020_0000), 0x42);
    }

    #[test]
    fn kseg1_bios_alias_reads_bios() {
        let mut core = PsxCore::new();
        let mut bios = vec![0u8; BIOS_SIZE];
        bios[0] = 0x99;
        core.execute(Command::LoadBios(bios)).unwrap();
        // 0xBFC0_0000 (KSEG1) and 0x1FC0_0000 (physical) alias the same byte.
        assert_eq!(core.memory().read8(0xBFC0_0000), 0x99);
        assert_eq!(core.memory().read8(0x1FC0_0000), 0x99);
    }

    #[test]
    fn framebuffer_has_expected_size() {
        let core = PsxCore::new();
        assert_eq!(core.framebuffer_rgba().len(), FRAME_RGBA_BYTES);
    }

    #[test]
    fn save_and_load_state_round_trip() {
        let mut core = PsxCore::new();
        core.memory_mut().write8(0x2000, 0x7E);
        core.execute(Command::SetControllerState {
            port: 1,
            buttons: 0x0A,
        })
        .unwrap();
        let snap = core.save_state();

        let mut other = PsxCore::new();
        other.load_state(&snap);
        assert_eq!(other.memory().read8(0x2000), 0x7E);
        assert_eq!(other.controllers[1], 0x0A);
    }

    #[test]
    fn snapshot_serde_round_trip() {
        let mut core = PsxCore::new();
        core.memory_mut().write8(0x10, 0x55);
        let snap = core.save_state();
        let json = serde_json::to_string(&snap).unwrap();
        let back: CoreSnapshot = serde_json::from_str(&json).unwrap();
        assert_eq!(snap, back);
    }
}
