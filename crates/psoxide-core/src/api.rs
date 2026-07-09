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
use crate::cdrom::{Cdrom, Disc};
use crate::cpu::execute::Bus;
use crate::cpu::{Cpu, CpuSnapshot, poll_interrupt, step};
use crate::dma::Dma;
use crate::gpu::Gpu;
use crate::iostubs::{CACHE_CTRL_REG, CacheCtrl, MemCtrl, Sio0, Spu};
use crate::irq::{Irq, IrqLine};
use crate::timers::{TIMERS_BASE, TIMERS_END, Timers};

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
    /// Insert a disc into the CD-ROM drive.
    LoadDisc(Disc),
    /// Eject the currently inserted disc, if any.
    EjectDisc,
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

/// Adapter that lets the CPU drive [`Memory`] and the memory-mapped peripherals
/// (GPU, DMA, interrupt controller) through the [`Bus`] trait.
///
/// RAM / BIOS / scratchpad accesses decompose into byte accesses so region
/// routing lives in [`Memory`]. Accesses that land in [`BusRegion::IoPorts`]
/// are intercepted here and routed to the peripheral registers instead.
struct CoreBus<'a> {
    mem: &'a mut Memory,
    gpu: &'a mut Gpu,
    dma: &'a mut Dma,
    irq: &'a mut Irq,
    timers: &'a mut Timers,
    memctrl: &'a mut MemCtrl,
    cache_ctrl: &'a mut CacheCtrl,
    sio0: &'a mut Sio0,
    cdrom: &'a mut Cdrom,
    spu: &'a mut Spu,
}

impl CoreBus<'_> {
    /// Returns `true` if the physical address falls in the I/O register window.
    #[inline]
    fn is_io(phys: u32) -> bool {
        matches!(map_region(phys), BusRegion::IoPorts)
    }

    fn io_read32(&mut self, phys: u32) -> u32 {
        match phys {
            0x1F80_1810 => self.gpu.gpuread(),
            0x1F80_1814 => self.gpu.gpustat(),
            0x1F80_1070 => self.irq.read_stat(),
            0x1F80_1074 => self.irq.read_mask(),
            0x1F80_1080..=0x1F80_10FF => self.dma.read32(phys),
            TIMERS_BASE..=TIMERS_END => self.timers.read32(phys),
            _ if MemCtrl::contains(phys) => self.memctrl.read32(phys),
            _ if Sio0::contains(phys) => self.sio0.read32(phys),
            _ if Cdrom::contains(phys) => self.cdrom.read32(phys),
            _ if Spu::contains(phys) => self.spu.read32(phys),
            _ => 0,
        }
    }

    fn io_write32(&mut self, phys: u32, val: u32) {
        match phys {
            0x1F80_1810 => self.gpu.gp0(val),
            0x1F80_1814 => self.gpu.gp1(val),
            0x1F80_1070 => self.irq.write_stat(val),
            0x1F80_1074 => self.irq.write_mask(val),
            0x1F80_1080..=0x1F80_10FF => {
                self.dma
                    .write32(phys, val, self.mem, self.gpu, self.cdrom, self.irq);
            }
            TIMERS_BASE..=TIMERS_END => self.timers.write32(phys, val),
            _ if MemCtrl::contains(phys) => self.memctrl.write32(phys, val),
            _ if Sio0::contains(phys) => self.sio0.write32(phys, val),
            _ if Cdrom::contains(phys) => self.cdrom.write32(phys, val),
            _ if Spu::contains(phys) => self.spu.write32(phys, val),
            // Other I/O ports are stubbed (ignored).
            _ => {}
        }
    }

    fn io_read16(&mut self, phys: u32) -> u16 {
        match phys {
            0x1F80_1070 => self.irq.read_stat() as u16,
            0x1F80_1074 => self.irq.read_mask() as u16,
            TIMERS_BASE..=TIMERS_END => self.timers.read16(phys),
            _ if MemCtrl::contains(phys) => self.memctrl.read32(phys & !0x3) as u16,
            _ if Sio0::contains(phys) => self.sio0.read16(phys),
            _ if Cdrom::contains(phys) => self.cdrom.read16(phys),
            _ if Spu::contains(phys) => self.spu.read16(phys),
            _ => 0,
        }
    }

    fn io_write16(&mut self, phys: u32, val: u16) {
        match phys {
            // Acknowledge only the low half; preserve the (unused) high bits.
            0x1F80_1070 => self.irq.write_stat(u32::from(val) | 0xFFFF_0000),
            0x1F80_1074 => {
                let hi = self.irq.read_mask() & 0xFFFF_0000;
                self.irq.write_mask(hi | u32::from(val));
            }
            TIMERS_BASE..=TIMERS_END => self.timers.write16(phys, val),
            _ if Sio0::contains(phys) => self.sio0.write16(phys, val),
            _ if Cdrom::contains(phys) => self.cdrom.write16(phys, val),
            _ if Spu::contains(phys) => self.spu.write16(phys, val),
            _ => {}
        }
    }

    fn io_read8(&mut self, phys: u32) -> u8 {
        match phys {
            _ if Sio0::contains(phys) => self.sio0.read8(phys),
            _ if Cdrom::contains(phys) => self.cdrom.read8(phys),
            _ if Spu::contains(phys) => self.spu.read8(phys),
            _ => 0,
        }
    }

    fn io_write8(&mut self, phys: u32, val: u8) {
        match phys {
            _ if Sio0::contains(phys) => self.sio0.write8(phys, val),
            _ if Cdrom::contains(phys) => self.cdrom.write8(phys, val),
            _ if Spu::contains(phys) => self.spu.write8(phys, val),
            _ => {}
        }
    }
}

impl Bus for CoreBus<'_> {
    fn load8(&mut self, addr: u32) -> u8 {
        let phys = mask_region(addr);
        if Self::is_io(phys) {
            return self.io_read8(phys);
        }
        self.mem.read8(addr)
    }
    fn load16(&mut self, addr: u32) -> u16 {
        let phys = mask_region(addr);
        if Self::is_io(phys) {
            return self.io_read16(phys);
        }
        u16::from_le_bytes([self.mem.read8(addr), self.mem.read8(addr.wrapping_add(1))])
    }
    fn load32(&mut self, addr: u32) -> u32 {
        let phys = mask_region(addr);
        if Self::is_io(phys) {
            return self.io_read32(phys);
        }
        if phys == CACHE_CTRL_REG {
            return self.cache_ctrl.read32();
        }
        u32::from_le_bytes([
            self.mem.read8(addr),
            self.mem.read8(addr.wrapping_add(1)),
            self.mem.read8(addr.wrapping_add(2)),
            self.mem.read8(addr.wrapping_add(3)),
        ])
    }
    fn store8(&mut self, addr: u32, value: u8) {
        let phys = mask_region(addr);
        if Self::is_io(phys) {
            self.io_write8(phys, value);
            return;
        }
        self.mem.write8(addr, value);
    }
    fn store16(&mut self, addr: u32, value: u16) {
        let phys = mask_region(addr);
        if Self::is_io(phys) {
            self.io_write16(phys, value);
            return;
        }
        let b = value.to_le_bytes();
        self.mem.write8(addr, b[0]);
        self.mem.write8(addr.wrapping_add(1), b[1]);
    }
    fn store32(&mut self, addr: u32, value: u32) {
        let phys = mask_region(addr);
        if Self::is_io(phys) {
            self.io_write32(phys, value);
            return;
        }
        if phys == CACHE_CTRL_REG {
            self.cache_ctrl.write32(value);
            return;
        }
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
    /// GPU state (VRAM + registers).
    pub gpu: Gpu,
    /// DMA controller state.
    pub dma: Dma,
    /// Interrupt controller state.
    pub irq: Irq,
    /// Hardware timer / root-counter state.
    #[serde(default)]
    pub timers: Timers,
    /// Memory-control register stub.
    #[serde(default)]
    pub memctrl: MemCtrl,
    /// Cache-control register stub.
    #[serde(default)]
    pub cache_ctrl: CacheCtrl,
    /// SIO0 / joypad register stub.
    #[serde(default)]
    pub sio0: Sio0,
    /// CD-ROM register stub.
    #[serde(default)]
    pub cdrom: Cdrom,
    /// SPU register file stub.
    #[serde(default)]
    pub spu: Spu,
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
    gpu: Gpu,
    dma: Dma,
    irq: Irq,
    timers: Timers,
    memctrl: MemCtrl,
    cache_ctrl: CacheCtrl,
    sio0: Sio0,
    cdrom: Cdrom,
    spu: Spu,
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
            gpu: Gpu::new(),
            dma: Dma::new(),
            irq: Irq::new(),
            timers: Timers::new(),
            memctrl: MemCtrl::new(),
            cache_ctrl: CacheCtrl::new(),
            sio0: Sio0::new(),
            cdrom: Cdrom::new(),
            spu: Spu::new(),
            paused: false,
            controllers: [0; 2],
        }
    }

    /// Returns a shared reference to the GPU.
    #[must_use]
    pub fn gpu(&self) -> &Gpu {
        &self.gpu
    }

    /// Returns a mutable reference to the GPU (for test harnesses that drive
    /// GP0/GP1 directly).
    pub fn gpu_mut(&mut self) -> &mut Gpu {
        &mut self.gpu
    }

    /// Returns a shared reference to the interrupt controller.
    #[must_use]
    pub fn irq(&self) -> &Irq {
        &self.irq
    }

    /// Returns a shared reference to the DMA controller.
    #[must_use]
    pub fn dma(&self) -> &Dma {
        &self.dma
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

    /// Reads general-purpose register `index` (architectural value). Useful for
    /// test harnesses that high-level-emulate BIOS calls.
    #[must_use]
    pub fn reg(&self, index: usize) -> u32 {
        self.cpu.regs[index]
    }

    /// Writes general-purpose register `index` directly into both register
    /// banks so the value is immediately visible (no load-delay). For test
    /// harnesses that stage BIOS-call results.
    pub fn set_reg(&mut self, index: usize, value: u32) {
        if index != 0 {
            self.cpu.regs[index] = value;
            self.cpu.out_regs[index] = value;
        }
    }

    /// Reads coprocessor-0 register `index`.
    #[must_use]
    pub fn cop0(&self, index: usize) -> u32 {
        self.cpu.cop0[index]
    }

    /// Writes coprocessor-0 register `index`. For test harnesses that
    /// high-level-emulate the BIOS exception handler.
    pub fn set_cop0(&mut self, index: usize, value: u32) {
        self.cpu.cop0[index] = value;
    }

    /// Raises a VBlank interrupt and advances the interlace field, exactly as
    /// [`Command::StepFrame`] does once per frame. Exposed for test harnesses
    /// that drive the CPU via [`Command::StepCpu`] but still need `VSync`-based
    /// programs to make progress.
    pub fn raise_vblank(&mut self) {
        self.gpu.field = !self.gpu.field;
        self.irq.set(IrqLine::VBlank);
    }

    /// Builds a transient [`CoreBus`] borrowing every peripheral, for one-off
    /// bus accesses that are not part of a CPU instruction step.
    fn core_bus(&mut self) -> CoreBus<'_> {
        CoreBus {
            mem: &mut self.mem,
            gpu: &mut self.gpu,
            dma: &mut self.dma,
            irq: &mut self.irq,
            timers: &mut self.timers,
            memctrl: &mut self.memctrl,
            cache_ctrl: &mut self.cache_ctrl,
            sio0: &mut self.sio0,
            cdrom: &mut self.cdrom,
            spu: &mut self.spu,
        }
    }

    /// Performs an 8-bit store through the full system bus, routing to device
    /// registers exactly as a CPU `sb` would.
    ///
    /// This is the frontend/integration seam for driving memory-mapped
    /// peripherals (e.g. the CD-ROM controller at `0x1F80_1800..=0x1F80_1803`)
    /// without hand-assembling guest code.
    pub fn store8(&mut self, addr: u32, value: u8) {
        self.core_bus().store8(addr, value);
    }

    /// Performs a 32-bit store through the full system bus (device routing
    /// included), exactly as a CPU `sw` would. Used to program the DMA and
    /// interrupt-controller register files from a frontend/test.
    pub fn store32(&mut self, addr: u32, value: u32) {
        self.core_bus().store32(addr, value);
    }

    /// Performs an 8-bit load through the full system bus, popping device FIFOs
    /// exactly as a CPU `lb` would.
    pub fn load8(&mut self, addr: u32) -> u8 {
        self.core_bus().load8(addr)
    }

    /// Performs a 32-bit load through the full system bus (device routing
    /// included), exactly as a CPU `lw` would.
    pub fn load32(&mut self, addr: u32) -> u32 {
        self.core_bus().load32(addr)
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
            Command::LoadDisc(disc) => self.cdrom.insert_disc(disc),
            Command::EjectDisc => self.cdrom.eject(),
            Command::Reset => self.cpu.reset(),
            Command::StepCpu => self.step_cpu(),
            Command::StepFrame => {
                for _ in 0..STEPS_PER_FRAME {
                    self.step_cpu();
                }
                // Once per frame: advance the interlace field and raise VBlank.
                self.gpu.field = !self.gpu.field;
                self.irq.set(IrqLine::VBlank);
            }
            Command::SetControllerState { port, buttons } => {
                if let Some(slot) = self.controllers.get_mut(port as usize) {
                    *slot = buttons;
                }
                // Feed the held buttons into the SIO0 pad so a transfer reads
                // them out. `set_buttons` ignores out-of-range ports.
                self.sio0.set_buttons(port as usize, buttons);
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
        // Advance the hardware timers by one CPU cycle first, so a timer that
        // reaches its target/overflow this cycle can deliver its interrupt at
        // this same instruction boundary.
        self.timers.tick(1, &mut self.irq);
        // Advance the CD-ROM controller so a queued command response can latch
        // and raise its interrupt this cycle.
        self.cdrom.tick(1, &mut self.irq);

        // Advance the SIO0 controller ACK timer by the same cycle, so a
        // scheduled controller /ACK can raise IRQ7 at this instruction boundary.
        self.sio0.tick(1, &mut self.irq);

        // Deliver a pending, unmasked hardware interrupt at the instruction
        // boundary before fetching the next instruction. With reset state
        // (interrupts disabled) this is a no-op.
        if poll_interrupt(&mut self.cpu, self.irq.pending()) {
            self.cpu.cycles = self.cpu.cycles.wrapping_add(1);
            return;
        }
        let mut bus = CoreBus {
            mem: &mut self.mem,
            gpu: &mut self.gpu,
            dma: &mut self.dma,
            irq: &mut self.irq,
            timers: &mut self.timers,
            memctrl: &mut self.memctrl,
            cache_ctrl: &mut self.cache_ctrl,
            sio0: &mut self.sio0,
            cdrom: &mut self.cdrom,
            spu: &mut self.spu,
        };
        step(&mut self.cpu, &mut bus);
    }

    /// Renders the current display area from VRAM to a 320×240 RGBA buffer.
    ///
    /// The display start position is read from the GPU (GP1 0x05). When the
    /// display is disabled the frame is all black. 15bpp (BGR555) mode is fully
    /// supported; 24bpp mode (used by the Sony boot logo) is best-effort,
    /// unpacking the packed 24-bit byte stream from the VRAM row.
    #[must_use]
    pub fn framebuffer_rgba(&self) -> Vec<u8> {
        let mut frame = vec![0u8; FRAME_RGBA_BYTES];
        if !self.gpu.display_enabled {
            // All black, opaque.
            for px in frame.chunks_exact_mut(4) {
                px[3] = 0xFF;
            }
            return frame;
        }

        let vx = self.gpu.display_vram_x;
        let vy = self.gpu.display_vram_y;

        if self.gpu.color_depth_24 {
            // 24bpp: each output row is 320 pixels = 960 bytes = 480 u16 words.
            for oy in 0..FRAME_HEIGHT {
                let row_y = vy.wrapping_add(oy as u16);
                for ox in 0..FRAME_WIDTH {
                    // Byte offset of this pixel within the VRAM row.
                    let byte = ox * 3;
                    let word0 = self.gpu.vram_at(vx.wrapping_add((byte / 2) as u16), row_y);
                    let word1 = self
                        .gpu
                        .vram_at(vx.wrapping_add((byte / 2 + 1) as u16), row_y);
                    let bytes = [
                        word0 as u8,
                        (word0 >> 8) as u8,
                        word1 as u8,
                        (word1 >> 8) as u8,
                    ];
                    let sub = byte % 2;
                    let r = bytes[sub];
                    let g = bytes[sub + 1];
                    let b = bytes[sub + 2];
                    let i = (oy * FRAME_WIDTH + ox) * 4;
                    frame[i] = r;
                    frame[i + 1] = g;
                    frame[i + 2] = b;
                    frame[i + 3] = 0xFF;
                }
            }
        } else {
            // 15bpp BGR555.
            for oy in 0..FRAME_HEIGHT {
                let row_y = vy.wrapping_add(oy as u16);
                for ox in 0..FRAME_WIDTH {
                    let p = self.gpu.vram_at(vx.wrapping_add(ox as u16), row_y);
                    let i = (oy * FRAME_WIDTH + ox) * 4;
                    frame[i] = ((p & 0x1F) << 3) as u8;
                    frame[i + 1] = (((p >> 5) & 0x1F) << 3) as u8;
                    frame[i + 2] = (((p >> 10) & 0x1F) << 3) as u8;
                    frame[i + 3] = 0xFF;
                }
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
            gpu: self.gpu.clone(),
            dma: self.dma.clone(),
            irq: self.irq.clone(),
            timers: self.timers.clone(),
            memctrl: self.memctrl.clone(),
            cache_ctrl: self.cache_ctrl.clone(),
            sio0: self.sio0.clone(),
            cdrom: self.cdrom.clone(),
            spu: self.spu.clone(),
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
        self.gpu = snap.gpu.clone();
        self.dma = snap.dma.clone();
        self.irq = snap.irq.clone();
        self.timers = snap.timers.clone();
        self.memctrl = snap.memctrl.clone();
        self.cache_ctrl = snap.cache_ctrl.clone();
        self.sio0 = snap.sio0.clone();
        self.cdrom = snap.cdrom.clone();
        self.spu = snap.spu.clone();
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

    #[test]
    fn gp0_write_via_bus_reaches_gpu() {
        let mut core = PsxCore::new();
        {
            let mut bus = CoreBus {
                mem: &mut core.mem,
                gpu: &mut core.gpu,
                dma: &mut core.dma,
                irq: &mut core.irq,
                timers: &mut core.timers,
                memctrl: &mut core.memctrl,
                cache_ctrl: &mut core.cache_ctrl,
                sio0: &mut core.sio0,
                cdrom: &mut core.cdrom,
                spu: &mut core.spu,
            };
            // Fill red 16x16 at (0,0) through the GP0 port.
            bus.store32(0x1F80_1810, 0x0200_00FF);
            bus.store32(0x1F80_1810, 0x0000_0000);
            bus.store32(0x1F80_1810, 0x0010_0010);
        }
        assert_eq!(
            core.gpu.vram_at(0, 0),
            crate::gpu::rgb_to_bgr555(0xFF, 0, 0)
        );
    }

    #[test]
    fn gpustat_read_via_bus_has_ready_bits() {
        let mut core = PsxCore::new();
        let val = {
            let mut bus = CoreBus {
                mem: &mut core.mem,
                gpu: &mut core.gpu,
                dma: &mut core.dma,
                irq: &mut core.irq,
                timers: &mut core.timers,
                memctrl: &mut core.memctrl,
                cache_ctrl: &mut core.cache_ctrl,
                sio0: &mut core.sio0,
                cdrom: &mut core.cdrom,
                spu: &mut core.spu,
            };
            bus.load32(0x1F80_1814)
        };
        assert_ne!(val, 0);
        assert_ne!(val & (1 << 26), 0);
        assert_ne!(val & (1 << 28), 0);
    }

    #[test]
    fn irq_registers_via_bus() {
        let mut core = PsxCore::new();
        let mut bus = CoreBus {
            mem: &mut core.mem,
            gpu: &mut core.gpu,
            dma: &mut core.dma,
            irq: &mut core.irq,
            timers: &mut core.timers,
            memctrl: &mut core.memctrl,
            cache_ctrl: &mut core.cache_ctrl,
            sio0: &mut core.sio0,
            cdrom: &mut core.cdrom,
            spu: &mut core.spu,
        };
        bus.store32(0x1F80_1074, 0x1); // I_MASK = VBlank
        assert_eq!(bus.load32(0x1F80_1074), 0x1);
    }

    #[test]
    fn narrow_io_read_adapts_width_against_backed_register() {
        // A legal narrow read of a backed I/O register returns the correctly
        // width-adapted value (byte lane, half lane, zero-extended word) — the
        // width-policing the ps1-tests `io-access-bitwidth` RAM/scratchpad/SPU
        // rows exercise. No wrong-width access traps here; only misaligned word
        // accesses (handled in the CPU) raise an address error.
        let mut core = PsxCore::new();
        let mut bus = CoreBus {
            mem: &mut core.mem,
            gpu: &mut core.gpu,
            dma: &mut core.dma,
            irq: &mut core.irq,
            timers: &mut core.timers,
            memctrl: &mut core.memctrl,
            cache_ctrl: &mut core.cache_ctrl,
            sio0: &mut core.sio0,
            cdrom: &mut core.cdrom,
            spu: &mut core.spu,
        };
        // SPU register file (0x1F80_1C00) is byte-addressable backing store.
        bus.store16(0x1F80_1C00, 0xABCD);
        assert_eq!(bus.load8(0x1F80_1C00), 0xCD, "byte lane");
        assert_eq!(bus.load8(0x1F80_1C01), 0xAB, "high byte lane");
        assert_eq!(bus.load16(0x1F80_1C00), 0xABCD, "half read");
        assert_eq!(bus.load32(0x1F80_1C00), 0x0000_ABCD, "word zero-extends");
    }

    #[test]
    fn framebuffer_reflects_vram_when_enabled() {
        let mut core = PsxCore::new();
        core.gpu.display_enabled = true;
        core.gpu.color_depth_24 = false;
        // White pixel at display origin (0,0).
        core.gpu.set_vram(0, 0, 0x7FFF);
        let frame = core.framebuffer_rgba();
        assert_eq!(frame.len(), FRAME_RGBA_BYTES);
        assert_eq!(&frame[0..4], &[0xF8, 0xF8, 0xF8, 0xFF]);
    }

    #[test]
    fn framebuffer_black_when_display_disabled() {
        let mut core = PsxCore::new();
        core.gpu.display_enabled = false;
        core.gpu.set_vram(0, 0, 0x7FFF);
        let frame = core.framebuffer_rgba();
        assert_eq!(&frame[0..4], &[0, 0, 0, 0xFF]);
    }

    #[test]
    fn save_load_round_trips_gpu_vram() {
        let mut core = PsxCore::new();
        core.gpu.set_vram(3, 3, 0x1234);
        core.irq.set(IrqLine::VBlank);
        let snap = core.save_state();
        let mut other = PsxCore::new();
        other.load_state(&snap);
        assert_eq!(other.gpu.vram_at(3, 3), 0x1234);
        assert_eq!(other.irq.read_stat(), snap.irq.read_stat());
    }

    #[test]
    fn step_frame_raises_vblank_and_toggles_field() {
        let mut core = PsxCore::new();
        let field_before = core.gpu.field;
        core.execute(Command::StepFrame).unwrap();
        assert_ne!(core.gpu.field, field_before);
        assert_ne!(core.irq.read_stat() & 0x1, 0, "VBlank bit should be set");
    }

    #[test]
    fn hardware_interrupt_taken_when_enabled() {
        let mut core = PsxCore::new();
        core.set_pc(0);
        // Enable interrupts: IEc (bit 0) and IM for the hardware line (bit 10),
        // clearing BEV so the handler vectors to RAM.
        core.cpu.cop0[crate::cpu::COP0_SR] = 0x1 | (1 << 10);
        core.irq.write_mask(1 << IrqLine::VBlank.bit());
        core.irq.set(IrqLine::VBlank);
        assert!(core.irq.pending());
        core.execute(Command::StepCpu).unwrap();
        assert_eq!(core.pc(), 0x8000_0080, "interrupt should vector to handler");
    }

    #[test]
    fn hardware_interrupt_not_taken_when_disabled() {
        let mut core = PsxCore::new();
        core.set_pc(0);
        // Interrupts globally disabled (reset SR has IEc=0).
        core.irq.write_mask(1 << IrqLine::VBlank.bit());
        core.irq.set(IrqLine::VBlank);
        assert!(core.irq.pending());
        core.execute(Command::StepCpu).unwrap();
        // No interrupt: PC simply advanced past the (zero/NOP) instruction.
        assert_eq!(core.pc(), 0x4);
    }
}
