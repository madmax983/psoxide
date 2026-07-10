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
use crate::iostubs::{CACHE_CTRL_REG, CacheCtrl, MemCtrl, Sio0};
use crate::irq::{Irq, IrqLine};
use crate::mdec::Mdec;
use crate::spu::Spu;
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
/// CPU cycles elapsed per [`Command::StepFrame`]. With the cycle-accurate access
/// timing model an instruction no longer costs a fixed one cycle, so `StepFrame`
/// paces by a cycle budget (not an instruction count) to hold ~60fps regardless
/// of how many wait states the running code incurs.
pub const CYCLES_PER_FRAME: u64 = CPU_CLOCK_HZ / FRAMES_PER_SECOND;

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
    /// L2 trigger.
    L2,
    /// R2 trigger.
    R2,
    /// L3 (left analog-stick click; DualShock only).
    L3,
    /// R3 (right analog-stick click; DualShock only).
    R3,
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
            Self::L3 => 1 << 1,
            Self::R3 => 1 << 2,
            Self::Start => 1 << 3,
            Self::Up => 1 << 4,
            Self::Right => 1 << 5,
            Self::Down => 1 << 6,
            Self::Left => 1 << 7,
            Self::L2 => 1 << 8,
            Self::R2 => 1 << 9,
            Self::L1 => 1 << 10,
            Self::R1 => 1 << 11,
            Self::Triangle => 1 << 12,
            Self::Circle => 1 << 13,
            Self::Cross => 1 << 14,
            Self::Square => 1 << 15,
        }
    }
}

/// The kind of controller device attached to a port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerKind {
    /// No controller plugged in.
    Disconnected,
    /// A standard digital pad (SCPH-1080).
    Digital,
    /// A DualShock / DualAnalog pad (SCPH-1200).
    Analog,
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
    /// Insert a memory card into a controller-port slot. `data` is padded or
    /// truncated to the 128 KB card size.
    InsertMemoryCard {
        /// Slot index (0 or 1).
        slot: u8,
        /// Card image bytes (128 KB expected; padded/truncated otherwise).
        data: Vec<u8>,
    },
    /// Eject the memory card in a slot, if any.
    EjectMemoryCard {
        /// Slot index (0 or 1).
        slot: u8,
    },
    /// Clear the dirty flag on a memory card after the frontend has flushed its
    /// image to persistent storage. (Query is `&self` and cannot clear it.)
    ClearMemoryCardDirty {
        /// Slot index (0 or 1).
        slot: u8,
    },
    /// Reset the CPU to the BIOS entry vector.
    Reset,
    /// Execute one CPU instruction.
    StepCpu,
    /// Execute a frame's worth of instructions ([`STEPS_PER_FRAME`]).
    ///
    /// Honours the paused flag: while paused this is a **no-op** (the machine
    /// does not advance, no VBlank is raised), so a frontend can call it
    /// unconditionally every host frame and get correct pause behaviour for
    /// free. Use [`Command::FrameStep`] to advance a single frame while paused.
    StepFrame,
    /// Advance exactly one frame, **even while paused** (the single-step /
    /// frame-advance control). Unlike [`Command::StepFrame`] it ignores the
    /// paused flag; the pause state itself is left unchanged.
    FrameStep,
    /// Replace a controller port's button bitfield.
    SetControllerState {
        /// Controller port index (0 or 1).
        port: u8,
        /// Pressed-button bitfield.
        buttons: u16,
    },
    /// Set the kind of controller attached to a port (digital / analog /
    /// disconnected), preserving the held buttons across the change.
    SetControllerType {
        /// Controller port index (0 or 1).
        port: u8,
        /// Device kind to attach.
        kind: ControllerKind,
    },
    /// Update a controller's analog-stick axes (`0x80` = centre). Promotes a
    /// non-analog port to an analog pad.
    SetControllerSticks {
        /// Controller port index (0 or 1).
        port: u8,
        /// Right stick `(x, y)`.
        right: (u8, u8),
        /// Left stick `(x, y)`.
        left: (u8, u8),
    },
    /// Simulate a press of the pad's physical "Analog" toggle button, flipping
    /// analog mode unless the host has locked it. A no-op for a non-analog port.
    SetControllerAnalogButton {
        /// Controller port index (0 or 1).
        port: u8,
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
    /// Return the memory-card image + dirty flag for a slot.
    MemoryCard {
        /// Slot index (0 or 1).
        slot: u8,
    },
    /// Return the analog pad's small/large rumble-motor actuation for a port.
    ControllerRumble {
        /// Controller port index (0 or 1).
        port: u8,
    },
    /// Return audio-pacing counters (SPU output-queue fill + monotonic produced/
    /// dropped sample counts) and the monotonic emulated-cycle count, for a
    /// frontend HUD. Counters only: the core does not pace frames.
    AudioStatus,
}

/// Audio-pacing / emulation-speed bookkeeping for a frontend HUD
/// ([`CoreQuery::AudioStatus`]). All fields are counters the core already
/// maintains; the frontend derives rates/speed from deltas over wall time — the
/// core does no host-side pacing itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AudioStatus {
    /// Interleaved-stereo sample-pairs currently queued in the SPU (the output
    /// buffer's fill level, drained by [`PsxCore::drain_audio`]).
    pub queued_sample_pairs: usize,
    /// Monotonic count of stereo sample-pairs the SPU has generated since
    /// power-on.
    pub samples_produced: u64,
    /// Monotonic count of stereo sample-pairs dropped because the output queue
    /// saturated (the frontend fell behind draining) — the core-side audio
    /// under/over-run signal.
    pub samples_dropped: u64,
    /// Monotonic emulated CPU-cycle count (the clock `StepFrame`/`FrameStep`
    /// advance). A HUD divides its delta by [`CPU_CLOCK_HZ`] and wall time to
    /// get emulated-vs-real speed.
    pub emulated_cycles: u64,
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
    /// [`CoreQuery::MemoryCard`] response. `present` is `false` (with an empty
    /// `data` and `dirty == false`) when no card is inserted in the slot.
    MemoryCard {
        /// Whether a card is inserted in the queried slot.
        present: bool,
        /// The 128 KB card image (empty when `present` is `false`).
        data: Vec<u8>,
        /// Whether the card has unsaved modifications.
        dirty: bool,
    },
    /// [`CoreQuery::ControllerRumble`] response. `present` is `false` (with
    /// `small`/`large` both 0) when the port holds no analog pad.
    ControllerRumble {
        /// Whether the queried port holds an analog pad.
        present: bool,
        /// Small-motor actuation last latched from a poll.
        small: u8,
        /// Large-motor actuation last latched from a poll.
        large: u8,
    },
    /// [`CoreQuery::AudioStatus`] response.
    AudioStatus(AudioStatus),
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

    /// Reads a little-endian half-word, resolving the region once and reading
    /// both bytes from the backing slice. For an aligned access this is exactly
    /// `u16::from_le_bytes([read8(virt), read8(virt+1)])`: RAM/scratchpad/BIOS
    /// return their contiguous bytes and every other region returns 0 (matching
    /// the byte path's `_ => 0`). Callers must only use this for non-I/O regions
    /// (the bus routes I/O separately); word-alignment guarantees the two bytes
    /// stay within one region so folding/masking are equivalent to the byte path.
    #[must_use]
    pub fn read16(&self, virt: u32) -> u16 {
        let phys = mask_region(virt);
        match map_region(phys) {
            BusRegion::MainRam => {
                let b = (phys & MAIN_RAM_MASK) as usize;
                u16::from_le_bytes([self.ram[b], self.ram[b + 1]])
            }
            BusRegion::Scratchpad => {
                let b = (phys & 0x3FF) as usize;
                u16::from_le_bytes([self.scratchpad[b], self.scratchpad[b + 1]])
            }
            BusRegion::Bios => {
                let o = (phys - 0x1FC0_0000) as usize;
                u16::from_le_bytes([
                    self.bios.get(o).copied().unwrap_or(0),
                    self.bios.get(o + 1).copied().unwrap_or(0),
                ])
            }
            _ => 0,
        }
    }

    /// Reads a little-endian word; see [`Memory::read16`] for the equivalence
    /// argument. Exactly `u32::from_le_bytes([read8(virt)..read8(virt+3)])` for
    /// an aligned, non-I/O access.
    #[must_use]
    pub fn read32(&self, virt: u32) -> u32 {
        let phys = mask_region(virt);
        match map_region(phys) {
            BusRegion::MainRam => {
                let b = (phys & MAIN_RAM_MASK) as usize;
                u32::from_le_bytes([
                    self.ram[b],
                    self.ram[b + 1],
                    self.ram[b + 2],
                    self.ram[b + 3],
                ])
            }
            BusRegion::Scratchpad => {
                let b = (phys & 0x3FF) as usize;
                u32::from_le_bytes([
                    self.scratchpad[b],
                    self.scratchpad[b + 1],
                    self.scratchpad[b + 2],
                    self.scratchpad[b + 3],
                ])
            }
            BusRegion::Bios => {
                let o = (phys - 0x1FC0_0000) as usize;
                u32::from_le_bytes([
                    self.bios.get(o).copied().unwrap_or(0),
                    self.bios.get(o + 1).copied().unwrap_or(0),
                    self.bios.get(o + 2).copied().unwrap_or(0),
                    self.bios.get(o + 3).copied().unwrap_or(0),
                ])
            }
            _ => 0,
        }
    }

    /// Writes a little-endian half-word, resolving the region once. Exactly
    /// mirrors the byte path (`write8` twice): only RAM and scratchpad are
    /// writable; BIOS/I/O/unmapped are ignored. For non-I/O callers with an
    /// aligned address this is equivalent to two `write8` calls.
    pub fn write16(&mut self, virt: u32, value: u16) {
        let phys = mask_region(virt);
        let bytes = value.to_le_bytes();
        match map_region(phys) {
            BusRegion::MainRam => {
                let b = (phys & MAIN_RAM_MASK) as usize;
                self.ram[b] = bytes[0];
                self.ram[b + 1] = bytes[1];
            }
            BusRegion::Scratchpad => {
                let b = (phys & 0x3FF) as usize;
                self.scratchpad[b] = bytes[0];
                self.scratchpad[b + 1] = bytes[1];
            }
            _ => {}
        }
    }

    /// Writes a little-endian word; see [`Memory::write16`]. Equivalent to four
    /// `write8` calls for an aligned, non-I/O access.
    pub fn write32(&mut self, virt: u32, value: u32) {
        let phys = mask_region(virt);
        let bytes = value.to_le_bytes();
        match map_region(phys) {
            BusRegion::MainRam => {
                let b = (phys & MAIN_RAM_MASK) as usize;
                self.ram[b] = bytes[0];
                self.ram[b + 1] = bytes[1];
                self.ram[b + 2] = bytes[2];
                self.ram[b + 3] = bytes[3];
            }
            BusRegion::Scratchpad => {
                let b = (phys & 0x3FF) as usize;
                self.scratchpad[b] = bytes[0];
                self.scratchpad[b + 1] = bytes[1];
                self.scratchpad[b + 2] = bytes[2];
                self.scratchpad[b + 3] = bytes[3];
            }
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
    mdec: &'a mut Mdec,
    /// CPU-cycle cost of the data access performed by the current instruction
    /// (0 = none). Set once per load/store at the top-level [`Bus`] entry from
    /// the original access width and address, read back by `step_cpu` to charge
    /// wait states to the hardware timers.
    access_cost: u32,
    /// Observation cycle for the lazy scheduler: the absolute CPU cycle the
    /// devices must be caught up to before any device register is read/written
    /// during this instruction (`= cpu.cycles + 1`, matching the naive path's
    /// post-top-tick device clock). Ignored when `device_clock` is `None`.
    obs: u64,
    /// Absolute cycle the four per-cycle devices have been advanced to. `None`
    /// for one-off (non-instruction) bus accesses, which are pre-synchronized by
    /// the caller and never need in-bus catch-up.
    device_clock: Option<&'a mut u64>,
    /// Set `true` when this access touched a device register window (so the
    /// scheduler recomputes the next-event deadline after the instruction).
    io_touched: bool,
}

impl CoreBus<'_> {
    /// Returns `true` if the physical address falls in the I/O register window.
    #[inline]
    fn is_io(phys: u32) -> bool {
        matches!(map_region(phys), BusRegion::IoPorts)
    }

    /// Called before dispatching a device-register access. Marks the access as
    /// I/O (so the scheduler recomputes the next-event deadline afterwards) and,
    /// in scheduled mode, catches the per-cycle devices up to the observation
    /// cycle so the register read/write sees the same device state the naive
    /// path (which top-ticks the devices before executing) would present.
    ///
    /// In the non-deadline branch `obs < next_deadline`, so this catch-up
    /// crosses no device event and cannot set a spurious `I_STAT` bit; in the
    /// deadline branch the devices are already at `obs`, so it is a no-op.
    #[inline]
    fn touch_io(&mut self) {
        self.io_touched = true;
        let obs = self.obs;
        let Some(dc) = self.device_clock.as_deref_mut() else {
            return;
        };
        catch_up_devices(
            dc,
            obs,
            self.timers,
            self.cdrom,
            self.spu,
            self.sio0,
            self.irq,
        );
    }

    /// Records the wait-state cost of a data access of `width_bytes` at physical
    /// address `phys`, from the current memory-control timing configuration.
    #[inline]
    fn charge(&mut self, phys: u32, width_bytes: u32) {
        self.access_cost = crate::timing::access_cycles(
            crate::timing::access_class(phys),
            width_bytes,
            &self.memctrl.timing(),
        );
    }

    fn io_read32(&mut self, phys: u32) -> u32 {
        match phys {
            // Expansion region 2 (debug board) is unpopulated: reads float the
            // bus high (all-ones open bus).
            0x1F80_2000..=0x1F80_2FFF => 0xFFFF_FFFF,
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
            _ if Mdec::contains(phys) => self.mdec.read32(phys),
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
                self.dma.write32(
                    phys, val, self.mem, self.gpu, self.cdrom, self.spu, self.mdec, self.irq,
                );
            }
            TIMERS_BASE..=TIMERS_END => self.timers.write32(phys, val),
            _ if MemCtrl::contains(phys) => self.memctrl.write32(phys, val),
            _ if Sio0::contains(phys) => self.sio0.write32(phys, val),
            _ if Cdrom::contains(phys) => self.cdrom.write32(phys, val),
            _ if Spu::contains(phys) => self.spu.write32(phys, val),
            _ if Mdec::contains(phys) => self.mdec.write32(phys, val),
            // Other I/O ports are stubbed (ignored).
            _ => {}
        }
    }

    fn io_read16(&mut self, phys: u32) -> u16 {
        match phys {
            // Expansion region 2 open bus (see `io_read32`).
            0x1F80_2000..=0x1F80_2FFF => 0xFFFF,
            0x1F80_1814 => self.gpu.gpustat() as u16,
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
            // Expansion region 2 open bus (see `io_read32`).
            0x1F80_2000..=0x1F80_2FFF => 0xFF,
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
        self.charge(phys, 1);
        if Self::is_io(phys) {
            self.touch_io();
            return self.io_read8(phys);
        }
        self.mem.read8(addr)
    }
    fn load16(&mut self, addr: u32) -> u16 {
        let phys = mask_region(addr);
        self.charge(phys, 2);
        if Self::is_io(phys) {
            self.touch_io();
            return self.io_read16(phys);
        }
        self.mem.read16(addr)
    }
    fn load32(&mut self, addr: u32) -> u32 {
        let phys = mask_region(addr);
        self.charge(phys, 4);
        if Self::is_io(phys) {
            self.touch_io();
            return self.io_read32(phys);
        }
        if phys == CACHE_CTRL_REG {
            return self.cache_ctrl.read32();
        }
        self.mem.read32(addr)
    }
    fn store8(&mut self, addr: u32, value: u8) {
        let phys = mask_region(addr);
        self.charge(phys, 1);
        if Self::is_io(phys) {
            self.touch_io();
            self.io_write8(phys, value);
            return;
        }
        self.mem.write8(addr, value);
    }
    fn store16(&mut self, addr: u32, value: u16) {
        let phys = mask_region(addr);
        self.charge(phys, 2);
        if Self::is_io(phys) {
            self.touch_io();
            self.io_write16(phys, value);
            return;
        }
        self.mem.write16(addr, value);
    }
    fn store32(&mut self, addr: u32, value: u32) {
        let phys = mask_region(addr);
        self.charge(phys, 4);
        if Self::is_io(phys) {
            self.touch_io();
            self.io_write32(phys, value);
            return;
        }
        if phys == CACHE_CTRL_REG {
            self.cache_ctrl.write32(value);
            return;
        }
        self.mem.write32(addr, value);
    }

    #[inline]
    fn set_access_cost(&mut self, cycles: u32) {
        self.access_cost = cycles;
    }
}

/// Advances the four per-cycle devices (timers, CD-ROM, SPU, SIO0) from
/// `*device_clock` up to `target`, delivering interrupts through `irq`.
///
/// The device order mirrors the naive per-instruction loop exactly — timers,
/// then CD-ROM (with the decoded-CD-audio → SPU bridge), then SPU, then SIO0 —
/// so the observable result is identical to ticking each device one cycle at a
/// time. Timers, SPU and SIO0 tick in a single batched call because their
/// batched tick is proven equivalent to per-cycle stepping. The CD-ROM + SPU
/// pair is batched too **except** while the CD-ROM is producing audio, when it
/// is stepped one cycle at a time so decoded CD frames reach the SPU in the
/// same order (and at the same sample boundaries) the naive loop delivered them.
fn catch_up_devices(
    device_clock: &mut u64,
    target: u64,
    timers: &mut Timers,
    cdrom: &mut Cdrom,
    spu: &mut Spu,
    sio0: &mut Sio0,
    irq: &mut Irq,
) {
    let cur = *device_clock;
    if target <= cur {
        return;
    }
    // The gap is bounded well under u32 in practice (the SPU deadline caps it
    // near CYCLES_PER_SAMPLE); chunk in u32 steps to stay total-correct anyway.
    let mut remaining = target - cur;
    while remaining > 0 {
        let step = u32::try_from(remaining.min(u64::from(u32::MAX))).unwrap_or(u32::MAX);
        if cdrom.is_cd_audio_active() {
            // Timers and SIO0 are independent of the CD→SPU bridge, so they can
            // still batch; the CD-ROM/SPU pair steps per cycle to keep decoded
            // frame delivery ordered with SPU sample consumption.
            timers.tick(step, irq);
            for _ in 0..step {
                cdrom.tick(1, irq);
                if cdrom.has_cd_audio() {
                    let frames = cdrom.take_cd_audio();
                    spu.push_cd_audio_samples(&frames);
                }
                spu.tick(1, irq);
            }
            sio0.tick(step, irq);
        } else {
            // No CD audio is produced in this window, so batching the CD-ROM and
            // SPU ticks (with the bridge between them) is exactly equivalent.
            timers.tick(step, irq);
            cdrom.tick(step, irq);
            if cdrom.has_cd_audio() {
                let frames = cdrom.take_cd_audio();
                spu.push_cd_audio_samples(&frames);
            }
            spu.tick(step, irq);
            sio0.tick(step, irq);
        }
        remaining -= u64::from(step);
    }
    *device_clock = target;
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
    /// MDEC (macroblock decoder) state.
    #[serde(default)]
    pub mdec: Mdec,
    // ---- identity metadata (added after v0.1; all `#[serde(default)]` so
    // legacy `.ss` files written before identity metadata still load) --------
    /// Emulator core version (`CARGO_PKG_VERSION`) that wrote the snapshot.
    /// Empty for a legacy snapshot that predates identity metadata.
    #[serde(default)]
    pub core_version: String,
    /// Identity hash of the BIOS image loaded when the snapshot was taken
    /// (`None` = no BIOS, or a legacy snapshot). Validated on
    /// [`PsxCore::load_state_checked`].
    #[serde(default)]
    pub bios_hash: Option<u64>,
    /// Identity hash of the disc image mounted when the snapshot was taken
    /// (`None` = no disc, or a legacy snapshot). Validated on
    /// [`PsxCore::load_state_checked`].
    #[serde(default)]
    pub disc_hash: Option<u64>,
}

/// Outcome of a successful [`PsxCore::load_state_checked`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoadStateOk {
    /// The snapshot's identity metadata matched the loaded BIOS/disc and it was
    /// applied.
    Loaded,
    /// The snapshot carried no identity metadata (a legacy `.ss` file written
    /// before identity was added). It was applied **without** validation — the
    /// frontend may wish to surface this.
    LoadedLegacy,
}

/// Typed failure from [`PsxCore::load_state_checked`]. On any of these the
/// snapshot is **not** applied — the running machine is left untouched — so the
/// frontend can present the mismatch and decide what to do.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LoadStateError {
    /// The snapshot's BIOS identity differs from the currently loaded BIOS.
    /// `expected` is the snapshot's hash, `actual` the running core's.
    BiosMismatch {
        /// BIOS hash recorded in the snapshot.
        expected: Option<u64>,
        /// BIOS hash of the currently loaded image.
        actual: Option<u64>,
    },
    /// The snapshot's disc identity differs from the currently mounted disc.
    /// `expected` is the snapshot's hash, `actual` the running core's.
    DiscMismatch {
        /// Disc hash recorded in the snapshot.
        expected: Option<u64>,
        /// Disc hash of the currently mounted disc.
        actual: Option<u64>,
    },
    /// The snapshot was written by a different core version. The serialized
    /// format is still compatible (deserialization already succeeded), so a
    /// frontend may choose to force-load via [`PsxCore::load_state`].
    VersionMismatch {
        /// Core version that wrote the snapshot.
        expected: String,
        /// This build's core version.
        actual: String,
    },
}

impl fmt::Display for LoadStateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BiosMismatch { expected, actual } => write!(
                f,
                "save state was made with a different BIOS (snapshot {expected:?}, loaded {actual:?})"
            ),
            Self::DiscMismatch { expected, actual } => write!(
                f,
                "save state was made with a different disc (snapshot {expected:?}, loaded {actual:?})"
            ),
            Self::VersionMismatch { expected, actual } => write!(
                f,
                "save state was written by psoxide {expected} (this build is {actual})"
            ),
        }
    }
}

impl std::error::Error for LoadStateError {}

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
    mdec: Mdec,
    paused: bool,
    controllers: [u16; 2],
    /// When `true` (the default), [`Self::step_cpu`] uses the lazy device
    /// scheduler ([`Self::step_cpu_scheduled`]); when `false` it uses the
    /// original per-instruction fan-out ([`Self::step_cpu_naive`]). The two
    /// paths are behaviourally identical (bit-for-bit); the flag exists so
    /// differential tests can A/B them, and so a frontend can opt out.
    ///
    /// Not part of [`CoreSnapshot`]: it is an execution-strategy toggle, not
    /// machine state.
    scheduler_enabled: bool,
    /// Absolute CPU cycle the four per-cycle devices (timers, CD-ROM, SPU,
    /// SIO0) have been advanced to. In naive mode this tracks `cpu.cycles`; in
    /// scheduled mode the devices lag `cpu.cycles` and are reconciled lazily.
    /// Not serialized (reconstructed on load).
    device_clock: u64,
    /// Cached minimum absolute CPU cycle at which any device would next set an
    /// `I_STAT` bit (`u64::MAX` = no device event pending). The scheduler runs
    /// instructions freely while `cpu.cycles + 1 < next_deadline`. Not
    /// serialized (recomputed on load).
    next_deadline: u64,
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
            mdec: Mdec::new(),
            paused: false,
            controllers: [0; 2],
            scheduler_enabled: true,
            device_clock: 0,
            next_deadline: 0,
        }
    }

    /// Selects the CPU-stepping strategy: the lazy device scheduler (`true`,
    /// the default) or the original per-instruction device fan-out (`false`).
    /// Both are bit-for-bit equivalent; this exists for differential testing
    /// and for a frontend that wants to force the reference path. Switching
    /// resynchronizes the device clock to `cpu.cycles`.
    pub fn set_scheduler_enabled(&mut self, enabled: bool) {
        // Bring the devices fully up to date before changing strategy so the
        // two paths start from an identical, rest-consistent state.
        self.catch_up_all(self.cpu.cycles);
        self.scheduler_enabled = enabled;
        self.device_clock = self.cpu.cycles;
        self.next_deadline = 0;
    }

    /// Returns whether the lazy device scheduler is active.
    #[must_use]
    pub fn is_scheduler_enabled(&self) -> bool {
        self.scheduler_enabled
    }

    /// Flushes the lazily-scheduled devices up to `cpu.cycles`, so every device
    /// reflects the full elapsed time (a no-op in naive mode, where the devices
    /// already track `cpu.cycles`). Exposed for differential tests that need the
    /// scheduled and reference strategies in a directly-comparable, rest-
    /// consistent state; it eagerly performs the device catch-up the next
    /// scheduled step would do anyway, so it does not change observable
    /// behaviour.
    pub fn sync_devices(&mut self) {
        self.catch_up_all(self.cpu.cycles);
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

    /// Drains all queued interleaved-stereo audio samples (L, R, L, R, ...) the
    /// SPU has produced since the last drain. The desktop frontend calls this
    /// once per frame and feeds the result to the host audio device.
    ///
    /// The SPU emits 44.1 kHz stereo, so a `StepFrame` yields ~735 sample pairs
    /// (1470 `i16` values). The internal queue is capped at ~1 second of audio,
    /// so a headless or paused run cannot grow it unbounded.
    pub fn drain_audio(&mut self) -> Vec<i16> {
        self.spu.drain_samples()
    }

    /// Builds a transient [`CoreBus`] borrowing every peripheral, for one-off
    /// bus accesses that are not part of a CPU instruction step.
    ///
    /// `device_clock` is `None`: these accesses do not tick the per-cycle
    /// devices themselves. The public [`Self::store8`]/[`Self::load8`] wrappers
    /// synchronize the devices to `cpu.cycles` around the access instead, so a
    /// frontend/test poke of a device register observes the same state the
    /// scheduled step loop would have at this rest point.
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
            mdec: &mut self.mdec,
            access_cost: 0,
            obs: self.cpu.cycles,
            device_clock: None,
            io_touched: false,
        }
    }

    /// Synchronizes the per-cycle devices to `cpu.cycles` before a one-off
    /// (non-instruction) bus access, then rebuilds the deadline afterwards, so
    /// frontend/test register pokes are consistent with the scheduled loop. A
    /// no-op in naive mode (the devices already track `cpu.cycles`).
    fn sync_for_one_off(&mut self) {
        if self.scheduler_enabled {
            self.catch_up_all(self.cpu.cycles);
        }
    }
    fn resync_after_one_off(&mut self) {
        if self.scheduler_enabled {
            self.recompute_deadline();
        }
    }

    /// Performs an 8-bit store through the full system bus, routing to device
    /// registers exactly as a CPU `sb` would.
    ///
    /// This is the frontend/integration seam for driving memory-mapped
    /// peripherals (e.g. the CD-ROM controller at `0x1F80_1800..=0x1F80_1803`)
    /// without hand-assembling guest code.
    pub fn store8(&mut self, addr: u32, value: u8) {
        self.sync_for_one_off();
        self.core_bus().store8(addr, value);
        self.resync_after_one_off();
    }

    /// Performs a 16-bit store through the full system bus (device routing
    /// included), exactly as a CPU `sh` would. Used to program the 16-bit SPU
    /// register file from a frontend/test.
    pub fn store16(&mut self, addr: u32, value: u16) {
        self.sync_for_one_off();
        self.core_bus().store16(addr, value);
        self.resync_after_one_off();
    }

    /// Performs a 32-bit store through the full system bus (device routing
    /// included), exactly as a CPU `sw` would. Used to program the DMA and
    /// interrupt-controller register files from a frontend/test.
    pub fn store32(&mut self, addr: u32, value: u32) {
        self.sync_for_one_off();
        self.core_bus().store32(addr, value);
        self.resync_after_one_off();
    }

    /// Performs an 8-bit load through the full system bus, popping device FIFOs
    /// exactly as a CPU `lb` would.
    pub fn load8(&mut self, addr: u32) -> u8 {
        self.sync_for_one_off();
        let v = self.core_bus().load8(addr);
        self.resync_after_one_off();
        v
    }

    /// Performs a 16-bit load through the full system bus (device routing
    /// included), exactly as a CPU `lh` would.
    pub fn load16(&mut self, addr: u32) -> u16 {
        self.sync_for_one_off();
        let v = self.core_bus().load16(addr);
        self.resync_after_one_off();
        v
    }

    /// Performs a 32-bit load through the full system bus (device routing
    /// included), exactly as a CPU `lw` would.
    pub fn load32(&mut self, addr: u32) -> u32 {
        self.sync_for_one_off();
        let v = self.core_bus().load32(addr);
        self.resync_after_one_off();
        v
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
            Command::InsertMemoryCard { slot, data } => {
                self.sio0.insert_card(slot as usize, data);
            }
            Command::EjectMemoryCard { slot } => self.sio0.eject_card(slot as usize),
            Command::ClearMemoryCardDirty { slot } => {
                self.sio0.clear_card_dirty(slot as usize);
            }
            Command::Reset => self.cpu.reset(),
            Command::StepCpu => self.step_cpu(),
            Command::StepFrame => {
                // Paused gates a normal frame advance: no machine progress and
                // no VBlank while paused, so the frontend can call this every
                // host frame without a pause guard of its own.
                if !self.paused {
                    self.advance_frame();
                }
            }
            Command::FrameStep => self.advance_frame(),
            Command::SetControllerState { port, buttons } => {
                if let Some(slot) = self.controllers.get_mut(port as usize) {
                    *slot = buttons;
                }
                // Feed the held buttons into the SIO0 pad so a transfer reads
                // them out. `set_buttons` ignores out-of-range ports.
                self.sio0.set_buttons(port as usize, buttons);
            }
            Command::SetControllerType { port, kind } => {
                self.sio0.set_controller_type(port as usize, kind);
            }
            Command::SetControllerSticks { port, right, left } => {
                self.sio0.set_sticks(port as usize, right, left);
            }
            Command::SetControllerAnalogButton { port } => {
                self.sio0.press_analog_button(port as usize);
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
            CoreQuery::MemoryCard { slot } => match self.sio0.card_image(slot as usize) {
                Some((data, dirty)) => QueryResult::MemoryCard {
                    present: true,
                    data,
                    dirty,
                },
                None => QueryResult::MemoryCard {
                    present: false,
                    data: Vec::new(),
                    dirty: false,
                },
            },
            CoreQuery::ControllerRumble { port } => match self.sio0.motor_state(port as usize) {
                Some((small, large)) => QueryResult::ControllerRumble {
                    present: true,
                    small,
                    large,
                },
                None => QueryResult::ControllerRumble {
                    present: false,
                    small: 0,
                    large: 0,
                },
            },
            CoreQuery::AudioStatus => QueryResult::AudioStatus(AudioStatus {
                queued_sample_pairs: self.spu.queued_sample_pairs(),
                samples_produced: self.spu.samples_produced(),
                samples_dropped: self.spu.samples_dropped(),
                emulated_cycles: self.cpu.cycles,
            }),
        }
    }

    /// Runs one frame's worth of CPU cycles and raises the once-per-frame
    /// VBlank, ignoring the paused flag. Shared by [`Command::StepFrame`] (which
    /// gates on paused) and [`Command::FrameStep`] (which does not).
    fn advance_frame(&mut self) {
        // Pace by a CPU-cycle budget rather than a fixed instruction count:
        // with wait-state timing an instruction costs a variable number of
        // cycles, so a cycle budget keeps the frame duration stable. `step_cpu`
        // always advances `cpu.cycles` by at least 1, so this loop terminates.
        let target = self.cpu.cycles.wrapping_add(CYCLES_PER_FRAME);
        while self.cpu.cycles < target {
            self.step_cpu();
        }
        // Once per frame: advance the interlace field and raise VBlank.
        self.gpu.field = !self.gpu.field;
        self.irq.set(IrqLine::VBlank);
    }

    /// Executes one CPU instruction (plus its device/interrupt bookkeeping),
    /// dispatching to the lazy scheduler or the reference per-instruction loop.
    fn step_cpu(&mut self) {
        if self.scheduler_enabled {
            self.step_cpu_scheduled();
        } else {
            self.step_cpu_naive();
        }
    }

    /// Reference implementation: tick every per-cycle device by one cycle,
    /// poll for an interrupt, execute one instruction, then charge the extra
    /// wait-state cycles to the CPU counter and every device. This is the
    /// behaviour the lazy scheduler ([`Self::step_cpu_scheduled`]) reproduces
    /// bit-for-bit; it is kept compiled for differential testing and as the
    /// fallback path.
    fn step_cpu_naive(&mut self) {
        // Advance the hardware timers by one CPU cycle first, so a timer that
        // reaches its target/overflow this cycle can deliver its interrupt at
        // this same instruction boundary.
        self.timers.tick(1, &mut self.irq);
        // Advance the CD-ROM controller so a queued command response can latch
        // and raise its interrupt this cycle.
        self.cdrom.tick(1, &mut self.irq);
        // Hand any CD-audio frames the CD-ROM controller decoded this cycle
        // (XA-ADPCM / CD-DA) to the SPU, which mixes them through its CD input.
        if self.cdrom.has_cd_audio() {
            let cd_frames = self.cdrom.take_cd_audio();
            self.spu.push_cd_audio_samples(&cd_frames);
        }
        // Advance the SPU by one CPU cycle: it emits an audio sample every 768
        // cycles and raises its interrupt when the SPU IRQ address is matched.
        self.spu.tick(1, &mut self.irq);

        // Advance the SIO0 controller ACK timer by the same cycle, so a
        // scheduled controller /ACK can raise IRQ7 at this instruction boundary.
        self.sio0.tick(1, &mut self.irq);
        // Keep the shared device clock in step with `cpu.cycles` so save-state
        // flushing (and a later switch to the scheduler) stays consistent.
        self.device_clock = self.cpu.cycles.wrapping_add(1);

        // Deliver a pending, unmasked hardware interrupt at the instruction
        // boundary before fetching the next instruction. With reset state
        // (interrupts disabled) this is a no-op.
        if poll_interrupt(&mut self.cpu, self.irq.pending()) {
            self.cpu.cycles = self.cpu.cycles.wrapping_add(1);
            self.device_clock = self.cpu.cycles;
            return;
        }

        // Cost model: an instruction costs `1 + fetch_wait + data_wait` CPU
        // cycles. `execute::step` already accounts the base 1 (and the four
        // devices were ticked by 1 above); here we compute the *extra* wait
        // states and charge them once, keeping `cpu.cycles` and the device tick
        // totals in lockstep. `fetch_cycles`/`access_cycles` return the whole
        // access cost, so each `wait = cost - 1`.
        //
        // The instruction fetch cost is computed before building the bus (it
        // borrows `memctrl`); a cached (KUSEG/KSEG0) fetch is 1 cycle (i-cache
        // hit model) so `fetch_wait` is 0 in the common case, and a load run
        // from cached code reports its pure data-access cost — exactly the
        // `access-time` golden.
        let fetch_wait = crate::timing::fetch_cycles(self.cpu.pc, &self.memctrl.timing()) - 1;

        let data_cost = {
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
                mdec: &mut self.mdec,
                access_cost: 0,
                obs: self.cpu.cycles,
                device_clock: None,
                io_touched: false,
            };
            step(&mut self.cpu, &mut bus);
            bus.access_cost
        };
        let data_wait = data_cost.saturating_sub(1);

        let extra = fetch_wait + data_wait;
        if extra != 0 {
            // Charge the wait states to every per-cycle device and to the CPU
            // cycle counter so a guest timing loop (Timer2, sysclk source) sees
            // region-aware access latency.
            self.timers.tick(extra, &mut self.irq);
            self.cdrom.tick(extra, &mut self.irq);
            self.spu.tick(extra, &mut self.irq);
            self.sio0.tick(extra, &mut self.irq);
            self.cpu.cycles = self.cpu.cycles.wrapping_add(u64::from(extra));
        }
        self.device_clock = self.cpu.cycles;
    }

    /// Lazy device-scheduler step. Behaviourally identical to
    /// [`Self::step_cpu_naive`], but instead of ticking all four per-cycle
    /// devices every instruction it lets them lag and reconciles them only when
    /// an instruction reaches a cached next-event deadline (or touches a device
    /// register). See the module notes / PR for the equivalence argument.
    fn step_cpu_scheduled(&mut self) {
        let c = self.cpu.cycles;
        // Observation cycle: the naive path top-ticks the devices to `c + 1`
        // before polling and before the instruction executes.
        let obs = c.wrapping_add(1);

        // If a device could set an `I_STAT` bit at or before this observation
        // cycle, catch every device up to `obs` (and recompute the deadline)
        // before we poll, so the bit is delivered at exactly the same boundary
        // the naive loop would deliver it.
        if obs >= self.next_deadline {
            self.catch_up_all(obs);
        }

        // Deliver a pending, unmasked hardware interrupt at the instruction
        // boundary. Below the deadline the device bits of `I_STAT` are frozen
        // but correct, while software-set bits (DMA / VBlank) are live, so this
        // poll matches the naive poll's result.
        if poll_interrupt(&mut self.cpu, self.irq.pending()) {
            self.cpu.cycles = c.wrapping_add(1);
            return;
        }

        // Same cost model as the naive path; fetch cost is read from the PC of
        // the instruction being fetched, before it executes.
        let fetch_wait = crate::timing::fetch_cycles(self.cpu.pc, &self.memctrl.timing()) - 1;

        let (data_cost, io_touched) = {
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
                mdec: &mut self.mdec,
                access_cost: 0,
                obs,
                device_clock: Some(&mut self.device_clock),
                io_touched: false,
            };
            step(&mut self.cpu, &mut bus);
            (bus.access_cost, bus.io_touched)
        };
        let data_wait = data_cost.saturating_sub(1);

        // Only the CPU cycle counter advances by the wait states; the devices
        // stay lagged and are reconciled at the next deadline crossing.
        let extra = fetch_wait + data_wait;
        self.cpu.cycles = self.cpu.cycles.wrapping_add(u64::from(extra));

        // A device register access may have changed a device's next event
        // (started a read, set a timer target, queued a transmit, keyed a
        // voice on, acked an interrupt), so rebuild the deadline.
        if io_touched {
            self.recompute_deadline();
        }
    }

    /// Advances every per-cycle device from `device_clock` up to `target`
    /// (delivering interrupts) and rebuilds the next-event deadline. `target`
    /// must be `>= device_clock`.
    fn catch_up_all(&mut self, target: u64) {
        catch_up_devices(
            &mut self.device_clock,
            target,
            &mut self.timers,
            &mut self.cdrom,
            &mut self.spu,
            &mut self.sio0,
            &mut self.irq,
        );
        self.recompute_deadline();
    }

    /// Recomputes [`Self::next_deadline`] as the earliest absolute CPU cycle at
    /// which any device would next set an `I_STAT` bit (`u64::MAX` if none).
    fn recompute_deadline(&mut self) {
        let base = self.device_clock;
        let mut best = u64::MAX;
        let mut consider = |offset: Option<u64>| {
            if let Some(o) = offset {
                best = best.min(base.saturating_add(o));
            }
        };
        consider(self.timers.cycles_to_next_event());
        consider(self.cdrom.cycles_to_next_event());
        consider(self.spu.cycles_to_next_event());
        consider(self.sio0.cycles_to_next_event());
        self.next_deadline = best;
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
    ///
    /// Takes `&mut self` because it first flushes the lazily-scheduled devices
    /// up to `cpu.cycles` so their serialized state is canonical — byte-for-byte
    /// what the reference per-instruction loop would serialize at this rest
    /// point. At a rest point the flush crosses no un-fired device event (the
    /// next deadline is always beyond `cpu.cycles`), so it sets no `I_STAT` bit
    /// and produces no audio the naive path would not also have. The scheduler
    /// bookkeeping (`device_clock`, `next_deadline`) is deliberately **not**
    /// part of [`CoreSnapshot`], keeping the snapshot format backward
    /// compatible; it is reconstructed on load.
    pub fn save_state(&mut self) -> CoreSnapshot {
        self.catch_up_all(self.cpu.cycles);
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
            mdec: self.mdec.clone(),
            core_version: env!("CARGO_PKG_VERSION").to_string(),
            bios_hash: self.bios_identity(),
            disc_hash: self.disc_identity(),
        }
    }

    /// Cheap stable identity hash of a byte slice (deterministic across runs —
    /// [`std::collections::hash_map::DefaultHasher`] is seeded with fixed keys),
    /// so a hash saved into a snapshot compares equal in a later session.
    fn hash_bytes(bytes: &[u8]) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut h = std::collections::hash_map::DefaultHasher::new();
        bytes.hash(&mut h);
        h.finish()
    }

    /// Identity hash of the loaded BIOS image (`None` when no BIOS is loaded).
    #[must_use]
    pub fn bios_identity(&self) -> Option<u64> {
        if self.mem.bios.len() == BIOS_SIZE {
            Some(Self::hash_bytes(&self.mem.bios))
        } else {
            None
        }
    }

    /// Identity hash of the mounted disc image (`None` when no disc is mounted).
    #[must_use]
    pub fn disc_identity(&self) -> Option<u64> {
        self.cdrom.disc_image().map(Self::hash_bytes)
    }

    /// Restores a snapshot after validating its identity metadata against the
    /// currently loaded BIOS/disc and this build's core version.
    ///
    /// - A **legacy** snapshot (no identity metadata — e.g. a `.ss` file written
    ///   before identity was added) is applied unvalidated and reported as
    ///   [`LoadStateOk::LoadedLegacy`], so old save states keep working.
    /// - A BIOS or disc identity mismatch returns [`LoadStateError::BiosMismatch`]
    ///   / [`LoadStateError::DiscMismatch`] and **does not** apply the snapshot.
    /// - A core-version difference returns [`LoadStateError::VersionMismatch`]
    ///   (also without applying); the format is compatible, so a frontend may
    ///   force it through the unvalidated [`Self::load_state`].
    ///
    /// # Errors
    ///
    /// Returns a [`LoadStateError`] (leaving the machine untouched) when the
    /// snapshot's identity does not match the running core.
    pub fn load_state_checked(
        &mut self,
        snap: &CoreSnapshot,
    ) -> Result<LoadStateOk, LoadStateError> {
        // Legacy snapshots carry no identity at all: empty version and both
        // hashes absent. Accept them without validation for backward compat.
        let legacy =
            snap.core_version.is_empty() && snap.bios_hash.is_none() && snap.disc_hash.is_none();
        if legacy {
            self.load_state(snap);
            return Ok(LoadStateOk::LoadedLegacy);
        }

        let bios_actual = self.bios_identity();
        if snap.bios_hash != bios_actual {
            return Err(LoadStateError::BiosMismatch {
                expected: snap.bios_hash,
                actual: bios_actual,
            });
        }
        let disc_actual = self.disc_identity();
        if snap.disc_hash != disc_actual {
            return Err(LoadStateError::DiscMismatch {
                expected: snap.disc_hash,
                actual: disc_actual,
            });
        }
        let current_version = env!("CARGO_PKG_VERSION");
        if !snap.core_version.is_empty() && snap.core_version != current_version {
            return Err(LoadStateError::VersionMismatch {
                expected: snap.core_version.clone(),
                actual: current_version.to_string(),
            });
        }

        self.load_state(snap);
        Ok(LoadStateOk::Loaded)
    }

    /// Restores a previously captured save-state snapshot **unconditionally**
    /// (no identity validation). Prefer [`Self::load_state_checked`] in a
    /// frontend so a mismatched game/BIOS is caught; this force-apply path
    /// exists for tests and for a frontend that has already decided to override
    /// a [`LoadStateError`].
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
        self.mdec = snap.mdec.clone();
        // Reconstruct the (non-serialized) scheduler bookkeeping: the restored
        // devices are consistent as of `cpu.cycles`, and the deadline is forced
        // to recompute on the next scheduled step (the first `obs >= 0` catch-up
        // ticks the devices by one cycle, matching the naive top-tick).
        self.device_clock = self.cpu.cycles;
        self.next_deadline = 0;
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
                mdec: &mut core.mdec,
                access_cost: 0,
                obs: 0,
                device_clock: None,
                io_touched: false,
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
                mdec: &mut core.mdec,
                access_cost: 0,
                obs: 0,
                device_clock: None,
                io_touched: false,
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
            mdec: &mut core.mdec,
            access_cost: 0,
            obs: 0,
            device_clock: None,
            io_touched: false,
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
            mdec: &mut core.mdec,
            access_cost: 0,
            obs: 0,
            device_clock: None,
            io_touched: false,
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
    fn step_frame_is_noop_while_paused() {
        let mut core = PsxCore::new();
        core.execute(Command::Pause).unwrap();
        let field_before = core.gpu.field;
        let cycles_before = core.cpu.cycles;
        let stat_before = core.irq.read_stat();
        core.execute(Command::StepFrame).unwrap();
        // Paused: no machine progress, no field flip, no VBlank raised.
        assert_eq!(core.cpu.cycles, cycles_before, "cycles must not advance");
        assert_eq!(core.gpu.field, field_before, "field must not flip");
        assert_eq!(core.irq.read_stat(), stat_before, "VBlank must not be set");
    }

    #[test]
    fn frame_step_advances_one_frame_while_paused() {
        let mut core = PsxCore::new();
        core.execute(Command::Pause).unwrap();
        let field_before = core.gpu.field;
        let cycles_before = core.cpu.cycles;
        core.execute(Command::FrameStep).unwrap();
        assert!(core.cpu.cycles > cycles_before, "cycles should advance");
        assert_ne!(core.gpu.field, field_before, "field should flip");
        assert_ne!(core.irq.read_stat() & 0x1, 0, "VBlank should be set");
        // FrameStep leaves the pause state itself untouched.
        assert!(core.is_paused(), "still paused after a frame-step");
    }

    #[test]
    fn step_frame_advances_when_not_paused() {
        let mut core = PsxCore::new();
        let cycles_before = core.cpu.cycles;
        core.execute(Command::StepFrame).unwrap();
        assert!(core.cpu.cycles > cycles_before);
    }

    #[test]
    fn audio_status_query_reports_counters() {
        let mut core = PsxCore::new();
        // Fresh core: nothing produced/dropped/queued, zero cycles.
        let QueryResult::AudioStatus(s0) = core.query(CoreQuery::AudioStatus) else {
            panic!("expected AudioStatus");
        };
        assert_eq!(s0.queued_sample_pairs, 0);
        assert_eq!(s0.samples_produced, 0);
        assert_eq!(s0.samples_dropped, 0);
        assert_eq!(s0.emulated_cycles, 0);
        // Run a frame: the SPU produces samples and the cycle counter advances.
        core.execute(Command::StepFrame).unwrap();
        let QueryResult::AudioStatus(s1) = core.query(CoreQuery::AudioStatus) else {
            panic!("expected AudioStatus");
        };
        assert!(s1.emulated_cycles > 0, "cycles advanced");
        assert!(s1.samples_produced > 0, "SPU produced samples");
        assert!(
            s1.queued_sample_pairs > 0,
            "produced samples are queued until drained"
        );
        // Draining empties the queue but leaves the monotonic counter.
        let _ = core.drain_audio();
        let QueryResult::AudioStatus(s2) = core.query(CoreQuery::AudioStatus) else {
            panic!("expected AudioStatus");
        };
        assert_eq!(s2.queued_sample_pairs, 0, "queue drained");
        assert_eq!(
            s2.samples_produced, s1.samples_produced,
            "produced counter is monotonic across a drain"
        );
    }

    #[test]
    fn load_state_checked_accepts_matching_identity() {
        let mut bios = vec![0u8; BIOS_SIZE];
        bios[0] = 0xAB;
        let mut core = PsxCore::new();
        core.execute(Command::LoadBios(bios.clone())).unwrap();
        let snap = core.save_state();
        assert!(snap.bios_hash.is_some());
        assert_eq!(snap.core_version, env!("CARGO_PKG_VERSION"));

        let mut other = PsxCore::new();
        other.execute(Command::LoadBios(bios)).unwrap();
        assert_eq!(
            other.load_state_checked(&snap),
            Ok(LoadStateOk::Loaded),
            "matching BIOS identity should load"
        );
    }

    #[test]
    fn load_state_checked_rejects_bios_mismatch() {
        let mut bios_a = vec![0u8; BIOS_SIZE];
        bios_a[0] = 0x11;
        let mut core = PsxCore::new();
        core.execute(Command::LoadBios(bios_a)).unwrap();
        let snap = core.save_state();

        let mut other = PsxCore::new();
        let mut bios_b = vec![0u8; BIOS_SIZE];
        bios_b[0] = 0x22;
        other.execute(Command::LoadBios(bios_b)).unwrap();
        let before = other.pc();
        match other.load_state_checked(&snap) {
            Err(LoadStateError::BiosMismatch { expected, actual }) => {
                assert_eq!(expected, snap.bios_hash);
                assert_eq!(actual, other.bios_identity());
            }
            other => panic!("expected BiosMismatch, got {other:?}"),
        }
        assert_eq!(other.pc(), before, "machine left untouched on mismatch");
    }

    #[test]
    fn load_state_checked_accepts_legacy_snapshot() {
        // Simulate an old `.ss` file: identity fields at their serde defaults.
        let mut core = PsxCore::new();
        core.memory_mut().write8(0x40, 0x5A);
        let mut snap = core.save_state();
        snap.core_version = String::new();
        snap.bios_hash = None;
        snap.disc_hash = None;

        let mut other = PsxCore::new();
        assert_eq!(
            other.load_state_checked(&snap),
            Ok(LoadStateOk::LoadedLegacy),
            "legacy snapshot loads without validation"
        );
        assert_eq!(other.memory().read8(0x40), 0x5A);
    }

    #[test]
    fn load_state_checked_reports_version_mismatch() {
        let mut core = PsxCore::new();
        let mut snap = core.save_state();
        // A snapshot from a different core version (but no BIOS/disc, so those
        // match): the version mismatch is surfaced.
        snap.core_version = "9.9.9".to_string();
        let mut other = PsxCore::new();
        match other.load_state_checked(&snap) {
            Err(LoadStateError::VersionMismatch { expected, actual }) => {
                assert_eq!(expected, "9.9.9");
                assert_eq!(actual, env!("CARGO_PKG_VERSION"));
            }
            other => panic!("expected VersionMismatch, got {other:?}"),
        }
    }

    #[test]
    fn legacy_snapshot_json_without_metadata_deserializes() {
        // A CoreSnapshot JSON that predates the identity fields must still
        // deserialize (the new fields are `#[serde(default)]`).
        let mut core = PsxCore::new();
        let snap = core.save_state();
        let mut value: serde_json::Value = serde_json::to_value(&snap).unwrap();
        let obj = value.as_object_mut().unwrap();
        obj.remove("core_version");
        obj.remove("bios_hash");
        obj.remove("disc_hash");
        // Also strip the SPU counter fields to mimic a truly old snapshot.
        if let Some(spu) = obj.get_mut("spu").and_then(|s| s.as_object_mut()) {
            spu.remove("samples_produced");
            spu.remove("samples_dropped");
        }
        let back: CoreSnapshot = serde_json::from_value(value).unwrap();
        assert_eq!(back.core_version, "");
        assert_eq!(back.bios_hash, None);
        assert_eq!(back.disc_hash, None);
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
