//! Psoxide core emulation library.
//!
//! Pure Sony PlayStation (PSX) emulation with no I/O dependencies. Frontends
//! drive the emulator via [`Command`] and poll state via [`CoreQuery`].
//!
//! This crate is the CPU + bus foundation of the emulator: the MIPS R3000A
//! interpreter, coprocessor-0 basics, the segmented memory map, a software GPU
//! with VRAM and a command FIFO, DMA (GPU + OTC + CD-ROM channels), the
//! interrupt controller, the GTE (coprocessor 2) geometry engine, and the
//! CD-ROM controller. The SPU (audio) is not yet implemented.

pub mod api;
pub mod bus;
pub mod cdrom;
pub mod cpu;
pub mod dma;
pub mod gpu;
pub mod gte;
pub mod iostubs;
pub mod irq;
pub mod timers;

pub use api::{
    BIOS_IMAGE_SIZE, Button, Command, CoreError, CoreQuery, CoreSnapshot, EmulatorState,
    FRAME_HEIGHT, FRAME_RGBA_BYTES, FRAME_WIDTH, Memory, PsxCore, QueryResult,
};
pub use bus::{BusRegion, map_region, mask_region};
pub use cdrom::{Cdrom, Disc, DiscTrack};
pub use cpu::{
    COP0_BADVADDR, COP0_CAUSE, COP0_EPC, COP0_SR, Cpu, CpuSnapshot, Instruction, decode,
};
pub use dma::Dma;
pub use gpu::Gpu;
pub use gte::Gte;
pub use irq::{Irq, IrqLine};
pub use timers::Timers;
