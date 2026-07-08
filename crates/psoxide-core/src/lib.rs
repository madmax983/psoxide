//! Psoxide core emulation library.
//!
//! Pure Sony PlayStation (PSX) emulation with no I/O dependencies. Frontends
//! drive the emulator via [`Command`] and poll state via [`CoreQuery`].
//!
//! This crate is the CPU + bus foundation of the emulator: the MIPS R3000A
//! interpreter, coprocessor-0 basics, the segmented memory map, a software GPU
//! with VRAM and a command FIFO, DMA (GPU + OTC channels), and the interrupt
//! controller. SPU, CD-ROM, and the GTE are not yet implemented.

pub mod api;
pub mod bus;
pub mod cpu;
pub mod dma;
pub mod gpu;
pub mod irq;

pub use api::{
    BIOS_IMAGE_SIZE, Button, Command, CoreError, CoreQuery, CoreSnapshot, EmulatorState,
    FRAME_HEIGHT, FRAME_RGBA_BYTES, FRAME_WIDTH, Memory, PsxCore, QueryResult,
};
pub use bus::{BusRegion, map_region, mask_region};
pub use cpu::{Cpu, CpuSnapshot, Instruction, decode};
pub use dma::Dma;
pub use gpu::Gpu;
pub use irq::{Irq, IrqLine};
