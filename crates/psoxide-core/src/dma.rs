//! DMA controller (7 channels).
//!
//! The PlayStation has seven DMA channels that move blocks of data between main
//! RAM and devices without CPU involvement. This module implements the register
//! file for all seven channels plus synchronous execution of the two channels
//! the boot path exercises:
//!
//! * **Channel 2 (GPU)** — block and linked-list (ordering-table) transfers to
//!   GP0, and block transfers from GPUREAD.
//! * **Channel 6 (OTC)** — the ordering-table clear that seeds an empty linked
//!   list in RAM for the GPU DMA to walk.
//!
//! The remaining channels (MDEC in/out, CD-ROM, SPU, PIO) are register-only:
//! their MADR/BCR/CHCR read and write back but no transfer is performed.
//!
//! Transfers run synchronously on the CHCR "start" trigger; on completion the
//! busy/trigger bits are cleared and, if enabled, a DMA interrupt is raised.

use serde::{Deserialize, Serialize};

use crate::api::Memory;
use crate::bus::MAIN_RAM_MASK;
use crate::cdrom::Cdrom;
use crate::gpu::Gpu;
use crate::irq::{Irq, IrqLine};

/// Number of DMA channels.
pub const CHANNELS: usize = 7;

/// GPU DMA channel index.
pub const CH_GPU: usize = 2;
/// CD-ROM DMA channel index.
pub const CH_CDROM: usize = 3;
/// OTC (ordering-table clear) DMA channel index.
pub const CH_OTC: usize = 6;

/// DMA controller state.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Dma {
    /// Per-channel base address registers (MADR).
    pub madr: [u32; CHANNELS],
    /// Per-channel block-control registers (BCR).
    pub bcr: [u32; CHANNELS],
    /// Per-channel channel-control registers (CHCR).
    pub chcr: [u32; CHANNELS],
    /// DMA primary control register (DPCR, 0x1F80_10F0).
    pub dpcr: u32,
    /// DMA interrupt register (DICR, 0x1F80_10F4).
    pub dicr: u32,
}

impl Default for Dma {
    fn default() -> Self {
        Self::new()
    }
}

impl Dma {
    /// Creates a controller with power-on register defaults.
    #[must_use]
    pub fn new() -> Self {
        Self {
            madr: [0; CHANNELS],
            bcr: [0; CHANNELS],
            chcr: [0; CHANNELS],
            dpcr: 0x0765_4321, // reset value per nocash
            dicr: 0,
        }
    }

    /// Reads a 32-bit DMA register at physical `addr`.
    #[must_use]
    pub fn read32(&self, addr: u32) -> u32 {
        match addr {
            0x1F80_10F0 => self.dpcr,
            0x1F80_10F4 => self.dicr,
            _ if (0x1F80_1080..=0x1F80_10EF).contains(&addr) => {
                let ch = ((addr - 0x1F80_1080) / 0x10) as usize;
                match (addr - 0x1F80_1080) % 0x10 {
                    0x0 => self.madr[ch],
                    0x4 => self.bcr[ch],
                    0x8 => self.chcr[ch],
                    _ => 0,
                }
            }
            _ => 0,
        }
    }

    /// Writes a 32-bit DMA register. Writing a CHCR whose start/trigger bits are
    /// set executes the channel's transfer synchronously.
    pub fn write32(
        &mut self,
        addr: u32,
        val: u32,
        mem: &mut Memory,
        gpu: &mut Gpu,
        cdrom: &mut Cdrom,
        irq: &mut Irq,
    ) {
        match addr {
            0x1F80_10F0 => self.dpcr = val,
            0x1F80_10F4 => {
                // DICR: bits 24-30 are per-channel IRQ flags (write-1-to-clear);
                // bits 0-23 are read/write control. Bit 31 is recomputed.
                let ack = val & 0x7F00_0000;
                let control = val & 0x00FF_FFFF;
                let flags = self.dicr & 0x7F00_0000 & !ack;
                self.dicr = control | flags;
                self.update_dicr_master();
            }
            _ if (0x1F80_1080..=0x1F80_10EF).contains(&addr) => {
                let ch = ((addr - 0x1F80_1080) / 0x10) as usize;
                match (addr - 0x1F80_1080) % 0x10 {
                    0x0 => self.madr[ch] = val,
                    0x4 => self.bcr[ch] = val,
                    0x8 => {
                        self.chcr[ch] = val;
                        if Self::is_triggered(val) {
                            self.run_channel(ch, mem, gpu, cdrom, irq);
                        }
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }

    /// A channel starts when the enable/busy bit (24) is set. For manual-sync
    /// channels the trigger bit (28) must also be set.
    fn is_triggered(chcr: u32) -> bool {
        let enable = chcr & (1 << 24) != 0;
        let sync = (chcr >> 9) & 0x3;
        if sync == 0 {
            enable && (chcr & (1 << 28) != 0)
        } else {
            enable
        }
    }

    fn run_channel(
        &mut self,
        ch: usize,
        mem: &mut Memory,
        gpu: &mut Gpu,
        cdrom: &mut Cdrom,
        irq: &mut Irq,
    ) {
        match ch {
            CH_GPU => self.run_gpu(ch, mem, gpu),
            CH_CDROM => self.run_cdrom(ch, mem, cdrom),
            CH_OTC => self.run_otc(ch, mem),
            _ => {}
        }
        // Clear the busy (24) and trigger (28) bits to signal completion.
        self.chcr[ch] &= !((1 << 24) | (1 << 28));
        self.raise_completion(ch, irq);
    }

    /// CD-ROM DMA (channel 3): a device→RAM block copy that pulls sector words
    /// from the CD-ROM data FIFO. Word count is `size * blocks` from BCR.
    fn run_cdrom(&mut self, ch: usize, mem: &mut Memory, cdrom: &mut Cdrom) {
        let bcr = self.bcr[ch];
        let size = bcr & 0xFFFF;
        let blocks = (bcr >> 16) & 0xFFFF;
        let words = size.max(1) * blocks.max(1);
        let step: i64 = if self.chcr[ch] & 0x2 != 0 { -4 } else { 4 };
        let mut addr = self.madr[ch] & 0x1F_FFFC;
        for _ in 0..words {
            let word = cdrom.read_data_word();
            write_ram(mem, addr, word);
            addr = (addr as i64 + step) as u32 & 0x1F_FFFC;
        }
        self.madr[ch] = addr;
    }

    /// Raises a DMA interrupt for `ch` if its DICR enable and master-enable bits
    /// are set, latching the per-channel flag.
    fn raise_completion(&mut self, ch: usize, irq: &mut Irq) {
        let channel_enable = self.dicr & (1 << (16 + ch)) != 0;
        let master_enable = self.dicr & (1 << 23) != 0;
        if channel_enable && master_enable {
            self.dicr |= 1 << (24 + ch); // set the channel flag
            self.update_dicr_master();
            irq.set(IrqLine::Dma);
        }
    }

    /// Recomputes the DICR master interrupt flag (bit 31): the force bit, or the
    /// master enable ANDed with any enabled-and-flagged channel.
    fn update_dicr_master(&mut self) {
        let force = self.dicr & (1 << 15) != 0;
        let master_enable = self.dicr & (1 << 23) != 0;
        let enables = (self.dicr >> 16) & 0x7F;
        let flags = (self.dicr >> 24) & 0x7F;
        let active = master_enable && (enables & flags) != 0;
        if force || active {
            self.dicr |= 1 << 31;
        } else {
            self.dicr &= !(1 << 31);
        }
    }

    fn run_gpu(&mut self, ch: usize, mem: &mut Memory, gpu: &mut Gpu) {
        let chcr = self.chcr[ch];
        let sync = (chcr >> 9) & 0x3;
        match sync {
            2 => self.run_gpu_linked_list(ch, mem, gpu),
            1 => self.run_gpu_block(ch, mem, gpu),
            _ => {
                // Immediate: word count = BCR low 16 bits.
                let words = (self.bcr[ch] & 0xFFFF).max(1);
                self.run_gpu_words(ch, mem, gpu, words);
            }
        }
    }

    fn run_gpu_block(&mut self, ch: usize, mem: &mut Memory, gpu: &mut Gpu) {
        let bcr = self.bcr[ch];
        let size = bcr & 0xFFFF;
        let blocks = (bcr >> 16) & 0xFFFF;
        let total = size.max(1) * blocks.max(1);
        self.run_gpu_words(ch, mem, gpu, total);
    }

    fn run_gpu_words(&mut self, ch: usize, mem: &mut Memory, gpu: &mut Gpu, words: u32) {
        let chcr = self.chcr[ch];
        let to_device = chcr & 0x1 != 0;
        let step: i64 = if chcr & 0x2 != 0 { -4 } else { 4 };
        let mut addr = self.madr[ch] & 0x1F_FFFC;
        for _ in 0..words {
            if to_device {
                let word = read_ram(mem, addr);
                gpu.gp0(word);
            } else {
                let word = gpu.gpuread();
                write_ram(mem, addr, word);
            }
            addr = (addr as i64 + step) as u32 & 0x1F_FFFC;
        }
        self.madr[ch] = addr;
    }

    fn run_gpu_linked_list(&mut self, ch: usize, mem: &mut Memory, gpu: &mut Gpu) {
        let mut addr = self.madr[ch] & 0x1F_FFFC;
        let mut guard = 0u32;
        loop {
            let header = read_ram(mem, addr);
            let count = header >> 24;
            for i in 1..=count {
                let word = read_ram(mem, addr.wrapping_add(4 * i) & 0x1F_FFFC);
                gpu.gp0(word);
            }
            let next = header & 0xFF_FFFF;
            if next & 0x80_0000 != 0 || next == 0xFF_FFFF {
                break;
            }
            addr = next & 0x1F_FFFC;
            guard += 1;
            if guard >= 0x10000 {
                break; // malformed list safety cap
            }
        }
        self.madr[ch] = 0x00FF_FFFF;
    }

    fn run_otc(&mut self, ch: usize, mem: &mut Memory) {
        // Ordering-table clear: build a descending linked list ending in the
        // 0xFF_FFFF end marker.
        let count = {
            let c = self.bcr[ch] & 0xFFFF;
            if c == 0 { 0x1_0000 } else { c }
        };
        let mut addr = self.madr[ch] & 0x1F_FFFC;
        for _ in 0..count.saturating_sub(1) {
            let prev = (addr.wrapping_sub(4)) & 0xFF_FFFF;
            write_ram(mem, addr, prev);
            addr = addr.wrapping_sub(4) & 0x1F_FFFC;
        }
        write_ram(mem, addr, 0x00FF_FFFF);
    }
}

/// Reads a word from main RAM (word-aligned, folded into the 2MB region).
fn read_ram(mem: &Memory, addr: u32) -> u32 {
    let base = (addr & MAIN_RAM_MASK & !0x3) as usize;
    u32::from_le_bytes([
        mem.ram[base],
        mem.ram[base + 1],
        mem.ram[base + 2],
        mem.ram[base + 3],
    ])
}

/// Writes a word to main RAM (word-aligned, folded into the 2MB region).
fn write_ram(mem: &mut Memory, addr: u32, val: u32) {
    let base = (addr & MAIN_RAM_MASK & !0x3) as usize;
    let b = val.to_le_bytes();
    mem.ram[base..base + 4].copy_from_slice(&b);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gpu::rgb_to_bgr555;

    fn setup() -> (Dma, Memory, Gpu, Cdrom, Irq) {
        (
            Dma::new(),
            Memory::new(),
            Gpu::new(),
            Cdrom::new(),
            Irq::new(),
        )
    }

    #[test]
    fn otc_builds_descending_list() {
        let (mut dma, mut mem, mut gpu, mut cdrom, mut irq) = setup();
        // OTC over 4 entries starting at 0x100.
        dma.madr[CH_OTC] = 0x100;
        dma.bcr[CH_OTC] = 4;
        // sync=0 manual: enable + trigger.
        dma.write32(
            0x1F80_1080 + (CH_OTC as u32) * 0x10 + 0x8,
            (1 << 24) | (1 << 28),
            &mut mem,
            &mut gpu,
            &mut cdrom,
            &mut irq,
        );
        assert_eq!(read_ram(&mem, 0x100), 0x0FC & 0xFF_FFFF); // → 0x0FC
        assert_eq!(read_ram(&mem, 0x0FC), 0x0F8);
        assert_eq!(read_ram(&mem, 0x0F8), 0x0F4);
        assert_eq!(read_ram(&mem, 0x0F4), 0x00FF_FFFF); // end marker
        // Busy bit cleared on completion.
        assert_eq!(dma.chcr[CH_OTC] & (1 << 24), 0);
    }

    #[test]
    fn linked_list_dma_drives_gp0_fill() {
        let (mut dma, mut mem, mut gpu, mut cdrom, mut irq) = setup();
        // Build a one-node list at 0x200: header (count=3) + a fill command.
        // Fill red 16x16 at (0,0).
        write_ram(&mut mem, 0x200, (3 << 24) | 0x00FF_FFFF); // count 3, next=end
        write_ram(&mut mem, 0x204, 0x0200_00FF); // fill red
        write_ram(&mut mem, 0x208, 0x0000_0000); // (0,0)
        write_ram(&mut mem, 0x20C, 0x0010_0010); // 16x16
        dma.madr[CH_GPU] = 0x200;
        // CHCR: enable, direction RAM->device (bit0), sync mode 2 (linked list).
        let chcr = (1 << 24) | 0x1 | (2 << 9);
        dma.write32(
            0x1F80_1080 + (CH_GPU as u32) * 0x10 + 0x8,
            chcr,
            &mut mem,
            &mut gpu,
            &mut cdrom,
            &mut irq,
        );
        assert_eq!(gpu.vram_at(0, 0), rgb_to_bgr555(0xFF, 0, 0));
        assert_eq!(dma.chcr[CH_GPU] & (1 << 24), 0);
    }

    #[test]
    fn block_dma_cpu_to_vram_loads_pixels() {
        let (mut dma, mut mem, mut gpu, mut cdrom, mut irq) = setup();
        // Prepare a CPU->VRAM image load header + pixel data in RAM.
        write_ram(&mut mem, 0x300, 0xA000_0000); // CPU->VRAM
        write_ram(&mut mem, 0x304, 0x0000_0000); // dst (0,0)
        write_ram(&mut mem, 0x308, 0x0001_0002); // 2x1
        write_ram(&mut mem, 0x30C, 0xBBBB_AAAA); // pixels AAAA,BBBB
        dma.madr[CH_GPU] = 0x300;
        // Block DMA: 4 words, one block. sync mode 1.
        dma.bcr[CH_GPU] = 4; // size=4 blocks=0->1
        let chcr = (1 << 24) | 0x1 | (1 << 9);
        dma.write32(
            0x1F80_1080 + (CH_GPU as u32) * 0x10 + 0x8,
            chcr,
            &mut mem,
            &mut gpu,
            &mut cdrom,
            &mut irq,
        );
        assert_eq!(gpu.vram_at(0, 0), 0xAAAA);
        assert_eq!(gpu.vram_at(1, 0), 0xBBBB);
    }

    #[test]
    fn dma_completion_raises_irq_when_enabled() {
        let (mut dma, mut mem, mut gpu, mut cdrom, mut irq) = setup();
        // Enable DMA IRQ for the OTC channel + master enable.
        dma.dicr = (1 << 23) | (1 << (16 + CH_OTC));
        dma.madr[CH_OTC] = 0x100;
        dma.bcr[CH_OTC] = 2;
        dma.write32(
            0x1F80_1080 + (CH_OTC as u32) * 0x10 + 0x8,
            (1 << 24) | (1 << 28),
            &mut mem,
            &mut gpu,
            &mut cdrom,
            &mut irq,
        );
        assert!(irq.read_stat() & (1 << IrqLine::Dma.bit()) != 0);
        assert_ne!(dma.dicr & (1 << (24 + CH_OTC)), 0);
    }

    #[test]
    fn register_read_write_roundtrip() {
        let (mut dma, mut mem, mut gpu, mut cdrom, mut irq) = setup();
        dma.write32(
            0x1F80_1080,
            0x1234,
            &mut mem,
            &mut gpu,
            &mut cdrom,
            &mut irq,
        ); // ch0 MADR
        assert_eq!(dma.read32(0x1F80_1080), 0x1234);
        dma.write32(
            0x1F80_10F0,
            0xDEAD_BEEF,
            &mut mem,
            &mut gpu,
            &mut cdrom,
            &mut irq,
        );
        assert_eq!(dma.read32(0x1F80_10F0), 0xDEAD_BEEF);
    }

    #[test]
    fn cdrom_dma_copies_data_fifo_to_ram() {
        let (mut dma, mut mem, mut gpu, mut cdrom, mut irq) = setup();
        // Stage 8 bytes in the CD-ROM sector buffer and load the data FIFO.
        cdrom.set_sector_buffer_for_test((0..8).collect());
        cdrom.write8(0x1F80_1803, 0x80); // BFRD: load data FIFO
        dma.madr[CH_CDROM] = 0x400;
        // Block DMA: 2 words, one block. sync mode 1, device→RAM.
        dma.bcr[CH_CDROM] = 2;
        let chcr = (1 << 24) | (1 << 9);
        dma.write32(
            0x1F80_1080 + (CH_CDROM as u32) * 0x10 + 0x8,
            chcr,
            &mut mem,
            &mut gpu,
            &mut cdrom,
            &mut irq,
        );
        assert_eq!(read_ram(&mem, 0x400), 0x0302_0100);
        assert_eq!(read_ram(&mem, 0x404), 0x0706_0504);
        assert_eq!(dma.chcr[CH_CDROM] & (1 << 24), 0);
    }
}
