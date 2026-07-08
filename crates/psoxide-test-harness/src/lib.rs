//! ROM/program-based integration test scaffolding for psoxide.
//!
//! Provides helpers to build a [`PsxCore`], stage a hand-assembled MIPS
//! program into main RAM, run it for a fixed number of instructions, and
//! inspect the resulting register state.
//!
//! Real CPU test ROMs (Amidog `psxtest_cpu`, JaCzekanski `ps1-tests`,
//! PeterLemon PSX demos) would live under `tests/roms/` and be driven through
//! [`Harness::load_bios`]. See `README.md`.

use psoxide_core::{Command, CoreQuery, CpuSnapshot, PsxCore, QueryResult};

/// Base address in KUSEG main RAM where staged test programs are placed.
pub const PROGRAM_BASE: u32 = 0x0000_0000;

/// A thin wrapper around [`PsxCore`] for writing deterministic CPU tests.
pub struct Harness {
    core: PsxCore,
}

impl Default for Harness {
    fn default() -> Self {
        Self::new()
    }
}

impl Harness {
    /// Creates a fresh harness with an empty machine.
    #[must_use]
    pub fn new() -> Self {
        Self {
            core: PsxCore::new(),
        }
    }

    /// Stages a sequence of 32-bit little-endian instruction words into main
    /// RAM starting at [`PROGRAM_BASE`], and points the CPU at it.
    pub fn load_program(&mut self, words: &[u32]) {
        let mem = self.core.memory_mut();
        for (i, &word) in words.iter().enumerate() {
            let addr = PROGRAM_BASE + (i as u32) * 4;
            let bytes = word.to_le_bytes();
            for (b, byte) in bytes.iter().enumerate() {
                mem.write8(addr + b as u32, *byte);
            }
        }
        self.core.set_pc(PROGRAM_BASE);
    }

    /// Loads a 512KB BIOS image (for booting real test ROMs later).
    ///
    /// # Errors
    ///
    /// Propagates [`psoxide_core::CoreError`] on a wrong-sized image.
    pub fn load_bios(&mut self, image: Vec<u8>) -> Result<(), psoxide_core::CoreError> {
        self.core.execute(Command::LoadBios(image))
    }

    /// Runs `n` CPU instructions.
    pub fn run(&mut self, n: usize) {
        for _ in 0..n {
            let _ = self.core.execute(Command::StepCpu);
        }
    }

    /// Returns the current CPU register snapshot.
    #[must_use]
    pub fn registers(&self) -> CpuSnapshot {
        match self.core.query(CoreQuery::Registers) {
            QueryResult::Registers(snap) => *snap,
            _ => unreachable!("Registers query returns Registers"),
        }
    }

    /// Returns general-purpose register `index`.
    #[must_use]
    pub fn reg(&self, index: usize) -> u32 {
        self.registers().regs[index]
    }

    /// Returns a mutable reference to the underlying core.
    pub fn core_mut(&mut self) -> &mut PsxCore {
        &mut self.core
    }

    /// Reads a word of main RAM (little-endian).
    #[must_use]
    pub fn read_word(&self, addr: u32) -> u32 {
        match self.core.query(CoreQuery::Memory { addr, len: 4 }) {
            QueryResult::Memory(bytes) => {
                u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
            }
            _ => unreachable!("Memory query returns Memory"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn harness_runs_simple_program() {
        // addiu $t0,$zero,2 ; addiu $t1,$zero,3 ; add $t2,$t0,$t1
        let mut h = Harness::new();
        h.load_program(&[
            (0x09 << 26) | (8 << 16) | 2,
            (0x09 << 26) | (9 << 16) | 3,
            (8 << 21) | (9 << 16) | (10 << 11) | 0x20,
        ]);
        h.run(3);
        assert_eq!(h.reg(10), 5);
    }
}
