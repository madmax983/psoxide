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

/// Sentinel return address staged into `$ra` by [`Harness::load_exe`]. When a
/// side-loaded PS-EXE returns to this address, [`Harness::run_hle`] stops.
pub const HLE_RETURN_ADDR: u32 = 0x0000_0000;

/// A thin wrapper around [`PsxCore`] for writing deterministic CPU tests.
pub struct Harness {
    core: PsxCore,
    /// Captured BIOS TTY output (bytes written via `std_out_putchar`/`puts`).
    tty: Vec<u8>,
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
            tty: Vec::new(),
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

    /// Side-loads a PlayStation PS-EXE image into RAM and stages the CPU to
    /// begin execution at its entry point.
    ///
    /// The PS-EXE format is little-endian with a fixed 0x800-byte header:
    /// magic `b"PS-X EXE"` (@0x00), initial PC (@0x10), initial GP (@0x14),
    /// destination address `t_addr` (@0x18), body length `t_size` (@0x1C),
    /// initial SP base `s_addr` (@0x30) and `s_offset` (@0x34). The body starts
    /// at file offset 0x800.
    ///
    /// Registers are staged through a state snapshot so that `$gp`, `$sp`,
    /// `$fp`, and the sentinel `$ra` are live on the very first instruction.
    ///
    /// # Errors
    ///
    /// Returns `Err` if the image is too small, has a bad magic, or its body
    /// overruns the file.
    pub fn load_exe(&mut self, exe: &[u8]) -> Result<(), String> {
        if exe.len() < 0x800 {
            return Err(format!(
                "PS-EXE too small: {} bytes (need >= 0x800 header)",
                exe.len()
            ));
        }
        if &exe[0..8] != b"PS-X EXE" {
            return Err("bad PS-EXE magic (expected \"PS-X EXE\")".to_string());
        }

        let rd = |off: usize| -> u32 {
            u32::from_le_bytes([exe[off], exe[off + 1], exe[off + 2], exe[off + 3]])
        };
        let pc = rd(0x10);
        let gp = rd(0x14);
        let t_addr = rd(0x18);
        let t_size = rd(0x1C);
        let s_addr = rd(0x30);
        let s_offset = rd(0x34);

        let body_start = 0x800usize;
        let body_end = body_start
            .checked_add(t_size as usize)
            .ok_or_else(|| "PS-EXE t_size overflow".to_string())?;
        if body_end > exe.len() {
            return Err(format!(
                "PS-EXE body overruns file: need {body_end} bytes, have {}",
                exe.len()
            ));
        }
        let body = &exe[body_start..body_end];

        let mem = self.core.memory_mut();
        for (i, &byte) in body.iter().enumerate() {
            mem.write8(t_addr.wrapping_add(i as u32), byte);
        }

        let mut snap = self.core.save_state();
        snap.cpu.pc = pc;
        snap.cpu.next_pc = pc.wrapping_add(4);
        snap.cpu.regs[28] = gp;
        let sp = if s_addr != 0 {
            s_addr.wrapping_add(s_offset)
        } else {
            0x801F_FFF0
        };
        snap.cpu.regs[29] = sp;
        snap.cpu.regs[30] = sp;
        snap.cpu.regs[31] = HLE_RETURN_ADDR;
        self.core.load_state(&snap);

        Ok(())
    }

    /// Runs a side-loaded PS-EXE with high-level emulation (HLE) of the BIOS
    /// TTY calls, capturing character output into the internal buffer.
    ///
    /// Execution stops early (returning the iteration count) when the program
    /// returns to [`HLE_RETURN_ADDR`]; otherwise runs the full `max_steps`.
    pub fn run_hle(&mut self, max_steps: usize) -> usize {
        for i in 0..max_steps {
            let pc = self.core.pc();
            if pc == HLE_RETURN_ADDR {
                return i;
            }
            let vec = pc & 0x1FFF_FFFF;
            if vec == 0xA0 || vec == 0xB0 || vec == 0xC0 {
                self.hle_bios_call(vec);
                continue;
            }
            let _ = self.core.execute(Command::StepCpu);
        }
        max_steps
    }

    /// High-level-emulates a BIOS jump-table call at masked address `vec`
    /// (0xA0/0xB0/0xC0), capturing TTY output, then returns to `$ra`.
    fn hle_bios_call(&mut self, vec: u32) {
        let regs = self.registers().regs;
        let func = regs[9] & 0xFF; // $t1 selects the table entry
        let arg = regs[4]; // $a0
        let ra = regs[31];

        match (vec, func) {
            // std_out_putchar
            (0xA0, 0x3C) | (0xB0, 0x3D) => {
                self.tty.push(arg as u8);
            }
            // std_out_puts (NUL-terminated string at $a0)
            (0xA0, 0x3E) | (0xB0, 0x3F) => {
                let mut addr = arg;
                for _ in 0..4096 {
                    let byte = self.core.memory().read8(addr);
                    if byte == 0 {
                        break;
                    }
                    self.tty.push(byte);
                    addr = addr.wrapping_add(1);
                }
            }
            // Any other BIOS call is a no-op for CPU testing purposes.
            _ => {}
        }

        self.core.set_pc(ra);
    }

    /// Returns the captured TTY output as a lossy UTF-8 string.
    #[must_use]
    pub fn tty(&self) -> String {
        String::from_utf8_lossy(&self.tty).into_owned()
    }

    /// Returns the raw captured TTY bytes.
    #[must_use]
    pub fn tty_bytes(&self) -> &[u8] {
        &self.tty
    }

    /// Clears the captured TTY buffer.
    pub fn clear_tty(&mut self) {
        self.tty.clear();
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
