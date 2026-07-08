//! MIPS R3000A register file and CPU state.
//!
//! The [`Cpu`] owns the general-purpose register file, the program counter
//! pair used to model the branch delay slot, the HI/LO multiply registers, the
//! coprocessor-0 system-control registers, and the pending load used to model
//! the load delay slot.
//!
//! ## Delay slots
//!
//! * **Branch delay slot** — modelled with [`Cpu::pc`] and [`Cpu::next_pc`].
//!   Each step fetches at `pc`, advances `pc = next_pc` and `next_pc += 4`; a
//!   branch overwrites `next_pc`, so the instruction already fetched into the
//!   delay slot still executes.
//! * **Load delay slot** — modelled with a two-bank register file
//!   ([`Cpu::regs`] / [`Cpu::out_regs`]) plus a [`Cpu::pending_load`]. A load's
//!   result lands in `out_regs` only at the *start* of the following
//!   instruction, so that instruction still reads the register's old value.

use super::CpuSnapshot;

/// Coprocessor-0 register index for the bad virtual address (`BadVaddr`).
pub const COP0_BADVADDR: usize = 8;
/// Coprocessor-0 register index for the status register (`SR`).
pub const COP0_SR: usize = 12;
/// Coprocessor-0 register index for the cause register (`CAUSE`).
pub const COP0_CAUSE: usize = 13;
/// Coprocessor-0 register index for the exception program counter (`EPC`).
pub const COP0_EPC: usize = 14;

/// `SR` bit 22: bootstrap exception vectors. When set, exceptions vector to
/// `0xBFC0_0180` (in BIOS); when clear, to `0x8000_0080` (in RAM).
pub const SR_BEV: u32 = 1 << 22;

/// The CPU reset vector — the KSEG1 (uncached) BIOS entry point.
pub const RESET_PC: u32 = 0xBFC0_0000;

/// The MIPS R3000A register file and control state.
#[derive(Debug, Clone)]
pub struct Cpu {
    /// General-purpose registers as seen by the *current* instruction's reads.
    /// `regs[0]` is hardwired to zero.
    pub regs: [u32; 32],
    /// Output register bank; writes land here and are copied into [`Cpu::regs`]
    /// after the instruction completes (load-delay model).
    pub out_regs: [u32; 32],
    /// Program counter of the instruction being fetched next.
    pub pc: u32,
    /// Address of the instruction after `pc` (branch delay model).
    pub next_pc: u32,
    /// Address of the instruction currently executing (used for `EPC`).
    pub current_pc: u32,
    /// Multiply/divide HI result register.
    pub hi: u32,
    /// Multiply/divide LO result register.
    pub lo: u32,
    /// Coprocessor-0 system-control registers (indexed 0..=31 by `rd`).
    pub cop0: [u32; 32],
    /// Pending load: `(register, value)` committed before the next
    /// instruction's operands are read. Register 0 means "no load".
    pub pending_load: (u8, u32),
    /// Set when the current instruction is a taken branch (for the delay slot).
    pub branch: bool,
    /// Set when the current instruction sits in a branch delay slot.
    pub delay_slot: bool,
    /// Total instructions/cycles retired.
    pub cycles: u64,
}

impl Cpu {
    /// Creates a CPU in its post-reset state (PC at the BIOS reset vector).
    #[must_use]
    pub fn new() -> Self {
        let mut cpu = Self {
            regs: [0; 32],
            out_regs: [0; 32],
            pc: 0,
            next_pc: 0,
            current_pc: 0,
            hi: 0,
            lo: 0,
            cop0: [0; 32],
            pending_load: (0, 0),
            branch: false,
            delay_slot: false,
            cycles: 0,
        };
        cpu.reset();
        cpu
    }

    /// Resets the CPU to its power-on state.
    pub fn reset(&mut self) {
        self.regs = [0; 32];
        self.out_regs = [0; 32];
        self.pc = RESET_PC;
        self.next_pc = RESET_PC.wrapping_add(4);
        self.current_pc = RESET_PC;
        self.hi = 0;
        self.lo = 0;
        self.cop0 = [0; 32];
        // Boot with BEV set, matching R3000A reset behaviour.
        self.cop0[COP0_SR] = SR_BEV;
        self.pending_load = (0, 0);
        self.branch = false;
        self.delay_slot = false;
        self.cycles = 0;
    }

    /// Reads general-purpose register `index` as visible to the current
    /// instruction. Register 0 always reads as zero.
    #[must_use]
    #[inline]
    pub fn reg(&self, index: u8) -> u32 {
        self.regs[index as usize]
    }

    /// Writes general-purpose register `index` into the output bank. Writes to
    /// register 0 are discarded (it stays hardwired to zero).
    #[inline]
    pub fn set_reg(&mut self, index: u8, value: u32) {
        self.out_regs[index as usize] = value;
        self.out_regs[0] = 0;
    }

    /// Reads a coprocessor-0 register.
    #[must_use]
    #[inline]
    pub fn cop0(&self, index: usize) -> u32 {
        self.cop0[index]
    }

    /// Returns the current status register (`SR`).
    #[must_use]
    #[inline]
    pub fn sr(&self) -> u32 {
        self.cop0[COP0_SR]
    }

    /// Captures a serializable snapshot of the CPU state.
    #[must_use]
    pub fn snapshot(&self) -> CpuSnapshot {
        CpuSnapshot {
            regs: self.regs,
            pc: self.pc,
            next_pc: self.next_pc,
            hi: self.hi,
            lo: self.lo,
            cop0: self.cop0,
            pending_load: self.pending_load,
            cycles: self.cycles,
        }
    }

    /// Restores the CPU state from a snapshot.
    pub fn restore(&mut self, snap: &CpuSnapshot) {
        self.regs = snap.regs;
        self.out_regs = snap.regs;
        self.pc = snap.pc;
        self.next_pc = snap.next_pc;
        self.current_pc = snap.pc;
        self.hi = snap.hi;
        self.lo = snap.lo;
        self.cop0 = snap.cop0;
        self.pending_load = snap.pending_load;
        self.branch = false;
        self.delay_slot = false;
        self.cycles = snap.cycles;
    }
}

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_state() {
        let cpu = Cpu::new();
        assert_eq!(cpu.pc, RESET_PC);
        assert_eq!(cpu.next_pc, RESET_PC + 4);
        assert_eq!(cpu.reg(0), 0);
        assert_eq!(cpu.sr() & SR_BEV, SR_BEV);
        assert_eq!(cpu.cycles, 0);
    }

    #[test]
    fn register_zero_is_hardwired() {
        let mut cpu = Cpu::new();
        cpu.set_reg(0, 0xDEAD_BEEF);
        assert_eq!(cpu.out_regs[0], 0);
    }

    #[test]
    fn set_reg_writes_to_output_bank() {
        let mut cpu = Cpu::new();
        cpu.set_reg(5, 0x1234);
        // Input bank unchanged until committed by the step loop.
        assert_eq!(cpu.reg(5), 0);
        assert_eq!(cpu.out_regs[5], 0x1234);
    }

    #[test]
    fn snapshot_roundtrip() {
        let mut cpu = Cpu::new();
        cpu.regs[3] = 0xAAAA;
        cpu.hi = 0x1111;
        cpu.lo = 0x2222;
        cpu.cop0[COP0_SR] = 0x0F;
        cpu.pending_load = (7, 0x99);
        cpu.cycles = 128;

        let snap = cpu.snapshot();
        let mut restored = Cpu::new();
        restored.restore(&snap);

        assert_eq!(restored.regs[3], 0xAAAA);
        assert_eq!(restored.hi, 0x1111);
        assert_eq!(restored.lo, 0x2222);
        assert_eq!(restored.cop0[COP0_SR], 0x0F);
        assert_eq!(restored.pending_load, (7, 0x99));
        assert_eq!(restored.cycles, 128);
    }
}
