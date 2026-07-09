//! MIPS R3000A CPU emulation.
//!
//! The R3000A is a 32-bit little-endian RISC processor:
//! - 32 general-purpose registers (`r0` hardwired to zero)
//! - `HI`/`LO` multiply/divide result registers
//! - a coprocessor-0 for system control (SR, CAUSE, EPC, BadVaddr)
//! - explicit branch and load delay slots
//!
//! In the PlayStation it runs at ~33.8688 MHz.
//!
//! The module is split into:
//! - [`decode`] — a pure decoder from a 32-bit word to [`decode::Instruction`].
//! - [`engine`] — the [`Cpu`] register file and delay-slot state.
//! - [`execute`] — the [`Bus`] trait and the interpreter ([`step`]).

pub mod decode;
pub mod engine;
pub mod execute;

pub use decode::{Instruction, decode};
pub use engine::{COP0_BADVADDR, COP0_CAUSE, COP0_EPC, COP0_SR, Cpu, RESET_PC};
pub use execute::{Bus, execute_instruction, poll_interrupt, step};

use crate::gte::Gte;
use serde::{Deserialize, Serialize};

/// Serializable snapshot of the CPU register file and control state.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CpuSnapshot {
    /// General-purpose registers (`regs[0]` is always zero).
    pub regs: [u32; 32],
    /// Program counter.
    pub pc: u32,
    /// Address of the instruction after `pc` (branch delay model).
    pub next_pc: u32,
    /// Multiply/divide HI register.
    pub hi: u32,
    /// Multiply/divide LO register.
    pub lo: u32,
    /// Coprocessor-0 registers (indexed 0..=31).
    pub cop0: [u32; 32],
    /// Coprocessor-2 (GTE) geometry-transformation engine state.
    pub gte: Gte,
    /// Pending load delay: `(register, value)`.
    pub pending_load: (u8, u32),
    /// Retired instruction/cycle count.
    pub cycles: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapshot_serde_round_trip() {
        let cpu = Cpu::new();
        let snap = cpu.snapshot();
        let json = serde_json::to_string(&snap).unwrap();
        let back: CpuSnapshot = serde_json::from_str(&json).unwrap();
        assert_eq!(snap, back);
    }

    #[test]
    fn reset_snapshot_has_reset_pc() {
        let cpu = Cpu::new();
        assert_eq!(cpu.snapshot().pc, RESET_PC);
    }
}
