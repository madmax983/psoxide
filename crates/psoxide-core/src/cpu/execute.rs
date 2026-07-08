//! MIPS R3000A interpreter.
//!
//! Provides the [`Bus`] abstraction the CPU uses for memory access and the
//! [`step`] / [`execute_instruction`] functions that implement every decoded
//! [`Instruction`]. The PlayStation is **little-endian**.

use super::decode::{Instruction, decode};
use super::engine::{COP0_BADVADDR, COP0_CAUSE, COP0_EPC, COP0_SR, Cpu, SR_BEV};

/// Memory interface used by the CPU. Loads return the value at `addr`; stores
/// take `&mut self`. All accesses are little-endian.
pub trait Bus {
    /// Loads a byte.
    fn load8(&mut self, addr: u32) -> u8;
    /// Loads a little-endian halfword.
    fn load16(&mut self, addr: u32) -> u16;
    /// Loads a little-endian word.
    fn load32(&mut self, addr: u32) -> u32;
    /// Stores a byte.
    fn store8(&mut self, addr: u32, value: u8);
    /// Stores a little-endian halfword.
    fn store16(&mut self, addr: u32, value: u16);
    /// Stores a little-endian word.
    fn store32(&mut self, addr: u32, value: u32);
}

// ── Exception codes (CAUSE ExcCode field) ───────────────────────────────

/// Address error on load / instruction fetch.
pub const EXC_ADEL: u32 = 0x04;
/// Address error on store.
pub const EXC_ADES: u32 = 0x05;
/// System call.
pub const EXC_SYSCALL: u32 = 0x08;
/// Breakpoint.
pub const EXC_BREAK: u32 = 0x09;
/// Reserved (illegal) instruction.
pub const EXC_RI: u32 = 0x0A;
/// Arithmetic overflow.
pub const EXC_OVERFLOW: u32 = 0x0C;

/// `SR` bit 16: isolate cache. When set, stores are absorbed by the cache and
/// do not reach main memory (the BIOS uses this while flushing the cache).
const SR_ISOLATE_CACHE: u32 = 1 << 16;

#[inline]
fn sign_extend(imm: u16) -> u32 {
    imm as i16 as i32 as u32
}

/// Fetches, decodes, and executes one instruction, advancing the branch and
/// load delay slots.
pub fn step<B: Bus>(cpu: &mut Cpu, bus: &mut B) {
    cpu.current_pc = cpu.pc;

    // Instruction fetch must be word-aligned.
    if cpu.current_pc & 0x3 != 0 {
        cpu.cop0[COP0_BADVADDR] = cpu.current_pc;
        enter_exception(cpu, EXC_ADEL);
        cpu.cycles = cpu.cycles.wrapping_add(1);
        return;
    }

    let raw = bus.load32(cpu.pc);

    // Advance the program counter pair; the delay-slot flag tracks whether the
    // instruction we are about to run sits after a taken branch.
    cpu.delay_slot = cpu.branch;
    cpu.branch = false;
    cpu.pc = cpu.next_pc;
    cpu.next_pc = cpu.next_pc.wrapping_add(4);

    // Commit the pending load into the output bank *before* this instruction
    // reads its operands (load delay slot).
    let (reg, value) = cpu.pending_load;
    cpu.set_reg(reg, value);
    cpu.pending_load = (0, 0);

    execute_instruction(cpu, bus, decode(raw));

    // Publish the output bank as the new architectural register file.
    cpu.regs = cpu.out_regs;
    cpu.cycles = cpu.cycles.wrapping_add(1);
}

/// Executes a single already-decoded [`Instruction`]. Callers normally use
/// [`step`], which also handles fetch and delay-slot bookkeeping.
pub fn execute_instruction<B: Bus>(cpu: &mut Cpu, bus: &mut B, insn: Instruction) {
    match insn {
        // ── Shifts ──────────────────────────────────────────────────────
        Instruction::Sll { rd, rt, shamt } => cpu.set_reg(rd, cpu.reg(rt) << shamt),
        Instruction::Srl { rd, rt, shamt } => cpu.set_reg(rd, cpu.reg(rt) >> shamt),
        Instruction::Sra { rd, rt, shamt } => {
            cpu.set_reg(rd, ((cpu.reg(rt) as i32) >> shamt) as u32);
        }
        Instruction::Sllv { rd, rt, rs } => cpu.set_reg(rd, cpu.reg(rt) << (cpu.reg(rs) & 0x1F)),
        Instruction::Srlv { rd, rt, rs } => cpu.set_reg(rd, cpu.reg(rt) >> (cpu.reg(rs) & 0x1F)),
        Instruction::Srav { rd, rt, rs } => {
            cpu.set_reg(rd, ((cpu.reg(rt) as i32) >> (cpu.reg(rs) & 0x1F)) as u32);
        }

        // ── ALU register ────────────────────────────────────────────────
        Instruction::Add { rd, rs, rt } => {
            let a = cpu.reg(rs) as i32;
            let b = cpu.reg(rt) as i32;
            match a.checked_add(b) {
                Some(v) => cpu.set_reg(rd, v as u32),
                None => enter_exception(cpu, EXC_OVERFLOW),
            }
        }
        Instruction::Addu { rd, rs, rt } => {
            cpu.set_reg(rd, cpu.reg(rs).wrapping_add(cpu.reg(rt)));
        }
        Instruction::Sub { rd, rs, rt } => {
            let a = cpu.reg(rs) as i32;
            let b = cpu.reg(rt) as i32;
            match a.checked_sub(b) {
                Some(v) => cpu.set_reg(rd, v as u32),
                None => enter_exception(cpu, EXC_OVERFLOW),
            }
        }
        Instruction::Subu { rd, rs, rt } => {
            cpu.set_reg(rd, cpu.reg(rs).wrapping_sub(cpu.reg(rt)));
        }
        Instruction::And { rd, rs, rt } => cpu.set_reg(rd, cpu.reg(rs) & cpu.reg(rt)),
        Instruction::Or { rd, rs, rt } => cpu.set_reg(rd, cpu.reg(rs) | cpu.reg(rt)),
        Instruction::Xor { rd, rs, rt } => cpu.set_reg(rd, cpu.reg(rs) ^ cpu.reg(rt)),
        Instruction::Nor { rd, rs, rt } => cpu.set_reg(rd, !(cpu.reg(rs) | cpu.reg(rt))),
        Instruction::Slt { rd, rs, rt } => {
            let v = ((cpu.reg(rs) as i32) < (cpu.reg(rt) as i32)) as u32;
            cpu.set_reg(rd, v);
        }
        Instruction::Sltu { rd, rs, rt } => {
            let v = (cpu.reg(rs) < cpu.reg(rt)) as u32;
            cpu.set_reg(rd, v);
        }

        // ── HI / LO ─────────────────────────────────────────────────────
        Instruction::Mfhi { rd } => cpu.set_reg(rd, cpu.hi),
        Instruction::Mflo { rd } => cpu.set_reg(rd, cpu.lo),
        Instruction::Mthi { rs } => cpu.hi = cpu.reg(rs),
        Instruction::Mtlo { rs } => cpu.lo = cpu.reg(rs),
        Instruction::Mult { rs, rt } => {
            let product = i64::from(cpu.reg(rs) as i32) * i64::from(cpu.reg(rt) as i32);
            cpu.lo = product as u32;
            cpu.hi = (product >> 32) as u32;
        }
        Instruction::Multu { rs, rt } => {
            let product = u64::from(cpu.reg(rs)) * u64::from(cpu.reg(rt));
            cpu.lo = product as u32;
            cpu.hi = (product >> 32) as u32;
        }
        Instruction::Div { rs, rt } => {
            let n = cpu.reg(rs) as i32;
            let d = cpu.reg(rt) as i32;
            if d == 0 {
                cpu.hi = n as u32;
                cpu.lo = if n >= 0 { 0xFFFF_FFFF } else { 1 };
            } else if n == i32::MIN && d == -1 {
                cpu.hi = 0;
                cpu.lo = i32::MIN as u32;
            } else {
                cpu.lo = (n / d) as u32;
                cpu.hi = (n % d) as u32;
            }
        }
        Instruction::Divu { rs, rt } => {
            let n = cpu.reg(rs);
            let d = cpu.reg(rt);
            if d == 0 {
                cpu.hi = n;
                cpu.lo = 0xFFFF_FFFF;
            } else {
                cpu.lo = n / d;
                cpu.hi = n % d;
            }
        }

        // ── ALU immediate ───────────────────────────────────────────────
        Instruction::Addi { rt, rs, imm } => {
            let a = cpu.reg(rs) as i32;
            let b = sign_extend(imm) as i32;
            match a.checked_add(b) {
                Some(v) => cpu.set_reg(rt, v as u32),
                None => enter_exception(cpu, EXC_OVERFLOW),
            }
        }
        Instruction::Addiu { rt, rs, imm } => {
            cpu.set_reg(rt, cpu.reg(rs).wrapping_add(sign_extend(imm)));
        }
        Instruction::Slti { rt, rs, imm } => {
            let v = ((cpu.reg(rs) as i32) < (sign_extend(imm) as i32)) as u32;
            cpu.set_reg(rt, v);
        }
        Instruction::Sltiu { rt, rs, imm } => {
            let v = (cpu.reg(rs) < sign_extend(imm)) as u32;
            cpu.set_reg(rt, v);
        }
        Instruction::Andi { rt, rs, imm } => cpu.set_reg(rt, cpu.reg(rs) & u32::from(imm)),
        Instruction::Ori { rt, rs, imm } => cpu.set_reg(rt, cpu.reg(rs) | u32::from(imm)),
        Instruction::Xori { rt, rs, imm } => cpu.set_reg(rt, cpu.reg(rs) ^ u32::from(imm)),
        Instruction::Lui { rt, imm } => cpu.set_reg(rt, u32::from(imm) << 16),

        // ── Jumps ───────────────────────────────────────────────────────
        Instruction::J { target } => {
            cpu.next_pc = (cpu.pc & 0xF000_0000) | (target << 2);
            cpu.branch = true;
        }
        Instruction::Jal { target } => {
            let ra = cpu.pc.wrapping_add(4);
            cpu.set_reg(31, ra);
            cpu.next_pc = (cpu.pc & 0xF000_0000) | (target << 2);
            cpu.branch = true;
        }
        Instruction::Jr { rs } => {
            cpu.next_pc = cpu.reg(rs);
            cpu.branch = true;
        }
        Instruction::Jalr { rd, rs } => {
            let ra = cpu.pc.wrapping_add(4);
            let target = cpu.reg(rs);
            cpu.set_reg(rd, ra);
            cpu.next_pc = target;
            cpu.branch = true;
        }

        // ── Branches ────────────────────────────────────────────────────
        Instruction::Beq { rs, rt, imm } => {
            if cpu.reg(rs) == cpu.reg(rt) {
                branch(cpu, imm);
            }
        }
        Instruction::Bne { rs, rt, imm } => {
            if cpu.reg(rs) != cpu.reg(rt) {
                branch(cpu, imm);
            }
        }
        Instruction::Blez { rs, imm } => {
            if (cpu.reg(rs) as i32) <= 0 {
                branch(cpu, imm);
            }
        }
        Instruction::Bgtz { rs, imm } => {
            if (cpu.reg(rs) as i32) > 0 {
                branch(cpu, imm);
            }
        }
        Instruction::Bltz { rs, imm } => {
            if (cpu.reg(rs) as i32) < 0 {
                branch(cpu, imm);
            }
        }
        Instruction::Bgez { rs, imm } => {
            if (cpu.reg(rs) as i32) >= 0 {
                branch(cpu, imm);
            }
        }
        Instruction::Bltzal { rs, imm } => {
            let ra = cpu.pc.wrapping_add(4);
            cpu.set_reg(31, ra);
            if (cpu.reg(rs) as i32) < 0 {
                branch(cpu, imm);
            }
        }
        Instruction::Bgezal { rs, imm } => {
            let ra = cpu.pc.wrapping_add(4);
            cpu.set_reg(31, ra);
            if (cpu.reg(rs) as i32) >= 0 {
                branch(cpu, imm);
            }
        }

        // ── Loads ───────────────────────────────────────────────────────
        Instruction::Lb { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            let value = bus.load8(addr) as i8 as i32 as u32;
            cpu.pending_load = (rt, value);
        }
        Instruction::Lbu { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            let value = u32::from(bus.load8(addr));
            cpu.pending_load = (rt, value);
        }
        Instruction::Lh { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            if addr & 1 != 0 {
                cpu.cop0[COP0_BADVADDR] = addr;
                enter_exception(cpu, EXC_ADEL);
            } else {
                let value = bus.load16(addr) as i16 as i32 as u32;
                cpu.pending_load = (rt, value);
            }
        }
        Instruction::Lhu { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            if addr & 1 != 0 {
                cpu.cop0[COP0_BADVADDR] = addr;
                enter_exception(cpu, EXC_ADEL);
            } else {
                let value = u32::from(bus.load16(addr));
                cpu.pending_load = (rt, value);
            }
        }
        Instruction::Lw { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            if addr & 3 != 0 {
                cpu.cop0[COP0_BADVADDR] = addr;
                enter_exception(cpu, EXC_ADEL);
            } else {
                let value = bus.load32(addr);
                cpu.pending_load = (rt, value);
            }
        }
        Instruction::Lwl { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            // Merge with the value currently in the load delay slot so that a
            // back-to-back LWL/LWR pair targeting the same register composes.
            let current = cpu.out_regs[rt as usize];
            let aligned = bus.load32(addr & !3);
            let value = match addr & 3 {
                0 => (current & 0x00FF_FFFF) | (aligned << 24),
                1 => (current & 0x0000_FFFF) | (aligned << 16),
                2 => (current & 0x0000_00FF) | (aligned << 8),
                _ => aligned,
            };
            cpu.pending_load = (rt, value);
        }
        Instruction::Lwr { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            let current = cpu.out_regs[rt as usize];
            let aligned = bus.load32(addr & !3);
            let value = match addr & 3 {
                0 => aligned,
                1 => (current & 0xFF00_0000) | (aligned >> 8),
                2 => (current & 0xFFFF_0000) | (aligned >> 16),
                _ => (current & 0xFFFF_FF00) | (aligned >> 24),
            };
            cpu.pending_load = (rt, value);
        }

        // ── Stores ──────────────────────────────────────────────────────
        Instruction::Sb { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            if !cache_isolated(cpu) {
                bus.store8(addr, cpu.reg(rt) as u8);
            }
        }
        Instruction::Sh { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            if addr & 1 != 0 {
                cpu.cop0[COP0_BADVADDR] = addr;
                enter_exception(cpu, EXC_ADES);
            } else if !cache_isolated(cpu) {
                bus.store16(addr, cpu.reg(rt) as u16);
            }
        }
        Instruction::Sw { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            if addr & 3 != 0 {
                cpu.cop0[COP0_BADVADDR] = addr;
                enter_exception(cpu, EXC_ADES);
            } else if !cache_isolated(cpu) {
                bus.store32(addr, cpu.reg(rt));
            }
        }
        Instruction::Swl { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            if !cache_isolated(cpu) {
                let aligned = addr & !3;
                let v = cpu.reg(rt);
                let mem = bus.load32(aligned);
                let merged = match addr & 3 {
                    0 => (mem & 0xFFFF_FF00) | (v >> 24),
                    1 => (mem & 0xFFFF_0000) | (v >> 16),
                    2 => (mem & 0xFF00_0000) | (v >> 8),
                    _ => v,
                };
                bus.store32(aligned, merged);
            }
        }
        Instruction::Swr { rt, rs, imm } => {
            let addr = cpu.reg(rs).wrapping_add(sign_extend(imm));
            if !cache_isolated(cpu) {
                let aligned = addr & !3;
                let v = cpu.reg(rt);
                let mem = bus.load32(aligned);
                let merged = match addr & 3 {
                    0 => v,
                    1 => (mem & 0x0000_00FF) | (v << 8),
                    2 => (mem & 0x0000_FFFF) | (v << 16),
                    _ => (mem & 0x00FF_FFFF) | (v << 24),
                };
                bus.store32(aligned, merged);
            }
        }

        // ── Coprocessor 0 ───────────────────────────────────────────────
        Instruction::Mfc0 { rt, rd } => {
            // Coprocessor moves also occupy the load delay slot.
            cpu.pending_load = (rt, cpu.cop0[rd as usize]);
        }
        Instruction::Mtc0 { rt, rd } => {
            cpu.cop0[rd as usize] = cpu.reg(rt);
        }
        Instruction::Rfe => rfe(cpu),

        // ── Coprocessor 2 (GTE): decoded but not executed ───────────────
        Instruction::Cop2 { .. } => {}

        // ── Traps ───────────────────────────────────────────────────────
        Instruction::Syscall => enter_exception(cpu, EXC_SYSCALL),
        Instruction::Break => enter_exception(cpu, EXC_BREAK),
        Instruction::Illegal { .. } => enter_exception(cpu, EXC_RI),
    }
}

/// Computes and installs a relative branch target into `next_pc`.
#[inline]
fn branch(cpu: &mut Cpu, imm: u16) {
    let offset = sign_extend(imm) << 2;
    cpu.next_pc = cpu.pc.wrapping_add(offset);
    cpu.branch = true;
}

#[inline]
fn cache_isolated(cpu: &Cpu) -> bool {
    cpu.sr() & SR_ISOLATE_CACHE != 0
}

/// Enters the exception handler: saves `EPC`, records the cause (with the `BD`
/// bit when in a delay slot), pushes the SR mode/interrupt stack, and jumps to
/// the appropriate vector.
fn enter_exception(cpu: &mut Cpu, cause: u32) {
    let handler = if cpu.sr() & SR_BEV != 0 {
        0xBFC0_0180
    } else {
        0x8000_0080
    };

    // Push the 3-deep kernel/interrupt-enable stack (bits 0..=5 of SR).
    let mode = cpu.cop0[COP0_SR] & 0x3F;
    cpu.cop0[COP0_SR] &= !0x3F;
    cpu.cop0[COP0_SR] |= (mode << 2) & 0x3F;

    // Record ExcCode (bits 2..=6) and BD (bit 31) in CAUSE.
    let mut cause_reg = cpu.cop0[COP0_CAUSE] & !0x8000_007C;
    cause_reg |= (cause & 0x1F) << 2;

    let epc = if cpu.delay_slot {
        cause_reg |= 1 << 31;
        cpu.current_pc.wrapping_sub(4)
    } else {
        cpu.current_pc
    };
    cpu.cop0[COP0_CAUSE] = cause_reg;
    cpu.cop0[COP0_EPC] = epc;

    // Vector immediately; no delay slot on the jump into the handler.
    cpu.pc = handler;
    cpu.next_pc = handler.wrapping_add(4);
    cpu.branch = false;
    cpu.delay_slot = false;
}

/// Restore-from-exception: pops the SR mode/interrupt-enable stack.
fn rfe(cpu: &mut Cpu) {
    let mode = cpu.cop0[COP0_SR] & 0x3F;
    cpu.cop0[COP0_SR] &= !0x3F;
    cpu.cop0[COP0_SR] |= mode >> 2;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::engine::SR_BEV;

    /// Simple flat little-endian memory for interpreter tests.
    struct TestBus {
        mem: Vec<u8>,
    }

    impl TestBus {
        fn new() -> Self {
            Self {
                mem: vec![0; 0x10000],
            }
        }
        fn write_u32(&mut self, addr: u32, value: u32) {
            self.store32(addr, value);
        }
    }

    impl Bus for TestBus {
        fn load8(&mut self, addr: u32) -> u8 {
            self.mem[addr as usize & 0xFFFF]
        }
        fn load16(&mut self, addr: u32) -> u16 {
            let a = addr as usize & 0xFFFF;
            u16::from_le_bytes([self.mem[a], self.mem[a + 1]])
        }
        fn load32(&mut self, addr: u32) -> u32 {
            let a = addr as usize & 0xFFFF;
            u32::from_le_bytes([
                self.mem[a],
                self.mem[a + 1],
                self.mem[a + 2],
                self.mem[a + 3],
            ])
        }
        fn store8(&mut self, addr: u32, value: u8) {
            self.mem[addr as usize & 0xFFFF] = value;
        }
        fn store16(&mut self, addr: u32, value: u16) {
            let a = addr as usize & 0xFFFF;
            let b = value.to_le_bytes();
            self.mem[a] = b[0];
            self.mem[a + 1] = b[1];
        }
        fn store32(&mut self, addr: u32, value: u32) {
            let a = addr as usize & 0xFFFF;
            let b = value.to_le_bytes();
            self.mem[a..a + 4].copy_from_slice(&b);
        }
    }

    /// Builds a CPU whose PC points at a program loaded into a fresh bus at
    /// `0x0000_0000`. Clears BEV so exceptions vector to RAM (`0x8000_0080`),
    /// which our test bus can hold, and clears cache isolation.
    fn setup(program: &[u32]) -> (Cpu, TestBus) {
        let mut bus = TestBus::new();
        for (i, &word) in program.iter().enumerate() {
            bus.write_u32(i as u32 * 4, word);
        }
        let mut cpu = Cpu::new();
        cpu.pc = 0;
        cpu.next_pc = 4;
        cpu.cop0[COP0_SR] = 0;
        (cpu, bus)
    }

    fn r_type(rs: u32, rt: u32, rd: u32, shamt: u32, funct: u32) -> u32 {
        (rs << 21) | (rt << 16) | (rd << 11) | (shamt << 6) | funct
    }
    fn i_type(op: u32, rs: u32, rt: u32, imm: u16) -> u32 {
        (op << 26) | (rs << 21) | (rt << 16) | u32::from(imm)
    }

    fn run(cpu: &mut Cpu, bus: &mut TestBus, steps: usize) {
        for _ in 0..steps {
            step(cpu, bus);
        }
    }

    #[test]
    fn addu_adds_without_trap() {
        // addiu $t0,$zero,5 ; addiu $t1,$zero,7 ; addu $t2,$t0,$t1
        let (mut cpu, mut bus) = setup(&[
            i_type(0x09, 0, 8, 5),
            i_type(0x09, 0, 9, 7),
            r_type(8, 9, 10, 0, 0x21),
        ]);
        run(&mut cpu, &mut bus, 3);
        assert_eq!(cpu.reg(10), 12);
    }

    #[test]
    fn add_overflow_traps_and_preserves_dest() {
        // Load INT_MAX into $t0, then add 1 with trapping ADD.
        // lui $t0,0x7fff ; ori $t0,$t0,0xffff ; addiu $t1,$zero,1 ; add $t2,$t0,$t1
        let (mut cpu, mut bus) = setup(&[
            i_type(0x0F, 0, 8, 0x7FFF),
            i_type(0x0D, 8, 8, 0xFFFF),
            i_type(0x09, 0, 9, 1),
            r_type(8, 9, 10, 0, 0x20),
        ]);
        run(&mut cpu, &mut bus, 4);
        // Destination unchanged and an overflow exception was taken.
        assert_eq!(cpu.reg(10), 0);
        assert_eq!((cpu.cop0[COP0_CAUSE] >> 2) & 0x1F, EXC_OVERFLOW);
    }

    #[test]
    fn addiu_sign_extends_immediate() {
        // addiu $t0,$zero,0xFFFF  => -1
        let (mut cpu, mut bus) = setup(&[i_type(0x09, 0, 8, 0xFFFF)]);
        run(&mut cpu, &mut bus, 1);
        assert_eq!(cpu.reg(8), 0xFFFF_FFFF);
    }

    #[test]
    fn lui_ori_compose_constant() {
        // lui $t0,0xABCD ; ori $t0,$t0,0x1234
        let (mut cpu, mut bus) = setup(&[i_type(0x0F, 0, 8, 0xABCD), i_type(0x0D, 8, 8, 0x1234)]);
        run(&mut cpu, &mut bus, 2);
        assert_eq!(cpu.reg(8), 0xABCD_1234);
    }

    #[test]
    fn slt_and_sltu_signedness() {
        // $t0 = -1, $t1 = 1
        let (mut cpu, mut bus) = setup(&[
            i_type(0x09, 0, 8, 0xFFFF), // -1
            i_type(0x09, 0, 9, 1),      // 1
            r_type(8, 9, 10, 0, 0x2A),  // slt  $t2,$t0,$t1 -> 1 (signed -1<1)
            r_type(8, 9, 11, 0, 0x2B),  // sltu $t3,$t0,$t1 -> 0 (unsigned 0xFFFFFFFF<1 false)
        ]);
        run(&mut cpu, &mut bus, 4);
        assert_eq!(cpu.reg(10), 1);
        assert_eq!(cpu.reg(11), 0);
    }

    #[test]
    fn shifts_sll_srl_sra() {
        // $t0 = 0x8000_0000
        let (mut cpu, mut bus) = setup(&[
            i_type(0x0F, 0, 8, 0x8000), // lui $t0,0x8000
            r_type(0, 8, 9, 4, 0x00),   // sll  $t1,$t0,4  -> 0x0000_0000
            r_type(0, 8, 10, 4, 0x02),  // srl  $t2,$t0,4  -> 0x0800_0000
            r_type(0, 8, 11, 4, 0x03),  // sra  $t3,$t0,4  -> 0xF800_0000
        ]);
        run(&mut cpu, &mut bus, 4);
        assert_eq!(cpu.reg(9), 0x0000_0000);
        assert_eq!(cpu.reg(10), 0x0800_0000);
        assert_eq!(cpu.reg(11), 0xF800_0000);
    }

    #[test]
    fn branch_delay_slot_executes() {
        // beq $zero,$zero,+2 (skip to index 3) ; delay: addiu $t0,$zero,1 ;
        // addiu $t0,$t0,10 (skipped) ; addiu $t1,$zero,7 (target)
        let (mut cpu, mut bus) = setup(&[
            i_type(0x04, 0, 0, 2),  // beq -> target = pc(4) + 2*4 + 4 = index 3
            i_type(0x09, 0, 8, 1),  // delay slot: $t0 = 1  (executes)
            i_type(0x09, 8, 8, 10), // skipped
            i_type(0x09, 0, 9, 7),  // target: $t1 = 7
        ]);
        run(&mut cpu, &mut bus, 3); // branch, delay slot, target
        assert_eq!(cpu.reg(8), 1); // delay slot ran, skipped add did not
        assert_eq!(cpu.reg(9), 7); // landed on target
    }

    #[test]
    fn branch_not_taken_falls_through() {
        // bne $zero,$zero,+8 (never) ; addiu $t0,$zero,5
        let (mut cpu, mut bus) = setup(&[i_type(0x05, 0, 0, 8), i_type(0x09, 0, 8, 5)]);
        run(&mut cpu, &mut bus, 2);
        assert_eq!(cpu.reg(8), 5);
    }

    #[test]
    fn jal_links_and_jr_returns() {
        // 0: jal 0x4 (target index 4) ; 1: delay addiu $t0,$zero,1 ;
        // 4: jr $ra ; 5: delay addiu $t1,$zero,2
        let (mut cpu, mut bus) = setup(&[
            0x0C00_0000 | 4,       // jal -> target = 4<<2 = 0x10 (index 4)
            i_type(0x09, 0, 8, 1), // delay slot
            0,
            0,
            r_type(31, 0, 0, 0, 0x08), // jr $ra
            i_type(0x09, 0, 9, 2),     // delay slot of jr
        ]);
        run(&mut cpu, &mut bus, 4); // jal, delay, jr, delay
        assert_eq!(cpu.reg(8), 1);
        assert_eq!(cpu.reg(9), 2);
        // $ra should point at instruction after jal's delay slot (index 2 => 0x8).
        assert_eq!(cpu.reg(31), 0x8);
        // After jr returns, pc should be back at 0x8.
        assert_eq!(cpu.pc, 0x8);
    }

    #[test]
    fn load_delay_slot_reads_old_value() {
        // Prime memory word at 0x100 = 0xDEAD_BEEF.
        // addiu $t0,$zero,0x100 ; addiu $t1,$zero,0x1111 ;
        // lw $t1,0($t0) ; addu $t2,$t1,$zero (delay slot sees OLD $t1)
        let (mut cpu, mut bus) = setup(&[
            i_type(0x09, 0, 8, 0x100),  // $t0 = 0x100
            i_type(0x09, 0, 9, 0x1111), // $t1 = 0x1111
            i_type(0x23, 8, 9, 0),      // lw $t1,0($t0)
            r_type(9, 0, 10, 0, 0x21),  // addu $t2,$t1,$zero
        ]);
        bus.write_u32(0x100, 0xDEAD_BEEF);
        run(&mut cpu, &mut bus, 4);
        // $t2 saw the OLD $t1 (0x1111), not the freshly loaded word.
        assert_eq!(cpu.reg(10), 0x1111);
        // But $t1 itself is now the loaded value.
        assert_eq!(cpu.reg(9), 0xDEAD_BEEF);
    }

    #[test]
    fn mult_and_mfhi_mflo() {
        // $t0 = 0x0001_0000 ; $t1 = 0x0001_0000 ; mult -> hi:lo = 0x1_0000_0000
        let (mut cpu, mut bus) = setup(&[
            i_type(0x0F, 0, 8, 1),     // lui $t0,1 -> 0x0001_0000
            i_type(0x0F, 0, 9, 1),     // lui $t1,1
            r_type(8, 9, 0, 0, 0x18),  // mult $t0,$t1
            r_type(0, 0, 10, 0, 0x10), // mfhi $t2
            r_type(0, 0, 11, 0, 0x12), // mflo $t3
        ]);
        run(&mut cpu, &mut bus, 5);
        assert_eq!(cpu.reg(10), 1); // hi
        assert_eq!(cpu.reg(11), 0); // lo
    }

    #[test]
    fn div_signed_and_edge_cases() {
        let mut cpu = Cpu::new();
        let mut bus = TestBus::new();
        cpu.regs[8] = (-7i32) as u32;
        cpu.regs[9] = 2;
        execute_instruction(&mut cpu, &mut bus, Instruction::Div { rs: 8, rt: 9 });
        assert_eq!(cpu.lo as i32, -3); // -7 / 2
        assert_eq!(cpu.hi as i32, -1); // -7 % 2

        // Divide by zero (positive dividend): lo=-1, hi=dividend.
        cpu.regs[8] = 5;
        cpu.regs[9] = 0;
        execute_instruction(&mut cpu, &mut bus, Instruction::Div { rs: 8, rt: 9 });
        assert_eq!(cpu.lo, 0xFFFF_FFFF);
        assert_eq!(cpu.hi, 5);

        // INT_MIN / -1 overflow: lo=INT_MIN, hi=0.
        cpu.regs[8] = i32::MIN as u32;
        cpu.regs[9] = (-1i32) as u32;
        execute_instruction(&mut cpu, &mut bus, Instruction::Div { rs: 8, rt: 9 });
        assert_eq!(cpu.lo, i32::MIN as u32);
        assert_eq!(cpu.hi, 0);
    }

    #[test]
    fn divu_by_zero() {
        let mut cpu = Cpu::new();
        let mut bus = TestBus::new();
        cpu.regs[8] = 100;
        cpu.regs[9] = 0;
        execute_instruction(&mut cpu, &mut bus, Instruction::Divu { rs: 8, rt: 9 });
        assert_eq!(cpu.lo, 0xFFFF_FFFF);
        assert_eq!(cpu.hi, 100);
    }

    #[test]
    fn lw_sw_round_trip() {
        // $t0=0x200 ; $t1=0xCAFE_F00D ; sw $t1,0($t0) ; lw $t2,0($t0)
        let (mut cpu, mut bus) = setup(&[
            i_type(0x0F, 0, 9, 0xCAFE), // lui $t1,0xCAFE
            i_type(0x0D, 9, 9, 0xF00D), // ori $t1,$t1,0xF00D
            i_type(0x09, 0, 8, 0x200),  // $t0 = 0x200
            i_type(0x2B, 8, 9, 0),      // sw $t1,0($t0)
            i_type(0x23, 8, 10, 0),     // lw $t2,0($t0)
        ]);
        run(&mut cpu, &mut bus, 5);
        // One more step to let the load delay commit into a readable reg.
        step(&mut cpu, &mut bus);
        assert_eq!(cpu.reg(10), 0xCAFE_F00D);
        assert_eq!(bus.load32(0x200), 0xCAFE_F00D);
    }

    #[test]
    fn lb_sign_extends_lbu_zero_extends() {
        let mut cpu = Cpu::new();
        let mut bus = TestBus::new();
        bus.store8(0x40, 0x80);
        cpu.regs[8] = 0x40;

        execute_instruction(
            &mut cpu,
            &mut bus,
            Instruction::Lb {
                rt: 9,
                rs: 8,
                imm: 0,
            },
        );
        assert_eq!(cpu.pending_load, (9, 0xFFFF_FF80));

        execute_instruction(
            &mut cpu,
            &mut bus,
            Instruction::Lbu {
                rt: 10,
                rs: 8,
                imm: 0,
            },
        );
        assert_eq!(cpu.pending_load, (10, 0x0000_0080));
    }

    #[test]
    fn syscall_raises_exception_and_sets_epc() {
        // Give BEV so the vector is the BIOS handler; place syscall at 0x4.
        let (mut cpu, mut bus) = setup(&[
            i_type(0x09, 0, 8, 1),    // 0x0: addiu (padding)
            r_type(0, 0, 0, 0, 0x0C), // 0x4: syscall
        ]);
        cpu.cop0[COP0_SR] = SR_BEV;
        run(&mut cpu, &mut bus, 2);
        assert_eq!((cpu.cop0[COP0_CAUSE] >> 2) & 0x1F, EXC_SYSCALL);
        assert_eq!(cpu.cop0[COP0_EPC], 0x4);
        assert_eq!(cpu.pc, 0xBFC0_0180);
    }

    #[test]
    fn syscall_in_delay_slot_sets_bd_and_backs_up_epc() {
        // beq $zero,$zero,+... (taken) ; delay slot = syscall
        let (mut cpu, mut bus) = setup(&[
            i_type(0x04, 0, 0, 4),    // 0x0: beq taken
            r_type(0, 0, 0, 0, 0x0C), // 0x4: syscall in delay slot
        ]);
        cpu.cop0[COP0_SR] = SR_BEV;
        run(&mut cpu, &mut bus, 2);
        // BD bit set, EPC points at the branch (0x0), not the delay slot.
        assert_ne!(cpu.cop0[COP0_CAUSE] & (1 << 31), 0);
        assert_eq!(cpu.cop0[COP0_EPC], 0x0);
    }

    #[test]
    fn rfe_pops_sr_stack() {
        let mut cpu = Cpu::new();
        let mut bus = TestBus::new();
        // Set a recognizable low-6 SR mode pattern.
        cpu.cop0[COP0_SR] = 0b00_1101;
        enter_exception(&mut cpu, EXC_SYSCALL);
        // Push shifted the mode left by two.
        assert_eq!(cpu.cop0[COP0_SR] & 0x3F, 0b11_0100);
        execute_instruction(&mut cpu, &mut bus, Instruction::Rfe);
        // Pop restores the low four bits.
        assert_eq!(cpu.cop0[COP0_SR] & 0xF, 0b1101);
    }

    #[test]
    fn mtc0_mfc0_round_trip_through_load_delay() {
        let (mut cpu, mut bus) = setup(&[
            i_type(0x09, 0, 8, 0x1F),                             // $t0 = 0x1F
            (0x10 << 26) | (0x04 << 21) | (8 << 16) | (12 << 11), // mtc0 $t0,$12 (SR)
            (0x10 << 26) | (9 << 16) | (12 << 11),                // mfc0 $t1,$12
            i_type(0x09, 0, 0, 0), // nop to commit the mfc0 load delay
        ]);
        run(&mut cpu, &mut bus, 4);
        assert_eq!(cpu.reg(9), 0x1F);
    }

    #[test]
    fn illegal_instruction_traps_reserved() {
        let (mut cpu, mut bus) = setup(&[0xFFFF_FFFF]);
        cpu.cop0[COP0_SR] = SR_BEV;
        run(&mut cpu, &mut bus, 1);
        assert_eq!((cpu.cop0[COP0_CAUSE] >> 2) & 0x1F, EXC_RI);
    }

    #[test]
    fn swl_swr_write_unaligned_word() {
        // Store 0x1122_3344 across an unaligned address using SWR+SWL.
        let mut cpu = Cpu::new();
        let mut bus = TestBus::new();
        cpu.regs[8] = 0x102; // base, unaligned
        cpu.out_regs[8] = 0x102;
        cpu.regs[9] = 0x1122_3344;
        cpu.out_regs[9] = 0x1122_3344;
        // SWR at 0x102 + SWL at 0x105 fully writes the 4 bytes at 0x102..0x106.
        execute_instruction(
            &mut cpu,
            &mut bus,
            Instruction::Swr {
                rt: 9,
                rs: 8,
                imm: 0,
            },
        );
        execute_instruction(
            &mut cpu,
            &mut bus,
            Instruction::Swl {
                rt: 9,
                rs: 8,
                imm: 3,
            },
        );
        assert_eq!(bus.load8(0x102), 0x44);
        assert_eq!(bus.load8(0x103), 0x33);
        assert_eq!(bus.load8(0x104), 0x22);
        assert_eq!(bus.load8(0x105), 0x11);

        // Read the unaligned word back with LWR+LWL, chaining through the load
        // delay slot (LWL merges with the value LWR left pending).
        execute_instruction(
            &mut cpu,
            &mut bus,
            Instruction::Lwr {
                rt: 10,
                rs: 8,
                imm: 0,
            },
        );
        let (r, v) = cpu.pending_load;
        cpu.out_regs[r as usize] = v;
        execute_instruction(
            &mut cpu,
            &mut bus,
            Instruction::Lwl {
                rt: 10,
                rs: 8,
                imm: 3,
            },
        );
        assert_eq!(cpu.pending_load, (10, 0x1122_3344));
    }
}
