//! Property tests: the decoder and interpreter never panic on arbitrary input,
//! `r0` stays zero, and memory access is total over the address space.

use proptest::prelude::*;
use psoxide_core::cpu::{Cpu, decode, engine, step};
use psoxide_core::{Command, CoreQuery, PsxCore, QueryResult, mask_region};

/// Minimal flat little-endian bus for stepping arbitrary instruction streams.
struct FlatBus {
    mem: Vec<u8>,
}

impl FlatBus {
    fn new() -> Self {
        Self {
            mem: vec![0; 0x2_0000],
        }
    }
}

impl psoxide_core::cpu::Bus for FlatBus {
    fn load8(&mut self, addr: u32) -> u8 {
        self.mem[addr as usize & 0x1_FFFF]
    }
    fn load16(&mut self, addr: u32) -> u16 {
        let a = addr as usize & 0x1_FFFF;
        u16::from_le_bytes([self.mem[a], self.mem[(a + 1) & 0x1_FFFF]])
    }
    fn load32(&mut self, addr: u32) -> u32 {
        let a = addr as usize & 0x1_FFFF;
        u32::from_le_bytes([
            self.mem[a],
            self.mem[(a + 1) & 0x1_FFFF],
            self.mem[(a + 2) & 0x1_FFFF],
            self.mem[(a + 3) & 0x1_FFFF],
        ])
    }
    fn store8(&mut self, addr: u32, value: u8) {
        self.mem[addr as usize & 0x1_FFFF] = value;
    }
    fn store16(&mut self, addr: u32, value: u16) {
        let a = addr as usize & 0x1_FFFF;
        let b = value.to_le_bytes();
        self.mem[a] = b[0];
        self.mem[(a + 1) & 0x1_FFFF] = b[1];
    }
    fn store32(&mut self, addr: u32, value: u32) {
        let b = value.to_le_bytes();
        for (i, byte) in b.iter().enumerate() {
            self.mem[(addr as usize + i) & 0x1_FFFF] = *byte;
        }
    }
}

proptest! {
    #[test]
    fn decode_never_panics(word in any::<u32>()) {
        let _ = decode(word);
    }

    #[test]
    fn mask_region_is_bounded(addr in any::<u32>()) {
        // Masking only clears bits, so the physical address never exceeds it.
        prop_assert!(mask_region(addr) <= addr);
    }

    #[test]
    fn stepping_arbitrary_stream_never_panics(words in prop::collection::vec(any::<u32>(), 1..64)) {
        let mut bus = FlatBus::new();
        for (i, &w) in words.iter().enumerate() {
            let b = w.to_le_bytes();
            let base = i * 4;
            bus.mem[base..base + 4].copy_from_slice(&b);
        }
        let mut cpu = Cpu::new();
        cpu.pc = 0;
        cpu.next_pc = 4;
        cpu.cop0[engine::COP0_SR] = 0; // vector to RAM on exception; stays in-bounds
        for _ in 0..words.len() {
            step(&mut cpu, &mut bus);
        }
    }

    #[test]
    fn register_zero_stays_zero(words in prop::collection::vec(any::<u32>(), 1..64)) {
        let mut bus = FlatBus::new();
        for (i, &w) in words.iter().enumerate() {
            let b = w.to_le_bytes();
            let base = i * 4;
            bus.mem[base..base + 4].copy_from_slice(&b);
        }
        let mut cpu = Cpu::new();
        cpu.pc = 0;
        cpu.next_pc = 4;
        cpu.cop0[engine::COP0_SR] = 0;
        for _ in 0..words.len() {
            step(&mut cpu, &mut bus);
            prop_assert_eq!(cpu.reg(0), 0);
        }
    }

    #[test]
    fn delayed_load_never_clobbers_following_write(mem_val in any::<u32>(), imm in any::<u16>()) {
        // A load into $t0 immediately followed by a register write into $t0 must
        // never let the (later-committing) load overwrite the write: on the
        // R3000 the delay-slot instruction's own writeback wins.
        //   lw    $t0, 0($t1)        ; $t1 preset to 0x400; queues $t0 = mem_val
        //   addiu $t0, $zero, imm    ; delay slot writes $t0 = sign_extend(imm)
        //   nop                      ; commit point
        let mut bus = FlatBus::new();
        bus.mem[0x400..0x404].copy_from_slice(&mem_val.to_le_bytes());
        let program = [
            (0x23u32 << 26) | (9 << 21) | (8 << 16), // lw $t0,0($t1)
            (0x09u32 << 26) | (8 << 16) | u32::from(imm), // addiu $t0,$zero,imm
            0,                                       // nop
        ];
        for (i, &w) in program.iter().enumerate() {
            let base = i * 4;
            bus.mem[base..base + 4].copy_from_slice(&w.to_le_bytes());
        }
        let mut cpu = Cpu::new();
        cpu.pc = 0;
        cpu.next_pc = 4;
        cpu.cop0[engine::COP0_SR] = 0;
        cpu.regs[9] = 0x400;
        cpu.out_regs[9] = 0x400;
        for _ in 0..program.len() {
            step(&mut cpu, &mut bus);
        }
        // $t0 holds the ADDIU result regardless of what the load fetched.
        let expected = imm as i16 as i32 as u32;
        prop_assert_eq!(cpu.reg(8), expected);
    }

    #[test]
    fn memory_query_never_panics(addr in any::<u32>(), len in 0u32..64) {
        let core = PsxCore::new();
        let result = core.query(CoreQuery::Memory { addr, len });
        if let QueryResult::Memory(bytes) = result {
            prop_assert_eq!(bytes.len(), len as usize);
        } else {
            prop_assert!(false, "expected Memory result");
        }
    }

    #[test]
    fn step_frame_from_reset_never_panics(seed in any::<u8>()) {
        // Load a tiny pseudo-random BIOS and step a bounded number of CPU
        // instructions from the reset vector without panicking.
        let mut core = PsxCore::new();
        let mut bios = vec![0u8; psoxide_core::BIOS_IMAGE_SIZE];
        for (i, byte) in bios.iter_mut().enumerate() {
            *byte = (i as u8) ^ seed;
        }
        core.execute(Command::LoadBios(bios)).unwrap();
        for _ in 0..2000 {
            core.execute(Command::StepCpu).unwrap();
        }
    }
}
