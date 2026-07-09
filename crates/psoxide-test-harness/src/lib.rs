//! ROM/program-based integration test scaffolding for psoxide.
//!
//! Provides helpers to build a [`PsxCore`], stage a hand-assembled MIPS
//! program into main RAM, run it for a fixed number of instructions, and
//! inspect the resulting register state.
//!
//! Real CPU test ROMs (Amidog `psxtest_cpu`, JaCzekanski `ps1-tests`,
//! PeterLemon PSX demos) would live under `tests/roms/` and be driven through
//! [`Harness::load_bios`]. See `README.md`.

use psoxide_core::{
    COP0_CAUSE, COP0_EPC, COP0_SR, Command, CoreQuery, CpuSnapshot, PsxCore, QueryResult,
};

/// Base address in KUSEG main RAM where staged test programs are placed.
pub const PROGRAM_BASE: u32 = 0x0000_0000;

/// Sentinel return address staged into `$ra` by [`Harness::load_exe`]. When a
/// side-loaded PS-EXE returns to this address, [`Harness::run_hle`] stops.
pub const HLE_RETURN_ADDR: u32 = 0x0000_0000;

/// Interval, in stepped instructions, at which [`Harness::run_hle`] injects a
/// VBlank interrupt. Pragmatic "one frame's worth of stepping" pacing so that
/// programs polling `VSync` progress within a bounded test budget; it is not
/// cycle-accurate (a real NTSC frame is ~564k CPU cycles).
const VBLANK_INTERVAL_STEPS: usize = 100_000;

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
            // Periodically raise VBlank so programs that poll VSync (I_STAT bit
            // 0) make progress instead of spinning forever. `StepCpu` alone
            // never raises VBlank (only `StepFrame` does), so we inject it here
            // roughly once per NTSC frame's worth of stepped instructions.
            if i != 0 && i % VBLANK_INTERVAL_STEPS == 0 {
                self.core.raise_vblank();
            }
            let pc = self.core.pc();
            if pc == HLE_RETURN_ADDR {
                return i;
            }
            let vec = pc & 0x1FFF_FFFF;
            if vec == 0xA0 || vec == 0xB0 || vec == 0xC0 {
                self.hle_bios_call(vec);
                continue;
            }
            // BIOS exception-handler HLE. When execution reaches a general
            // exception vector and no test-installed handler lives there (the
            // vector word is zero — i.e. no BIOS image and no custom handler),
            // emulate the minimal handler. If a real handler is present we fall
            // through and let the CPU execute it.
            if self.at_empty_exception_vector(pc) {
                self.hle_exception();
                continue;
            }
            let _ = self.core.execute(Command::StepCpu);
        }
        max_steps
    }

    /// Returns `true` when `pc` sits at a general exception vector
    /// (`0x8000_0080` RAM or `0xBFC0_0180` BIOS, in any segment alias) and the
    /// first handler word there is zero — meaning no BIOS and no test-installed
    /// custom handler, so the harness must stand in for the BIOS.
    fn at_empty_exception_vector(&self, pc: u32) -> bool {
        let phys = pc & 0x1FFF_FFFF;
        if phys != 0x80 && phys != 0x1FC0_0180 {
            return false;
        }
        // If code is present at the vector (nonzero), it is a real handler.
        self.core.memory().read8(pc) == 0
            && self.core.memory().read8(pc.wrapping_add(1)) == 0
            && self.core.memory().read8(pc.wrapping_add(2)) == 0
            && self.core.memory().read8(pc.wrapping_add(3)) == 0
    }

    /// Minimal high-level emulation of the BIOS general exception handler for
    /// side-loaded tests that run without a real BIOS image.
    ///
    /// Handles `syscall` (dispatching `EnterCriticalSection` / `ExitCriticalSection`
    /// by `$a0`) and hardware interrupts, then performs a return-from-exception:
    /// it pops the `SR` mode/interrupt stack (mirroring the CPU's `rfe`) and
    /// resumes at `EPC` (interrupts, re-execute) or `EPC + 4` (syscall, skip the
    /// trapping instruction).
    ///
    /// Limitation: if the trapping instruction was in a branch-delay slot
    /// (`CAUSE.BD`), `EPC` points at the branch and the delay slot is re-run on
    /// resume. Test programs do not place syscalls in delay slots, so this is
    /// benign in practice.
    fn hle_exception(&mut self) {
        let cause = self.core.cop0(COP0_CAUSE);
        let exccode = (cause >> 2) & 0x1F;
        let epc = self.core.cop0(COP0_EPC);

        const EXC_INT: u32 = 0x00;
        const EXC_SYSCALL: u32 = 0x08;

        // Return-from-exception first: pop the SR mode/interrupt-enable stack
        // exactly as the CPU's `rfe` does, restoring the pre-exception mode.
        // EnterCriticalSection / ExitCriticalSection then adjust IEc *on this
        // restored value*, so their effect persists after the handler returns
        // (setting IEc before the pop would just be shifted back out).
        let sr = self.core.cop0(COP0_SR);
        let mode = sr & 0x3F;
        let mut sr = (sr & !0x3F) | (mode >> 2);

        let resume = match exccode {
            EXC_SYSCALL => {
                match self.core.reg(4) {
                    // EnterCriticalSection: return the previous IEc in $v0, then
                    // disable interrupts (clear SR.IEc).
                    1 => {
                        self.core.set_reg(2, sr & 0x1);
                        sr &= !0x1;
                    }
                    // ExitCriticalSection: enable interrupts (set SR.IEc).
                    2 => sr |= 0x1,
                    // Any other syscall selector is a no-op for CPU testing.
                    _ => {}
                }
                // Resume *after* the syscall instruction.
                epc.wrapping_add(4)
            }
            // Hardware interrupt: acknowledge by returning to the interrupted
            // instruction (the driving I_STAT bit is cleared by device code).
            EXC_INT => epc,
            // Any other trap: best-effort skip of the faulting instruction.
            _ => epc.wrapping_add(4),
        };

        self.core.set_cop0(COP0_SR, sr);
        self.core.set_pc(resume);
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
                self.tty_push_cstr(arg);
            }
            // printf(fmt, ...) — A(0x3F). The format string is in $a0 and the
            // variadic arguments follow the o32 ABI (a1/a2/a3 then the stack).
            (0xA0, 0x3F) => {
                self.hle_printf(arg);
            }
            // Any other BIOS call is a no-op for CPU testing purposes.
            _ => {}
        }

        self.core.set_pc(ra);
    }

    /// Appends the NUL-terminated string at `addr` to the TTY buffer.
    fn tty_push_cstr(&mut self, mut addr: u32) {
        for _ in 0..8192 {
            let byte = self.core.memory().read8(addr);
            if byte == 0 {
                break;
            }
            self.tty.push(byte);
            addr = addr.wrapping_add(1);
        }
    }

    /// Reads the NUL-terminated string at `addr` into an owned byte vector.
    fn read_cstr(&self, mut addr: u32) -> Vec<u8> {
        let mut out = Vec::new();
        for _ in 0..8192 {
            let byte = self.core.memory().read8(addr);
            if byte == 0 {
                break;
            }
            out.push(byte);
            addr = addr.wrapping_add(1);
        }
        out
    }

    /// Fetches the `n`-th (0-based) variadic `printf` argument following the
    /// format string, per the MIPS o32 ABI: the first three come from
    /// `$a1`/`$a2`/`$a3`, and the rest from the stack at `$sp + 16 + 4*(n-3)`.
    fn printf_arg(&self, n: usize) -> u32 {
        match n {
            0 => self.core.reg(5),
            1 => self.core.reg(6),
            2 => self.core.reg(7),
            _ => {
                let sp = self.core.reg(29);
                self.read_word(sp.wrapping_add(16 + 4 * (n as u32 - 3)))
            }
        }
    }

    /// High-level-emulates BIOS `printf`, formatting into the TTY buffer.
    ///
    /// Supports the conversions the CPU test suites use — `d`, `i`, `u`, `x`,
    /// `X`, `o`, `c`, `s`, `p`, `%` — with optional `-`/`0` flags, a decimal or
    /// `*` field width, and an optional `.precision`. Length modifiers
    /// (`h`/`l`/`ll`) are accepted and ignored (all integers are 32-bit).
    fn hle_printf(&mut self, fmt_addr: u32) {
        let fmt = self.read_cstr(fmt_addr);
        let mut out: Vec<u8> = Vec::new();
        let mut argn = 0usize;
        let mut i = 0usize;
        while i < fmt.len() {
            let c = fmt[i];
            if c != b'%' {
                out.push(c);
                i += 1;
                continue;
            }
            i += 1;
            if i >= fmt.len() {
                break;
            }
            // Flags.
            let mut left = false;
            let mut zero = false;
            while i < fmt.len() {
                match fmt[i] {
                    b'-' => left = true,
                    b'0' => zero = true,
                    b'+' | b' ' | b'#' => {}
                    _ => break,
                }
                i += 1;
            }
            // Width (decimal or '*').
            let mut width = 0usize;
            if i < fmt.len() && fmt[i] == b'*' {
                width = self.printf_arg(argn) as i32 as usize;
                argn += 1;
                i += 1;
            } else {
                while i < fmt.len() && fmt[i].is_ascii_digit() {
                    width = width * 10 + usize::from(fmt[i] - b'0');
                    i += 1;
                }
            }
            // Precision.
            let mut precision: Option<usize> = None;
            if i < fmt.len() && fmt[i] == b'.' {
                i += 1;
                let mut p = 0usize;
                if i < fmt.len() && fmt[i] == b'*' {
                    p = self.printf_arg(argn) as i32 as usize;
                    argn += 1;
                    i += 1;
                } else {
                    while i < fmt.len() && fmt[i].is_ascii_digit() {
                        p = p * 10 + usize::from(fmt[i] - b'0');
                        i += 1;
                    }
                }
                precision = Some(p);
            }
            // Length modifiers (ignored).
            while i < fmt.len() && matches!(fmt[i], b'h' | b'l' | b'L' | b'z' | b'j' | b't') {
                i += 1;
            }
            if i >= fmt.len() {
                break;
            }
            let spec = fmt[i];
            i += 1;

            let body: Vec<u8> = match spec {
                b'%' => vec![b'%'],
                b'c' => {
                    let a = self.printf_arg(argn);
                    argn += 1;
                    vec![a as u8]
                }
                b's' => {
                    let a = self.printf_arg(argn);
                    argn += 1;
                    let mut s = self.read_cstr(a);
                    if let Some(p) = precision {
                        s.truncate(p);
                    }
                    s
                }
                b'd' | b'i' => {
                    let a = self.printf_arg(argn) as i32;
                    argn += 1;
                    a.to_string().into_bytes()
                }
                b'u' => {
                    let a = self.printf_arg(argn);
                    argn += 1;
                    a.to_string().into_bytes()
                }
                b'x' => {
                    let a = self.printf_arg(argn);
                    argn += 1;
                    format!("{a:x}").into_bytes()
                }
                b'X' => {
                    let a = self.printf_arg(argn);
                    argn += 1;
                    format!("{a:X}").into_bytes()
                }
                b'o' => {
                    let a = self.printf_arg(argn);
                    argn += 1;
                    format!("{a:o}").into_bytes()
                }
                b'p' => {
                    let a = self.printf_arg(argn);
                    argn += 1;
                    format!("{a:08x}").into_bytes()
                }
                other => {
                    // Unknown specifier: emit verbatim.
                    argn += 1;
                    vec![b'%', other]
                }
            };

            // Apply field width padding (zero-pad only for right-justified
            // numeric output; string/char use space padding).
            let pad_zero = zero && !left && !matches!(spec, b's' | b'c' | b'%');
            if body.len() < width {
                let pad = width - body.len();
                if left {
                    out.extend_from_slice(&body);
                    out.extend(std::iter::repeat_n(b' ', pad));
                } else {
                    out.extend(std::iter::repeat_n(if pad_zero { b'0' } else { b' ' }, pad));
                    out.extend_from_slice(&body);
                }
            } else {
                out.extend_from_slice(&body);
            }
        }
        self.tty.extend_from_slice(&out);
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

    /// `ori rt, $zero, imm`.
    fn ori(rt: u32, imm: u32) -> u32 {
        (0x0D << 26) | (rt << 16) | (imm & 0xFFFF)
    }

    /// Wraps a body of instruction words in a minimal PS-EXE image loaded at
    /// `0x8001_0000`.
    fn build_exe(body_words: &[u32]) -> Vec<u8> {
        let mut body: Vec<u8> = Vec::new();
        for &w in body_words {
            body.extend_from_slice(&w.to_le_bytes());
        }
        let padded = body.len().div_ceil(0x800) * 0x800;
        body.resize(padded, 0);
        let mut exe = vec![0u8; 0x800];
        exe[0..8].copy_from_slice(b"PS-X EXE");
        let put = |exe: &mut Vec<u8>, off: usize, val: u32| {
            exe[off..off + 4].copy_from_slice(&val.to_le_bytes());
        };
        put(&mut exe, 0x10, 0x8001_0000); // PC
        put(&mut exe, 0x18, 0x8001_0000); // t_addr
        put(&mut exe, 0x1C, body.len() as u32); // t_size
        put(&mut exe, 0x30, 0x801F_FFF0); // s_addr
        exe.extend_from_slice(&body);
        exe
    }

    #[test]
    fn syscall_exception_round_trips_and_continues() {
        // Prove the BIOS exception HLE round-trips: run EnterCriticalSection then
        // ExitCriticalSection via `syscall`, then print "OK\n" — which only
        // happens if control returned from both syscalls to the following code.
        const SYSCALL: u32 = 0x0C; // SPECIAL funct
        let program = [
            ori(4, 1),                      // $a0 = 1  (EnterCriticalSection)
            SYSCALL,                        // syscall
            ori(4, 2),                      // $a0 = 2  (ExitCriticalSection)
            SYSCALL,                        // syscall
            ori(10, 0xB0),                  // $t2 = 0xB0  (B-table)
            ori(9, 0x3D),                   // $t1 = 0x3D  (std_out_putchar)
            ori(4, 0x4F),                   // $a0 = 'O'
            (10 << 21) | (31 << 11) | 0x09, // jalr $ra, $t2
            0,                              // delay slot
            ori(4, 0x4B),                   // $a0 = 'K'
            (10 << 21) | (31 << 11) | 0x09, // jalr $ra, $t2
            0,
            ori(4, 0x0A),                   // $a0 = '\n'
            (10 << 21) | (31 << 11) | 0x09, // jalr $ra, $t2
            0,
            ori(31, 0),        // $ra = sentinel 0
            (31 << 21) | 0x08, // jr $ra  -> stops run_hle
            0,
        ];
        let mut h = Harness::new();
        h.load_exe(&build_exe(&program)).expect("load_exe");
        let steps = h.run_hle(1000);
        assert!(steps < 1000, "should return via sentinel, ran {steps}");
        assert_eq!(
            h.tty(),
            "OK\n",
            "syscall round-trip should reach the prints"
        );
        // After ExitCriticalSection the interrupt-enable bit (SR.IEc) is set.
        assert_eq!(h.core.cop0(COP0_SR) & 0x1, 0x1, "IEc should be enabled");
    }

    #[test]
    fn printf_formats_common_conversions() {
        // printf("%d %x %s %c%%", -5, 0xABC, "hi", 'Z') via A(0x3F).
        let msg_addr = 0x8001_0000u32 + 0x400; // data area inside the image
        let str_addr = msg_addr + 0x40;
        let fmt = b"[%d|%08x|%s|%c%%]\0";
        let sref = b"hi\0";

        let mut h = Harness::new();
        // Empty body; we poke the data and drive printf directly.
        h.load_exe(&build_exe(&[0])).expect("load_exe");
        {
            let mem = h.core.memory_mut();
            for (i, b) in fmt.iter().enumerate() {
                mem.write8(msg_addr + i as u32, *b);
            }
            for (i, b) in sref.iter().enumerate() {
                mem.write8(str_addr + i as u32, *b);
            }
        }
        // Stage printf args: $a0=fmt, $a1=-5, $a2=0xABC, $a3=str, and 'Z' on the
        // stack (5th vararg at sp+16).
        h.core.set_reg(4, msg_addr);
        h.core.set_reg(5, (-5i32) as u32);
        h.core.set_reg(6, 0xABC);
        h.core.set_reg(7, str_addr);
        let sp = h.core.reg(29);
        h.core.memory_mut().write8(sp + 16, b'Z');
        h.hle_printf(msg_addr);
        assert_eq!(h.tty(), "[-5|00000abc|hi|Z%]");
    }
}
