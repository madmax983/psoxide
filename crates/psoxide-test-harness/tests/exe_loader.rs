//! PS-EXE side-loader + BIOS TTY HLE tests.

use psoxide_test_harness::Harness;

/// `ori rt, $zero, imm` (opcode 0x0D).
fn ori(rt: u32, imm: u32) -> u32 {
    (0x0D << 26) | (rt << 16) | (imm & 0xFFFF)
}
/// `jalr $ra, rs` (SPECIAL, funct 0x09).
fn jalr(rs: u32) -> u32 {
    (rs << 21) | (31 << 11) | 0x09
}
/// `jr rs` (SPECIAL, funct 0x08).
fn jr(rs: u32) -> u32 {
    (rs << 21) | 0x08
}

/// Builds a minimal PS-EXE image from a body of instruction words.
fn build_exe(pc: u32, t_addr: u32, s_addr: u32, body_words: &[u32]) -> Vec<u8> {
    let mut body: Vec<u8> = Vec::new();
    for &w in body_words {
        body.extend_from_slice(&w.to_le_bytes());
    }
    // Pad body to a multiple of 0x800.
    let padded = body.len().div_ceil(0x800) * 0x800;
    body.resize(padded, 0);

    let mut exe = vec![0u8; 0x800];
    exe[0..8].copy_from_slice(b"PS-X EXE");
    let put = |exe: &mut Vec<u8>, off: usize, val: u32| {
        exe[off..off + 4].copy_from_slice(&val.to_le_bytes());
    };
    put(&mut exe, 0x10, pc); // initial PC
    put(&mut exe, 0x14, 0); // initial GP
    put(&mut exe, 0x18, t_addr); // t_addr (dest)
    put(&mut exe, 0x1C, body.len() as u32); // t_size (padded body length)
    put(&mut exe, 0x30, s_addr); // s_addr
    put(&mut exe, 0x34, 0); // s_offset

    exe.extend_from_slice(&body);
    exe
}

#[test]
fn synthetic_exe_prints_ok_via_hle() {
    // $t1 = 9 (func), $t2 = 10 (B-table addr), $a0 = 4 (char), $ra = 31.
    let program = [
        ori(10, 0xB0), // ori $t2, $0, 0xB0   -> B-table
        ori(9, 0x3D),  // ori $t1, $0, 0x3D   -> std_out_putchar
        ori(4, 0x4F),  // ori $a0, $0, 'O'
        jalr(10),      // jalr $ra, $t2
        0,             // nop (delay slot)
        ori(4, 0x4B),  // ori $a0, $0, 'K'
        jalr(10),      // jalr $ra, $t2
        0,             // nop
        ori(4, 0x0A),  // ori $a0, $0, '\n'
        jalr(10),      // jalr $ra, $t2
        0,             // nop
        // The jalr calls above clobber $ra, so restore the sentinel before
        // returning: $ra = 0 -> jr $ra lands on HLE_RETURN_ADDR and stops.
        ori(31, 0x00), // ori $ra, $0, 0
        jr(31),        // jr $ra  ($ra = sentinel 0 -> stop)
        0,             // nop
    ];

    let mut h = Harness::new();
    let exe = build_exe(0x8001_0000, 0x8001_0000, 0x801F_FFF0, &program);
    h.load_exe(&exe).expect("load_exe should succeed");
    let steps = h.run_hle(1000);
    assert!(
        steps < 1000,
        "program should return via sentinel, ran {steps} steps"
    );
    assert_eq!(h.tty(), "OK\n", "captured TTY should be \"OK\\n\"");
}

#[test]
fn load_exe_rejects_bad_magic() {
    let mut h = Harness::new();
    let mut exe = vec![0u8; 0x800];
    exe[0..8].copy_from_slice(b"NOT-EXE!");
    assert!(h.load_exe(&exe).is_err());
}

#[test]
fn load_exe_rejects_too_small() {
    let mut h = Harness::new();
    let exe = vec![0u8; 0x100];
    assert!(h.load_exe(&exe).is_err());
}

/// Ad-hoc driver for running real PS-EXE test suites. Reads a path from the
/// `PSOXIDE_EXE` env var, side-loads it, runs with a large step budget, and
/// prints the captured TTY plus a step count. Ignored by default.
#[test]
#[ignore]
fn run_real_suite() {
    let path = std::env::var("PSOXIDE_EXE").expect("set PSOXIDE_EXE to the .exe path");
    let bytes = std::fs::read(&path).expect("read exe file");
    let mut h = Harness::new();
    h.load_exe(&bytes).expect("load_exe");
    let budget: usize = std::env::var("PSOXIDE_STEPS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(50_000_000);
    let steps = h.run_hle(budget);
    let terminated = steps < budget;
    eprintln!("=== PSOXIDE_EXE: {path} ===");
    eprintln!(
        "steps={steps} budget={budget} terminated_via_sentinel={terminated} final_pc={:#010x} tty_len={}",
        h.registers().pc,
        h.tty_bytes().len()
    );
    if let Ok(out) = std::env::var("PSOXIDE_OUT") {
        std::fs::write(&out, h.tty_bytes()).expect("write PSOXIDE_OUT");
        eprintln!("wrote TTY ({} bytes) to {out}", h.tty_bytes().len());
    }
    eprintln!("=== TTY BEGIN ===");
    print!("{}", h.tty());
    eprintln!("\n=== TTY END ===");
}
