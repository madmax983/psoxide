//! End-to-end CPU program tests driven through the public `PsxCore` API.

use psoxide_test_harness::Harness;

/// Assembles an I-type instruction word.
fn i_type(op: u32, rs: u32, rt: u32, imm: u16) -> u32 {
    (op << 26) | (rs << 21) | (rt << 16) | u32::from(imm)
}
/// Assembles an R-type (SPECIAL) instruction word.
fn r_type(rs: u32, rt: u32, rd: u32, shamt: u32, funct: u32) -> u32 {
    (rs << 21) | (rt << 16) | (rd << 11) | (shamt << 6) | funct
}

#[test]
fn sum_1_to_5_with_a_loop() {
    // Compute 1+2+3+4+5 = 15 in $s0 using a countdown loop in $t0.
    //
    // 0x00: addiu $s0,$zero,0      ; sum = 0
    // 0x04: addiu $t0,$zero,5      ; i = 5
    // 0x08: beq   $t0,$zero,+3     ; if i == 0 goto done (0x18)
    // 0x0C: addu  $s0,$s0,$t0      ; delay slot: sum += i
    // 0x10: beq   $zero,$zero,-3   ; goto loop (0x08)
    // 0x14: addiu $t0,$t0,-1       ; delay slot: i -= 1
    // 0x18: <done>  addiu $s0,$s0,0 (nop-ish landing pad)
    let program = [
        i_type(0x09, 0, 16, 0),     // 0x00 addiu $s0,$zero,0
        i_type(0x09, 0, 8, 5),      // 0x04 addiu $t0,$zero,5
        i_type(0x04, 8, 0, 3),      // 0x08 beq $t0,$zero,done
        r_type(16, 8, 16, 0, 0x21), // 0x0C addu $s0,$s0,$t0 (delay)
        i_type(0x04, 0, 0, 0xFFFD), // 0x10 beq $zero,$zero,loop (-3)
        i_type(0x09, 8, 8, 0xFFFF), // 0x14 addiu $t0,$t0,-1 (delay)
        i_type(0x09, 16, 16, 0),    // 0x18 done: addiu $s0,$s0,0
    ];

    let mut h = Harness::new();
    h.load_program(&program);
    // Plenty of instructions to converge the loop and land on `done`.
    h.run(40);
    assert_eq!(h.reg(16), 15, "sum 1..=5 should be 15");
}

#[test]
fn store_then_load_round_trips_through_ram() {
    // $t0 = 0xABCD_0001 ; store at 0x1000 ; load into $t2.
    let program = [
        i_type(0x0F, 0, 8, 0xABCD), // lui $t0,0xABCD
        i_type(0x0D, 8, 8, 0x0001), // ori $t0,$t0,1
        i_type(0x09, 0, 9, 0x1000), // addiu $t1,$zero,0x1000
        i_type(0x2B, 9, 8, 0),      // sw $t0,0($t1)
        i_type(0x23, 9, 10, 0),     // lw $t2,0($t1)
        i_type(0x09, 0, 0, 0),      // nop (commit load delay)
    ];

    let mut h = Harness::new();
    h.load_program(&program);
    h.run(6);
    assert_eq!(h.reg(10), 0xABCD_0001);
    assert_eq!(h.read_word(0x1000), 0xABCD_0001);
}
