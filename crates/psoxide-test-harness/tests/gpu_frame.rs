//! Golden-frame foundation tests for the GPU.
//!
//! These drive [`PsxCore`] directly (no external ROM) to prove that a GP0 fill
//! becomes visible through `framebuffer_rgba`, and that a GPU DMA linked list
//! also reaches VRAM. Deterministic and committed.

use psoxide_core::{Command, PsxCore};

/// Packs 8-bit R/G/B into the framebuffer's expected RGBA bytes (with the 3-bit
/// truncation the 15bpp path introduces).
fn expected_rgba(r: u8, g: u8, b: u8) -> [u8; 4] {
    [r & 0xF8, g & 0xF8, b & 0xF8, 0xFF]
}

#[test]
fn gp0_fill_is_visible_in_framebuffer() {
    let mut core = PsxCore::new();
    let gpu = core.gpu_mut();

    // Power-on reset, display from VRAM (0,0), 15bpp.
    gpu.gp1(0x0000_0000); // reset
    gpu.gp1(0x0500_0000); // display area start (0,0)
    gpu.gp1(0x0800_0000); // display mode: 15bpp, NTSC
    gpu.gp1(0x0300_0000); // display enable

    // Fill a red 16x16 block at VRAM (0,0) via GP0.
    gpu.gp0(0x0200_00FF); // fill, color = red (0xFF)
    gpu.gp0(0x0000_0000); // (0,0)
    gpu.gp0(0x0010_0010); // 16x16

    let frame = core.framebuffer_rgba();
    assert_eq!(frame.len(), 320 * 240 * 4);
    // Top-left pixel should be red.
    assert_eq!(&frame[0..4], &expected_rgba(0xFF, 0, 0));
    // A pixel outside the filled block is still clear (black).
    let i = 100 * 4;
    assert_eq!(&frame[i..i + 4], &[0, 0, 0, 0xFF]);
}

#[test]
fn textured_quad_is_visible_in_framebuffer() {
    let mut core = PsxCore::new();
    let gpu = core.gpu_mut();
    gpu.gp1(0x0000_0000); // reset
    gpu.gp1(0x0500_0000); // display area (0,0)
    gpu.gp1(0x0800_0000); // 15bpp NTSC
    gpu.gp1(0x0300_0000); // display enable

    // Drawing area covers the whole visible frame.
    gpu.gp0(0xE300_0000);
    gpu.gp0(0xE400_0000 | 319u32 | (239u32 << 10));

    // A 16×16 solid-green (BGR555 0x03E0) texture at off-screen page (0,256).
    for u in 0..16u16 {
        for v in 0..16u16 {
            gpu.set_vram(u, 256 + v, 0x03E0);
        }
    }

    // Raw textured flat quad (opcode 0x2D) covering (0,0)-(16,16). The texpage
    // (15bpp, page_y=256 → tp 0x0110) rides in vertex 1's texcoord word.
    for w in [
        0x2D00_0000u32, // raw textured quad
        0x0000_0000,    // v0 (0,0)
        0x0000_0000,    // uv0 (0,0), clut unused
        0x0000_0010,    // v1 (16,0)
        0x0110_000F,    // texpage 0x0110 + uv1 (15,0)
        0x0010_0010,    // v2 (16,16)
        0x0000_0F0F,    // uv2 (15,15)
        0x0010_0000,    // v3 (0,16)
        0x0000_0F00,    // uv3 (0,15)
    ] {
        gpu.gp0(w);
    }

    let frame = core.framebuffer_rgba();
    // An interior pixel of the quad samples the green texel.
    let i = (4 * 320 + 4) * 4;
    assert_eq!(&frame[i..i + 4], &expected_rgba(0, 0xFF, 0));
}

#[test]
fn gouraud_line_is_visible_and_interpolated_in_framebuffer() {
    let mut core = PsxCore::new();
    let gpu = core.gpu_mut();
    gpu.gp1(0x0000_0000); // reset
    gpu.gp1(0x0500_0000); // display area (0,0)
    gpu.gp1(0x0800_0000); // 15bpp NTSC
    gpu.gp1(0x0300_0000); // display enable

    // Drawing area covers the whole visible frame.
    gpu.gp0(0xE300_0000);
    gpu.gp0(0xE400_0000 | 319u32 | (239u32 << 10));

    // Gouraud line (opcode 0x50) red at (0,10) → blue at (100,10).
    gpu.gp0(0x5000_00FF); // shaded line, colour0 = red
    gpu.gp0(0x000A_0000); // v0 (0,10)
    gpu.gp0(0x00FF_0000); // colour1 = blue
    gpu.gp0(0x000A_0064); // v1 (100,10)

    let frame = core.framebuffer_rgba();
    // Endpoints match their vertex colours.
    let v0 = (10 * 320) * 4;
    assert_eq!(&frame[v0..v0 + 4], &expected_rgba(0xFF, 0, 0), "v0 red");
    let v1 = (10 * 320 + 100) * 4;
    assert_eq!(&frame[v1..v1 + 4], &expected_rgba(0, 0, 0xFF), "v1 blue");
    // The midpoint (50,10) is a red+blue blend — both channels present.
    let mid = (10 * 320 + 50) * 4;
    assert!(frame[mid] > 0, "midpoint has red");
    assert!(frame[mid + 2] > 0, "midpoint has blue");
    assert_ne!(
        &frame[mid..mid + 4],
        &expected_rgba(0xFF, 0, 0),
        "midpoint is not pure red"
    );
}

#[test]
fn gpu_dma_linked_list_fills_visible_region() {
    let mut core = PsxCore::new();
    core.gpu_mut().gp1(0x0000_0000);
    core.gpu_mut().gp1(0x0500_0000);
    core.gpu_mut().gp1(0x0800_0000);
    core.gpu_mut().gp1(0x0300_0000);

    // Build a one-node ordering-table entry in RAM at 0x1000: a fill command.
    let mem = core.memory_mut();
    let put = |mem: &mut psoxide_core::Memory, addr: u32, word: u32| {
        for (i, b) in word.to_le_bytes().iter().enumerate() {
            mem.write8(addr + i as u32, *b);
        }
    };
    put(mem, 0x1000, (3 << 24) | 0x00FF_FFFF); // header: 3 words, end marker
    put(mem, 0x1004, 0x0200_00FF); // fill red
    put(mem, 0x1008, 0x0020_0000); // (0,32)
    put(mem, 0x100C, 0x0010_0010); // 16x16

    // Program GPU DMA (channel 2) to walk the linked list by executing a small
    // staged CPU program that writes the DMA registers through the bus.
    stage_dma_kick(&mut core, 0x1000);
    core.execute(Command::StepFrame).ok();

    let frame = core.framebuffer_rgba();
    // The fill at VRAM (0,32) maps to framebuffer row 32.
    let i = (32 * 320) * 4;
    assert_eq!(&frame[i..i + 4], &expected_rgba(0xFF, 0, 0));
}

/// Stages a small MIPS program in RAM that programs GPU DMA channel 2 to walk a
/// linked list at `list_addr`, then points the CPU at it and runs a few steps.
fn stage_dma_kick(core: &mut PsxCore, list_addr: u32) {
    // Registers: DMA ch2 MADR=0x1F80_10A0, CHCR=0x1F80_10A8.
    // We assemble:
    //   lui  $t0, 0x1F80
    //   ori  $t0, $t0, 0x10A0
    //   lui  $t1, (list_addr>>16)
    //   ori  $t1, $t1, (list_addr&0xFFFF)
    //   sw   $t1, 0($t0)          ; MADR = list_addr
    //   lui  $t2, 0x0000
    //   ori  $t2, $t2, 0x0401     ; CHCR = enable|dir|mode2 => 0x0100_0401? build below
    // Simpler: we build CHCR = (1<<24)|1|(2<<9) = 0x0100_0401.
    let chcr = (1u32 << 24) | 0x1 | (2 << 9);
    let prog: [u32; 9] = [
        i_lui(8, 0x1F80),                         // lui $t0,0x1F80
        i_ori(8, 8, 0x10A0),                      // ori $t0,$t0,0x10A0
        i_lui(9, (list_addr >> 16) as u16),       // lui $t1, hi(list)
        i_ori(9, 9, (list_addr & 0xFFFF) as u16), // ori $t1,$t1, lo(list)
        i_sw(8, 9, 0x00),                         // sw $t1,0($t0)  -> MADR
        i_lui(10, (chcr >> 16) as u16),           // lui $t2, hi(chcr)
        i_ori(10, 10, (chcr & 0xFFFF) as u16),    // ori $t2,$t2, lo(chcr)
        i_sw(8, 10, 0x08),                        // sw $t2,8($t0)  -> CHCR (kicks DMA)
        0x0000_0000,                              // nop
    ];
    let mem = core.memory_mut();
    for (i, w) in prog.iter().enumerate() {
        for (b, byte) in w.to_le_bytes().iter().enumerate() {
            mem.write8(0x0000_2000 + (i as u32) * 4 + b as u32, *byte);
        }
    }
    core.set_pc(0x0000_2000);
    for _ in 0..12 {
        let _ = core.execute(Command::StepCpu);
    }
}

fn i_lui(rt: u32, imm: u16) -> u32 {
    (0x0F << 26) | (rt << 16) | u32::from(imm)
}
fn i_ori(rt: u32, rs: u32, imm: u16) -> u32 {
    (0x0D << 26) | (rs << 21) | (rt << 16) | u32::from(imm)
}
fn i_sw(rs: u32, rt: u32, imm: u16) -> u32 {
    (0x2B << 26) | (rs << 21) | (rt << 16) | u32::from(imm)
}
