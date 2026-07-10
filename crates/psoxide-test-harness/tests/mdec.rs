//! End-to-end MDEC (macroblock decoder) integration test.
//!
//! Exercises the assembled MDEC wiring added to the core: the two 32-bit ports
//! (`MDEC0` 0x1F80_1820 command/data, `MDEC1` 0x1F80_1824 control/status)
//! routed through `CoreBus::io_*32`, plus DMA channel 0 (MDECin: RAM → decoder)
//! and channel 1 (MDECout: decoder → RAM). No BIOS or game data is needed: a
//! synthetic flat-colour macroblock is decoded and the resulting BGR555 pixels
//! are checked in main RAM.
//!
//! The macroblock is DC-only, so its decoded image is a single flat colour that
//! can be computed by hand: with quant[0] = 8 and a q_scale=1, DC=16 first
//! coefficient the dequantized DC is 16*8 = 128; the IDCT of a DC-only block is
//! flat DC/8 = 16 for every pixel; monochrome-to-RGB adds the +0x80 unsigned
//! bias giving a grey byte of 0x90 on each channel; and 15-bit packing keeps the
//! top five bits of each channel: (0x90 >> 3) & 0x1F = 0x12, so each pixel is
//! 0x12 | (0x12<<5) | (0x12<<10) = 0x4A52, i.e. output words of 0x4A524A52.

use psoxide_test_harness::Harness;

const MDEC0: u32 = 0x1F80_1820; // command / data port
const MDEC1: u32 = 0x1F80_1824; // control / status port

// DMA register file (physical), channels 0 (MDECin) and 1 (MDECout).
const DMA0_MADR: u32 = 0x1F80_1080;
const DMA0_BCR: u32 = 0x1F80_1084;
const DMA0_CHCR: u32 = 0x1F80_1088;
const DMA1_MADR: u32 = 0x1F80_1090;
const DMA1_BCR: u32 = 0x1F80_1094;
const DMA1_CHCR: u32 = 0x1F80_1098;

const IN_ADDR: u32 = 0x0000_1000; // staged decode command + data words
const OUT_ADDR: u32 = 0x0000_2000; // decoded output destination

/// The canonical PSX IDCT scale table (see `mdec.rs`): the first row is 0x5A82.
fn psx_scale_table() -> [i16; 64] {
    let mut t = [0i16; 64];
    for a in 0..8 {
        for b in 0..8 {
            let kb = if b == 0 { 1.0f64 / 2.0f64.sqrt() } else { 1.0 };
            let c = kb
                * (((2 * a + 1) * b) as f64 * std::f64::consts::PI / 16.0).cos()
                * (2.0f64 / 8.0).sqrt()
                * 65536.0;
            t[a * 8 + b] = c.round().clamp(-32768.0, 32767.0) as i16;
        }
    }
    t
}

/// Encodes one DC-only block as a single 32-bit word: `[q_scale|dc]` then a
/// run of 63 which ends the block after just the DC coefficient.
fn dc_only_word(q_scale: u32, dc: u32) -> u32 {
    let n1 = ((q_scale & 0x3F) << 10) | (dc & 0x3FF);
    let n2: u32 = 63 << 10;
    n1 | (n2 << 16)
}

#[test]
fn mdec_decodes_flat_color_macroblock_through_dma() {
    let mut h = Harness::new();

    // --- Configure the decoder via the command port -----------------------
    // Set scale (IDCT) table: command 3, then 32 words = 64 signed halfwords.
    let scale = psx_scale_table();
    h.core_mut().store32(MDEC0, 3 << 29);
    for i in 0..32 {
        let lo = scale[2 * i] as u16 as u32;
        let hi = scale[2 * i + 1] as u16 as u32;
        h.core_mut().store32(MDEC0, lo | (hi << 16));
    }
    // Set quant tables: command 2 with colour flag (32 words: luma + chroma),
    // every entry = 8.
    h.core_mut().store32(MDEC0, (2 << 29) | 1);
    for _ in 0..32 {
        h.core_mut().store32(MDEC0, 0x0808_0808);
    }

    // --- Stage the decode command + 6-block macroblock in RAM -------------
    // Colour 15-bit decode (command 1, depth=3), 6 parameter words follow.
    let cmd = (1u32 << 29) | (3 << 27) | 6;
    let cr = dc_only_word(1, 0); // Cr: DC 0
    let cb = dc_only_word(1, 0); // Cb: DC 0
    let y = dc_only_word(1, 16); // Y1..Y4: DC 16 -> flat grey
    let words = [cmd, cr, cb, y, y, y, y];
    for (i, w) in words.iter().enumerate() {
        h.core_mut().store32(IN_ADDR + (i as u32) * 4, *w);
    }

    // --- DMA0 (MDECin): feed the 7 words RAM -> decoder --------------------
    h.core_mut().store32(DMA0_MADR, IN_ADDR);
    h.core_mut().store32(DMA0_BCR, 7); // 7 words, one block
    // enable | direction RAM->device (bit0) | sync mode 1 (bit9)
    h.core_mut().store32(DMA0_CHCR, (1 << 24) | 0x1 | (1 << 9));

    // Status: decode complete, output FIFO non-empty (bit31 clear).
    let status = h.core_mut().load32(MDEC1);
    assert_eq!(status & (1 << 31), 0, "output FIFO should be non-empty");

    // --- DMA1 (MDECout): drain 128 output words decoder -> RAM -------------
    h.core_mut().store32(DMA1_MADR, OUT_ADDR);
    h.core_mut().store32(DMA1_BCR, 128); // 15-bit colour macroblock = 128 words
    // enable | direction device->RAM (bit0 clear) | sync mode 1 (bit9)
    h.core_mut().store32(DMA1_CHCR, (1 << 24) | (1 << 9));

    // --- Verify the decoded flat 15-bit image in RAM ----------------------
    let expected: u32 = 0x4A52_4A52;
    for i in 0..128u32 {
        let w = h.core_mut().load32(OUT_ADDR + i * 4);
        assert_eq!(w, expected, "decoded output word {i} mismatch");
    }

    // Output FIFO fully drained, controller back to idle. The latched output
    // depth (3 = 15-bit) persists in the status register after the command, so
    // status is 0x8004_0000 with the depth field (bits 26-25 = 3) set.
    assert_eq!(
        h.core_mut().load32(MDEC1),
        0x8604_0000,
        "MDEC idle: out-FIFO empty (bit31), current block 4, depth 3 latched"
    );
}

#[test]
fn mdec_state_survives_snapshot_round_trip() {
    let mut h = Harness::new();
    // Put the MDEC into a non-default state: load a scale table and enable the
    // data-in DMA request.
    let scale = psx_scale_table();
    h.core_mut().store32(MDEC0, 3 << 29);
    for i in 0..32 {
        let lo = scale[2 * i] as u16 as u32;
        let hi = scale[2 * i + 1] as u16 as u32;
        h.core_mut().store32(MDEC0, lo | (hi << 16));
    }
    h.core_mut().store32(MDEC1, 1 << 30); // enable DMA-in request

    let snap = h.core_mut().save_state();
    let json = serde_json::to_string(&snap).unwrap();
    let back: psoxide_core::CoreSnapshot = serde_json::from_str(&json).unwrap();
    assert_eq!(snap, back);

    // Loading the snapshot into a fresh core preserves the enabled request bit.
    let mut other = psoxide_core::PsxCore::new();
    other.load_state(&back);
    assert_ne!(
        other.load32(MDEC1) & (1 << 28),
        0,
        "data-in request preserved"
    );
}
