//! Macroblock Decoder (MDEC).
//!
//! The MDEC is the PlayStation's JPEG-like still-image / FMV decompressor. It
//! turns a run-length-encoded, quantized, DCT-domain macroblock stream (as
//! produced by the `.STR`/`.BS` movie format) into raw RGB/YUV pixel data. It
//! is not a bus master itself: games feed it commands + compressed data through
//! DMA channel 0 (MDECin) and read decoded pixels back through DMA channel 1
//! (MDECout), or poll the two 32-bit I/O ports directly.
//!
//! * **MDEC0 (`0x1F80_1820`)** — write: command / parameter FIFO; read: decoded
//!   output-data FIFO.
//! * **MDEC1 (`0x1F80_1824`)** — write: control register (reset + DMA-request
//!   enables); read: status register.
//!
//! Because psoxide runs DMA blocks synchronously to completion (see `dma.rs`),
//! this controller decodes *eagerly*: parameter words are buffered until a
//! command's full parameter count has arrived, at which point the whole
//! macroblock stream is decoded and the packed output words are queued in the
//! output FIFO for MDECout / MDEC0 reads to drain. There is no per-cycle tick
//! and no dedicated interrupt line — completion is signalled by the DMA channel
//! finishing (`Dma::raise_completion` → `IrqLine::Dma`).
//!
//! ## Decode pipeline (per PSX-SPX "Macroblock Decoder (MDEC)", cross-checked
//! against DuckStation `mdec.cpp` and mednafen)
//!
//! 1. **RLE + dequantization** ([`decode_rle_block`]): the first halfword of a
//!    block carries a 6-bit quant scale and a signed-10-bit DC coefficient;
//!    following halfwords carry a 6-bit zero run-length and a signed-10-bit AC
//!    level. DC = `signed10(dc) * quant[0]`; AC =
//!    `(signed10(level) * quant[k] * q_scale + 4) / 8`; both clamped to
//!    `[-0x400, 0x3FF]`, and placed into the 8x8 block via the zig-zag order.
//!    A `q_scale` of 0 selects the raw `signed10(x) * 2` path. `0xFE00` is
//!    end-of-block padding; a run that pushes the coefficient index past 63
//!    ends the block.
//! 2. **IDCT** ([`Mdec::idct`]): a separable integer 2-D inverse DCT driven by
//!    the guest-loaded 64-entry scale table, with a `>> 32` (rounded) final
//!    normalization and a clamp to `[-128, 127]`.
//! 3. **Colour reconstruction**: a colour macroblock is six blocks in the order
//!    Cr, Cb, Y1, Y2, Y3, Y4 assembled into a 16x16 image (the chroma planes
//!    are 2x subsampled); a monochrome (4/8-bit) macroblock is a single 8x8 Y
//!    block. YUV→RGB uses the BT.601 integer coefficients.
//! 4. **Output packing**: 4-bit (2 mono px/byte), 8-bit (1 mono px/byte),
//!    15-bit (BGR555, two px/word, optional bit15), or 24-bit (RGB888 packed
//!    tightly across words). The signed-output bit selects a ±0 vs +0x80 bias.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

/// First MDEC I/O port (`MDEC0`: command / data).
pub const MDEC_BASE: u32 = 0x1F80_1820;
/// Last byte of the MDEC I/O window (`MDEC1`: control / status at `0x1824`).
pub const MDEC_END: u32 = 0x1F80_1827;

/// Zig-zag scan order (`zagzig`): maps the linear coefficient index in the RLE
/// stream to its position in the row-major 8x8 block. Verbatim from PSX-SPX /
/// DuckStation `mdec.cpp`.
const ZAGZIG: [usize; 64] = [
    0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5, 12, 19, 26, 33, 40, 48, 41, 34, 27, 20,
    13, 6, 7, 14, 21, 28, 35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51, 58, 59,
    52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
];

/// Command decoded from the first word written to MDEC0. Held while the
/// controller collects the remaining parameter words.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
enum Pending {
    /// Not collecting parameters; the next MDEC0 write is a command word.
    Idle,
    /// Decode-macroblock (command 1): collecting compressed data words.
    Decode,
    /// Set-quant-table (command 2): collecting 16 (luma) or 32 (luma+chroma) words.
    SetQuant {
        /// Whether the chroma table follows the luma table.
        color: bool,
    },
    /// Set-scale-table (command 3): collecting 32 words (64 signed halfwords).
    SetScale,
}

/// Sign-extends the low `bits` of `value` to a full [`i32`].
#[inline]
fn sign_extend(value: i32, bits: u32) -> i32 {
    let shift = 32 - bits;
    (value << shift) >> shift
}

/// The Macroblock Decoder controller.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Mdec {
    /// Luma inverse-quant table (64 bytes).
    iq_y: Vec<u8>,
    /// Chroma inverse-quant table (64 bytes).
    iq_uv: Vec<u8>,
    /// IDCT scale table (64 signed 16-bit entries).
    scale_table: Vec<i16>,

    /// Command currently collecting parameter words.
    pending: Pending,
    /// Parameter words still expected before the pending command runs.
    words_remaining: u32,
    /// Accumulated parameter words for the pending command.
    params: Vec<u32>,

    /// Latched output depth (0 = 4-bit, 1 = 8-bit, 2 = 24-bit, 3 = 15-bit).
    output_depth: u8,
    /// Latched signed-output flag.
    output_signed: bool,
    /// Latched bit15 value for 15-bit output.
    output_bit15: bool,

    /// Decoded output words awaiting readout (front = next).
    out_fifo: VecDeque<u32>,

    /// DMA0 (data-in) request enable.
    enable_dma_in: bool,
    /// DMA1 (data-out) request enable.
    enable_dma_out: bool,
}

impl Default for Mdec {
    fn default() -> Self {
        Self::new()
    }
}

impl Mdec {
    /// Creates a controller in the power-on / post-reset state (status
    /// `0x8004_0000`).
    #[must_use]
    pub fn new() -> Self {
        Self {
            iq_y: vec![0u8; 64],
            iq_uv: vec![0u8; 64],
            scale_table: vec![0i16; 64],
            pending: Pending::Idle,
            words_remaining: 0,
            params: Vec::new(),
            output_depth: 0,
            output_signed: false,
            output_bit15: false,
            out_fifo: VecDeque::new(),
            enable_dma_in: false,
            enable_dma_out: false,
        }
    }

    /// Returns `true` if `phys` falls in the MDEC I/O window.
    #[must_use]
    pub fn contains(phys: u32) -> bool {
        (MDEC_BASE..=MDEC_END).contains(&phys)
    }

    /// Reads one of the two 32-bit MDEC ports.
    pub fn read32(&mut self, phys: u32) -> u32 {
        match phys & !0x3 {
            MDEC_BASE => self.read_data_word(),
            _ => self.status(),
        }
    }

    /// Writes one of the two 32-bit MDEC ports.
    pub fn write32(&mut self, phys: u32, val: u32) {
        match phys & !0x3 {
            MDEC_BASE => self.write_command_word(val),
            _ => self.write_control(val),
        }
    }

    /// The MDEC status register (`MDEC1` read).
    #[must_use]
    pub fn status(&self) -> u32 {
        let mut s = 0u32;
        // bit31: data-out FIFO empty.
        if self.out_fifo.is_empty() {
            s |= 1 << 31;
        }
        // bit30: data-in FIFO full — the eager model never back-pressures.
        // bit29: command busy.
        if self.pending != Pending::Idle {
            s |= 1 << 29;
        }
        // bit28: data-in request (we always accept while DMA-in is enabled).
        if self.enable_dma_in {
            s |= 1 << 28;
        }
        // bit27: data-out request.
        if self.enable_dma_out && !self.out_fifo.is_empty() {
            s |= 1 << 27;
        }
        // bits26-25: output depth.
        s |= (u32::from(self.output_depth) & 0x3) << 25;
        // bit24: signed output.
        if self.output_signed {
            s |= 1 << 24;
        }
        // bit23: bit15 value.
        if self.output_bit15 {
            s |= 1 << 23;
        }
        // bits18-16: current output block. In the eager model a decode
        // completes instantly, so this reads the idle value (Cr index + 4).
        s |= 4 << 16;
        // bits15-0: parameter words remaining minus 1 (idle reads 0, matching
        // the documented reset value 0x8004_0000).
        if self.pending != Pending::Idle {
            s |= self.words_remaining.wrapping_sub(1) & 0xFFFF;
        }
        s
    }

    /// Handles a write to the MDEC1 control register.
    fn write_control(&mut self, val: u32) {
        if val & (1 << 31) != 0 {
            self.reset();
            return;
        }
        self.enable_dma_in = val & (1 << 30) != 0;
        self.enable_dma_out = val & (1 << 29) != 0;
    }

    /// Aborts any in-flight command and returns to the idle / reset state.
    fn reset(&mut self) {
        self.pending = Pending::Idle;
        self.words_remaining = 0;
        self.params.clear();
        self.out_fifo.clear();
        self.output_depth = 0;
        self.output_signed = false;
        self.output_bit15 = false;
        self.enable_dma_in = false;
        self.enable_dma_out = false;
    }

    /// Writes a command / parameter word to MDEC0 (also the DMA0 sink).
    pub fn write_command_word(&mut self, val: u32) {
        if self.pending == Pending::Idle {
            self.begin_command(val);
            return;
        }
        self.params.push(val);
        self.words_remaining = self.words_remaining.saturating_sub(1);
        if self.words_remaining == 0 {
            self.execute_command();
        }
    }

    /// Reads a decoded word from MDEC0 (also the DMA1 source). Returns
    /// `0xFFFF_FFFF` when the output FIFO is empty, matching hardware open-bus.
    pub fn read_data_word(&mut self) -> u32 {
        self.out_fifo.pop_front().unwrap_or(0xFFFF_FFFF)
    }

    /// Decodes the command word (first word of a new command).
    fn begin_command(&mut self, val: u32) {
        let cmd = (val >> 29) & 0x7;
        self.params.clear();
        match cmd {
            1 => {
                // Decode macroblock. Latch the output format from the command
                // word and collect `parameter_word_count` data words.
                self.output_depth = ((val >> 27) & 0x3) as u8;
                self.output_signed = (val >> 26) & 0x1 != 0;
                self.output_bit15 = (val >> 25) & 0x1 != 0;
                let count = val & 0xFFFF;
                if count == 0 {
                    self.pending = Pending::Idle;
                    return;
                }
                self.words_remaining = count;
                self.pending = Pending::Decode;
            }
            2 => {
                // Set quant table(s): bit0 selects luma-only (16 words) vs
                // luma+chroma (32 words).
                let color = val & 0x1 != 0;
                self.words_remaining = if color { 32 } else { 16 };
                self.pending = Pending::SetQuant { color };
            }
            3 => {
                // Set scale (IDCT) table: 32 words = 64 signed halfwords.
                self.words_remaining = 32;
                self.pending = Pending::SetScale;
            }
            _ => {
                // Commands 0/4..7 are no-ops.
                self.pending = Pending::Idle;
            }
        }
    }

    /// Runs the pending command now that all its parameter words have arrived.
    fn execute_command(&mut self) {
        match self.pending {
            Pending::SetQuant { color } => self.load_quant(color),
            Pending::SetScale => self.load_scale(),
            Pending::Decode => self.run_decode(),
            Pending::Idle => {}
        }
        self.pending = Pending::Idle;
        self.params.clear();
    }

    /// Loads the luma (and optionally chroma) inverse-quant table.
    fn load_quant(&mut self, color: bool) {
        let bytes: Vec<u8> = self.params.iter().flat_map(|w| w.to_le_bytes()).collect();
        if bytes.len() >= 64 {
            self.iq_y.copy_from_slice(&bytes[0..64]);
        }
        if color && bytes.len() >= 128 {
            self.iq_uv.copy_from_slice(&bytes[64..128]);
        }
    }

    /// Loads the 64-entry signed IDCT scale table.
    fn load_scale(&mut self) {
        for (i, w) in self.params.iter().enumerate() {
            if 2 * i + 1 < self.scale_table.len() {
                self.scale_table[2 * i] = (*w as u16) as i16;
                self.scale_table[2 * i + 1] = ((*w >> 16) as u16) as i16;
            }
        }
    }

    /// Decodes the buffered macroblock stream into packed output words.
    fn run_decode(&mut self) {
        // Unpack parameter words to the halfword coefficient stream.
        let mut hw: Vec<u16> = Vec::with_capacity(self.params.len() * 2);
        for w in &self.params {
            hw.push(*w as u16);
            hw.push((*w >> 16) as u16);
        }
        let mut pos = 0usize;

        if self.output_depth <= 1 {
            // Monochrome: a stream of single 8x8 Y blocks.
            let iq_y = self.iq_y.clone();
            while let Some(mut blk) = decode_rle_block(&hw, &mut pos, &iq_y) {
                self.idct(&mut blk);
                let rgb = self.yuv_to_mono(&blk);
                self.pack_output(&rgb);
            }
        } else {
            // Colour: repeated 6-block macroblocks (Cr, Cb, Y1..Y4).
            let iq_y = self.iq_y.clone();
            let iq_uv = self.iq_uv.clone();
            while let Some(mut cr) = decode_rle_block(&hw, &mut pos, &iq_uv) {
                let Some(mut cb) = decode_rle_block(&hw, &mut pos, &iq_uv) else {
                    break;
                };
                let mut ys = [[0i16; 64]; 4];
                let mut ok = true;
                for y in &mut ys {
                    if let Some(blk) = decode_rle_block(&hw, &mut pos, &iq_y) {
                        *y = blk;
                    } else {
                        ok = false;
                        break;
                    }
                }
                if !ok {
                    break;
                }
                self.idct(&mut cr);
                self.idct(&mut cb);
                for y in &mut ys {
                    self.idct(y);
                }
                let mut rgb = vec![0u32; 256];
                self.yuv_to_rgb(&mut rgb, 0, 0, &cr, &cb, &ys[0]);
                self.yuv_to_rgb(&mut rgb, 8, 0, &cr, &cb, &ys[1]);
                self.yuv_to_rgb(&mut rgb, 0, 8, &cr, &cb, &ys[2]);
                self.yuv_to_rgb(&mut rgb, 8, 8, &cr, &cb, &ys[3]);
                self.pack_output(&rgb);
            }
        }
    }

    /// The separable integer 2-D IDCT driven by the loaded scale table. Matches
    /// the PSX-SPX / DuckStation reference (`IDCT_Old`): two matrix passes with
    /// a `>> 32` rounded normalization and a `[-128, 127]` clamp.
    fn idct(&self, blk: &mut [i16; 64]) {
        let s = &self.scale_table;
        let mut temp = [0i64; 64];
        for x in 0..8 {
            for y in 0..8 {
                let mut sum: i64 = 0;
                for u in 0..8 {
                    sum += i64::from(blk[u * 8 + x]) * i64::from(s[y * 8 + u]);
                }
                temp[x + y * 8] = sum;
            }
        }
        for x in 0..8 {
            for y in 0..8 {
                let mut sum: i64 = 0;
                for u in 0..8 {
                    sum += temp[u + y * 8] * i64::from(s[x * 8 + u]);
                }
                let rounded = ((sum >> 32) + ((sum >> 31) & 1)) as i32;
                let v = sign_extend(rounded, 9).clamp(-128, 127);
                blk[x + y * 8] = v as i16;
            }
        }
    }

    /// Monochrome YUV→pixel: clamp Y to `[-128, 127]` and apply the signed /
    /// unsigned bias. Returns 64 packed pixel bytes (in the low byte of each
    /// element).
    fn yuv_to_mono(&self, yblk: &[i16; 64]) -> Vec<u32> {
        let addval: i32 = if self.output_signed { 0 } else { 0x80 };
        let mut out = vec![0u32; 64];
        for i in 0..64 {
            let v = sign_extend(i32::from(yblk[i]), 9).clamp(-128, 127) + addval;
            out[i] = (v as u32) & 0xFF;
        }
        out
    }

    /// Colour YUV→RGB for one 8x8 quadrant at `(xx, yy)` of the 16x16 output,
    /// using BT.601 integer coefficients (mednafen rounding, per DuckStation
    /// `YUVToRGB_New`). Chroma is 2x subsampled. Writes `R | G<<8 | B<<16` (each
    /// channel already biased and masked to a byte) into `rgb`.
    fn yuv_to_rgb(
        &self,
        rgb: &mut [u32],
        xx: usize,
        yy: usize,
        crblk: &[i16; 64],
        cbblk: &[i16; 64],
        yblk: &[i16; 64],
    ) {
        let addval: i32 = if self.output_signed { 0 } else { 0x80 };
        for y in 0..8 {
            for x in 0..8 {
                let ci = ((x + xx) / 2) + ((y + yy) / 2) * 8;
                let cr = i32::from(crblk[ci]);
                let cb = i32::from(cbblk[ci]);
                let yv = i32::from(yblk[x + y * 8]);

                let r_off = ((359 * cr) + 0x80) >> 8;
                let g_off = (((-88 * cb) & !0x1F) + ((-183 * cr) & !0x07) + 0x80) >> 8;
                let b_off = ((454 * cb) + 0x80) >> 8;

                let r = sign_extend(yv + r_off, 9).clamp(-128, 127) + addval;
                let g = sign_extend(yv + g_off, 9).clamp(-128, 127) + addval;
                let b = sign_extend(yv + b_off, 9).clamp(-128, 127) + addval;

                let idx = (x + xx) + (y + yy) * 16;
                rgb[idx] =
                    ((r as u32) & 0xFF) | (((g as u32) & 0xFF) << 8) | (((b as u32) & 0xFF) << 16);
            }
        }
    }

    /// Packs a decoded pixel block into output words per the latched depth and
    /// pushes them into the output FIFO.
    fn pack_output(&mut self, rgb: &[u32]) {
        match self.output_depth {
            // 4-bit monochrome: two pixels per byte, eight per word.
            0 => {
                for chunk in rgb.chunks_exact(8) {
                    let mut value = 0u32;
                    for (j, &p) in chunk.iter().enumerate() {
                        value |= ((p >> 4) & 0xF) << (j * 4);
                    }
                    self.out_fifo.push_back(value);
                }
            }
            // 8-bit monochrome: one pixel per byte, four per word.
            1 => {
                for chunk in rgb.chunks_exact(4) {
                    let value = (chunk[0] & 0xFF)
                        | ((chunk[1] & 0xFF) << 8)
                        | ((chunk[2] & 0xFF) << 16)
                        | ((chunk[3] & 0xFF) << 24);
                    self.out_fifo.push_back(value);
                }
            }
            // 24-bit RGB: packed tightly (three bytes per pixel) across words.
            2 => {
                let mut index = 0usize;
                let mut state = 0u8;
                let mut acc = 0u32;
                while index < rgb.len() {
                    match state {
                        0 => {
                            acc = rgb[index];
                            index += 1;
                            state = 1;
                        }
                        1 => {
                            acc |= (rgb[index] & 0xFF) << 24;
                            self.out_fifo.push_back(acc);
                            acc = rgb[index] >> 8;
                            index += 1;
                            state = 2;
                        }
                        2 => {
                            acc |= rgb[index] << 16;
                            self.out_fifo.push_back(acc);
                            acc = rgb[index] >> 16;
                            index += 1;
                            state = 3;
                        }
                        _ => {
                            acc |= rgb[index] << 8;
                            self.out_fifo.push_back(acc);
                            index += 1;
                            state = 0;
                        }
                    }
                }
            }
            // 15-bit BGR555: two pixels per word, optional bit15.
            _ => {
                let a: u32 = if self.output_bit15 { 0x8000 } else { 0 };
                for chunk in rgb.chunks_exact(2) {
                    let c0 = chunk[0];
                    let p0 = ((c0 >> 3) & 0x1F)
                        | (((c0 >> 11) & 0x1F) << 5)
                        | (((c0 >> 19) & 0x1F) << 10)
                        | a;
                    let c1 = chunk[1];
                    let p1 = ((c1 >> 3) & 0x1F)
                        | (((c1 >> 11) & 0x1F) << 5)
                        | (((c1 >> 19) & 0x1F) << 10)
                        | a;
                    self.out_fifo.push_back(p0 | (p1 << 16));
                }
            }
        }
    }

    /// Number of words currently queued in the output FIFO. For tests / DMA
    /// sizing.
    #[must_use]
    pub fn out_len(&self) -> usize {
        self.out_fifo.len()
    }
}

/// RLE-decodes and dequantizes one 8x8 block from the halfword stream starting
/// at `*pos`, advancing `*pos`. Returns `None` if the stream is exhausted before
/// a block can begin. Follows the PSX-SPX `rl_decode_block` reference.
fn decode_rle_block(hw: &[u16], pos: &mut usize, qt: &[u8]) -> Option<[i16; 64]> {
    let mut blk = [0i16; 64];

    // Skip end-of-block padding at the block start.
    let n = loop {
        if *pos >= hw.len() {
            return None;
        }
        let n = hw[*pos];
        *pos += 1;
        if n != 0xFE00 {
            break n;
        }
    };

    let q_scale = ((n >> 10) & 0x3F) as i32;
    let dc = sign_extend(i32::from(n & 0x3FF), 10);
    let mut val = if q_scale == 0 {
        dc * 2
    } else {
        dc * i32::from(qt[0])
    };
    val = val.clamp(-0x400, 0x3FF);
    let mut k: usize = 0;
    if q_scale > 0 {
        blk[ZAGZIG[k]] = val as i16;
    } else {
        blk[k] = val as i16;
    }

    loop {
        if *pos >= hw.len() {
            break;
        }
        let n = hw[*pos];
        *pos += 1;
        k += (((n >> 10) & 0x3F) as usize) + 1;
        if k < 64 {
            let level = sign_extend(i32::from(n & 0x3FF), 10);
            let mut val = if q_scale == 0 {
                level * 2
            } else {
                (level * i32::from(qt[k]) * q_scale + 4) / 8
            };
            val = val.clamp(-0x400, 0x3FF);
            if q_scale > 0 {
                blk[ZAGZIG[k]] = val as i16;
            } else {
                blk[k] = val as i16;
            }
        }
        if k >= 63 {
            break;
        }
    }

    Some(blk)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds the canonical PSX IDCT scale table:
    /// `M[a*8+b] = round(k(b) * cos((2a+1) b pi / 16) * sqrt(2/8) * 65536)`,
    /// clamped to signed 16-bit. This is the table the BIOS/`DecDCTvlc` loads;
    /// `M[0..8] == 0x5A82`.
    fn psx_scale_table() -> Vec<i16> {
        let mut t = vec![0i16; 64];
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

    /// A floating-point reference 2-D IDCT for cross-checking the integer path.
    /// `out[y*8+x] = sum_{u,v} A(u,y) A(v,x) coeff[u*8+v]` with the orthonormal
    /// DCT-III basis.
    fn reference_idct(coeff: &[i16; 64]) -> [i32; 64] {
        let a = |freq: usize, pos: usize| -> f64 {
            let k = if freq == 0 { 1.0 / 2.0f64.sqrt() } else { 1.0 };
            k * (2.0f64 / 8.0).sqrt()
                * (((2 * pos + 1) * freq) as f64 * std::f64::consts::PI / 16.0).cos()
        };
        let mut out = [0i32; 64];
        for y in 0..8 {
            for x in 0..8 {
                let mut sum = 0.0f64;
                for u in 0..8 {
                    for v in 0..8 {
                        sum += a(u, y) * a(v, x) * f64::from(coeff[u * 8 + v]);
                    }
                }
                out[y * 8 + x] = sum.round() as i32;
            }
        }
        out
    }

    fn mdec_with_scale() -> Mdec {
        let mut m = Mdec::new();
        m.scale_table = psx_scale_table();
        m
    }

    /// Encodes a DC-only block as two halfwords: `[q_scale|dc]`, then a run of
    /// 63 (EOB) so the block ends with only the DC coefficient placed.
    fn dc_only_word(q_scale: u32, dc: u32) -> u32 {
        let n1 = ((q_scale & 0x3F) << 10) | (dc & 0x3FF);
        let n2: u32 = 63 << 10; // run pushes k to 64 -> end of block
        n1 | (n2 << 16)
    }

    #[test]
    fn post_reset_status_is_documented_value() {
        let m = Mdec::new();
        assert_eq!(m.status(), 0x8004_0000);
    }

    #[test]
    fn control_reset_restores_status() {
        let mut m = Mdec::new();
        m.write32(0x1F80_1824, 1 << 30); // enable DMA-in
        assert_ne!(m.status(), 0x8004_0000);
        m.write32(0x1F80_1824, 1 << 31); // reset
        assert_eq!(m.status(), 0x8004_0000);
    }

    #[test]
    fn control_bits_set_dma_request_enables() {
        let mut m = Mdec::new();
        m.write32(0x1F80_1824, 1 << 30);
        // bit28 = data-in request when DMA-in enabled.
        assert_ne!(m.status() & (1 << 28), 0);
        assert_eq!(
            m.status() & (1 << 27),
            0,
            "no data-out request with empty FIFO"
        );

        m.write32(0x1F80_1824, 1 << 29); // enable DMA-out only
        assert_eq!(m.status() & (1 << 28), 0);
    }

    #[test]
    fn decode_command_latches_format_and_counts_params() {
        let mut m = Mdec::new();
        // Command 1, depth=3 (15-bit), signed=0, bit15=1, 3 param words.
        let cmd = (1 << 29) | (3 << 27) | (1 << 25) | 3;
        m.write32(0x1F80_1820, cmd);
        // Busy, depth=3, bit15 set, remaining = 3-1 = 2.
        let st = m.status();
        assert_ne!(st & (1 << 29), 0, "busy");
        assert_eq!((st >> 25) & 0x3, 3, "depth latched");
        assert_ne!(st & (1 << 23), 0, "bit15 latched");
        assert_eq!(st & 0xFFFF, 2, "words remaining minus 1");

        m.write32(0x1F80_1820, 0); // 1st param -> remaining 2
        assert_eq!(m.status() & 0xFFFF, 1);
        m.write32(0x1F80_1820, 0); // 2nd param -> remaining 1
        assert_eq!(m.status() & 0xFFFF, 0);
        m.write32(0x1F80_1820, 0); // 3rd param -> command completes, idle
        assert_eq!(m.status() & (1 << 29), 0, "idle after all params");
    }

    #[test]
    fn set_quant_luma_populates_table() {
        let mut m = Mdec::new();
        m.write32(0x1F80_1820, 2 << 29); // set quant, luma only -> 16 words
        // 16 words of 0x04040404 -> every luma quant entry = 4.
        for _ in 0..16 {
            m.write32(0x1F80_1820, 0x0404_0404);
        }
        assert!(m.iq_y.iter().all(|&q| q == 4));
        assert_eq!(m.status(), 0x8004_0000, "back to idle");
    }

    #[test]
    fn set_quant_color_populates_both_tables() {
        let mut m = Mdec::new();
        m.write32(0x1F80_1820, (2 << 29) | 1); // color -> 32 words
        for _ in 0..16 {
            m.write32(0x1F80_1820, 0x0202_0202); // luma = 2
        }
        for _ in 0..16 {
            m.write32(0x1F80_1820, 0x0505_0505); // chroma = 5
        }
        assert!(m.iq_y.iter().all(|&q| q == 2));
        assert!(m.iq_uv.iter().all(|&q| q == 5));
    }

    #[test]
    fn set_scale_populates_table() {
        let mut m = Mdec::new();
        m.write32(0x1F80_1820, 3 << 29); // set scale -> 32 words
        for i in 0..32u32 {
            // Two signed halfwords per word: 2i and 2i+1.
            let lo = (2 * i) as u16 as u32;
            let hi = (2 * i + 1) as u16 as u32;
            m.write32(0x1F80_1820, lo | (hi << 16));
        }
        for (i, &v) in m.scale_table.iter().enumerate() {
            assert_eq!(v, i as i16);
        }
    }

    #[test]
    fn idct_dc_only_is_flat() {
        // DC coefficient 128 -> flat 128/8 = 16 everywhere.
        let m = mdec_with_scale();
        let mut blk = [0i16; 64];
        blk[0] = 128;
        m.idct(&mut blk);
        for &v in blk.iter() {
            assert_eq!(v, 16, "DC-only IDCT must be flat 16");
        }
    }

    #[test]
    fn idct_matches_reference() {
        let m = mdec_with_scale();
        // A DC + a couple of AC coefficients.
        let mut coeff = [0i16; 64];
        coeff[0] = 200;
        coeff[1] = -60;
        coeff[8] = 40;
        coeff[9] = 15;
        let reference = reference_idct(&coeff);

        let mut blk = coeff;
        m.idct(&mut blk);
        for i in 0..64 {
            let r = reference[i].clamp(-128, 127);
            let got = i32::from(blk[i]);
            assert!(
                (got - r).abs() <= 2,
                "idx {i}: integer IDCT {got} vs reference {r}"
            );
        }
    }

    #[test]
    fn idct_impulse_matches_reference() {
        let m = mdec_with_scale();
        let mut coeff = [0i16; 64];
        coeff[10] = 100; // a single mid-frequency impulse
        let reference = reference_idct(&coeff);
        let mut blk = coeff;
        m.idct(&mut blk);
        for i in 0..64 {
            let r = reference[i].clamp(-128, 127);
            let got = i32::from(blk[i]);
            assert!((got - r).abs() <= 2, "idx {i}: {got} vs {r}");
        }
    }

    #[test]
    fn yuv_to_rgb_grayscale_when_chroma_zero() {
        // Zero chroma -> R=G=B=Y+0x80 (unsigned).
        let m = mdec_with_scale();
        let cr = [0i16; 64];
        let cb = [0i16; 64];
        let mut yblk = [0i16; 64];
        yblk[0] = 40;
        let mut rgb = vec![0u32; 256];
        m.yuv_to_rgb(&mut rgb, 0, 0, &cr, &cb, &yblk);
        let px = rgb[0];
        let r = px & 0xFF;
        let g = (px >> 8) & 0xFF;
        let b = (px >> 16) & 0xFF;
        assert_eq!(r, 40 + 0x80);
        assert_eq!(g, 40 + 0x80);
        assert_eq!(b, 40 + 0x80);
    }

    #[test]
    fn yuv_to_rgb_signed_bias() {
        // Signed output: Y=40, chroma 0 -> value 40 (no +0x80 bias).
        let mut m = mdec_with_scale();
        m.output_signed = true;
        let cr = [0i16; 64];
        let cb = [0i16; 64];
        let mut yblk = [0i16; 64];
        yblk[5] = -30;
        let mut rgb = vec![0u32; 256];
        m.yuv_to_rgb(&mut rgb, 0, 0, &cr, &cb, &yblk);
        let px = rgb[5];
        assert_eq!(px & 0xFF, (-30i32 as u32) & 0xFF, "signed red byte");
    }

    #[test]
    fn yuv_to_rgb_nonzero_chroma_bt601() {
        // Hand-check one BT.601 vector: Y=0, Cr=64, Cb=-32, unsigned.
        let m = mdec_with_scale();
        let mut cr = [0i16; 64];
        let mut cb = [0i16; 64];
        cr[0] = 64;
        cb[0] = -32;
        let yblk = [0i16; 64];
        let mut rgb = vec![0u32; 256];
        m.yuv_to_rgb(&mut rgb, 0, 0, &cr, &cb, &yblk);
        // r_off = (359*64 + 0x80) >> 8 = (22976 + 128) >> 8 = 23104 >> 8 = 90
        let r_off = ((359 * 64) + 0x80) >> 8;
        // g_off = (((-88*-32)&!0x1F) + ((-183*64)&!0x07) + 0x80) >> 8
        let g_off = (((-88 * -32) & !0x1F) + ((-183 * 64) & !0x07) + 0x80) >> 8;
        // b_off = (454*-32 + 0x80) >> 8
        let b_off = ((454 * -32) + 0x80) >> 8;
        let expect_r = (r_off.clamp(-128, 127) + 0x80) as u32 & 0xFF;
        let expect_g = (g_off.clamp(-128, 127) + 0x80) as u32 & 0xFF;
        let expect_b = (b_off.clamp(-128, 127) + 0x80) as u32 & 0xFF;
        let px = rgb[0];
        assert_eq!(px & 0xFF, expect_r, "R");
        assert_eq!((px >> 8) & 0xFF, expect_g, "G");
        assert_eq!((px >> 16) & 0xFF, expect_b, "B");
    }

    /// Drives a full mono decode of a single DC-only flat block at the given
    /// depth and returns the output words.
    fn decode_mono_flat(depth: u8, q_scale: u32, dc: u32, quant0: u8) -> Vec<u32> {
        let mut m = mdec_with_scale();
        m.iq_y[0] = quant0;
        // Decode command: depth, unsigned, one param word.
        let cmd = (1 << 29) | ((u32::from(depth) & 0x3) << 27) | 1;
        m.write32(0x1F80_1820, cmd);
        m.write32(0x1F80_1820, dc_only_word(q_scale, dc));
        let mut out = Vec::new();
        while !m.out_fifo.is_empty() {
            out.push(m.read_data_word());
        }
        out
    }

    #[test]
    fn decode_mono_8bit_flat_golden() {
        // q_scale=1, dc=16, quant[0]=8 -> DC coeff = 16*8 = 128 -> flat Y=16 ->
        // byte = 16 + 0x80 = 0x90. 8-bit mono = 16 words of 0x90909090.
        let out = decode_mono_flat(1, 1, 16, 8);
        assert_eq!(out.len(), 16, "8-bit mono = 16 words");
        for w in out {
            assert_eq!(w, 0x9090_9090);
        }
    }

    #[test]
    fn decode_mono_4bit_flat_golden() {
        // Same flat Y byte 0x90 -> nibble 0x9 -> 8 words of 0x99999999.
        let out = decode_mono_flat(0, 1, 16, 8);
        assert_eq!(out.len(), 8, "4-bit mono = 8 words");
        for w in out {
            assert_eq!(w, 0x9999_9999);
        }
    }

    /// Drives a full colour decode of a flat macroblock (DC-only Y, zero chroma)
    /// and returns the output words.
    fn decode_color_flat(depth: u8, q_scale: u32, y_dc: u32, quant0: u8) -> Vec<u32> {
        let mut m = mdec_with_scale();
        m.iq_y[0] = quant0;
        m.iq_uv[0] = quant0;
        // Six blocks: Cr, Cb (both zero DC), Y1..Y4 (flat DC).
        let cmd = (1 << 29) | ((u32::from(depth) & 0x3) << 27) | 6;
        m.write32(0x1F80_1820, cmd);
        // Cr, Cb: DC = 0.
        m.write32(0x1F80_1820, dc_only_word(q_scale, 0));
        m.write32(0x1F80_1820, dc_only_word(q_scale, 0));
        // Y1..Y4: flat DC.
        for _ in 0..4 {
            m.write32(0x1F80_1820, dc_only_word(q_scale, y_dc));
        }
        let mut out = Vec::new();
        while !m.out_fifo.is_empty() {
            out.push(m.read_data_word());
        }
        out
    }

    #[test]
    fn decode_color_15bit_flat_golden() {
        // Y DC = 16*8 = 128 -> flat Y=16 -> gray byte 0x90. 15-bit channel =
        // (0x90 >> 3) & 0x1F = 0x12. pixel = 0x12 | 0x12<<5 | 0x12<<10 = 0x4A52.
        let out = decode_color_flat(3, 1, 16, 8);
        assert_eq!(out.len(), 128, "15-bit colour = 128 words");
        let ch = (0x90u32 >> 3) & 0x1F;
        let pixel = ch | (ch << 5) | (ch << 10);
        let expected = pixel | (pixel << 16);
        for w in out {
            assert_eq!(w, expected, "flat 15-bit pixel");
        }
    }

    #[test]
    fn decode_color_24bit_flat_golden() {
        // Flat gray 0x90 in every channel -> every byte 0x90 -> words 0x90909090.
        let out = decode_color_flat(2, 1, 16, 8);
        assert_eq!(out.len(), 192, "24-bit colour = 192 words");
        for w in out {
            assert_eq!(w, 0x9090_9090);
        }
    }

    #[test]
    fn read_empty_fifo_is_open_bus() {
        let mut m = Mdec::new();
        assert_eq!(m.read_data_word(), 0xFFFF_FFFF);
    }

    #[test]
    fn serde_round_trip() {
        let mut m = mdec_with_scale();
        m.iq_y[3] = 7;
        m.write32(0x1F80_1824, 1 << 30);
        let json = serde_json::to_string(&m).unwrap();
        let back: Mdec = serde_json::from_str(&json).unwrap();
        assert_eq!(m, back);
    }
}
