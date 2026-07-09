//! GPU emulation: command FIFO, VRAM, and a software rasterizer.
//!
//! The PlayStation GPU is driven through two ports:
//!
//! * **GP0** (0x1F80_1810 write) — rendering / VRAM-transfer commands. Many are
//!   multi-word, so [`Gpu::gp0`] runs a small state machine that accumulates the
//!   required number of words before executing a command.
//! * **GP1** (0x1F80_1814 write) — display / control commands (reset, DMA
//!   direction, display area, display mode, ...).
//!
//! Reads mirror these: reading 0x1F80_1810 returns `GPUREAD` (VRAM→CPU transfer
//! data or the latched GP0(0x10) info word); reading 0x1F80_1814 returns
//! `GPUSTAT`.
//!
//! VRAM is a 1024×512 array of 16-bit pixels (BGR555). The rasterizer supports
//! flat/Gouraud triangles and quads, textured triangles/quads/rectangles
//! (4bpp/8bpp CLUT + 15bpp direct sampling, colour modulation, per-texel
//! semi-transparency, and the texture window), monochrome/textured rectangles,
//! flat and Gouraud lines/poly-lines, fills, and VRAM↔VRAM / CPU↔VRAM block
//! transfers. It also honours all four semi-transparency blend modes, ordered
//! dithering, the mask bit, and the top-left fill rule. Poly-lines are parsed to
//! their terminator and each segment is colour-interpolated between its own two
//! endpoints, routed through the shared shade/plot path (mask, dither on
//! Gouraud segments, semi-transparency).

use serde::{Deserialize, Serialize};

/// VRAM width in 16-bit pixels.
pub const VRAM_WIDTH: usize = 1024;
/// VRAM height in 16-bit pixels.
pub const VRAM_HEIGHT: usize = 512;
/// Total VRAM size in 16-bit pixels.
pub const VRAM_PIXELS: usize = VRAM_WIDTH * VRAM_HEIGHT;

/// A running CPU↔VRAM or VRAM↔CPU rectangle transfer.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
struct VramTransfer {
    /// Rectangle origin X (VRAM pixel).
    x: u16,
    /// Rectangle origin Y (VRAM pixel).
    y: u16,
    /// Rectangle width in pixels (>= 1).
    w: u16,
    /// Rectangle height in pixels (>= 1).
    h: u16,
    /// Current X offset within the rectangle.
    cur_x: u16,
    /// Current Y offset within the rectangle.
    cur_y: u16,
}

impl VramTransfer {
    fn new(x: u16, y: u16, w: u16, h: u16) -> Self {
        Self {
            x,
            y,
            w: w.max(1),
            h: h.max(1),
            cur_x: 0,
            cur_y: 0,
        }
    }

    /// Advances the cursor by one pixel; returns `true` when the rectangle is
    /// fully consumed.
    fn advance(&mut self) -> bool {
        self.cur_x += 1;
        if self.cur_x >= self.w {
            self.cur_x = 0;
            self.cur_y += 1;
        }
        self.cur_y >= self.h
    }
}

/// The PlayStation GPU.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Gpu {
    /// 1024×512 16-bit (BGR555) video RAM.
    #[serde(with = "vram_serde")]
    pub vram: Vec<u16>,

    // ── Drawing area / offset ────────────────────────────────────────────
    /// Drawing area top-left X (inclusive).
    pub draw_x0: u16,
    /// Drawing area top-left Y (inclusive).
    pub draw_y0: u16,
    /// Drawing area bottom-right X (inclusive).
    pub draw_x1: u16,
    /// Drawing area bottom-right Y (inclusive).
    pub draw_y1: u16,
    /// Signed drawing offset X applied to primitive vertices.
    pub draw_off_x: i16,
    /// Signed drawing offset Y applied to primitive vertices.
    pub draw_off_y: i16,

    // ── Texture / attribute state (GPUSTAT low bits) ─────────────────────
    /// Texture page base X (64-pixel units, bits 0-3 of GPUSTAT).
    pub tex_page_x: u8,
    /// Texture page base Y (256-pixel units, bit 4 of GPUSTAT).
    pub tex_page_y: u8,
    /// Semi-transparency mode (bits 5-6).
    pub semi_transparency: u8,
    /// Texture color depth (bits 7-8): 0=4bit,1=8bit,2=15bit.
    pub tex_depth: u8,
    /// Dither enable (bit 9).
    pub dither: bool,
    /// Draw-to-display-area enable (bit 10).
    pub draw_to_display: bool,
    /// Force mask bit on writes (bit 11).
    pub mask_set: bool,
    /// Check mask bit before writing (bit 12).
    pub mask_check: bool,
    /// Texture disable (bit 15).
    pub tex_disable: bool,
    /// Texture window mask X.
    pub tex_window_mask_x: u8,
    /// Texture window mask Y.
    pub tex_window_mask_y: u8,
    /// Texture window offset X.
    pub tex_window_off_x: u8,
    /// Texture window offset Y.
    pub tex_window_off_y: u8,

    // ── Display state ────────────────────────────────────────────────────
    /// Display area start X in VRAM (GP1 0x05).
    pub display_vram_x: u16,
    /// Display area start Y in VRAM (GP1 0x05).
    pub display_vram_y: u16,
    /// Horizontal display range (GP1 0x06), packed X1|X2<<12.
    pub display_h_range: u32,
    /// Vertical display range (GP1 0x07), packed Y1|Y2<<10.
    pub display_v_range: u32,
    /// Horizontal resolution selector (GP1 0x08 bits 0-1).
    pub hres1: u8,
    /// Horizontal resolution 368 selector (GP1 0x08 bit 6).
    pub hres2: bool,
    /// Vertical resolution (GP1 0x08 bit 2): false=240, true=480.
    pub vres_480: bool,
    /// Video mode (GP1 0x08 bit 3): false=NTSC, true=PAL.
    pub pal: bool,
    /// Display color depth (GP1 0x08 bit 4): false=15bpp, true=24bpp.
    pub color_depth_24: bool,
    /// Vertical interlace (GP1 0x08 bit 5).
    pub interlace: bool,
    /// Whether the display is enabled (GPUSTAT bit 23 is the inverse).
    pub display_enabled: bool,

    /// DMA direction (GP1 0x04): 0=off,1=fifo,2=cpu→gp0,3=gpuread→cpu.
    pub dma_direction: u8,

    /// Latched value returned by GPUREAD outside of a store transfer.
    pub gpuread_latch: u32,

    /// Interlace field toggle (GPUSTAT bits 13 & 31).
    pub field: bool,

    /// GPU interrupt request (GPUSTAT bit 24).
    pub irq: bool,

    // ── Command FIFO / transfer state ────────────────────────────────────
    /// Accumulated words of the in-flight GP0 command.
    cmd_buffer: Vec<u32>,
    /// Total words needed to complete the in-flight command (0 = idle).
    cmd_words_needed: usize,
    /// Set while accumulating a variable-length poly-line.
    polyline_active: bool,
    /// Whether the active poly-line is Gouraud-shaded.
    polyline_shaded: bool,
    /// Active CPU→VRAM load transfer (GP0 0xA0).
    load_transfer: Option<VramTransfer>,
    /// Active VRAM→CPU store transfer (GP0 0xC0, drives GPUREAD).
    store_transfer: Option<VramTransfer>,
}

/// Serializes VRAM compactly as a byte blob rather than a giant JSON array.
mod vram_serde {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S: Serializer>(vram: &[u16], s: S) -> Result<S::Ok, S::Error> {
        let bytes: Vec<u8> = vram.iter().flat_map(|p| p.to_le_bytes()).collect();
        bytes.serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Vec<u16>, D::Error> {
        let bytes: Vec<u8> = Vec::deserialize(d)?;
        let mut out = vec![0u16; super::VRAM_PIXELS];
        for (i, chunk) in bytes.chunks_exact(2).enumerate() {
            if i >= out.len() {
                break;
            }
            out[i] = u16::from_le_bytes([chunk[0], chunk[1]]);
        }
        Ok(out)
    }
}

impl Default for Gpu {
    fn default() -> Self {
        Self::new()
    }
}

/// Packs an 8-bit-per-channel RGB color into BGR555.
#[inline]
#[must_use]
pub fn rgb_to_bgr555(r: u8, g: u8, b: u8) -> u16 {
    ((u16::from(b) >> 3) << 10) | ((u16::from(g) >> 3) << 5) | (u16::from(r) >> 3)
}

/// Extracts the R/G/B 8-bit channels of a 24-bit command color word.
#[inline]
fn color_channels(word: u32) -> (u8, u8, u8) {
    (word as u8, (word >> 8) as u8, (word >> 16) as u8)
}

/// Sign-extends an 11-bit vertex coordinate packed in the low 16 bits.
#[inline]
fn coord_component(half: u16) -> i32 {
    // Vertices are 11-bit signed; sign-extend from bit 10.
    let v = (half & 0x7FF) as i32;
    if v & 0x400 != 0 { v - 0x800 } else { v }
}

/// Decodes a packed `y<<16 | x` vertex word into signed (x, y).
#[inline]
fn decode_vertex(word: u32) -> (i32, i32) {
    (
        coord_component(word as u16),
        coord_component((word >> 16) as u16),
    )
}

impl Gpu {
    /// Creates a GPU with zeroed VRAM and power-on defaults.
    #[must_use]
    pub fn new() -> Self {
        Self {
            vram: vec![0u16; VRAM_PIXELS],
            draw_x0: 0,
            draw_y0: 0,
            draw_x1: 0,
            draw_y1: 0,
            draw_off_x: 0,
            draw_off_y: 0,
            tex_page_x: 0,
            tex_page_y: 0,
            semi_transparency: 0,
            tex_depth: 0,
            dither: false,
            draw_to_display: false,
            mask_set: false,
            mask_check: false,
            tex_disable: false,
            tex_window_mask_x: 0,
            tex_window_mask_y: 0,
            tex_window_off_x: 0,
            tex_window_off_y: 0,
            display_vram_x: 0,
            display_vram_y: 0,
            display_h_range: 0,
            display_v_range: 0,
            hres1: 0,
            hres2: false,
            vres_480: false,
            pal: false,
            color_depth_24: false,
            interlace: false,
            display_enabled: false,
            dma_direction: 0,
            gpuread_latch: 0,
            field: false,
            irq: false,
            cmd_buffer: Vec::new(),
            cmd_words_needed: 0,
            polyline_active: false,
            polyline_shaded: false,
            load_transfer: None,
            store_transfer: None,
        }
    }

    // ── VRAM helpers ─────────────────────────────────────────────────────

    #[inline]
    fn vram_index(x: u16, y: u16) -> usize {
        let x = (x as usize) & (VRAM_WIDTH - 1);
        let y = (y as usize) & (VRAM_HEIGHT - 1);
        y * VRAM_WIDTH + x
    }

    /// Reads a VRAM pixel with wrap.
    #[must_use]
    pub fn vram_at(&self, x: u16, y: u16) -> u16 {
        self.vram[Self::vram_index(x, y)]
    }

    /// Writes a VRAM pixel with wrap.
    pub fn set_vram(&mut self, x: u16, y: u16, value: u16) {
        let idx = Self::vram_index(x, y);
        self.vram[idx] = value;
    }

    // ── GP0 command FIFO ─────────────────────────────────────────────────

    /// Feeds one word into the GP0 port.
    pub fn gp0(&mut self, word: u32) {
        // 1. Pixel data for an in-flight CPU→VRAM transfer.
        if self.load_transfer.is_some() {
            self.push_load_pixels(word);
            return;
        }

        // 2. Variable-length poly-line accumulation.
        if self.polyline_active {
            self.push_polyline_word(word);
            return;
        }

        // 3. Continue an in-flight multi-word command.
        if self.cmd_words_needed != 0 {
            self.cmd_buffer.push(word);
            if self.cmd_buffer.len() >= self.cmd_words_needed {
                self.execute_gp0();
            }
            return;
        }

        // 4. Idle: decode a new command.
        let opcode = (word >> 24) as u8;

        // Poly-lines are variable length and handled specially.
        if (0x40..=0x5F).contains(&opcode) && (word & 0x0800_0000) != 0 {
            self.polyline_active = true;
            self.polyline_shaded = (word & 0x1000_0000) != 0;
            self.cmd_buffer.clear();
            self.cmd_buffer.push(word);
            return;
        }

        let needed = Self::gp0_word_count(word);
        self.cmd_buffer.clear();
        self.cmd_buffer.push(word);
        if needed <= 1 {
            self.cmd_words_needed = 1;
            self.execute_gp0();
        } else {
            self.cmd_words_needed = needed;
        }
    }

    /// Returns the total number of words (including the command word) a GP0
    /// command occupies. Poly-lines are variable-length and excluded here.
    #[must_use]
    pub fn gp0_word_count(word: u32) -> usize {
        let opcode = (word >> 24) as u8;
        match opcode {
            0x02 => 3, // fill rectangle
            0x20..=0x3F => {
                // Polygon.
                let shaded = word & 0x1000_0000 != 0;
                let quad = word & 0x0800_0000 != 0;
                let textured = word & 0x0400_0000 != 0;
                let nv = if quad { 4 } else { 3 };
                1 + nv + if textured { nv } else { 0 } + if shaded { nv - 1 } else { 0 }
            }
            0x40..=0x5F => {
                // Single line (poly-lines handled by the state machine).
                if word & 0x1000_0000 != 0 { 4 } else { 3 }
            }
            0x60..=0x7F => {
                // Rectangle.
                let size = (word >> 27) & 0x3;
                let textured = word & 0x0400_0000 != 0;
                1 + 1 + if textured { 1 } else { 0 } + if size == 0 { 1 } else { 0 }
            }
            0x80..=0x9F => 4, // VRAM→VRAM copy
            0xA0..=0xBF => 3, // CPU→VRAM (header only; pixels follow)
            0xC0..=0xDF => 3, // VRAM→CPU (header only)
            // Single-word commands: NOP, clear cache, IRQ, env setup, unknown.
            _ => 1,
        }
    }

    fn execute_gp0(&mut self) {
        let cmd = self.cmd_buffer[0];
        let opcode = (cmd >> 24) as u8;
        match opcode {
            0x00 => {}                // NOP
            0x01 => {}                // Clear cache
            0x02 => self.fill_rect(), // Fill rectangle
            0x1F => self.irq = true,  // Interrupt request
            0x20..=0x3F => self.draw_polygon(),
            0x40..=0x5F => self.draw_single_line(),
            0x60..=0x7F => self.draw_rectangle(),
            0x80..=0x9F => self.vram_to_vram(),
            0xA0..=0xBF => self.begin_load_transfer(),
            0xC0..=0xDF => self.begin_store_transfer(),
            0xE1 => self.set_texpage(cmd),
            0xE2 => self.set_texture_window(cmd),
            0xE3 => {
                self.draw_x0 = (cmd & 0x3FF) as u16;
                self.draw_y0 = ((cmd >> 10) & 0x1FF) as u16;
            }
            0xE4 => {
                self.draw_x1 = (cmd & 0x3FF) as u16;
                self.draw_y1 = ((cmd >> 10) & 0x1FF) as u16;
            }
            0xE5 => {
                // Signed 11-bit drawing offset.
                self.draw_off_x = coord_component(cmd as u16) as i16;
                self.draw_off_y = coord_component((cmd >> 11) as u16) as i16;
            }
            0xE6 => {
                self.mask_set = cmd & 0x1 != 0;
                self.mask_check = cmd & 0x2 != 0;
            }
            _ => {} // Unknown: treat as no-op (already consumed).
        }
        self.reset_fifo();
    }

    fn reset_fifo(&mut self) {
        self.cmd_buffer.clear();
        self.cmd_words_needed = 0;
    }

    fn set_texpage(&mut self, cmd: u32) {
        self.set_texpage_low(cmd);
        self.dither = (cmd >> 9) & 0x1 != 0;
        self.draw_to_display = (cmd >> 10) & 0x1 != 0;
        self.tex_disable = (cmd >> 11) & 0x1 != 0;
    }

    /// Latches the GPUSTAT bits 0–8 texpage fields (page X/Y base,
    /// semi-transparency mode, texture depth) from a texpage word's low bits.
    /// Shared by GP0(E1) and the texpage attribute carried in a textured
    /// polygon's vertex-1 texcoord word (PSX-SPX Render-Polygon "Texpage").
    fn set_texpage_low(&mut self, cmd: u32) {
        self.tex_page_x = (cmd & 0xF) as u8;
        self.tex_page_y = ((cmd >> 4) & 0x1) as u8;
        self.semi_transparency = ((cmd >> 5) & 0x3) as u8;
        self.tex_depth = ((cmd >> 7) & 0x3) as u8;
    }

    fn set_texture_window(&mut self, cmd: u32) {
        self.tex_window_mask_x = (cmd & 0x1F) as u8;
        self.tex_window_mask_y = ((cmd >> 5) & 0x1F) as u8;
        self.tex_window_off_x = ((cmd >> 10) & 0x1F) as u8;
        self.tex_window_off_y = ((cmd >> 15) & 0x1F) as u8;
    }

    // ── Poly-line accumulation ───────────────────────────────────────────

    fn push_polyline_word(&mut self, word: u32) {
        // A poly-line vertex list is terminated by a word matching the
        // 0x5000_5000 pattern (0xF000_F000 mask). The terminator can only
        // appear where a vertex is expected — after at least the first vertex.
        let have = self.cmd_buffer.len();
        let terminator_slot = if self.polyline_shaded {
            // Shaded layout: [0]=cmd+col0, [1]=v0, [2]=col1, [3]=v1, [4]=col2,
            // [5]=v2, ... — colours at even indices, vertices at odd. The
            // terminator follows a vertex, i.e. it occupies the next colour slot
            // (even index ≥ 2).
            have >= 2 && have.is_multiple_of(2)
        } else {
            // Flat layout: [0]=cmd+col, [1]=v0, [2]=v1, ... — the terminator
            // follows a vertex (any index ≥ 1).
            have >= 1
        };
        if terminator_slot && (word & 0xF000_F000) == 0x5000_5000 {
            self.render_polyline();
            self.polyline_active = false;
            self.polyline_shaded = false;
            self.cmd_buffer.clear();
            return;
        }
        self.cmd_buffer.push(word);
    }

    fn render_polyline(&mut self) {
        // Extract each vertex with its colour, then draw every segment with
        // per-vertex colour interpolation between its own two endpoints (a
        // shaded poly-line carries a colour per vertex; PSX-SPX). Every segment
        // is routed through the shared shade/plot path so it honours the mask
        // bit, dithering (Gouraud segments only), and semi-transparency.
        let words = self.cmd_buffer.clone();
        if words.is_empty() {
            return;
        }
        let header = words[0];
        let semi = header & 0x0200_0000 != 0;
        let flags = PrimFlags {
            textured: false,
            raw: false,
            semi,
            gouraud: self.polyline_shaded,
            dither_allowed: true,
        };
        // (screen position, 8-bit vertex colour) per vertex.
        let mut verts: Vec<LineVertex> = Vec::new();
        if self.polyline_shaded {
            // Layout: [0]=cmd+col0, [1]=v0, [2]=col1, [3]=v1, ... vertices at odd
            // indices, each preceded by its colour ([0] carries v0's colour).
            let mut color = color_channels(header);
            let mut i = 1;
            while i < words.len() {
                verts.push((self.offset_vertex(words[i]), color));
                if i + 1 < words.len() {
                    color = color_channels(words[i + 1]);
                }
                i += 2;
            }
        } else {
            let color = color_channels(header);
            for w in &words[1..] {
                verts.push((self.offset_vertex(*w), color));
            }
        }
        for pair in verts.windows(2) {
            self.draw_line_shaded(pair[0].0, pair[1].0, pair[0].1, pair[1].1, &flags);
        }
    }

    /// Decodes a packed vertex word and applies the signed drawing offset.
    #[inline]
    fn offset_vertex(&self, word: u32) -> (i32, i32) {
        let (vx, vy) = decode_vertex(word);
        (
            vx + i32::from(self.draw_off_x),
            vy + i32::from(self.draw_off_y),
        )
    }

    // ── CPU↔VRAM transfers ───────────────────────────────────────────────

    fn begin_load_transfer(&mut self) {
        let dst = self.cmd_buffer[1];
        let dim = self.cmd_buffer[2];
        let x = (dst & 0x3FF) as u16;
        let y = ((dst >> 16) & 0x1FF) as u16;
        let mut w = (dim & 0xFFFF) as u16 & 0x3FF;
        let mut h = ((dim >> 16) & 0xFFFF) as u16 & 0x1FF;
        if w == 0 {
            w = 0x400;
        }
        if h == 0 {
            h = 0x200;
        }
        self.load_transfer = Some(VramTransfer::new(x, y, w, h));
        self.reset_fifo();
    }

    fn push_load_pixels(&mut self, word: u32) {
        // Each word carries two 16-bit pixels.
        for pixel in [word as u16, (word >> 16) as u16] {
            let done = {
                let t = self.load_transfer.as_ref().unwrap();
                let px = t.x.wrapping_add(t.cur_x);
                let py = t.y.wrapping_add(t.cur_y);
                self.set_vram(px, py, pixel);
                self.load_transfer.as_mut().unwrap().advance()
            };
            if done {
                self.load_transfer = None;
                break;
            }
        }
    }

    fn begin_store_transfer(&mut self) {
        let src = self.cmd_buffer[1];
        let dim = self.cmd_buffer[2];
        let x = (src & 0x3FF) as u16;
        let y = ((src >> 16) & 0x1FF) as u16;
        let mut w = (dim & 0xFFFF) as u16 & 0x3FF;
        let mut h = ((dim >> 16) & 0xFFFF) as u16 & 0x1FF;
        if w == 0 {
            w = 0x400;
        }
        if h == 0 {
            h = 0x200;
        }
        self.store_transfer = Some(VramTransfer::new(x, y, w, h));
        self.reset_fifo();
    }

    fn vram_to_vram(&mut self) {
        let src = self.cmd_buffer[1];
        let dst = self.cmd_buffer[2];
        let dim = self.cmd_buffer[3];
        let sx = (src & 0x3FF) as u16;
        let sy = ((src >> 16) & 0x1FF) as u16;
        let dx = (dst & 0x3FF) as u16;
        let dy = ((dst >> 16) & 0x1FF) as u16;
        let mut w = (dim & 0xFFFF) as u16 & 0x3FF;
        let mut h = ((dim >> 16) & 0xFFFF) as u16 & 0x1FF;
        if w == 0 {
            w = 0x400;
        }
        if h == 0 {
            h = 0x200;
        }
        for row in 0..h {
            for col in 0..w {
                let v = self.vram_at(sx.wrapping_add(col), sy.wrapping_add(row));
                self.set_vram(dx.wrapping_add(col), dy.wrapping_add(row), v);
            }
        }
    }

    // ── Rasterizer ───────────────────────────────────────────────────────

    fn fill_rect(&mut self) {
        // GP0(0x02): fill ignores draw area / offset / mask.
        let color = self.cmd_buffer[0];
        let xy = self.cmd_buffer[1];
        let wh = self.cmd_buffer[2];
        let (r, g, b) = color_channels(color);
        let pixel = rgb_to_bgr555(r, g, b);
        let x = (xy & 0x3F0) as u16; // aligned to 16
        let y = ((xy >> 16) & 0x1FF) as u16;
        // Width rounds up to a multiple of 16, height masked to 0x1FF.
        let w = (((wh & 0xFFFF) + 0x0F) & 0x3F0) as u16;
        let h = ((wh >> 16) & 0x1FF) as u16;
        for row in 0..h {
            for col in 0..w {
                self.set_vram(x.wrapping_add(col), y.wrapping_add(row), pixel);
            }
        }
    }

    fn draw_polygon(&mut self) {
        let cmd = self.cmd_buffer[0];
        let gouraud = cmd & 0x1000_0000 != 0;
        let quad = cmd & 0x0800_0000 != 0;
        let textured = cmd & 0x0400_0000 != 0;
        // Bit 25 = semi-transparency (the primitive is a "semi-transparent"
        // primitive); bit 24 = texture-blend mode, 0 = blended/modulated,
        // 1 = raw texture (PSX-SPX: GP0 render-command bit layout). Bit 24 is
        // only meaningful for textured primitives.
        let semi = cmd & 0x0200_0000 != 0;
        let raw = textured && (cmd & 0x0100_0000 != 0);
        let nv = if quad { 4 } else { 3 };

        // Parse vertices, per-vertex colours (Gouraud) and texcoords. For a
        // textured polygon the CLUT lives in the high half of vertex 0's
        // texcoord word and the texpage in the high half of vertex 1's.
        let mut verts = [Vert::default(); 4];
        let base_color = color_channels(cmd);
        let mut idx = 1usize;
        let mut clut_word = 0u32;
        let mut page_word = 0u32;
        // Parsing walks `cmd_buffer` sequentially (idx advances by vertex kind),
        // so a plain index loop is clearer than zipping the vertex array.
        #[allow(clippy::needless_range_loop)]
        for i in 0..nv {
            let (r, g, b) = if gouraud {
                if i == 0 {
                    base_color
                } else {
                    let c = color_channels(self.cmd_buffer[idx]);
                    idx += 1;
                    c
                }
            } else {
                base_color
            };
            let vword = self.cmd_buffer[idx];
            idx += 1;
            let (mut u, mut v) = (0i32, 0i32);
            if textured {
                let tword = self.cmd_buffer[idx];
                idx += 1;
                u = (tword & 0xFF) as i32;
                v = ((tword >> 8) & 0xFF) as i32;
                if i == 0 {
                    clut_word = tword;
                }
                if i == 1 {
                    page_word = tword;
                }
            }
            let (vx, vy) = decode_vertex(vword);
            verts[i] = Vert {
                x: vx + i32::from(self.draw_off_x),
                y: vy + i32::from(self.draw_off_y),
                r: i32::from(r),
                g: i32::from(g),
                b: i32::from(b),
                u,
                v,
            };
        }

        let tex = self.poly_texinfo(clut_word, page_word, textured);
        if textured {
            // A textured polygon's texpage attribute (high half of vertex-1's
            // texcoord word) also reloads the persistent draw-mode/GPUSTAT
            // texpage bits (0–8) exactly like GP0(E1), so a subsequent
            // untextured primitive's semi-transparency mode and a subsequent
            // textured rectangle's page pick it up (PSX-SPX Render-Polygon).
            // Textured rectangles carry no texpage word and never latch.
            self.set_texpage_low(page_word >> 16);
        }
        let flags = PrimFlags {
            textured,
            raw,
            semi,
            gouraud,
            dither_allowed: true,
        };

        // Quad split (0,1,2)+(0,2,3): the two triangles share edge 0-2, which is
        // traversed in opposite directions once each is normalised to positive
        // area, so the top-left fill rule covers the seam exactly once.
        self.raster_triangle(verts[0], verts[1], verts[2], &flags, &tex);
        if quad {
            self.raster_triangle(verts[0], verts[2], verts[3], &flags, &tex);
        }
    }

    /// Builds texture-sampling state for a textured polygon from its CLUT
    /// (vertex 0 texcoord word) and texpage (vertex 1 texcoord word).
    fn poly_texinfo(&self, clut_word: u32, page_word: u32, textured: bool) -> TexInfo {
        if !textured {
            return TexInfo::default();
        }
        let clut = (clut_word >> 16) & 0xFFFF;
        let tp = (page_word >> 16) & 0xFFFF;
        TexInfo {
            clut_x: ((clut & 0x3F) * 16) as u16,
            clut_y: ((clut >> 6) & 0x1FF) as u16,
            page_x: ((tp & 0xF) * 64) as u16,
            page_y: (((tp >> 4) & 1) * 256) as u16,
            semi_mode: ((tp >> 5) & 3) as u8,
            depth: ((tp >> 7) & 3) as u8,
        }
    }

    /// Rasterizes a triangle with barycentric Gouraud/texture interpolation,
    /// clipped to the drawing area and honouring the top-left fill rule.
    fn raster_triangle(&mut self, a: Vert, b: Vert, c: Vert, flags: &PrimFlags, tex: &TexInfo) {
        let (a, mut b, mut c) = (a, b, c);
        let mut area = edge((a.x, a.y), (b.x, b.y), (c.x, c.y));
        if area == 0 {
            return; // degenerate
        }
        // Normalise to positive area (CCW in screen space) so the barycentric
        // weights are all non-negative inside and the top-left rule is uniform.
        if area < 0 {
            std::mem::swap(&mut b, &mut c);
            area = -area;
        }
        let pa = (a.x, a.y);
        let pb = (b.x, b.y);
        let pc = (c.x, c.y);

        let min_x = a.x.min(b.x).min(c.x).max(i32::from(self.draw_x0));
        let max_x = a.x.max(b.x).max(c.x).min(i32::from(self.draw_x1));
        let min_y = a.y.min(b.y).min(c.y).max(i32::from(self.draw_y0));
        let max_y = a.y.max(b.y).max(c.y).min(i32::from(self.draw_y1));

        // Top-left fill rule (PSX-SPX "Rasterization Topleft-rule"): a pixel that
        // lies exactly on an edge is drawn only when that edge is a top or left
        // edge, so shared edges between adjacent triangles are covered once (no
        // double blend for semi-transparent primitives, no gaps).
        let tl_bc = is_top_left(pb, pc);
        let tl_ca = is_top_left(pc, pa);
        let tl_ab = is_top_left(pa, pb);

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let p = (x, y);
                let w0 = edge(pb, pc, p); // weight of vertex a
                let w1 = edge(pc, pa, p); // weight of vertex b
                let w2 = edge(pa, pb, p); // weight of vertex c
                let inside = (w0 > 0 || (w0 == 0 && tl_bc))
                    && (w1 > 0 || (w1 == 0 && tl_ca))
                    && (w2 > 0 || (w2 == 0 && tl_ab));
                if !inside {
                    continue;
                }
                let aa = i64::from(area);
                let interp = |va: i32, vb: i32, vc: i32| -> i32 {
                    ((i64::from(w0) * i64::from(va)
                        + i64::from(w1) * i64::from(vb)
                        + i64::from(w2) * i64::from(vc))
                        / aa) as i32
                };
                let r8 = interp(a.r, b.r, c.r);
                let g8 = interp(a.g, b.g, c.g);
                let b8 = interp(a.b, b.b, c.b);
                let u = interp(a.u, b.u, c.u);
                let v = interp(a.v, b.v, c.v);
                self.shade_and_plot(x, y, r8, g8, b8, u, v, flags, tex);
            }
        }
    }

    fn draw_rectangle(&mut self) {
        let cmd = self.cmd_buffer[0];
        let size = (cmd >> 27) & 0x3;
        let textured = cmd & 0x0400_0000 != 0;
        let semi = cmd & 0x0200_0000 != 0;
        let raw = textured && (cmd & 0x0100_0000 != 0);
        let xy = self.cmd_buffer[1];
        let (vx, vy) = decode_vertex(xy);
        let x0 = vx + i32::from(self.draw_off_x);
        let y0 = vy + i32::from(self.draw_off_y);

        // Textured rects carry their CLUT in the high half of the (single)
        // texcoord word; the texpage comes from the latched GP0(E1) state. The
        // texcoord low/second byte give the top-left U/V, stepping per pixel.
        let (u0, v0, tex) = if textured {
            let tword = self.cmd_buffer[2];
            let clut = (tword >> 16) & 0xFFFF;
            let tex = TexInfo {
                clut_x: ((clut & 0x3F) * 16) as u16,
                clut_y: ((clut >> 6) & 0x1FF) as u16,
                page_x: u16::from(self.tex_page_x) * 64,
                page_y: u16::from(self.tex_page_y) * 256,
                semi_mode: self.semi_transparency,
                depth: self.tex_depth,
            };
            ((tword & 0xFF) as i32, ((tword >> 8) & 0xFF) as i32, tex)
        } else {
            (0, 0, TexInfo::default())
        };

        // If textured, cmd_buffer[2] is the texcoord/clut; size word follows.
        let (w, h) = match size {
            1 => (1i32, 1i32),
            2 => (8, 8),
            3 => (16, 16),
            _ => {
                let dim_idx = if textured { 3 } else { 2 };
                let dim = self.cmd_buffer[dim_idx];
                ((dim & 0xFFFF) as i32, ((dim >> 16) & 0xFFFF) as i32)
            }
        };
        let (r, g, b) = color_channels(cmd);
        // Rectangles are never dithered on real hardware (PSX-SPX), even when a
        // textured rect is colour-modulated — hence dither_allowed = false.
        let flags = PrimFlags {
            textured,
            raw,
            semi,
            gouraud: false,
            dither_allowed: false,
        };
        for row in 0..h {
            for col in 0..w {
                self.shade_and_plot(
                    x0 + col,
                    y0 + row,
                    i32::from(r),
                    i32::from(g),
                    i32::from(b),
                    u0 + col,
                    v0 + row,
                    &flags,
                    &tex,
                );
            }
        }
    }

    /// Samples a texel at texture-space `(u, v)`, applying the texture window and
    /// the CLUT/direct-colour decode for the active texture depth. Returns the
    /// raw BGR555 texel, or `None` when the texel is `0x0000` (fully transparent
    /// — such texels are skipped even for opaque primitives, per PSX-SPX).
    fn sample_texel(&self, u: i32, v: i32, tex: &TexInfo) -> Option<u16> {
        // Texture window (GP0 E2): masked bits are replaced by the offset so a
        // sub-region tiles. mask_x = win bits0-4, off_x = bits10-14 (×8), etc.
        let mask_x = u32::from(self.tex_window_mask_x) * 8;
        let mask_y = u32::from(self.tex_window_mask_y) * 8;
        let off_x = (u32::from(self.tex_window_off_x) & u32::from(self.tex_window_mask_x)) * 8;
        let off_y = (u32::from(self.tex_window_off_y) & u32::from(self.tex_window_mask_y)) * 8;
        let uu = (((u as u32) & 0xFF) & !mask_x) | off_x;
        let vv = (((v as u32) & 0xFF) & !mask_y) | off_y;
        let uu = uu as u16;
        let vv = vv as u16;

        // `vram_at` wraps X with 0x3FF and Y with 0x1FF, so raw sums are fine.
        let texel = match tex.depth {
            0 => {
                // 4bpp CLUT: four texels per VRAM halfword.
                let hw = self.vram_at(
                    tex.page_x.wrapping_add(uu >> 2),
                    tex.page_y.wrapping_add(vv),
                );
                let index = (hw >> ((uu & 3) * 4)) & 0xF;
                self.vram_at(tex.clut_x.wrapping_add(index), tex.clut_y)
            }
            1 => {
                // 8bpp CLUT: two texels per halfword.
                let hw = self.vram_at(
                    tex.page_x.wrapping_add(uu >> 1),
                    tex.page_y.wrapping_add(vv),
                );
                let index = (hw >> ((uu & 1) * 8)) & 0xFF;
                self.vram_at(tex.clut_x.wrapping_add(index), tex.clut_y)
            }
            _ => {
                // 15bpp direct (depth 2; depth 3 is treated as 15bpp).
                self.vram_at(tex.page_x.wrapping_add(uu), tex.page_y.wrapping_add(vv))
            }
        };
        if texel == 0 { None } else { Some(texel) }
    }

    /// Shades one candidate pixel — texture sampling, colour modulation,
    /// dithering — then blends (semi-transparency) and writes it to VRAM.
    #[allow(clippy::too_many_arguments)]
    fn shade_and_plot(
        &mut self,
        x: i32,
        y: i32,
        r8: i32,
        g8: i32,
        b8: i32,
        u: i32,
        v: i32,
        flags: &PrimFlags,
        tex: &TexInfo,
    ) {
        // Per-channel values carried at ~8-bit precision so dithering can act
        // before the final 5-bit quantize.
        let (cr, cg, cb);
        let force_msb;
        let do_semi;
        let semi_mode;
        if flags.textured {
            let texel = match self.sample_texel(u, v, tex) {
                Some(t) => t,
                None => return, // transparent texel — pixel skipped entirely
            };
            let (tr, tg, tb) = unpack5(texel);
            force_msb = texel & 0x8000 != 0;
            semi_mode = tex.semi_mode;
            // PSX-SPX: bit 25 marks the primitive semi-transparent; for textured
            // primitives the per-texel STP bit (bit 15) additionally gates the
            // blend. (Opaque textured prims still store STP into bit 15.)
            do_semi = flags.semi && force_msb;
            if flags.raw {
                // Raw texture: no modulation — expand the 5-bit texel to 8-bit.
                cr = i32::from(tr) << 3;
                cg = i32::from(tg) << 3;
                cb = i32::from(tb) << 3;
            } else {
                // Modulate: out5 = min(0x1F, tex5*col8 >> 7). Computed here as
                // (tex5*col8) >> 4 to keep ~8-bit precision for dithering;
                // ((tex5*col8) >> 4) >> 3 == (tex5*col8) >> 7 (col8 0x80 = 1.0).
                cr = (i32::from(tr) * r8) >> 4;
                cg = (i32::from(tg) * g8) >> 4;
                cb = (i32::from(tb) * b8) >> 4;
            }
        } else {
            cr = r8;
            cg = g8;
            cb = b8;
            force_msb = false;
            do_semi = flags.semi;
            semi_mode = self.semi_transparency;
        }

        // Dithering applies to Gouraud shading and modulated textures only — not
        // to raw textures or flat untextured fills, and never to rectangles
        // (dither_allowed = false there) (PSX-SPX "Dithering").
        let dither = self.dither
            && flags.dither_allowed
            && (flags.gouraud || (flags.textured && !flags.raw));
        let (r5, g5, b5) = if dither {
            let d = DITHER_MATRIX[(y & 3) as usize][(x & 3) as usize];
            (quant(cr + d), quant(cg + d), quant(cb + d))
        } else {
            (quant(cr), quant(cg), quant(cb))
        };
        self.write_pixel(x, y, r5, g5, b5, do_semi, semi_mode, force_msb);
    }

    /// Writes a shaded 5-bit-per-channel pixel to VRAM, applying the drawing-area
    /// clip, semi-transparency blending against the destination, and the mask
    /// bit (GP0 E6): check-mask skips destinations whose bit 15 is set; set-mask
    /// (or a set texel STP) forces bit 15 on the written pixel.
    #[allow(clippy::too_many_arguments)]
    fn write_pixel(
        &mut self,
        x: i32,
        y: i32,
        r5: u8,
        g5: u8,
        b5: u8,
        semi: bool,
        semi_mode: u8,
        force_msb: bool,
    ) {
        if x < i32::from(self.draw_x0)
            || x > i32::from(self.draw_x1)
            || y < i32::from(self.draw_y0)
            || y > i32::from(self.draw_y1)
        {
            return;
        }
        if x < 0 || y < 0 || x >= VRAM_WIDTH as i32 || y >= VRAM_HEIGHT as i32 {
            return;
        }
        let idx = Self::vram_index(x as u16, y as u16);
        let bg = self.vram[idx];
        if self.mask_check && (bg & 0x8000) != 0 {
            return; // check-mask-before-draw: destination is masked
        }
        let (fr, fg, fb) = if semi {
            blend(semi_mode, unpack5(bg), (r5, g5, b5))
        } else {
            (r5, g5, b5)
        };
        let mut out = pack555(fr, fg, fb);
        if self.mask_set || force_msb {
            out |= 0x8000;
        }
        self.vram[idx] = out;
    }

    fn draw_single_line(&mut self) {
        let cmd = self.cmd_buffer[0];
        let shaded = cmd & 0x1000_0000 != 0;
        let semi = cmd & 0x0200_0000 != 0;
        let c0 = color_channels(cmd);
        // Shaded layout: [0]=cmd+col0, [1]=v0, [2]=col1, [3]=v1. Flat: [0]=cmd+col,
        // [1]=v0, [2]=v1 (both endpoints share the single colour).
        let (a, c1, bb) = if shaded {
            (
                self.offset_vertex(self.cmd_buffer[1]),
                color_channels(self.cmd_buffer[2]),
                self.offset_vertex(self.cmd_buffer[3]),
            )
        } else {
            (
                self.offset_vertex(self.cmd_buffer[1]),
                c0,
                self.offset_vertex(self.cmd_buffer[2]),
            )
        };
        let flags = PrimFlags {
            textured: false,
            raw: false,
            semi,
            gouraud: shaded,
            dither_allowed: true,
        };
        self.draw_line_shaded(a, bb, c0, c1, &flags);
    }

    /// Draws a Bresenham line from `a` to `b`, linearly interpolating the 8-bit
    /// per-channel colour from `ca` at `a` to `cb` at `b` by parametric pixel
    /// position. Each pixel is routed through [`Gpu::shade_and_plot`], so the
    /// segment honours the drawing-area clip, the mask bit, dithering (when the
    /// primitive is Gouraud-shaded and dither is enabled), and semi-transparency
    /// — reusing the exact same helpers as triangles/rectangles. A monochrome
    /// line passes `ca == cb`, yielding a constant colour with no dithering.
    fn draw_line_shaded(
        &mut self,
        a: (i32, i32),
        b: (i32, i32),
        ca: (u8, u8, u8),
        cb: (u8, u8, u8),
        flags: &PrimFlags,
    ) {
        let (mut x0, mut y0) = a;
        let (x1, y1) = b;
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        // Pixel count along the dominant axis; the line spans `steps + 1` pixels.
        let steps = dx.max(dy);
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx - dy;
        let tex = TexInfo::default();
        let mut i = 0i32;
        loop {
            // Parametric colour at position i/steps along the segment.
            let (r8, g8, b8) = if steps == 0 {
                (i32::from(ca.0), i32::from(ca.1), i32::from(ca.2))
            } else {
                let lerp = |s: u8, e: u8| -> i32 {
                    i32::from(s) + (i32::from(e) - i32::from(s)) * i / steps
                };
                (lerp(ca.0, cb.0), lerp(ca.1, cb.1), lerp(ca.2, cb.2))
            };
            self.shade_and_plot(x0, y0, r8, g8, b8, 0, 0, flags, &tex);
            if x0 == x1 && y0 == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                x0 += sx;
            }
            if e2 < dx {
                err += dx;
                y0 += sy;
            }
            i += 1;
        }
    }

    // ── GP1 control port ─────────────────────────────────────────────────

    /// Feeds one word into the GP1 port.
    pub fn gp1(&mut self, word: u32) {
        let opcode = (word >> 24) as u8;
        match opcode {
            0x00 => self.reset(),
            0x01 => {
                // Reset command buffer / abort transfers.
                self.cmd_buffer.clear();
                self.cmd_words_needed = 0;
                self.polyline_active = false;
                self.load_transfer = None;
                self.store_transfer = None;
            }
            0x02 => self.irq = false, // Acknowledge GPU IRQ.
            0x03 => self.display_enabled = word & 0x1 == 0,
            0x04 => self.dma_direction = (word & 0x3) as u8,
            0x05 => {
                self.display_vram_x = (word & 0x3FF) as u16;
                self.display_vram_y = ((word >> 10) & 0x1FF) as u16;
            }
            0x06 => self.display_h_range = word & 0xFF_FFFF,
            0x07 => self.display_v_range = word & 0xF_FFFF,
            0x08 => {
                self.hres1 = (word & 0x3) as u8;
                self.vres_480 = word & 0x4 != 0;
                self.pal = word & 0x8 != 0;
                self.color_depth_24 = word & 0x10 != 0;
                self.interlace = word & 0x20 != 0;
                self.hres2 = word & 0x40 != 0;
            }
            0x10 => self.gpu_info(word),
            _ => {} // Unknown: ignore.
        }
    }

    fn gpu_info(&mut self, word: u32) {
        // GP1(0x10): latch a value for the next GPUREAD.
        self.gpuread_latch = match word & 0xF {
            0x2 => {
                u32::from(self.tex_window_mask_x)
                    | (u32::from(self.tex_window_mask_y) << 5)
                    | (u32::from(self.tex_window_off_x) << 10)
                    | (u32::from(self.tex_window_off_y) << 15)
            }
            0x3 => u32::from(self.draw_x0) | (u32::from(self.draw_y0) << 10),
            0x4 => u32::from(self.draw_x1) | (u32::from(self.draw_y1) << 10),
            0x5 => (self.draw_off_x as u32 & 0x7FF) | ((self.draw_off_y as u32 & 0x7FF) << 11),
            _ => self.gpuread_latch,
        };
    }

    /// Resets all GPU state to power-on defaults (GP1 0x00).
    pub fn reset(&mut self) {
        let vram = std::mem::take(&mut self.vram);
        *self = Self::new();
        self.vram = vram;
    }

    // ── Register reads ───────────────────────────────────────────────────

    /// Composes the GPUSTAT register (read of 0x1F80_1814).
    #[must_use]
    pub fn gpustat(&self) -> u32 {
        let mut stat = 0u32;
        stat |= u32::from(self.tex_page_x) & 0xF;
        stat |= (u32::from(self.tex_page_y) & 0x1) << 4;
        stat |= (u32::from(self.semi_transparency) & 0x3) << 5;
        stat |= (u32::from(self.tex_depth) & 0x3) << 7;
        stat |= u32::from(self.dither) << 9;
        stat |= u32::from(self.draw_to_display) << 10;
        stat |= u32::from(self.mask_set) << 11;
        stat |= u32::from(self.mask_check) << 12;
        stat |= u32::from(self.field) << 13;
        // bit 14 reverse = 0
        stat |= u32::from(self.tex_disable) << 15;
        stat |= u32::from(self.hres2) << 16;
        stat |= (u32::from(self.hres1) & 0x3) << 17;
        stat |= u32::from(self.vres_480) << 19;
        stat |= u32::from(self.pal) << 20;
        stat |= u32::from(self.color_depth_24) << 21;
        stat |= u32::from(self.interlace) << 22;
        // bit 23: display DISABLED (inverted).
        stat |= u32::from(!self.display_enabled) << 23;
        stat |= u32::from(self.irq) << 24;

        // bit 25: DMA / data request, derived from DMA direction.
        let dma_req = match self.dma_direction {
            1 => 1,                                        // FIFO
            2 => 1,                                        // CPU→GP0: mirrors bit 28
            3 => u32::from(self.store_transfer.is_some()), // GPUREAD→CPU: mirrors bit 27
            _ => 0,
        };
        stat |= dma_req << 25;

        // The BIOS spins on these ready bits, so always report ready.
        stat |= 1 << 26; // ready to receive command
        stat |= u32::from(self.store_transfer.is_some()) << 27; // ready to send VRAM
        stat |= 1 << 28; // ready to receive DMA block
        stat |= (u32::from(self.dma_direction) & 0x3) << 29;
        stat |= u32::from(self.field) << 31;
        stat
    }

    /// Reads the GPUREAD register (read of 0x1F80_1810).
    pub fn gpuread(&mut self) -> u32 {
        if self.store_transfer.is_some() {
            let mut word = 0u32;
            for shift in [0u32, 16] {
                let (px, py) = {
                    let t = self.store_transfer.as_ref().unwrap();
                    (t.x.wrapping_add(t.cur_x), t.y.wrapping_add(t.cur_y))
                };
                let pixel = self.vram_at(px, py);
                word |= u32::from(pixel) << shift;
                let finished = self.store_transfer.as_mut().unwrap().advance();
                if finished {
                    self.store_transfer = None;
                    break;
                }
            }
            self.gpuread_latch = word;
            word
        } else {
            self.gpuread_latch
        }
    }

    /// Returns whether a VRAM→CPU store transfer is currently active.
    #[must_use]
    pub fn store_active(&self) -> bool {
        self.store_transfer.is_some()
    }

    /// Returns whether the GP0 FIFO is mid-command (for tests).
    #[must_use]
    pub fn is_idle(&self) -> bool {
        self.cmd_words_needed == 0
            && !self.polyline_active
            && self.load_transfer.is_none()
            && self.store_transfer.is_none()
    }
}

// ── Free helpers ─────────────────────────────────────────────────────────

/// Twice the signed area of triangle (a, b, c) — the edge function.
#[inline]
fn edge(a: (i32, i32), b: (i32, i32), c: (i32, i32)) -> i32 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

/// Unpacks a BGR555 pixel into 5-bit-per-channel R/G/B (0..=31).
#[inline]
fn unpack5(p: u16) -> (u8, u8, u8) {
    (
        (p & 0x1F) as u8,
        ((p >> 5) & 0x1F) as u8,
        ((p >> 10) & 0x1F) as u8,
    )
}

/// Packs 5-bit-per-channel R/G/B (0..=31) into BGR555 (mask bit cleared).
#[inline]
fn pack555(r: u8, g: u8, b: u8) -> u16 {
    (u16::from(b & 0x1F) << 10) | (u16::from(g & 0x1F) << 5) | u16::from(r & 0x1F)
}

/// Clamps an ~8-bit component to `[0, 255]` and quantizes it to 5 bits (0..=31).
#[inline]
fn quant(v: i32) -> u8 {
    (v.clamp(0, 255) >> 3) as u8
}

/// Blends foreground `f` over background `b` (5-bit components) using one of the
/// four PSX semi-transparency modes (PSX-SPX "Semi Transparency"):
/// `0: B/2+F/2`, `1: B+F`, `2: B-F`, `3: B+F/4` (with clamping).
#[inline]
fn blend(mode: u8, b: (u8, u8, u8), f: (u8, u8, u8)) -> (u8, u8, u8) {
    let ch = |bg: u8, fg: u8| -> u8 {
        match mode {
            0 => ((u16::from(bg) + u16::from(fg)) >> 1) as u8,
            1 => (u16::from(bg) + u16::from(fg)).min(0x1F) as u8,
            2 => bg.saturating_sub(fg),
            3 => (u16::from(bg) + u16::from(fg >> 2)).min(0x1F) as u8,
            _ => fg,
        }
    };
    (ch(b.0, f.0), ch(b.1, f.1), ch(b.2, f.2))
}

/// Whether a directed edge `start → end` is a top or left edge under the PSX
/// top-left fill rule, given positive-area (CCW, Y-down) triangles. A left edge
/// goes downward (`dy > 0`); a top edge is horizontal moving left
/// (`dy == 0 && dx > 0`).
#[inline]
fn is_top_left(start: (i32, i32), end: (i32, i32)) -> bool {
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    dy > 0 || (dy == 0 && dx > 0)
}

/// The 4×4 signed ordered-dither matrix (PSX-SPX "Dithering"), indexed
/// `[y & 3][x & 3]`, added to each ~8-bit component before the 5-bit quantize.
const DITHER_MATRIX: [[i32; 4]; 4] = [
    [-4, 0, -3, 1],
    [2, -2, 3, -1],
    [-3, 1, -4, 0],
    [3, -1, 2, -2],
];

/// A poly-line vertex: screen position `(x, y)` paired with its 8-bit R/G/B
/// vertex colour, used to interpolate colour along each segment.
type LineVertex = ((i32, i32), (u8, u8, u8));

/// A rasterizer vertex: screen position, 8-bit vertex colour, and texcoords.
#[derive(Clone, Copy, Default)]
struct Vert {
    x: i32,
    y: i32,
    r: i32,
    g: i32,
    b: i32,
    u: i32,
    v: i32,
}

/// Per-primitive shading flags shared by every pixel of a primitive.
struct PrimFlags {
    /// The primitive samples a texture.
    textured: bool,
    /// Raw texture (bit 24) — skip colour modulation.
    raw: bool,
    /// Semi-transparent primitive (bit 25).
    semi: bool,
    /// Gouraud-shaded (enables dithering).
    gouraud: bool,
    /// Whether dithering may apply (false for rectangles).
    dither_allowed: bool,
}

/// Texture-sampling state (CLUT + texpage) resolved for a textured primitive.
#[derive(Default)]
struct TexInfo {
    /// CLUT (palette) top-left X in VRAM pixels.
    clut_x: u16,
    /// CLUT top-left Y in VRAM pixels.
    clut_y: u16,
    /// Texture-page base X in VRAM pixels.
    page_x: u16,
    /// Texture-page base Y in VRAM pixels.
    page_y: u16,
    /// Texture colour depth: 0 = 4bpp CLUT, 1 = 8bpp CLUT, 2/3 = 15bpp direct.
    depth: u8,
    /// Semi-transparency blend mode for this primitive.
    semi_mode: u8,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(gpu: &mut Gpu, words: &[u32]) {
        for &w in words {
            gpu.gp0(w);
        }
    }

    #[test]
    fn word_count_polygons() {
        // Flat triangle: cmd + 3 verts = 4.
        assert_eq!(Gpu::gp0_word_count(0x2000_0000), 4);
        // Gouraud triangle: cmd(+col0) + 3 verts + 2 colors = 6.
        assert_eq!(Gpu::gp0_word_count(0x3000_0000), 6);
        // Textured flat triangle: cmd + 3 verts + 3 uv = 7.
        assert_eq!(Gpu::gp0_word_count(0x2400_0000), 7);
        // Textured Gouraud quad: cmd + 4 verts + 4 uv + 3 colors = 12.
        assert_eq!(Gpu::gp0_word_count(0x3C00_0000), 12);
        // Flat quad: cmd + 4 verts = 5.
        assert_eq!(Gpu::gp0_word_count(0x2800_0000), 5);
    }

    #[test]
    fn word_count_rects_and_lines() {
        assert_eq!(Gpu::gp0_word_count(0x6000_0000), 3); // variable rect
        assert_eq!(Gpu::gp0_word_count(0x6800_0000), 2); // 1x1 rect
        assert_eq!(Gpu::gp0_word_count(0x7400_0000), 3); // 8x8 textured rect
        assert_eq!(Gpu::gp0_word_count(0x4000_0000), 3); // flat line
        assert_eq!(Gpu::gp0_word_count(0x5000_0000), 4); // shaded line
        assert_eq!(Gpu::gp0_word_count(0x0200_0000), 3); // fill
    }

    #[test]
    fn back_to_back_fills_do_not_desync() {
        // Two fill rects fed back-to-back should both land, proving the FIFO
        // returns to idle between commands.
        let mut gpu = Gpu::new();
        feed(
            &mut gpu,
            &[
                0x0200_00FF, // fill red at...
                0x0000_0000, // (0,0)
                0x0010_0010, // 16x16
            ],
        );
        assert!(gpu.is_idle());
        feed(
            &mut gpu,
            &[
                0x0200_FF00, // fill green at...
                0x0020_0000, // (0,32)  (x in low half, y in high half)
                0x0010_0010, // 16x16
            ],
        );
        assert!(gpu.is_idle());
        assert_eq!(gpu.vram_at(0, 0), rgb_to_bgr555(0xFF, 0, 0));
        assert_eq!(gpu.vram_at(0, 32), rgb_to_bgr555(0, 0xFF, 0));
    }

    #[test]
    fn fill_rect_writes_pixels() {
        let mut gpu = Gpu::new();
        feed(&mut gpu, &[0x0200_00FF, 0x0000_0000, 0x0008_0008]);
        // Width rounds up to a multiple of 16.
        for y in 0..8 {
            for x in 0..8 {
                assert_eq!(gpu.vram_at(x, y), rgb_to_bgr555(0xFF, 0, 0));
            }
        }
    }

    #[test]
    fn flat_triangle_fills_interior_and_clips() {
        let mut gpu = Gpu::new();
        // Drawing area covers the whole VRAM top-left.
        gpu.gp0(0xE300_0000); // draw area TL (0,0)
        gpu.gp0(0xE400_0000 | (100u32) | (100u32 << 10)); // BR (100,100)
        // Flat white triangle (0,0)-(20,0)-(0,20).
        feed(
            &mut gpu,
            &[
                0x20FF_FFFF,
                0x0000_0000, // v0 (0,0)
                0x0000_0014, // v1 (20,0)
                0x0014_0000, // v2 (0,20)
            ],
        );
        // Interior point should be set.
        assert_ne!(gpu.vram_at(3, 3), 0);
        // A point well outside the triangle stays clear.
        assert_eq!(gpu.vram_at(90, 90), 0);
    }

    #[test]
    fn triangle_respects_draw_area_clip() {
        let mut gpu = Gpu::new();
        // Restrict the drawing area to a small box away from the origin.
        gpu.gp0(0xE300_0000 | 50u32 | (50u32 << 10)); // TL (50,50)
        gpu.gp0(0xE400_0000 | 60u32 | (60u32 << 10)); // BR (60,60)
        // Large triangle (0,0)-(200,0)-(0,200) so (52,52) lies well inside it.
        feed(
            &mut gpu,
            &[0x20FF_FFFF, 0x0000_0000, 0x0000_00C8, 0x00C8_0000],
        );
        // Origin is inside the triangle but outside the clip box.
        assert_eq!(gpu.vram_at(0, 0), 0);
        // A point inside both is drawn.
        assert_ne!(gpu.vram_at(52, 52), 0);
    }

    #[test]
    fn draw_offset_shifts_primitive() {
        let mut gpu = Gpu::new();
        gpu.gp0(0xE300_0000);
        gpu.gp0(0xE400_0000 | 200u32 | (200u32 << 10));
        // Offset of (100,100).
        gpu.gp0(0xE500_0000 | 100u32 | (100u32 << 11));
        feed(
            &mut gpu,
            &[0x20FF_FFFF, 0x0000_0000, 0x0000_0014, 0x0014_0000],
        );
        // The triangle now sits around (100,100), not the origin.
        assert_eq!(gpu.vram_at(3, 3), 0);
        assert_ne!(gpu.vram_at(103, 103), 0);
    }

    #[test]
    fn gouraud_triangle_interpolates() {
        let mut gpu = Gpu::new();
        gpu.gp0(0xE300_0000);
        gpu.gp0(0xE400_0000 | 100u32 | (100u32 << 10));
        // Gouraud triangle: red, green, blue corners of a big triangle.
        feed(
            &mut gpu,
            &[
                0x3000_00FF, // cmd + color0 = red
                0x0000_0000, // v0 (0,0)
                0x0000_FF00, // color1 = green
                0x0000_0032, // v1 (50,0)
                0x00FF_0000, // color2 = blue
                0x0032_0000, // v2 (0,50)
            ],
        );
        // Near v0 the pixel should be reddish (r channel dominant).
        let p = gpu.vram_at(1, 1);
        let (r, g, b) = unpack5(p);
        assert!(
            r > g && r > b,
            "expected red-dominant near v0, got {r},{g},{b}"
        );
    }

    #[test]
    fn rectangle_fills() {
        let mut gpu = Gpu::new();
        gpu.gp0(0xE300_0000);
        gpu.gp0(0xE400_0000 | 100u32 | (100u32 << 10));
        // 8x8 monochrome rect at (10,10).
        feed(&mut gpu, &[0x7000_00FF, 0x000A_000A]);
        assert_eq!(gpu.vram_at(12, 12), rgb_to_bgr555(0xFF, 0, 0));
        assert_eq!(gpu.vram_at(20, 20), 0);
    }

    #[test]
    fn cpu_to_vram_then_vram_to_cpu_round_trips() {
        let mut gpu = Gpu::new();
        // CPU→VRAM: 2x2 rect at (5,5).
        gpu.gp0(0xA000_0000);
        gpu.gp0(0x0005_0005); // dst (5,5)
        gpu.gp0(0x0002_0002); // 2x2 = 4 pixels = 2 words
        gpu.gp0(0x2222_1111); // pixels (1111,2222)
        gpu.gp0(0x4444_3333); // pixels (3333,4444)
        assert!(gpu.is_idle());
        assert_eq!(gpu.vram_at(5, 5), 0x1111);
        assert_eq!(gpu.vram_at(6, 5), 0x2222);
        assert_eq!(gpu.vram_at(5, 6), 0x3333);
        assert_eq!(gpu.vram_at(6, 6), 0x4444);

        // VRAM→CPU: read the same rect back.
        gpu.gp0(0xC000_0000);
        gpu.gp0(0x0005_0005);
        gpu.gp0(0x0002_0002);
        assert!(gpu.store_active());
        assert_eq!(gpu.gpuread(), 0x2222_1111);
        assert_eq!(gpu.gpuread(), 0x4444_3333);
        assert!(!gpu.store_active());
    }

    #[test]
    fn vram_to_vram_copy() {
        let mut gpu = Gpu::new();
        gpu.set_vram(0, 0, 0xBEEF);
        gpu.set_vram(1, 0, 0xCAFE);
        // Copy 2x1 block from (0,0) to (10,10).
        gpu.gp0(0x8000_0000);
        gpu.gp0(0x0000_0000); // src (0,0)
        gpu.gp0(0x000A_000A); // dst (10,10)
        gpu.gp0(0x0001_0002); // 2x1
        assert_eq!(gpu.vram_at(10, 10), 0xBEEF);
        assert_eq!(gpu.vram_at(11, 10), 0xCAFE);
    }

    #[test]
    fn gp1_reset_clears_state() {
        let mut gpu = Gpu::new();
        gpu.gp0(0xE300_0000 | 50u32);
        gpu.dma_direction = 2;
        gpu.display_enabled = true;
        gpu.gp1(0x0000_0000); // reset
        assert_eq!(gpu.draw_x0, 0);
        assert_eq!(gpu.dma_direction, 0);
        assert!(!gpu.display_enabled);
    }

    #[test]
    fn gpustat_ready_bits_set_after_reset() {
        let gpu = Gpu::new();
        let stat = gpu.gpustat();
        assert_ne!(stat & (1 << 26), 0, "ready-to-receive-command must be set");
        assert_ne!(stat & (1 << 28), 0, "ready-to-receive-DMA must be set");
    }

    #[test]
    fn gpustat_display_enable_bit_inverted() {
        let mut gpu = Gpu::new();
        // Disabled by default → bit 23 set.
        assert_ne!(gpu.gpustat() & (1 << 23), 0);
        gpu.gp1(0x0300_0000); // display enable (bit0=0 → on)
        assert_eq!(gpu.gpustat() & (1 << 23), 0);
        gpu.gp1(0x0300_0001); // display off
        assert_ne!(gpu.gpustat() & (1 << 23), 0);
    }

    #[test]
    fn gpustat_reflects_dma_direction() {
        let mut gpu = Gpu::new();
        gpu.gp1(0x0400_0002); // dma dir = 2
        let stat = gpu.gpustat();
        assert_eq!((stat >> 29) & 0x3, 2);
    }

    #[test]
    fn polyline_parses_to_terminator_without_desync() {
        let mut gpu = Gpu::new();
        gpu.gp0(0xE300_0000);
        gpu.gp0(0xE400_0000 | 200u32 | (200u32 << 10));
        // Flat poly-line: cmd+color, v0, v1, v2, terminator.
        gpu.gp0(0x48FF_FFFF); // flat poly-line (bit 27 set)
        gpu.gp0(0x0000_0000); // v0
        gpu.gp0(0x0000_0014); // v1
        gpu.gp0(0x0014_0014); // v2
        gpu.gp0(0x5555_5555); // terminator (matches 0x5000_5000 mask)
        assert!(
            gpu.is_idle(),
            "poly-line should terminate and return to idle"
        );
        // A following fill must still land correctly.
        feed(&mut gpu, &[0x0200_00FF, 0x0040_0000, 0x0010_0010]);
        assert_eq!(gpu.vram_at(0, 64), rgb_to_bgr555(0xFF, 0, 0));
    }

    #[test]
    fn unknown_opcode_is_single_word_noop() {
        let mut gpu = Gpu::new();
        gpu.gp0(0x1300_0000); // misc no-op range
        assert!(gpu.is_idle());
        // FIFO still works afterwards.
        feed(&mut gpu, &[0x0200_00FF, 0x0000_0000, 0x0010_0010]);
        assert_eq!(gpu.vram_at(0, 0), rgb_to_bgr555(0xFF, 0, 0));
    }

    #[test]
    fn gp0_irq_sets_status_bit() {
        let mut gpu = Gpu::new();
        gpu.gp0(0x1F00_0000);
        assert!(gpu.irq);
        assert_ne!(gpu.gpustat() & (1 << 24), 0);
        gpu.gp1(0x0200_0000); // ack
        assert!(!gpu.irq);
    }

    // ── Textured rendering ───────────────────────────────────────────────

    /// Sets the drawing area to cover the top-left 128×128 of VRAM.
    fn open_draw_area(gpu: &mut Gpu) {
        gpu.gp0(0xE300_0000); // draw area TL (0,0)
        gpu.gp0(0xE400_0000 | 127u32 | (127u32 << 10)); // BR (127,127)
    }

    #[test]
    fn textured_rect_4bpp_clut_decode() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        // Texpage: base (0, 256), 4bpp CLUT (depth 0).
        gpu.gp0(0xE100_0010);
        // Texture halfword at (0,256): nibbles [t3 t2 t1 t0] = index 2,1,2,1.
        gpu.set_vram(0, 256, 0x2121);
        // CLUT at (0,300): entry 1 = red, entry 2 = green.
        gpu.set_vram(1, 300, 0x001F);
        gpu.set_vram(2, 300, 0x03E0);
        // CLUT selector: clut_x=0, clut_y=300 → field 300<<6 = 0x4B00.
        // Raw textured variable rect (opcode 0x65) at (10,10), 4×1.
        feed(
            &mut gpu,
            &[
                0x6500_0000, // raw textured rect
                0x000A_000A, // vertex (10,10)
                0x4B00_0000, // texcoord u=0 v=0, clut=0x4B00
                0x0001_0004, // size 4×1
            ],
        );
        assert_eq!(gpu.vram_at(10, 10), 0x001F, "u0 -> clut[1] red");
        assert_eq!(gpu.vram_at(11, 10), 0x03E0, "u1 -> clut[2] green");
        assert_eq!(gpu.vram_at(12, 10), 0x001F, "u2 -> clut[1] red");
        assert_eq!(gpu.vram_at(13, 10), 0x03E0, "u3 -> clut[2] green");
    }

    #[test]
    fn textured_rect_8bpp_clut_decode() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        // Texpage base (0,256), 8bpp CLUT (depth 1 → bit7).
        gpu.gp0(0xE100_0090);
        // Halfword at (0,256): low byte = index 1, high byte = index 2.
        gpu.set_vram(0, 256, 0x0201);
        gpu.set_vram(1, 300, 0x001F); // clut[1] red
        gpu.set_vram(2, 300, 0x03E0); // clut[2] green
        feed(
            &mut gpu,
            &[
                0x6500_0000, // raw textured rect
                0x000A_000A, // vertex (10,10)
                0x4B00_0000, // clut=0x4B00
                0x0001_0002, // size 2×1
            ],
        );
        assert_eq!(gpu.vram_at(10, 10), 0x001F, "u0 -> clut[1]");
        assert_eq!(gpu.vram_at(11, 10), 0x03E0, "u1 -> clut[2]");
    }

    #[test]
    fn textured_rect_15bpp_direct_sample() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        // Texpage base (0,256), 15bpp direct (depth 2 → bit8).
        gpu.gp0(0xE100_0110);
        gpu.set_vram(0, 256, 0x1234);
        gpu.set_vram(1, 256, 0x03E0);
        feed(
            &mut gpu,
            &[
                0x6500_0000, // raw textured rect
                0x000A_000A, // vertex (10,10)
                0x0000_0000, // texcoord u=0 v=0 (clut unused for 15bpp)
                0x0001_0002, // size 2×1
            ],
        );
        assert_eq!(gpu.vram_at(10, 10), 0x1234);
        assert_eq!(gpu.vram_at(11, 10), 0x03E0);
    }

    #[test]
    fn transparent_texel_is_skipped() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        gpu.gp0(0xE100_0110); // 15bpp
        gpu.set_vram(0, 256, 0x0000); // fully transparent texel
        gpu.set_vram(10, 10, 0x7FFF); // pre-existing pixel
        feed(
            &mut gpu,
            &[0x6500_0000, 0x000A_000A, 0x0000_0000, 0x0001_0001],
        );
        // The 0x0000 texel is skipped even for an opaque primitive.
        assert_eq!(gpu.vram_at(10, 10), 0x7FFF);
    }

    #[test]
    fn textured_triangle_interpolates_uv() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        // A 4×4 texture patch at page base (0,256): distinct colour per column.
        for u in 0..4u16 {
            for v in 0..4u16 {
                gpu.set_vram(u, 256 + v, 0x0008 * (u + 1));
            }
        }
        // For a textured polygon the texpage is in vertex 1's texcoord word high
        // half (same layout as GP0 E1 low half): 15bpp, page_y=1 → tp = 0x0110.
        feed(
            &mut gpu,
            &[
                0x2500_0000, // raw textured flat triangle
                0x0000_0000, // v0 (0,0)
                0x0000_0000, // v0 clut (unused) + uv (0,0)
                0x0000_0010, // v1 (16,0)
                0x0110_0003, // v1 texpage=0x0110 + uv (3,0)
                0x0010_0000, // v2 (0,16)
                0x0000_0300, // v2 uv (0,3)
            ],
        );
        // Interior pixel (2,2) interpolates to texel (u=0,v=0) — the barycentric
        // weights give u = (32*3)/256 = 0, v = 0 — so it samples the patch's
        // (0,0) texel 0x0008 verbatim (raw texture, no modulation).
        assert_eq!(gpu.vram_at(2, 2), 0x0008);
    }

    #[test]
    fn semi_transparency_all_four_modes() {
        // B (background) and F (foreground) as 5-bit components.
        let bg = pack555(20, 10, 4);
        let f8 = 0x00C0_8040u32; // R=0x40,G=0x80,C=0xC0 → F5 = (8,16,24)
        let cases = [
            (0u32, (14u8, 13u8, 14u8)),
            (1, (28, 26, 28)),
            (2, (12, 0, 0)),
            (3, (22, 14, 10)),
        ];
        for (mode, expect) in cases {
            let mut gpu = Gpu::new();
            open_draw_area(&mut gpu);
            gpu.gp0(0xE100_0000 | (mode << 5)); // latch semi mode via E1
            gpu.set_vram(10, 10, bg);
            // Untextured semi-transparent 1×1 rect (opcode 0x6A).
            feed(&mut gpu, &[0x6A00_0000 | f8, 0x000A_000A]);
            assert_eq!(
                gpu.vram_at(10, 10),
                pack555(expect.0, expect.1, expect.2),
                "semi mode {mode}"
            );
        }
    }

    #[test]
    fn blend_mode0_rounds_full_sum() {
        // Mode 0 is (B+F)/2 over the full sum, not (B/2)+(F/2): odd operands
        // must not each lose their low bit before the add.
        // B=3,F=3 → 3 (the truncating form gave 2); B=5,F=7 → 6 (gave 5);
        // B=1,F=3 → 2 (matches either form).
        assert_eq!(blend(0, (3, 5, 1), (3, 7, 3)), (3, 6, 2));
    }

    #[test]
    fn textured_polygon_latches_texpage_into_gpustat() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        // Textured flat triangle. Vertex-1's texcoord word high half carries the
        // texpage attribute (same bit layout as GP0 E1's low half):
        // page_x=2, page_y=1, semi=2, depth=1 → 0x00D2.
        feed(
            &mut gpu,
            &[
                0x2400_0000, // textured flat triangle
                0x0000_0000, // v0 pos (0,0)
                0x0000_0000, // v0 clut + uv
                0x0000_0010, // v1 pos (16,0)
                0x00D2_0000, // v1 texpage=0x00D2 + uv (0,0)
                0x0010_0000, // v2 pos (0,16)
                0x0000_0000, // v2 uv (0,0)
            ],
        );
        // The polygon reloads GPUSTAT bits 0–8 exactly like GP0(E1) would.
        assert_eq!(gpu.gpustat() & 0x1FF, 0xD2, "polygon latched texpage");

        // A textured rectangle carries no texpage word, so it must leave the
        // latched texpage untouched (it draws using the latched page/mode).
        feed(&mut gpu, &[0x6C00_0000, 0x0005_0005, 0x0000_0000]);
        assert_eq!(gpu.gpustat() & 0x1FF, 0xD2, "rect did not change texpage");
    }

    #[test]
    fn modulation_neutral_and_half() {
        // 15bpp texel with all channels = 16.
        let texel = pack555(16, 16, 16);
        // Neutral colour 0x80 leaves the texel unchanged.
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        gpu.gp0(0xE100_0110);
        gpu.set_vram(0, 256, texel);
        feed(&mut gpu, &[0x6C80_8080, 0x000A_000A, 0x0000_0000]); // modulated 1×1
        assert_eq!(gpu.vram_at(10, 10), pack555(16, 16, 16));

        // Colour 0x40 halves each channel.
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        gpu.gp0(0xE100_0110);
        gpu.set_vram(0, 256, texel);
        feed(&mut gpu, &[0x6C40_4040, 0x000A_000A, 0x0000_0000]);
        assert_eq!(gpu.vram_at(10, 10), pack555(8, 8, 8));

        // Raw texture (opcode bit24) ignores the colour entirely.
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        gpu.gp0(0xE100_0110);
        gpu.set_vram(0, 256, texel);
        feed(&mut gpu, &[0x6D40_4040, 0x000A_000A, 0x0000_0000]);
        assert_eq!(gpu.vram_at(10, 10), pack555(16, 16, 16));
    }

    #[test]
    fn texture_window_masks_uv() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        gpu.gp0(0xE100_0110); // 15bpp, base (0,256)
        gpu.set_vram(0, 256, 0x1234); // texel at u=0
        gpu.set_vram(8, 256, 0x5678); // texel at u=8
        // Texture window mask_x = 1 (×8 = 8): u=8 wraps to u=0.
        gpu.gp0(0xE200_0001);
        feed(
            &mut gpu,
            &[0x6500_0000, 0x000A_000A, 0x0000_0008, 0x0001_0001], // raw, u=8
        );
        assert_eq!(gpu.vram_at(10, 10), 0x1234, "u=8 masked back to u=0");
    }

    #[test]
    fn mask_bit_check_and_set() {
        // check-mask: a destination with bit15 set is not overwritten.
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        gpu.gp0(0xE600_0002); // check-mask-before-draw
        gpu.set_vram(10, 10, 0xBEEF); // bit15 set
        feed(&mut gpu, &[0x6800_00FF, 0x000A_000A]); // opaque 1×1 red rect
        assert_eq!(gpu.vram_at(10, 10), 0xBEEF, "masked pixel preserved");

        // set-mask: written pixels get bit15 forced on.
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        gpu.gp0(0xE600_0001); // set-mask-while-drawing
        feed(&mut gpu, &[0x6800_00FF, 0x000A_000A]);
        assert_ne!(gpu.vram_at(10, 10) & 0x8000, 0, "written pixel masked");
    }

    #[test]
    fn textured_stp_texel_blends_when_semi() {
        // A texel with the STP bit set, on a semi-transparent primitive, blends.
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        gpu.gp0(0xE100_0110); // 15bpp, semi mode 0 (average)
        let texel = 0x8000 | pack555(16, 16, 16); // STP set
        gpu.set_vram(0, 256, texel);
        gpu.set_vram(10, 10, pack555(8, 8, 8)); // background
        // Semi-transparent raw textured 1×1 rect (opcode 0x67).
        feed(
            &mut gpu,
            &[0x6700_0000, 0x000A_000A, 0x0000_0000, 0x0001_0001],
        );
        // Mode 0: (bg/2 + fg/2) = (4+8) = 12 per channel; STP forces bit15.
        assert_eq!(gpu.vram_at(10, 10), 0x8000 | pack555(12, 12, 12));
    }

    #[test]
    fn dither_applies_to_gouraud_only_when_enabled() {
        // With dithering enabled, a flat mid-grey Gouraud triangle gains a
        // spatially-varying low bit; the same fill without dithering is uniform.
        let build = |dither: bool| -> Gpu {
            let mut gpu = Gpu::new();
            open_draw_area(&mut gpu);
            // E1: dither bit (9) optionally set.
            gpu.gp0(0xE100_0000 | (u32::from(dither) << 9));
            // Gouraud triangle, all three vertices colour 0x83 (just above a
            // 5-bit boundary) so dithering can push some pixels up/down. The
            // right triangle (0,0)-(40,0)-(0,40) keeps row y=4 well inside.
            feed(
                &mut gpu,
                &[
                    0x3083_8383, // gouraud tri, colour0 = 0x83 grey
                    0x0000_0000, // v0 (0,0)
                    0x0083_8383, // colour1
                    0x0000_0028, // v1 (40,0)
                    0x0083_8383, // colour2
                    0x0028_0000, // v2 (0,40)
                ],
            );
            gpu
        };
        let plain = build(false);
        let dithered = build(true);
        // Undithered: every interior pixel on the row is identical.
        assert_eq!(
            plain.vram_at(10, 4),
            plain.vram_at(11, 4),
            "no dither → uniform"
        );
        // Dithered: at least one interior pixel differs from another on the row.
        let mut varies = false;
        for x in 6..30u16 {
            if dithered.vram_at(x, 4) != dithered.vram_at(6, 4) {
                varies = true;
                break;
            }
        }
        assert!(varies, "dither → spatial variation");
    }

    // ── Lines ────────────────────────────────────────────────────────────

    #[test]
    fn monochrome_line_renders_single_colour() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        // Flat line (opcode 0x40) red from (0,0) to (10,0).
        feed(
            &mut gpu,
            &[
                0x4000_00FF, // flat line, colour red
                0x0000_0000, // v0 (0,0)
                0x0000_000A, // v1 (10,0)
            ],
        );
        let red = rgb_to_bgr555(0xFF, 0, 0);
        assert_eq!(gpu.vram_at(0, 0), red, "endpoint");
        assert_eq!(gpu.vram_at(5, 0), red, "midpoint uniform");
        assert_eq!(gpu.vram_at(10, 0), red, "endpoint");
    }

    #[test]
    fn shaded_line_interpolates_between_endpoint_colours() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        // Gouraud line (opcode 0x50): red at v0 (0,0), blue at v1 (10,0).
        feed(
            &mut gpu,
            &[
                0x5000_00FF, // shaded line, colour0 = red
                0x0000_0000, // v0 (0,0)
                0x00FF_0000, // colour1 = blue
                0x0000_000A, // v1 (10,0)
            ],
        );
        // Endpoints match their vertex colours exactly.
        assert_eq!(gpu.vram_at(0, 0), rgb_to_bgr555(0xFF, 0, 0), "v0 red");
        assert_eq!(gpu.vram_at(10, 0), rgb_to_bgr555(0, 0, 0xFF), "v1 blue");
        // The midpoint is a blend of both endpoints — not the first-vertex colour.
        let mid = gpu.vram_at(5, 0);
        let (r, g, b) = unpack5(mid);
        assert!(r > 0 && b > 0, "midpoint blends red+blue, got {r},{g},{b}");
        assert_eq!(g, 0, "no green anywhere on this line");
        assert_ne!(mid, rgb_to_bgr555(0xFF, 0, 0), "midpoint is not pure red");
    }

    #[test]
    fn shaded_polyline_interpolates_each_segment() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        // Gouraud poly-line (opcode 0x58, bit27 set): red→green→blue across two
        // segments (0,0)→(10,0)→(20,0).
        gpu.gp0(0x5800_00FF); // shaded poly-line, colour0 = red
        gpu.gp0(0x0000_0000); // v0 (0,0)
        gpu.gp0(0x0000_FF00); // colour1 = green
        gpu.gp0(0x0000_000A); // v1 (10,0)
        gpu.gp0(0x00FF_0000); // colour2 = blue
        gpu.gp0(0x0000_0014); // v2 (20,0)
        gpu.gp0(0x5555_5555); // terminator
        assert!(gpu.is_idle());
        // Vertices carry their exact colours.
        assert_eq!(gpu.vram_at(0, 0), rgb_to_bgr555(0xFF, 0, 0), "v0 red");
        assert_eq!(gpu.vram_at(10, 0), rgb_to_bgr555(0, 0xFF, 0), "v1 green");
        assert_eq!(gpu.vram_at(20, 0), rgb_to_bgr555(0, 0, 0xFF), "v2 blue");
        // Segment 1 midpoint blends red+green; segment 2 midpoint blends green+blue.
        let (r1, g1, b1) = unpack5(gpu.vram_at(5, 0));
        assert!(
            r1 > 0 && g1 > 0 && b1 == 0,
            "seg1 red→green: {r1},{g1},{b1}"
        );
        let (r2, g2, b2) = unpack5(gpu.vram_at(15, 0));
        assert!(
            r2 == 0 && g2 > 0 && b2 > 0,
            "seg2 green→blue: {r2},{g2},{b2}"
        );
    }

    #[test]
    fn semi_transparent_line_blends_against_background() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        gpu.gp0(0xE100_0000); // latch semi mode 0 (B/2 + F/2)
        let bg = pack555(20, 10, 4);
        gpu.set_vram(5, 5, bg);
        // Semi-transparent flat line (opcode 0x42) across y=5, colour F5=(8,16,24).
        feed(
            &mut gpu,
            &[
                0x42C0_8040, // semi flat line, R=0x40 G=0x80 B=0xC0
                0x0005_0000, // v0 (0,5)
                0x0005_000A, // v1 (10,5)
            ],
        );
        // Mode 0 average: ((20+8)/2, (10+16)/2, (4+24)/2) = (14,13,14).
        assert_eq!(gpu.vram_at(5, 5), pack555(14, 13, 14));
    }

    #[test]
    fn mask_check_blocks_a_line_pixel() {
        let mut gpu = Gpu::new();
        open_draw_area(&mut gpu);
        gpu.gp0(0xE600_0002); // check-mask-before-draw
        gpu.set_vram(5, 5, 0xBEEF); // bit15 set → masked destination
        // Opaque flat line across y=5.
        feed(&mut gpu, &[0x4000_00FF, 0x0005_0000, 0x0005_000A]);
        assert_eq!(gpu.vram_at(5, 5), 0xBEEF, "masked line pixel preserved");
        // A neighbouring unmasked pixel on the same line is still drawn.
        assert_eq!(gpu.vram_at(4, 5), rgb_to_bgr555(0xFF, 0, 0));
    }
}
