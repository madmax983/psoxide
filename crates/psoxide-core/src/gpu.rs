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
//! flat/Gouraud triangles and quads, monochrome rectangles, flat lines, fills,
//! and VRAM↔VRAM / CPU↔VRAM block transfers. Textured primitives are parsed
//! (correct word counts, no FIFO desync) but rendered as flat shaded fills —
//! real texture sampling is a documented gap. Poly-lines are parsed to their
//! terminator and each segment is drawn flat with the first color.

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
        self.tex_page_x = (cmd & 0xF) as u8;
        self.tex_page_y = ((cmd >> 4) & 0x1) as u8;
        self.semi_transparency = ((cmd >> 5) & 0x3) as u8;
        self.tex_depth = ((cmd >> 7) & 0x3) as u8;
        self.dither = (cmd >> 9) & 0x1 != 0;
        self.draw_to_display = (cmd >> 10) & 0x1 != 0;
        self.tex_disable = (cmd >> 11) & 0x1 != 0;
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
        let is_vertex_slot = if self.polyline_shaded {
            // Shaded: cmd, col?, v0, col, v1, col, v2, ... colors at even indices
            // after the header. Word layout: [0]=cmd+col0, [1]=v0, [2]=col1,
            // [3]=v1, ... vertices at odd indices.
            have >= 1 && have % 2 == 1
        } else {
            // Flat: cmd+col, v0, v1, v2, ... vertices at index >= 1.
            have >= 1
        };
        if is_vertex_slot && (word & 0xF000_F000) == 0x5000_5000 {
            self.render_polyline();
            self.polyline_active = false;
            self.polyline_shaded = false;
            self.cmd_buffer.clear();
            return;
        }
        self.cmd_buffer.push(word);
    }

    fn render_polyline(&mut self) {
        // Extract vertices and (for flat) the single color; draw each segment
        // with Bresenham using the first color. Gouraud shading across the line
        // is approximated with the first vertex color (documented gap).
        let words = self.cmd_buffer.clone();
        if words.is_empty() {
            return;
        }
        let base_color = words[0];
        let mut verts: Vec<(i32, i32)> = Vec::new();
        if self.polyline_shaded {
            let mut i = 1;
            while i < words.len() {
                verts.push(decode_vertex(words[i]));
                i += 2; // skip the following color word
            }
        } else {
            for w in &words[1..] {
                verts.push(decode_vertex(*w));
            }
        }
        let (r, g, b) = color_channels(base_color);
        let color = rgb_to_bgr555(r, g, b);
        for pair in verts.windows(2) {
            self.draw_line(pair[0], pair[1], color);
        }
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

    /// Writes a pixel if it lies within the drawing area.
    #[inline]
    fn plot_clipped(&mut self, x: i32, y: i32, color: u16) {
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
        self.set_vram(x as u16, y as u16, color);
    }

    fn draw_polygon(&mut self) {
        let cmd = self.cmd_buffer[0];
        let shaded = cmd & 0x1000_0000 != 0;
        let quad = cmd & 0x0800_0000 != 0;
        let textured = cmd & 0x0400_0000 != 0;
        let nv = if quad { 4 } else { 3 };

        // Parse vertices and their colors from the accumulated buffer.
        let mut verts: Vec<(i32, i32)> = Vec::with_capacity(nv);
        let mut colors: Vec<u16> = Vec::with_capacity(nv);
        let base_color = cmd;
        let mut idx = 1usize;
        for v in 0..nv {
            let color_word = if shaded {
                if v == 0 {
                    base_color
                } else {
                    let c = self.cmd_buffer[idx];
                    idx += 1;
                    c
                }
            } else {
                base_color
            };
            let vword = self.cmd_buffer[idx];
            idx += 1;
            if textured {
                idx += 1; // skip texcoord/palette/page word
            }
            let (vx, vy) = decode_vertex(vword);
            verts.push((
                vx + i32::from(self.draw_off_x),
                vy + i32::from(self.draw_off_y),
            ));
            let (r, g, b) = color_channels(color_word);
            colors.push(rgb_to_bgr555(r, g, b));
        }

        // Textured polygons are rendered flat (documented gap): use a neutral
        // mid-gray if untextured shading is unavailable, else the vertex color.
        self.raster_triangle(
            verts[0], verts[1], verts[2], colors[0], colors[1], colors[2],
        );
        if quad {
            self.raster_triangle(
                verts[1], verts[2], verts[3], colors[1], colors[2], colors[3],
            );
        }
    }

    /// Rasterizes a triangle with barycentric Gouraud interpolation, clipped to
    /// the drawing area.
    #[allow(clippy::too_many_arguments)]
    fn raster_triangle(
        &mut self,
        a: (i32, i32),
        b: (i32, i32),
        c: (i32, i32),
        ca: u16,
        cb: u16,
        cc: u16,
    ) {
        let min_x = a.0.min(b.0).min(c.0).max(i32::from(self.draw_x0));
        let max_x = a.0.max(b.0).max(c.0).min(i32::from(self.draw_x1));
        let min_y = a.1.min(b.1).min(c.1).max(i32::from(self.draw_y0));
        let max_y = a.1.max(b.1).max(c.1).min(i32::from(self.draw_y1));

        let area = edge(a, b, c);
        if area == 0 {
            return; // degenerate
        }

        // Unpack the three vertex colors for interpolation.
        let (ra, ga, ba) = unpack_bgr555(ca);
        let (rb, gb, bb) = unpack_bgr555(cb);
        let (rc, gc, bc) = unpack_bgr555(cc);

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let p = (x, y);
                let w0 = edge(b, c, p);
                let w1 = edge(c, a, p);
                let w2 = edge(a, b, p);
                // Inside test works for either winding.
                let inside = (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0);
                if !inside {
                    continue;
                }
                let (l0, l1, l2) = (
                    w0 as f32 / area as f32,
                    w1 as f32 / area as f32,
                    w2 as f32 / area as f32,
                );
                let r = (l0 * ra as f32 + l1 * rb as f32 + l2 * rc as f32) as u8;
                let g = (l0 * ga as f32 + l1 * gb as f32 + l2 * gc as f32) as u8;
                let bl = (l0 * ba as f32 + l1 * bb as f32 + l2 * bc as f32) as u8;
                self.plot_clipped(x, y, pack5(r, g, bl));
            }
        }
    }

    fn draw_rectangle(&mut self) {
        let cmd = self.cmd_buffer[0];
        let size = (cmd >> 27) & 0x3;
        let textured = cmd & 0x0400_0000 != 0;
        let xy = self.cmd_buffer[1];
        let (vx, vy) = decode_vertex(xy);
        let x0 = vx + i32::from(self.draw_off_x);
        let y0 = vy + i32::from(self.draw_off_y);
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
        let color = rgb_to_bgr555(r, g, b);
        for row in 0..h {
            for col in 0..w {
                self.plot_clipped(x0 + col, y0 + row, color);
            }
        }
    }

    fn draw_single_line(&mut self) {
        let cmd = self.cmd_buffer[0];
        let shaded = cmd & 0x1000_0000 != 0;
        let (r, g, b) = color_channels(cmd);
        let color = rgb_to_bgr555(r, g, b);
        let (v0, v1) = if shaded {
            (
                decode_vertex(self.cmd_buffer[1]),
                decode_vertex(self.cmd_buffer[3]),
            )
        } else {
            (
                decode_vertex(self.cmd_buffer[1]),
                decode_vertex(self.cmd_buffer[2]),
            )
        };
        let a = (
            v0.0 + i32::from(self.draw_off_x),
            v0.1 + i32::from(self.draw_off_y),
        );
        let bb = (
            v1.0 + i32::from(self.draw_off_x),
            v1.1 + i32::from(self.draw_off_y),
        );
        self.draw_line(a, bb, color);
    }

    /// Draws a Bresenham line clipped to the drawing area.
    fn draw_line(&mut self, a: (i32, i32), b: (i32, i32), color: u16) {
        let (mut x0, mut y0) = a;
        let (x1, y1) = b;
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        loop {
            self.plot_clipped(x0, y0, color);
            if x0 == x1 && y0 == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x0 += sx;
            }
            if e2 <= dx {
                err += dx;
                y0 += sy;
            }
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

/// Unpacks a BGR555 pixel into 8-bit-per-channel R/G/B.
#[inline]
fn unpack_bgr555(p: u16) -> (u8, u8, u8) {
    let r = ((p & 0x1F) << 3) as u8;
    let g = (((p >> 5) & 0x1F) << 3) as u8;
    let b = (((p >> 10) & 0x1F) << 3) as u8;
    (r, g, b)
}

/// Packs 8-bit R/G/B back into BGR555.
#[inline]
fn pack5(r: u8, g: u8, b: u8) -> u16 {
    rgb_to_bgr555(r, g, b)
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
        let (r, g, b) = unpack_bgr555(p);
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
}
