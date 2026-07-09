//! Geometry Transformation Engine (GTE, coprocessor 2).
//!
//! The GTE is the PlayStation's fixed-point vector/matrix coprocessor. It owns
//! two banks of 32 registers each — the **data** registers (`cop2r0..31`,
//! reached by `MFC2`/`MTC2`/`LWC2`/`SWC2`) and the **control** registers
//! (`cop2r32..63`, reached by `CFC2`/`CTC2`) — and a set of ~24 fixed-function
//! operations (`RTPS`, `NCDS`, `MVMVA`, …) driven by a 25-bit command word.
//!
//! This implementation follows the Nocash "psx-spx" GTE reference for register
//! layout, operation semantics, saturation/overflow flag bits, and the
//! Unsigned-Newton-Raphson (`UNR`) reciprocal used by the perspective
//! transform. All arithmetic is deterministic and side-effect free (no I/O), so
//! the whole engine is snapshot-serializable.

use serde::{Deserialize, Serialize};

/// The 257-entry Unsigned-Newton-Raphson reciprocal seed table used by the
/// perspective-transform division (`H / SZ`). These are the exact constants
/// from the GTE hardware (Nocash `unr_table`).
#[rustfmt::skip]
const UNR_TABLE: [u8; 257] = [
    0xFF, 0xFD, 0xFB, 0xF9, 0xF7, 0xF5, 0xF3, 0xF1, 0xEF, 0xEE, 0xEC, 0xEA, 0xE8, 0xE6, 0xE4, 0xE3,
    0xE1, 0xDF, 0xDD, 0xDC, 0xDA, 0xD8, 0xD6, 0xD5, 0xD3, 0xD1, 0xD0, 0xCE, 0xCD, 0xCB, 0xC9, 0xC8,
    0xC6, 0xC5, 0xC3, 0xC1, 0xC0, 0xBE, 0xBD, 0xBB, 0xBA, 0xB8, 0xB7, 0xB5, 0xB4, 0xB2, 0xB1, 0xB0,
    0xAE, 0xAD, 0xAB, 0xAA, 0xA9, 0xA7, 0xA6, 0xA4, 0xA3, 0xA2, 0xA0, 0x9F, 0x9E, 0x9C, 0x9B, 0x9A,
    0x99, 0x97, 0x96, 0x95, 0x94, 0x92, 0x91, 0x90, 0x8F, 0x8D, 0x8C, 0x8B, 0x8A, 0x89, 0x87, 0x86,
    0x85, 0x84, 0x83, 0x82, 0x81, 0x7F, 0x7E, 0x7D, 0x7C, 0x7B, 0x7A, 0x79, 0x78, 0x77, 0x75, 0x74,
    0x73, 0x72, 0x71, 0x70, 0x6F, 0x6E, 0x6D, 0x6C, 0x6B, 0x6A, 0x69, 0x68, 0x67, 0x66, 0x65, 0x64,
    0x63, 0x62, 0x61, 0x60, 0x5F, 0x5E, 0x5D, 0x5D, 0x5C, 0x5B, 0x5A, 0x59, 0x58, 0x57, 0x56, 0x55,
    0x54, 0x53, 0x53, 0x52, 0x51, 0x50, 0x4F, 0x4E, 0x4D, 0x4D, 0x4C, 0x4B, 0x4A, 0x49, 0x48, 0x48,
    0x47, 0x46, 0x45, 0x44, 0x43, 0x43, 0x42, 0x41, 0x40, 0x3F, 0x3F, 0x3E, 0x3D, 0x3C, 0x3C, 0x3B,
    0x3A, 0x39, 0x39, 0x38, 0x37, 0x36, 0x36, 0x35, 0x34, 0x33, 0x33, 0x32, 0x31, 0x31, 0x30, 0x2F,
    0x2E, 0x2E, 0x2D, 0x2C, 0x2C, 0x2B, 0x2A, 0x2A, 0x29, 0x28, 0x28, 0x27, 0x26, 0x26, 0x25, 0x24,
    0x24, 0x23, 0x22, 0x22, 0x21, 0x20, 0x20, 0x1F, 0x1E, 0x1E, 0x1D, 0x1D, 0x1C, 0x1B, 0x1B, 0x1A,
    0x19, 0x19, 0x18, 0x18, 0x17, 0x16, 0x16, 0x15, 0x15, 0x14, 0x14, 0x13, 0x12, 0x12, 0x11, 0x11,
    0x10, 0x0F, 0x0F, 0x0E, 0x0E, 0x0D, 0x0D, 0x0C, 0x0C, 0x0B, 0x0A, 0x0A, 0x09, 0x09, 0x08, 0x08,
    0x07, 0x07, 0x06, 0x06, 0x05, 0x05, 0x04, 0x04, 0x03, 0x03, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00,
    0x00,
];

/// The bits of the `FLAG` (control register 31) error-summary mask. Bit 31 of
/// `FLAG` is the logical OR of these bits (bits 30..=23 and 18..=13).
const FLAG_ERROR_MASK: u32 = 0x7F87_E000;

/// The Geometry Transformation Engine register file and state.
///
/// Registers are stored semantically (matrices as `i16` element grids, FIFOs as
/// small arrays) and re-assembled into the packed 32-bit view expected by
/// `MFC2`/`CFC2` on read. All fields are plain integers, so the struct is
/// trivially `Clone`/`PartialEq`/serde-serializable for save states.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct Gte {
    // ── Data registers ──────────────────────────────────────────────────
    /// The three input vectors V0, V1, V2, each `[x, y, z]` (`cop2r0..5`).
    v: [[i16; 3]; 3],
    /// Packed color/code register RGBC (`cop2r6`): R, G, B, CODE bytes.
    rgbc: u32,
    /// Average Z value OTZ (`cop2r7`).
    otz: u16,
    /// The interpolation/vector accumulators IR0..IR3 (`cop2r8..11`).
    ir: [i16; 4],
    /// Screen XY coordinate FIFO SXY0/1/2 (`cop2r12..14`), each `(x, y)`.
    sxy: [(i16, i16); 3],
    /// Screen Z coordinate FIFO SZ0/1/2/3 (`cop2r16..19`).
    sz: [u16; 4],
    /// Color FIFO RGB0/1/2 (`cop2r20..22`).
    rgb: [u32; 3],
    /// Prohibited scratch register RES1 (`cop2r23`) — stored raw.
    res1: u32,
    /// Maths accumulators MAC0..MAC3 (`cop2r24..27`).
    mac: [i32; 4],
    /// Leading-zero-count source LZCS (`cop2r30`).
    lzcs: i32,
    /// Leading-zero-count result LZCR (`cop2r31`).
    lzcr: u32,

    // ── Control registers ───────────────────────────────────────────────
    /// Rotation matrix RT (`cop2r32..36`).
    rt: [[i16; 3]; 3],
    /// Translation vector TRX/TRY/TRZ (`cop2r37..39`).
    tr: [i32; 3],
    /// Light-direction matrix LLM (`cop2r40..44`).
    llm: [[i16; 3]; 3],
    /// Background color RBK/GBK/BBK (`cop2r45..47`).
    bk: [i32; 3],
    /// Light-color matrix LCM (`cop2r48..52`).
    lcm: [[i16; 3]; 3],
    /// Far color RFC/GFC/BFC (`cop2r53..55`).
    fc: [i32; 3],
    /// Screen offset X OFX (`cop2r56`).
    ofx: i32,
    /// Screen offset Y OFY (`cop2r57`).
    ofy: i32,
    /// Projection-plane distance H (`cop2r58`) — written u16, read sign-extended.
    h: u16,
    /// Depth-cue coefficient DQA (`cop2r59`).
    dqa: i16,
    /// Depth-cue offset DQB (`cop2r60`).
    dqb: i32,
    /// Average-Z scale factor ZSF3 (`cop2r61`).
    zsf3: i16,
    /// Average-Z scale factor ZSF4 (`cop2r62`).
    zsf4: i16,
    /// Calculation error FLAG (`cop2r63`).
    flag: u32,
}

/// Packs two signed 16-bit halves into a 32-bit word (`lo` low, `hi` high).
#[inline]
fn pack(lo: i16, hi: i16) -> u32 {
    (u32::from(lo as u16)) | (u32::from(hi as u16) << 16)
}

/// Splits a 32-bit word into its two signed 16-bit halves `(lo, hi)`.
#[inline]
fn unpack(v: u32) -> (i16, i16) {
    (v as u16 as i16, (v >> 16) as u16 as i16)
}

impl Gte {
    /// Creates a GTE in its power-on (all-zero) state.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    // ════════════════════════════════════════════════════════════════════
    //  Data register access (cop2r0..31)
    // ════════════════════════════════════════════════════════════════════

    /// Reads GTE **data** register `rd` (`0..=31`) as a 32-bit word, applying
    /// the hardware sign/zero-extension quirks.
    #[must_use]
    pub fn read_data(&self, rd: u8) -> u32 {
        match rd & 0x1F {
            0 => pack(self.v[0][0], self.v[0][1]),
            1 => self.v[0][2] as i32 as u32,
            2 => pack(self.v[1][0], self.v[1][1]),
            3 => self.v[1][2] as i32 as u32,
            4 => pack(self.v[2][0], self.v[2][1]),
            5 => self.v[2][2] as i32 as u32,
            6 => self.rgbc,
            7 => u32::from(self.otz),
            8 => self.ir[0] as i32 as u32,
            9 => self.ir[1] as i32 as u32,
            10 => self.ir[2] as i32 as u32,
            11 => self.ir[3] as i32 as u32,
            12 => pack(self.sxy[0].0, self.sxy[0].1),
            13 => pack(self.sxy[1].0, self.sxy[1].1),
            14 => pack(self.sxy[2].0, self.sxy[2].1),
            // SXYP mirrors SXY2 on read.
            15 => pack(self.sxy[2].0, self.sxy[2].1),
            16 => u32::from(self.sz[0]),
            17 => u32::from(self.sz[1]),
            18 => u32::from(self.sz[2]),
            19 => u32::from(self.sz[3]),
            20 => self.rgb[0],
            21 => self.rgb[1],
            22 => self.rgb[2],
            23 => self.res1,
            24 => self.mac[0] as u32,
            25 => self.mac[1] as u32,
            26 => self.mac[2] as u32,
            27 => self.mac[3] as u32,
            // IRGB / ORGB: pack IR1/IR2/IR3 (each /0x80, clamped 0..0x1F) as 5:5:5.
            28 | 29 => self.read_irgb(),
            30 => self.lzcs as u32,
            31 => self.lzcr,
            _ => unreachable!(),
        }
    }

    /// Writes GTE **data** register `rd` (`0..=31`).
    pub fn write_data(&mut self, rd: u8, value: u32) {
        match rd & 0x1F {
            0 => {
                let (x, y) = unpack(value);
                self.v[0][0] = x;
                self.v[0][1] = y;
            }
            1 => self.v[0][2] = value as u16 as i16,
            2 => {
                let (x, y) = unpack(value);
                self.v[1][0] = x;
                self.v[1][1] = y;
            }
            3 => self.v[1][2] = value as u16 as i16,
            4 => {
                let (x, y) = unpack(value);
                self.v[2][0] = x;
                self.v[2][1] = y;
            }
            5 => self.v[2][2] = value as u16 as i16,
            6 => self.rgbc = value,
            7 => self.otz = value as u16,
            8 => self.ir[0] = value as u16 as i16,
            9 => self.ir[1] = value as u16 as i16,
            10 => self.ir[2] = value as u16 as i16,
            11 => self.ir[3] = value as u16 as i16,
            12 => self.sxy[0] = unpack(value),
            13 => self.sxy[1] = unpack(value),
            14 => self.sxy[2] = unpack(value),
            // SXYP write pushes the screen-XY FIFO.
            15 => {
                self.sxy[0] = self.sxy[1];
                self.sxy[1] = self.sxy[2];
                self.sxy[2] = unpack(value);
            }
            16 => self.sz[0] = value as u16,
            17 => self.sz[1] = value as u16,
            18 => self.sz[2] = value as u16,
            19 => self.sz[3] = value as u16,
            20 => self.rgb[0] = value,
            21 => self.rgb[1] = value,
            22 => self.rgb[2] = value,
            23 => self.res1 = value,
            24 => self.mac[0] = value as i32,
            25 => self.mac[1] = value as i32,
            26 => self.mac[2] = value as i32,
            27 => self.mac[3] = value as i32,
            28 => {
                // IRGB write: expand 5:5:5 into IR1/IR2/IR3 (each *0x80).
                self.ir[1] = ((value & 0x1F) * 0x80) as i16;
                self.ir[2] = (((value >> 5) & 0x1F) * 0x80) as i16;
                self.ir[3] = (((value >> 10) & 0x1F) * 0x80) as i16;
            }
            // ORGB is read-only; writes are ignored.
            29 => {}
            30 => {
                self.lzcs = value as i32;
                self.lzcr = leading_bit_count(value);
            }
            // LZCR is read-only; writes are ignored.
            31 => {}
            _ => unreachable!(),
        }
    }

    /// Packs IR1/IR2/IR3 into the 15-bit IRGB/ORGB representation (each channel
    /// `IRn / 0x80`, clamped to `0..=0x1F`).
    fn read_irgb(&self) -> u32 {
        let chan = |v: i16| (v / 0x80).clamp(0, 0x1F) as u32;
        chan(self.ir[1]) | (chan(self.ir[2]) << 5) | (chan(self.ir[3]) << 10)
    }

    // ════════════════════════════════════════════════════════════════════
    //  Control register access (cop2r32..63)
    // ════════════════════════════════════════════════════════════════════

    /// Reads GTE **control** register `rd` (`0..=31`, i.e. `cop2r32..63`).
    #[must_use]
    pub fn read_control(&self, rd: u8) -> u32 {
        match rd & 0x1F {
            0 => pack(self.rt[0][0], self.rt[0][1]),
            1 => pack(self.rt[0][2], self.rt[1][0]),
            2 => pack(self.rt[1][1], self.rt[1][2]),
            3 => pack(self.rt[2][0], self.rt[2][1]),
            4 => self.rt[2][2] as i32 as u32,
            5 => self.tr[0] as u32,
            6 => self.tr[1] as u32,
            7 => self.tr[2] as u32,
            8 => pack(self.llm[0][0], self.llm[0][1]),
            9 => pack(self.llm[0][2], self.llm[1][0]),
            10 => pack(self.llm[1][1], self.llm[1][2]),
            11 => pack(self.llm[2][0], self.llm[2][1]),
            12 => self.llm[2][2] as i32 as u32,
            13 => self.bk[0] as u32,
            14 => self.bk[1] as u32,
            15 => self.bk[2] as u32,
            16 => pack(self.lcm[0][0], self.lcm[0][1]),
            17 => pack(self.lcm[0][2], self.lcm[1][0]),
            18 => pack(self.lcm[1][1], self.lcm[1][2]),
            19 => pack(self.lcm[2][0], self.lcm[2][1]),
            20 => self.lcm[2][2] as i32 as u32,
            21 => self.fc[0] as u32,
            22 => self.fc[1] as u32,
            23 => self.fc[2] as u32,
            24 => self.ofx as u32,
            25 => self.ofy as u32,
            // H is written unsigned but read back sign-extended (hardware quirk).
            26 => self.h as i16 as i32 as u32,
            27 => self.dqa as i32 as u32,
            28 => self.dqb as u32,
            29 => self.zsf3 as i32 as u32,
            30 => self.zsf4 as i32 as u32,
            31 => self.flag,
            _ => unreachable!(),
        }
    }

    /// Writes GTE **control** register `rd` (`0..=31`, i.e. `cop2r32..63`).
    pub fn write_control(&mut self, rd: u8, value: u32) {
        match rd & 0x1F {
            0 => set_pair(&mut self.rt, 0, value),
            1 => {
                let (a, b) = unpack(value);
                self.rt[0][2] = a;
                self.rt[1][0] = b;
            }
            2 => {
                let (a, b) = unpack(value);
                self.rt[1][1] = a;
                self.rt[1][2] = b;
            }
            3 => {
                let (a, b) = unpack(value);
                self.rt[2][0] = a;
                self.rt[2][1] = b;
            }
            4 => self.rt[2][2] = value as u16 as i16,
            5 => self.tr[0] = value as i32,
            6 => self.tr[1] = value as i32,
            7 => self.tr[2] = value as i32,
            8 => set_pair(&mut self.llm, 0, value),
            9 => {
                let (a, b) = unpack(value);
                self.llm[0][2] = a;
                self.llm[1][0] = b;
            }
            10 => {
                let (a, b) = unpack(value);
                self.llm[1][1] = a;
                self.llm[1][2] = b;
            }
            11 => {
                let (a, b) = unpack(value);
                self.llm[2][0] = a;
                self.llm[2][1] = b;
            }
            12 => self.llm[2][2] = value as u16 as i16,
            13 => self.bk[0] = value as i32,
            14 => self.bk[1] = value as i32,
            15 => self.bk[2] = value as i32,
            16 => set_pair(&mut self.lcm, 0, value),
            17 => {
                let (a, b) = unpack(value);
                self.lcm[0][2] = a;
                self.lcm[1][0] = b;
            }
            18 => {
                let (a, b) = unpack(value);
                self.lcm[1][1] = a;
                self.lcm[1][2] = b;
            }
            19 => {
                let (a, b) = unpack(value);
                self.lcm[2][0] = a;
                self.lcm[2][1] = b;
            }
            20 => self.lcm[2][2] = value as u16 as i16,
            21 => self.fc[0] = value as i32,
            22 => self.fc[1] = value as i32,
            23 => self.fc[2] = value as i32,
            24 => self.ofx = value as i32,
            25 => self.ofy = value as i32,
            26 => self.h = value as u16,
            27 => self.dqa = value as u16 as i16,
            28 => self.dqb = value as i32,
            29 => self.zsf3 = value as u16 as i16,
            30 => self.zsf4 = value as u16 as i16,
            31 => {
                self.flag = value & 0x7FFF_F000;
                self.update_flag();
            }
            _ => unreachable!(),
        }
    }

    // ════════════════════════════════════════════════════════════════════
    //  FLAG / saturation helpers
    // ════════════════════════════════════════════════════════════════════

    /// Sets FLAG bit `bit`.
    #[inline]
    fn flag_set(&mut self, bit: u32) {
        self.flag |= 1 << bit;
    }

    /// Recomputes the FLAG error-summary bit (bit 31).
    #[inline]
    fn update_flag(&mut self) {
        if self.flag & FLAG_ERROR_MASK != 0 {
            self.flag |= 0x8000_0000;
        } else {
            self.flag &= !0x8000_0000;
        }
    }

    /// Overflow-checks a 44-bit MAC accumulator step for `MAC1..3`
    /// (`n ∈ 1..=3`), setting the positive/negative overflow flag, and returns
    /// the value truncated (sign-extended) to 44 bits.
    #[inline]
    fn mac_ext(&mut self, n: usize, value: i64) -> i64 {
        const MAX: i64 = (1 << 43) - 1;
        const MIN: i64 = -(1 << 43);
        if value > MAX {
            self.flag_set(match n {
                1 => 30,
                2 => 29,
                _ => 28,
            });
        } else if value < MIN {
            self.flag_set(match n {
                1 => 27,
                2 => 26,
                _ => 25,
            });
        }
        // Sign-extend from bit 43 (44-bit two's-complement wraparound).
        (value << 20) >> 20
    }

    /// Overflow-checks, truncates, and stores `MAC1..3` (`n ∈ 1..=3`) as
    /// `value` arithmetic-shifted right by `shift`.
    #[inline]
    fn set_mac(&mut self, n: usize, value: i64, shift: u32) {
        let truncated = self.mac_ext(n, value);
        self.mac[n] = (truncated >> shift) as i32;
    }

    /// Overflow-checks (32-bit) and stores `MAC0`.
    #[inline]
    fn set_mac0(&mut self, value: i64) {
        if value > i64::from(i32::MAX) {
            self.flag_set(16);
        } else if value < i64::from(i32::MIN) {
            self.flag_set(15);
        }
        self.mac[0] = value as i32;
    }

    /// Saturates `value` to the `IRn` range (`n ∈ 1..=3`) and stores it,
    /// flagging on clamp. `lm` selects the lower bound (`0` vs `-0x8000`).
    #[inline]
    fn set_ir(&mut self, n: usize, value: i32, lm: bool) {
        let min = if lm { 0 } else { -0x8000 };
        let flag = match n {
            1 => 24,
            2 => 23,
            _ => 22,
        };
        if value < min {
            self.flag_set(flag);
            self.ir[n] = min as i16;
        } else if value > 0x7FFF {
            self.flag_set(flag);
            self.ir[n] = 0x7FFF;
        } else {
            self.ir[n] = value as i16;
        }
    }

    /// Saturates `value` to the `IR0` range `[0, 0x1000]`, flagging on clamp.
    #[inline]
    fn set_ir0(&mut self, value: i32) {
        if value < 0 {
            self.flag_set(12);
            self.ir[0] = 0;
        } else if value > 0x1000 {
            self.flag_set(12);
            self.ir[0] = 0x1000;
        } else {
            self.ir[0] = value as i16;
        }
    }

    /// Saturates a color channel to `[0, 0xFF]`, flagging on clamp.
    #[inline]
    fn color_component(&mut self, value: i32, flag: u32) -> u32 {
        if value < 0 {
            self.flag_set(flag);
            0
        } else if value > 0xFF {
            self.flag_set(flag);
            0xFF
        } else {
            value as u32
        }
    }

    /// Saturates `value` to the SZ/OTZ range `[0, 0xFFFF]`, flagging on clamp.
    #[inline]
    fn saturate_sz(&mut self, value: i32) -> u16 {
        if value < 0 {
            self.flag_set(18);
            0
        } else if value > 0xFFFF {
            self.flag_set(18);
            0xFFFF
        } else {
            value as u16
        }
    }

    /// Pushes a value onto the screen-Z FIFO (saturated to `[0, 0xFFFF]`).
    fn push_sz(&mut self, value: i32) {
        self.sz[0] = self.sz[1];
        self.sz[1] = self.sz[2];
        self.sz[2] = self.sz[3];
        self.sz[3] = self.saturate_sz(value);
    }

    /// Pushes `(x, y)` onto the screen-XY FIFO, each saturated to
    /// `[-0x400, 0x3FF]` (flags 14/13).
    fn push_sxy(&mut self, x: i32, y: i32) {
        let sx = self.saturate_sxy(x, 14);
        let sy = self.saturate_sxy(y, 13);
        self.sxy[0] = self.sxy[1];
        self.sxy[1] = self.sxy[2];
        self.sxy[2] = (sx, sy);
    }

    #[inline]
    fn saturate_sxy(&mut self, value: i32, flag: u32) -> i16 {
        if value < -0x400 {
            self.flag_set(flag);
            -0x400
        } else if value > 0x3FF {
            self.flag_set(flag);
            0x3FF
        } else {
            value as i16
        }
    }

    /// Pushes the current `MAC1/2/3` (each `>> 4`, color-saturated) onto the
    /// color FIFO, preserving the CODE byte from RGBC.
    fn push_rgb(&mut self) {
        let r = self.color_component(self.mac[1] >> 4, 21);
        let g = self.color_component(self.mac[2] >> 4, 20);
        let b = self.color_component(self.mac[3] >> 4, 19);
        let code = (self.rgbc >> 24) & 0xFF;
        let v = r | (g << 8) | (b << 16) | (code << 24);
        self.rgb[0] = self.rgb[1];
        self.rgb[1] = self.rgb[2];
        self.rgb[2] = v;
    }

    // ════════════════════════════════════════════════════════════════════
    //  Shared datapath primitives
    // ════════════════════════════════════════════════════════════════════

    /// Returns input vector `k` (`0..=2`) as `[x, y, z]`.
    #[inline]
    fn vector(&self, k: usize) -> [i32; 3] {
        [
            i32::from(self.v[k][0]),
            i32::from(self.v[k][1]),
            i32::from(self.v[k][2]),
        ]
    }

    /// Returns the RGBC color bytes as `(r, g, b)` (each `0..=255`).
    #[inline]
    fn rgb_bytes(&self) -> (i64, i64, i64) {
        (
            i64::from(self.rgbc & 0xFF),
            i64::from((self.rgbc >> 8) & 0xFF),
            i64::from((self.rgbc >> 16) & 0xFF),
        )
    }

    /// Computes `[MAC1,MAC2,MAC3] = (Tr*0x1000 + Mat * Vec) >> (sf*12)` and the
    /// matching saturated `IR1/2/3`. `tr` elements are pre-widened to `i64`.
    fn mat_vec(&mut self, mat: [[i16; 3]; 3], vec: [i32; 3], tr: [i64; 3], sf: u32, lm: bool) {
        let shift = sf * 12;
        for i in 0..3 {
            let mut acc = tr[i] << 12;
            acc = self.mac_ext(i + 1, acc + i64::from(mat[i][0]) * i64::from(vec[0]));
            acc = self.mac_ext(i + 1, acc + i64::from(mat[i][1]) * i64::from(vec[1]));
            acc = self.mac_ext(i + 1, acc + i64::from(mat[i][2]) * i64::from(vec[2]));
            self.mac[i + 1] = (acc >> shift) as i32;
            self.set_ir(i + 1, self.mac[i + 1], lm);
        }
    }

    /// The shared far-color interpolation used by the depth-cue ops. Given a
    /// base color `[in1, in2, in3]` (44-bit, un-shifted), computes
    /// `MAC = (FC*0x1000 - in) >> shift` → `IR` (lm=false) →
    /// `MAC = (IR*IR0 + in) >> shift` → `IR` (lm).
    fn interpolate(&mut self, in1: i64, in2: i64, in3: i64, sf: u32, lm: bool) {
        let shift = sf * 12;
        self.set_mac(1, (i64::from(self.fc[0]) << 12) - in1, shift);
        self.set_mac(2, (i64::from(self.fc[1]) << 12) - in2, shift);
        self.set_mac(3, (i64::from(self.fc[2]) << 12) - in3, shift);
        self.set_ir(1, self.mac[1], false);
        self.set_ir(2, self.mac[2], false);
        self.set_ir(3, self.mac[3], false);
        let ir0 = i64::from(self.ir[0]);
        self.set_mac(1, i64::from(self.ir[1]) * ir0 + in1, shift);
        self.set_mac(2, i64::from(self.ir[2]) * ir0 + in2, shift);
        self.set_mac(3, i64::from(self.ir[3]) * ir0 + in3, shift);
        self.set_ir(1, self.mac[1], lm);
        self.set_ir(2, self.mac[2], lm);
        self.set_ir(3, self.mac[3], lm);
    }

    /// The Unsigned-Newton-Raphson reciprocal `(H * 0x20000 + SZ/2) / SZ`,
    /// clamped to `0x1FFFF`. Sets the divide-overflow flag (bit 17) and returns
    /// `0x1FFFF` when `H >= SZ*2`.
    fn unr_divide(&mut self, h: u16, sz3: u16) -> u32 {
        if u32::from(h) < u32::from(sz3) * 2 {
            let z = sz3.leading_zeros();
            let n = u64::from(h) << z;
            let mut d = u64::from(sz3) << z;
            let u = u64::from(UNR_TABLE[((d - 0x7FC0) >> 7) as usize]) + 0x101;
            d = (0x0200_0080 - d * u) >> 8;
            d = (0x0000_0080 + d * u) >> 8;
            (((n * d + 0x8000) >> 16) as u32).min(0x1FFFF)
        } else {
            self.flag_set(17);
            0x1FFFF
        }
    }

    // ════════════════════════════════════════════════════════════════════
    //  Command dispatch
    // ════════════════════════════════════════════════════════════════════

    /// Executes a GTE command (the 25-bit `imm25` field of a `COP2` command
    /// word). FLAG is reset at entry and its summary bit recomputed at exit.
    pub fn execute(&mut self, cmd: u32) {
        self.flag = 0;
        let sf = (cmd >> 19) & 1;
        let lm = (cmd >> 10) & 1 != 0;
        match cmd & 0x3F {
            0x01 => self.rtps(sf, lm),
            0x06 => self.nclip(),
            0x0C => self.op(sf, lm),
            0x10 => self.dpcs(sf, lm),
            0x11 => self.intpl(sf, lm),
            0x12 => self.mvmva(cmd, sf, lm),
            0x13 => self.ncd(0, sf, lm),
            0x14 => self.cdp(sf, lm),
            0x16 => {
                self.ncd(0, sf, lm);
                self.ncd(1, sf, lm);
                self.ncd(2, sf, lm);
            }
            0x1B => self.ncc(0, sf, lm),
            0x1C => self.cc(sf, lm),
            0x1E => self.nc(0, sf, lm),
            0x20 => {
                self.nc(0, sf, lm);
                self.nc(1, sf, lm);
                self.nc(2, sf, lm);
            }
            0x28 => self.sqr(sf, lm),
            0x29 => self.dcpl(sf, lm),
            0x2A => self.dpct(sf, lm),
            0x2D => self.avsz3(),
            0x2E => self.avsz4(),
            0x30 => self.rtpt(sf, lm),
            0x3D => self.gpf(sf, lm),
            0x3E => self.gpl(sf, lm),
            0x3F => {
                self.ncc(0, sf, lm);
                self.ncc(1, sf, lm);
                self.ncc(2, sf, lm);
            }
            // Unassigned GTE opcodes are a no-op (usable, never a trap).
            _ => {}
        }
        self.update_flag();
    }

    // ── Perspective transform (RTPS / RTPT) ─────────────────────────────

    fn rtps(&mut self, sf: u32, lm: bool) {
        let v = self.vector(0);
        self.rtp(v, sf, lm, true);
    }

    fn rtpt(&mut self, sf: u32, lm: bool) {
        for k in 0..3 {
            let v = self.vector(k);
            self.rtp(v, sf, lm, k == 2);
        }
    }

    /// Transforms a single vertex: rotation + translation, perspective divide,
    /// screen-XY FIFO push, and (on `last`) the depth-cue into IR0/MAC0.
    fn rtp(&mut self, vec: [i32; 3], sf: u32, lm: bool, last: bool) {
        let shift = sf * 12;
        let rt = self.rt;
        let tr = self.tr;

        // Rows X and Y: standard multiply-accumulate + saturate.
        for i in 0..2 {
            let mut acc = i64::from(tr[i]) << 12;
            acc = self.mac_ext(i + 1, acc + i64::from(rt[i][0]) * i64::from(vec[0]));
            acc = self.mac_ext(i + 1, acc + i64::from(rt[i][1]) * i64::from(vec[1]));
            acc = self.mac_ext(i + 1, acc + i64::from(rt[i][2]) * i64::from(vec[2]));
            self.mac[i + 1] = (acc >> shift) as i32;
            self.set_ir(i + 1, self.mac[i + 1], lm);
        }

        // Row Z: keep the full 44-bit accumulator for the SZ computation.
        let mut accz = i64::from(tr[2]) << 12;
        accz = self.mac_ext(3, accz + i64::from(rt[2][0]) * i64::from(vec[0]));
        accz = self.mac_ext(3, accz + i64::from(rt[2][1]) * i64::from(vec[1]));
        accz = self.mac_ext(3, accz + i64::from(rt[2][2]) * i64::from(vec[2]));
        self.mac[3] = (accz >> shift) as i32;

        // IR3 quirk: the saturation FLAG (bit 22) is tested against the
        // un-`sf`-shifted value (MAC3 >> 12) as if lm=0, but IR3 itself stores
        // MAC3 clamped with the requested `lm`.
        let z = (accz >> 12) as i32;
        if !(-0x8000..=0x7FFF).contains(&z) {
            self.flag_set(22);
        }
        let lo = if lm { 0 } else { -0x8000 };
        self.ir[3] = self.mac[3].clamp(lo, 0x7FFF) as i16;
        self.push_sz(z);

        // Perspective divide and screen projection.
        let n = i64::from(self.unr_divide(self.h, self.sz[3]));
        let mac0x = i64::from(self.ofx) + n * i64::from(self.ir[1]);
        self.set_mac0(mac0x);
        let sx = (mac0x >> 16) as i32;
        let mac0y = i64::from(self.ofy) + n * i64::from(self.ir[2]);
        self.set_mac0(mac0y);
        let sy = (mac0y >> 16) as i32;
        self.push_sxy(sx, sy);

        if last {
            let mac0d = i64::from(self.dqb) + n * i64::from(self.dqa);
            self.set_mac0(mac0d);
            self.set_ir0(self.mac[0] >> 12);
        }
    }

    // ── NCLIP / OP / SQR ────────────────────────────────────────────────

    fn nclip(&mut self) {
        let (sx0, sy0) = self.sxy[0];
        let (sx1, sy1) = self.sxy[1];
        let (sx2, sy2) = self.sxy[2];
        let (sx0, sy0) = (i64::from(sx0), i64::from(sy0));
        let (sx1, sy1) = (i64::from(sx1), i64::from(sy1));
        let (sx2, sy2) = (i64::from(sx2), i64::from(sy2));
        let v = sx0 * sy1 + sx1 * sy2 + sx2 * sy0 - sx0 * sy2 - sx1 * sy0 - sx2 * sy1;
        self.set_mac0(v);
    }

    fn op(&mut self, sf: u32, lm: bool) {
        let shift = sf * 12;
        let d1 = i64::from(self.rt[0][0]);
        let d2 = i64::from(self.rt[1][1]);
        let d3 = i64::from(self.rt[2][2]);
        let ir1 = i64::from(self.ir[1]);
        let ir2 = i64::from(self.ir[2]);
        let ir3 = i64::from(self.ir[3]);
        self.set_mac(1, ir3 * d2 - ir2 * d3, shift);
        self.set_mac(2, ir1 * d3 - ir3 * d1, shift);
        self.set_mac(3, ir2 * d1 - ir1 * d2, shift);
        self.set_ir(1, self.mac[1], lm);
        self.set_ir(2, self.mac[2], lm);
        self.set_ir(3, self.mac[3], lm);
    }

    fn sqr(&mut self, sf: u32, lm: bool) {
        let shift = sf * 12;
        self.set_mac(1, i64::from(self.ir[1]) * i64::from(self.ir[1]), shift);
        self.set_mac(2, i64::from(self.ir[2]) * i64::from(self.ir[2]), shift);
        self.set_mac(3, i64::from(self.ir[3]) * i64::from(self.ir[3]), shift);
        self.set_ir(1, self.mac[1], lm);
        self.set_ir(2, self.mac[2], lm);
        self.set_ir(3, self.mac[3], lm);
    }

    // ── Average Z (AVSZ3 / AVSZ4) ───────────────────────────────────────

    fn avsz3(&mut self) {
        let sum = i64::from(self.sz[1]) + i64::from(self.sz[2]) + i64::from(self.sz[3]);
        self.set_mac0(i64::from(self.zsf3) * sum);
        let otz = self.mac[0] >> 12;
        self.otz = self.saturate_sz(otz);
    }

    fn avsz4(&mut self) {
        let sum = i64::from(self.sz[0])
            + i64::from(self.sz[1])
            + i64::from(self.sz[2])
            + i64::from(self.sz[3]);
        self.set_mac0(i64::from(self.zsf4) * sum);
        let otz = self.mac[0] >> 12;
        self.otz = self.saturate_sz(otz);
    }

    // ── MVMVA ───────────────────────────────────────────────────────────

    fn mvmva(&mut self, cmd: u32, sf: u32, lm: bool) {
        let mx = (cmd >> 17) & 3;
        let vsel = (cmd >> 15) & 3;
        let cv = (cmd >> 13) & 3;

        let mat = match mx {
            0 => self.rt,
            1 => self.llm,
            2 => self.lcm,
            _ => self.garbage_matrix(),
        };
        let vec = match vsel {
            0 => self.vector(0),
            1 => self.vector(1),
            2 => self.vector(2),
            _ => [
                i32::from(self.ir[1]),
                i32::from(self.ir[2]),
                i32::from(self.ir[3]),
            ],
        };
        let tr = match cv {
            0 => [
                i64::from(self.tr[0]),
                i64::from(self.tr[1]),
                i64::from(self.tr[2]),
            ],
            1 => [
                i64::from(self.bk[0]),
                i64::from(self.bk[1]),
                i64::from(self.bk[2]),
            ],
            2 => [
                i64::from(self.fc[0]),
                i64::from(self.fc[1]),
                i64::from(self.fc[2]),
            ],
            _ => [0, 0, 0],
        };

        if cv == 2 {
            // Far-color translation triggers the well-known hardware bug: the
            // FC term and the first matrix column contribute only to a
            // prematurely-saturated IR (setting flags), while MAC is computed
            // from columns 2 and 3 alone (FC effectively dropped).
            let shift = sf * 12;
            for i in 0..3 {
                let tmp = self.mac_ext(
                    i + 1,
                    (tr[i] << 12) + i64::from(mat[i][0]) * i64::from(vec[0]),
                );
                self.set_ir(i + 1, (tmp >> shift) as i32, false);
                let mut acc = self.mac_ext(i + 1, i64::from(mat[i][1]) * i64::from(vec[1]));
                acc = self.mac_ext(i + 1, acc + i64::from(mat[i][2]) * i64::from(vec[2]));
                self.mac[i + 1] = (acc >> shift) as i32;
            }
        } else {
            self.mat_vec(mat, vec, tr, sf, lm);
        }
    }

    /// The bugged "garbage" matrix selected by `mx=3`
    /// (`[-R<<4, R<<4, IR0]`, `[RT13; 3]`, `[RT22; 3]`).
    fn garbage_matrix(&self) -> [[i16; 3]; 3] {
        let r4 = ((self.rgbc & 0xFF) as i16) << 4;
        [
            [-r4, r4, self.ir[0]],
            [self.rt[0][2]; 3],
            [self.rt[1][1]; 3],
        ]
    }

    // ── Color / lighting ops ────────────────────────────────────────────

    /// NCS/NCT body: normal → light color, output the light color directly.
    fn nc(&mut self, k: usize, sf: u32, lm: bool) {
        let v = self.vector(k);
        self.mat_vec(self.llm, v, [0, 0, 0], sf, lm);
        let ir = self.ir_vector();
        let bk = self.bk_i64();
        self.mat_vec(self.lcm, ir, bk, sf, lm);
        self.push_rgb();
    }

    /// NCCS/NCCT body: normal → light color, then RGB color multiply.
    fn ncc(&mut self, k: usize, sf: u32, lm: bool) {
        let v = self.vector(k);
        self.mat_vec(self.llm, v, [0, 0, 0], sf, lm);
        let ir = self.ir_vector();
        let bk = self.bk_i64();
        self.mat_vec(self.lcm, ir, bk, sf, lm);
        self.color_multiply(sf, lm);
        self.push_rgb();
    }

    /// NCDS/NCDT body: normal → light color, RGB color multiply, depth cue.
    fn ncd(&mut self, k: usize, sf: u32, lm: bool) {
        let v = self.vector(k);
        self.mat_vec(self.llm, v, [0, 0, 0], sf, lm);
        let ir = self.ir_vector();
        let bk = self.bk_i64();
        self.mat_vec(self.lcm, ir, bk, sf, lm);
        let (r, g, b) = self.rgb_bytes();
        let in1 = (r * i64::from(self.ir[1])) << 4;
        let in2 = (g * i64::from(self.ir[2])) << 4;
        let in3 = (b * i64::from(self.ir[3])) << 4;
        self.interpolate(in1, in2, in3, sf, lm);
        self.push_rgb();
    }

    /// CDP: light color from the current IR, then RGB color multiply + depth cue.
    fn cdp(&mut self, sf: u32, lm: bool) {
        let ir = self.ir_vector();
        let bk = self.bk_i64();
        self.mat_vec(self.lcm, ir, bk, sf, lm);
        let (r, g, b) = self.rgb_bytes();
        let in1 = (r * i64::from(self.ir[1])) << 4;
        let in2 = (g * i64::from(self.ir[2])) << 4;
        let in3 = (b * i64::from(self.ir[3])) << 4;
        self.interpolate(in1, in2, in3, sf, lm);
        self.push_rgb();
    }

    /// CC: light color from the current IR, then RGB color multiply (no cue).
    fn cc(&mut self, sf: u32, lm: bool) {
        let ir = self.ir_vector();
        let bk = self.bk_i64();
        self.mat_vec(self.lcm, ir, bk, sf, lm);
        self.color_multiply(sf, lm);
        self.push_rgb();
    }

    /// DCPL: RGB color multiply of the current IR, then depth cue.
    fn dcpl(&mut self, sf: u32, lm: bool) {
        let (r, g, b) = self.rgb_bytes();
        let in1 = (r * i64::from(self.ir[1])) << 4;
        let in2 = (g * i64::from(self.ir[2])) << 4;
        let in3 = (b * i64::from(self.ir[3])) << 4;
        self.interpolate(in1, in2, in3, sf, lm);
        self.push_rgb();
    }

    /// DPCS: depth cue of the RGBC color (`[R,G,B] << 16`).
    fn dpcs(&mut self, sf: u32, lm: bool) {
        let (r, g, b) = self.rgb_bytes();
        self.interpolate(r << 16, g << 16, b << 16, sf, lm);
        self.push_rgb();
    }

    /// DPCT: depth cue of the color FIFO front (RGB0), performed three times.
    fn dpct(&mut self, sf: u32, lm: bool) {
        for _ in 0..3 {
            let r = i64::from(self.rgb[0] & 0xFF);
            let g = i64::from((self.rgb[0] >> 8) & 0xFF);
            let b = i64::from((self.rgb[0] >> 16) & 0xFF);
            self.interpolate(r << 16, g << 16, b << 16, sf, lm);
            self.push_rgb();
        }
    }

    /// INTPL: interpolate the IR vector (`IR << 12`) towards the far color.
    fn intpl(&mut self, sf: u32, lm: bool) {
        let in1 = i64::from(self.ir[1]) << 12;
        let in2 = i64::from(self.ir[2]) << 12;
        let in3 = i64::from(self.ir[3]) << 12;
        self.interpolate(in1, in2, in3, sf, lm);
        self.push_rgb();
    }

    /// GPF: general-purpose interpolation `MAC = (IR0 * IR) >> shift`.
    fn gpf(&mut self, sf: u32, lm: bool) {
        let shift = sf * 12;
        let ir0 = i64::from(self.ir[0]);
        self.set_mac(1, ir0 * i64::from(self.ir[1]), shift);
        self.set_mac(2, ir0 * i64::from(self.ir[2]), shift);
        self.set_mac(3, ir0 * i64::from(self.ir[3]), shift);
        self.set_ir(1, self.mac[1], lm);
        self.set_ir(2, self.mac[2], lm);
        self.set_ir(3, self.mac[3], lm);
        self.push_rgb();
    }

    /// GPL: general-purpose interpolation with base
    /// `MAC = (IR0 * IR + MAC << shift) >> shift`.
    fn gpl(&mut self, sf: u32, lm: bool) {
        let shift = sf * 12;
        let ir0 = i64::from(self.ir[0]);
        let in1 = i64::from(self.mac[1]) << shift;
        let in2 = i64::from(self.mac[2]) << shift;
        let in3 = i64::from(self.mac[3]) << shift;
        self.set_mac(1, ir0 * i64::from(self.ir[1]) + in1, shift);
        self.set_mac(2, ir0 * i64::from(self.ir[2]) + in2, shift);
        self.set_mac(3, ir0 * i64::from(self.ir[3]) + in3, shift);
        self.set_ir(1, self.mac[1], lm);
        self.set_ir(2, self.mac[2], lm);
        self.set_ir(3, self.mac[3], lm);
        self.push_rgb();
    }

    /// `[MAC1,MAC2,MAC3] = ([R,G,B] * IR << 4) >> shift`, then saturate IR.
    fn color_multiply(&mut self, sf: u32, lm: bool) {
        let shift = sf * 12;
        let (r, g, b) = self.rgb_bytes();
        self.set_mac(1, (r * i64::from(self.ir[1])) << 4, shift);
        self.set_mac(2, (g * i64::from(self.ir[2])) << 4, shift);
        self.set_mac(3, (b * i64::from(self.ir[3])) << 4, shift);
        self.set_ir(1, self.mac[1], lm);
        self.set_ir(2, self.mac[2], lm);
        self.set_ir(3, self.mac[3], lm);
    }

    /// The current `[IR1, IR2, IR3]` widened to an `i32` vector.
    #[inline]
    fn ir_vector(&self) -> [i32; 3] {
        [
            i32::from(self.ir[1]),
            i32::from(self.ir[2]),
            i32::from(self.ir[3]),
        ]
    }

    /// The background color `[RBK, GBK, BBK]` widened to `i64`.
    #[inline]
    fn bk_i64(&self) -> [i64; 3] {
        [
            i64::from(self.bk[0]),
            i64::from(self.bk[1]),
            i64::from(self.bk[2]),
        ]
    }
}

/// Writes a packed 16:16 word into row `row` columns 0,1 of a 3x3 `i16` matrix.
#[inline]
fn set_pair(mat: &mut [[i16; 3]; 3], row: usize, value: u32) {
    let (a, b) = unpack(value);
    mat[row][0] = a;
    mat[row][1] = b;
}

/// Counts the leading bits equal to the sign bit of `value` (the LZCS→LZCR
/// operation): leading zeros for non-negative inputs, leading ones otherwise.
/// The result is always in `1..=32`.
#[inline]
fn leading_bit_count(value: u32) -> u32 {
    if value & 0x8000_0000 == 0 {
        value.leading_zeros()
    } else {
        (!value).leading_zeros()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Register round-trips ────────────────────────────────────────────

    #[test]
    fn data_vector_pack_roundtrip() {
        let mut gte = Gte::new();
        gte.write_data(0, 0x1234_ABCD); // VXY0: x=0xABCD, y=0x1234
        assert_eq!(gte.read_data(0), 0x1234_ABCD);
        gte.write_data(1, 0x0000_8001); // VZ0 = 0x8001 (sign-extended on read)
        assert_eq!(gte.read_data(1), 0xFFFF_8001);
    }

    #[test]
    fn otz_and_sz_zero_extend() {
        let mut gte = Gte::new();
        gte.write_data(7, 0x1234_8000);
        assert_eq!(gte.read_data(7), 0x0000_8000);
        gte.write_data(16, 0xFFFF_ABCD);
        assert_eq!(gte.read_data(16), 0x0000_ABCD);
    }

    #[test]
    fn ir_sign_extend() {
        let mut gte = Gte::new();
        gte.write_data(9, 0x0000_8000);
        assert_eq!(gte.read_data(9), 0xFFFF_8000);
        gte.write_data(8, 0x0000_7FFF);
        assert_eq!(gte.read_data(8), 0x0000_7FFF);
    }

    #[test]
    fn control_sign_extension_quirks() {
        let mut gte = Gte::new();
        // H is written unsigned but read back sign-extended.
        gte.write_control(26, 0x0000_8000);
        assert_eq!(gte.read_control(26), 0xFFFF_8000);
        // DQA / ZSF3 / ZSF4 are signed and read sign-extended.
        gte.write_control(27, 0x0000_9000);
        assert_eq!(gte.read_control(27), 0xFFFF_9000);
        gte.write_control(29, 0x0000_8123);
        assert_eq!(gte.read_control(29), 0xFFFF_8123);
        gte.write_control(30, 0x0000_8456);
        assert_eq!(gte.read_control(30), 0xFFFF_8456);
    }

    #[test]
    fn control_matrix_pack_roundtrip() {
        let mut gte = Gte::new();
        gte.write_control(0, 0x0002_0001); // RT11=1, RT12=2
        gte.write_control(1, 0x0004_0003); // RT13=3, RT21=4
        assert_eq!(gte.read_control(0), 0x0002_0001);
        assert_eq!(gte.read_control(1), 0x0004_0003);
        // RT33 sits in the low half, read sign-extended.
        gte.write_control(4, 0x0000_8000);
        assert_eq!(gte.read_control(4), 0xFFFF_8000);
    }

    #[test]
    fn control_32bit_roundtrip() {
        let mut gte = Gte::new();
        gte.write_control(5, 0xDEAD_BEEF); // TRX
        assert_eq!(gte.read_control(5), 0xDEAD_BEEF);
        gte.write_control(24, 0x1234_5678); // OFX
        assert_eq!(gte.read_control(24), 0x1234_5678);
        gte.write_control(28, 0xCAFE_F00D); // DQB
        assert_eq!(gte.read_control(28), 0xCAFE_F00D);
    }

    #[test]
    fn irgb_orgb_pack_unpack() {
        let mut gte = Gte::new();
        // Write IRGB expands into IR1/2/3 = channel * 0x80.
        gte.write_data(28, (1) | (2 << 5) | (3 << 10));
        assert_eq!(gte.read_data(9), 0x80); // IR1 = 1 * 0x80
        assert_eq!(gte.read_data(10), 0x100); // IR2 = 2 * 0x80
        assert_eq!(gte.read_data(11), 0x180); // IR3 = 3 * 0x80
        // Reading IRGB / ORGB repacks IR/0x80 clamped.
        assert_eq!(gte.read_data(28), (1) | (2 << 5) | (3 << 10));
        assert_eq!(gte.read_data(29), (1) | (2 << 5) | (3 << 10));
        // Out-of-range IR saturates on read.
        gte.write_data(9, 0xFFFF_FFFF); // IR1 = -1 -> clamps to 0
        gte.write_data(10, 0x0000_7FFF); // IR2 big -> clamps to 0x1F
        assert_eq!(gte.read_data(28) & 0x1F, 0);
        assert_eq!((gte.read_data(28) >> 5) & 0x1F, 0x1F);
    }

    #[test]
    fn lzcs_lzcr_counts() {
        let mut gte = Gte::new();
        let cases = [
            (0x0000_0000u32, 32u32),
            (0xFFFF_FFFF, 32),
            (0x7FFF_FFFF, 1),
            (0x8000_0000, 1),
            (0x0FFF_FFFF, 4),
            (0xF000_0000, 4),
            (0x0000_0001, 31),
        ];
        for (input, expect) in cases {
            gte.write_data(30, input);
            assert_eq!(gte.read_data(31), expect, "lzc({input:#010x})");
            assert_eq!(gte.read_data(30), input);
        }
    }

    #[test]
    fn sxyp_write_pushes_fifo() {
        let mut gte = Gte::new();
        gte.write_data(12, 0x0001_0001); // SXY0
        gte.write_data(13, 0x0002_0002); // SXY1
        gte.write_data(14, 0x0003_0003); // SXY2
        gte.write_data(15, 0x0004_0004); // SXYP push
        assert_eq!(gte.read_data(12), 0x0002_0002); // old SXY1
        assert_eq!(gte.read_data(13), 0x0003_0003); // old SXY2
        assert_eq!(gte.read_data(14), 0x0004_0004); // pushed value
        assert_eq!(gte.read_data(15), 0x0004_0004); // SXYP mirrors SXY2
    }

    #[test]
    fn res1_stores_raw() {
        let mut gte = Gte::new();
        gte.write_data(23, 0xABCD_1234);
        assert_eq!(gte.read_data(23), 0xABCD_1234);
    }

    // ── Operation results ───────────────────────────────────────────────

    /// Loads the identity rotation matrix (RT = I) with zero translation.
    fn identity_rt(gte: &mut Gte) {
        gte.write_control(0, pack(0x1000, 0)); // RT11=0x1000, RT12=0
        gte.write_control(1, pack(0, 0)); // RT13=0, RT21=0
        gte.write_control(2, pack(0x1000, 0)); // RT22=0x1000, RT23=0
        gte.write_control(3, pack(0, 0)); // RT31=0, RT32=0
        gte.write_control(4, 0x1000); // RT33=0x1000
        gte.write_control(5, 0); // TRX
        gte.write_control(6, 0); // TRY
        gte.write_control(7, 0); // TRZ
    }

    #[test]
    fn rtps_identity_projects() {
        let mut gte = Gte::new();
        identity_rt(&mut gte);
        gte.write_control(26, 0x0200); // H = 512
        gte.write_control(24, 0); // OFX
        gte.write_control(25, 0); // OFY
        gte.write_control(27, 0); // DQA
        gte.write_control(28, 0); // DQB
        // V0 = (64, 32, 256), sf=1 (>>12).
        gte.write_data(0, pack(64, 32));
        gte.write_data(1, 256);
        gte.execute(0x0008_0001); // RTPS, sf=1
        // With RT=I*0x1000 and sf shift 12, MAC1..3 == V0.
        assert_eq!(gte.mac[1], 64);
        assert_eq!(gte.mac[2], 32);
        assert_eq!(gte.mac[3], 256);
        assert_eq!(gte.ir[1], 64);
        assert_eq!(gte.sz[3], 256); // SZ3 = z
        // n = H*0x20000/SZ rounded = 512*0x20000/256 = 0x40000 -> clamp 0x1FFFF.
        // SX = (OFX + n*IR1) >> 16.
        let n = 0x1FFFF_i64;
        assert_eq!(gte.sxy[2].0 as i64, (n * 64) >> 16);
        assert_eq!(gte.sxy[2].1 as i64, (n * 32) >> 16);
    }

    #[test]
    fn rtps_divide_overflow_flag() {
        let mut gte = Gte::new();
        identity_rt(&mut gte);
        gte.write_control(26, 0xFFFF); // huge H
        gte.write_data(0, pack(0, 0));
        gte.write_data(1, 1); // tiny Z
        gte.execute(0x0008_0001);
        // H >= SZ*2 forces divide overflow (bit 17) and result 0x1FFFF.
        assert_ne!(gte.flag & (1 << 17), 0);
        assert_ne!(gte.flag & 0x8000_0000, 0); // summary bit set
    }

    #[test]
    fn nclip_cross_product() {
        let mut gte = Gte::new();
        gte.write_data(12, pack(0, 0)); // SXY0 = (0,0)
        gte.write_data(13, pack(10, 0)); // SXY1 = (10,0)
        gte.write_data(14, pack(0, 10)); // SXY2 = (0,10)
        gte.execute(0x06);
        // MAC0 = 0*0 + 10*10 + 0*0 - 0*10 - 10*0 - 0*0 = 100.
        assert_eq!(gte.mac[0], 100);
    }

    #[test]
    fn op_cross_product() {
        let mut gte = Gte::new();
        // D = diag(0x1000). IR = (1,2,3), sf=1.
        gte.write_control(0, pack(0x1000, 0));
        gte.write_control(2, pack(0x1000, 0));
        gte.write_control(4, 0x1000);
        gte.write_data(9, 1);
        gte.write_data(10, 2);
        gte.write_data(11, 3);
        gte.execute(0x0008_000C); // OP, sf=1
        // MAC1 = (IR3*D2 - IR2*D3)>>12 = (3*0x1000 - 2*0x1000)>>12 = 1.
        assert_eq!(gte.mac[1], 1);
        assert_eq!(gte.mac[2], -2); // (IR1*D3 - IR3*D1)>>12 = (1-3)
        assert_eq!(gte.mac[3], 1); // (IR2*D1 - IR1*D2)>>12 = (2-1)
    }

    #[test]
    fn sqr_squares_ir() {
        let mut gte = Gte::new();
        gte.write_data(9, 4);
        gte.write_data(10, 5);
        gte.write_data(11, 6);
        gte.execute(0x28); // SQR, sf=0
        assert_eq!(gte.mac[1], 16);
        assert_eq!(gte.mac[2], 25);
        assert_eq!(gte.mac[3], 36);
        assert_eq!(gte.ir[1], 16);
    }

    #[test]
    fn avsz3_averages() {
        let mut gte = Gte::new();
        gte.write_control(29, 0x0155); // ZSF3 ~ 1/3 * 0x1000
        gte.write_data(17, 300); // SZ1
        gte.write_data(18, 300); // SZ2
        gte.write_data(19, 300); // SZ3
        gte.execute(0x2D);
        // MAC0 = ZSF3 * 900 = 0x155 * 900 = 306900; OTZ = >>12 = 74.
        assert_eq!(gte.mac[0], 0x155 * 900);
        assert_eq!(gte.otz, ((0x155 * 900) >> 12) as u16);
    }

    #[test]
    fn mvmva_identity_matrix() {
        let mut gte = Gte::new();
        // RT = diag(0x1000), no translation. Vector = V0.
        gte.write_control(0, pack(0x1000, 0));
        gte.write_control(2, pack(0x1000, 0));
        gte.write_control(4, 0x1000);
        gte.write_data(0, pack(7, 9));
        gte.write_data(1, 11);
        // MVMVA sf=1, mx=0(RT), v=0(V0), cv=3(none). The zero mx/v fields are
        // written explicitly for documentation.
        #[allow(clippy::identity_op)]
        let cmd = 0x12 | (1 << 19) | (0 << 17) | (0 << 15) | (3 << 13);
        gte.execute(cmd);
        assert_eq!(gte.mac[1], 7);
        assert_eq!(gte.mac[2], 9);
        assert_eq!(gte.mac[3], 11);
    }

    #[test]
    fn flag_summary_bit() {
        let mut gte = Gte::new();
        // Force an IR1 saturation by a large SQR result with lm.
        gte.write_data(9, 0x7FFF);
        gte.execute(0x28); // SQR sf=0: MAC1 = 0x7FFF^2 huge -> IR1 saturates.
        assert_ne!(gte.flag & (1 << 24), 0); // IR1 saturated
        assert_ne!(gte.flag & 0x8000_0000, 0); // summary bit set
    }

    #[test]
    fn ncds_runs_and_pushes_color() {
        let mut gte = Gte::new();
        // Minimal setup: identity light matrices, mid-gray RGBC.
        gte.write_control(8, pack(0x1000, 0)); // LLM row0
        gte.write_control(10, pack(0x1000, 0));
        gte.write_control(12, 0x1000);
        gte.write_control(16, pack(0x1000, 0)); // LCM row0
        gte.write_control(18, pack(0x1000, 0));
        gte.write_control(20, 0x1000);
        gte.write_data(6, 0x0080_8080); // RGBC
        gte.write_data(0, pack(0x100, 0));
        gte.write_data(1, 0);
        // Should not panic and should push a color into the FIFO.
        gte.execute(0x0000_0013); // NCDS
        assert_ne!(gte.read_data(22), 0); // RGB2 populated
    }

    // ── UNR division ────────────────────────────────────────────────────

    #[test]
    fn unr_divide_basic() {
        let mut gte = Gte::new();
        // The reciprocal returns ~(H << 16) / SZ. H/SZ = 0.5 → ~0.5 * 0x10000.
        let r = gte.unr_divide(0x100, 0x200);
        assert!((0x7F00..=0x8100).contains(&r), "got {r:#x}");
        // A 1:1 ratio yields ~0x10000.
        assert!(
            (0xFF00..=0x1_0100).contains(&gte.unr_divide(0x100, 0x100)),
            "1:1 ratio"
        );
    }

    #[test]
    fn unr_divide_overflow() {
        let mut gte = Gte::new();
        let r = gte.unr_divide(0x400, 0x100); // H >= SZ*2
        assert_eq!(r, 0x1FFFF);
        assert_ne!(gte.flag & (1 << 17), 0);
    }

    // ── Serde ───────────────────────────────────────────────────────────

    #[test]
    fn serde_round_trip() {
        let mut gte = Gte::new();
        gte.write_data(0, 0x1234_5678);
        gte.write_control(5, 0xDEAD_BEEF);
        gte.execute(0x06);
        let json = serde_json::to_string(&gte).unwrap();
        let back: Gte = serde_json::from_str(&json).unwrap();
        assert_eq!(gte, back);
    }
}
