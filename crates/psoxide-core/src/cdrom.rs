//! PlayStation CD-ROM controller.
//!
//! This module implements the CD-ROM sub-controller mapped at
//! `0x1F80_1800..=0x1F80_1803`. Unlike the old read-back stub in
//! [`crate::iostubs`], this is a real state machine: it decodes the
//! index-banked register file, buffers the parameter/response/data FIFOs,
//! executes the command set a BIOS and game runtime issue, and delivers the
//! queued `INT1`/`INT2`/`INT3`/`INT5` responses (raising [`IrqLine::CdRom`])
//! a realistic number of cycles later from [`Cdrom::tick`].
//!
//! The controller is pure: it performs no host I/O and never panics on a guest
//! access. A disc is a raw 2352-byte-per-sector image handed in through
//! [`Cdrom::insert_disc`]; reads out of range deliver zeros rather than
//! trapping.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use crate::irq::{Irq, IrqLine};

/// Physical base of the CD-ROM controller register window.
pub const CDROM_BASE: u32 = 0x1F80_1800;
/// Physical end (inclusive) of the CD-ROM controller register window.
pub const CDROM_END: u32 = 0x1F80_1803;

/// Latency (CPU cycles) before the first (acknowledge) response of a command is
/// delivered — the `INT3`/`INT5` ack.
pub const FIRST_RESP_DELAY: i64 = 50_000;
/// Additional latency (CPU cycles) before the second response (`INT2`) of a
/// two-phase command is delivered.
pub const SECOND_RESP_DELAY: i64 = 50_000;

/// Single-speed (1x) sector read period in CPU cycles (33_868_800 / 75).
pub const READ_PERIOD_SINGLE: i64 = 451_584;
/// Double-speed (2x) sector read period in CPU cycles (33_868_800 / 150).
pub const READ_PERIOD_DOUBLE: i64 = 225_792;

/// Raw bytes per CD sector (2352 = full Mode-2 raw frame).
pub const SECTOR_RAW: usize = 2352;

/// Maximum entries the parameter and response FIFOs hold.
const FIFO_CAP: usize = 16;

/// Maximum decoded CD-audio frames buffered before the oldest is dropped. This
/// bounds the queue if audio decode outpaces the [`Cdrom::take_cd_audio`] drain
/// (e.g. back-to-back synthetic XA sectors read at 2x with no interleave gap).
const CD_AUDIO_CAP: usize = 16_384;

/// XA-ADPCM positive filter coefficients (index = filter 0..=4). Source: Nocash
/// PSX-SPX "CDROM XA Audio ADPCM Compression" (`pos_xa_adpcm_table`).
const XA_POS: [i32; 5] = [0, 60, 115, 98, 122];
/// XA-ADPCM negative filter coefficients (index = filter 0..=4). Source: Nocash
/// PSX-SPX "CDROM XA Audio ADPCM Compression" (`neg_xa_adpcm_table`).
const XA_NEG: [i32; 5] = [0, 0, -52, -55, -60];

/// The 150-sector (2-second) lead-in pregap between an absolute MSF address and
/// logical block 0.
const PREGAP: u32 = 150;

/// A single track descriptor in a disc's table of contents.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct DiscTrack {
    /// 1-based track number.
    pub number: u8,
    /// Logical block address where the track starts.
    pub start_lba: u32,
    /// `true` for a CD-DA (audio) track, `false` for a data track.
    pub audio: bool,
}

/// A mounted disc: a raw sector image plus its table of contents.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct Disc {
    /// Raw disc image, [`SECTOR_RAW`] bytes per sector.
    pub data: Vec<u8>,
    /// Track table. If empty, a default single data track is synthesized on
    /// insertion.
    pub tracks: Vec<DiscTrack>,
    /// Logical block address of the lead-out (end of the last track).
    pub lead_out_lba: u32,
}

impl Disc {
    /// Builds a disc from a raw image with a synthesized single-track TOC
    /// (track 1 data at LBA 0).
    #[must_use]
    pub fn from_bytes(data: Vec<u8>) -> Self {
        let lead_out_lba = (data.len() / SECTOR_RAW) as u32;
        Self {
            data,
            tracks: Vec::new(),
            lead_out_lba,
        }
    }

    /// Number of sectors in the image.
    #[must_use]
    pub fn sector_count(&self) -> u32 {
        (self.data.len() / SECTOR_RAW) as u32
    }
}

/// A response queued for delivery after a delay, modelling the way real
/// hardware serializes `INTn` responses.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
struct PendingResp {
    /// Remaining CPU cycles before this response can latch.
    delay: i64,
    /// Interrupt kind (`1`, `2`, `3`, or `5`).
    int_kind: u8,
    /// Response FIFO bytes delivered when this response latches.
    bytes: Vec<u8>,
}

/// Parsed XA subheader fields for one sector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct XaInfo {
    /// Subheader file number.
    file: u8,
    /// Subheader channel number.
    channel: u8,
    /// Submode bit2: this is an Audio sector.
    audio: bool,
    /// Submode bit5: Form-2 sector (2324-byte user area).
    form2: bool,
    /// Coding bit0-1: stereo (`true`) vs mono (`false`).
    stereo: bool,
    /// Coding bit2-3: 18900 Hz (`true`) vs 37800 Hz (`false`) source rate.
    rate18900: bool,
    /// Coding bit4-5: 8-bit (`true`) vs 4-bit (`false`) samples.
    bits8: bool,
}

/// The CD-ROM controller state machine.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Cdrom {
    /// Port index (low two bits of `0x1F80_1800`), selecting the register bank.
    index: u8,
    /// Interrupt enable register (low 5 bits).
    ie: u8,
    /// Currently latched interrupt kind (`0` = none, else the `INTn` number).
    flag: u8,
    /// `BUSYSTS`: a command is being processed.
    busy: bool,

    /// Parameter FIFO (guest → controller).
    params: VecDeque<u8>,
    /// Response FIFO (controller → guest).
    response: VecDeque<u8>,
    /// Data FIFO (sector bytes streamed to the CPU/DMA).
    data_fifo: VecDeque<u8>,
    /// Deliverable bytes of the most recently read sector.
    sector_buffer: Vec<u8>,

    /// Ordered queue of pending responses awaiting delivery.
    pending: VecDeque<PendingResp>,

    /// Setmode register (speed / sector-size / etc.).
    mode: u8,
    /// Spindle motor spinning.
    motor_on: bool,
    /// Data read in progress (ReadN/ReadS).
    reading: bool,
    /// Seek in progress (transient; reflected in the status byte).
    seeking: bool,
    /// CD-DA playback in progress.
    playing: bool,

    /// Setloc seek target (logical block address).
    seek_target: u32,
    /// Current read/play head position (logical block address).
    lba: u32,
    /// Setfilter file number.
    filter_file: u8,
    /// Setfilter channel number.
    filter_channel: u8,

    /// Countdown to the next per-sector `INT1` while reading.
    read_timer: i64,
    /// Header (8 bytes: MSF+mode+subheader) of the last sector read.
    last_header: [u8; 8],

    /// The mounted disc, if any.
    disc: Option<Disc>,

    /// `true` when audio output is muted (Mute/Demute commands).
    muted: bool,
    /// Track number playback started on (for CD-DA autopause at track end).
    play_track: u8,

    /// CD-audio volume matrix, unity-scaled at `0x80` = 1.0. Real BIOS programs
    /// these via `0x1F80_1801..0x1F80_1803` index 2/3; we default to a
    /// straight-through stereo mix so CD audio is audible even without a BIOS.
    /// `vol_ll`: CD-L → SPU-L, `vol_lr`: CD-L → SPU-R, `vol_rl`: CD-R → SPU-L,
    /// `vol_rr`: CD-R → SPU-R.
    vol_ll: u8,
    vol_lr: u8,
    vol_rl: u8,
    vol_rr: u8,

    /// Decoded 44.1kHz interleaved-stereo CD-audio frames (XA-ADPCM / CD-DA)
    /// awaiting drain by [`Cdrom::take_cd_audio`] into the SPU.
    cd_audio: VecDeque<(i16, i16)>,

    /// Persistent XA-ADPCM decode history (prev1, prev2) for the left channel.
    xa_hist_l: [i32; 2],
    /// Persistent XA-ADPCM decode history (prev1, prev2) for the right channel.
    xa_hist_r: [i32; 2],
    /// XA resampler phase, in source-samples scaled by 44100 (fixed-point so the
    /// controller stays `Eq`-comparable for snapshot round-trips).
    xa_resample_pos: i64,
    /// Last source sample fed to the XA resampler (left), for continuity.
    xa_last_l: i16,
    /// Last source sample fed to the XA resampler (right), for continuity.
    xa_last_r: i16,
}

impl Default for Cdrom {
    fn default() -> Self {
        Self::new()
    }
}

impl Cdrom {
    /// Creates a fresh controller with no disc and the motor spinning.
    #[must_use]
    pub fn new() -> Self {
        Self {
            index: 0,
            ie: 0,
            flag: 0,
            busy: false,
            params: VecDeque::new(),
            response: VecDeque::new(),
            data_fifo: VecDeque::new(),
            sector_buffer: Vec::new(),
            pending: VecDeque::new(),
            mode: 0,
            motor_on: true,
            reading: false,
            seeking: false,
            playing: false,
            seek_target: 0,
            lba: 0,
            filter_file: 0,
            filter_channel: 0,
            read_timer: 0,
            last_header: [0; 8],
            disc: None,
            muted: false,
            play_track: 1,
            vol_ll: 0x80,
            vol_lr: 0,
            vol_rl: 0,
            vol_rr: 0x80,
            cd_audio: VecDeque::new(),
            xa_hist_l: [0; 2],
            xa_hist_r: [0; 2],
            xa_resample_pos: 0,
            xa_last_l: 0,
            xa_last_r: 0,
        }
    }

    /// Drains and returns all queued decoded CD-audio frames (interleaved
    /// stereo, 44.1kHz). A later step mixes these into the SPU output.
    pub fn take_cd_audio(&mut self) -> Vec<(i16, i16)> {
        self.cd_audio.drain(..).collect()
    }

    /// Returns `true` if any decoded CD-audio frames are queued. Callers use
    /// this to avoid the per-instruction `take_cd_audio` Vec allocation when the
    /// drive has produced nothing (the common idle case).
    #[inline]
    #[must_use]
    pub fn has_cd_audio(&self) -> bool {
        !self.cd_audio.is_empty()
    }

    /// Returns `true` if `phys` falls in the CD-ROM register window.
    #[must_use]
    pub fn contains(phys: u32) -> bool {
        matches!(phys, CDROM_BASE..=CDROM_END)
    }

    /// Inserts a disc, synthesizing a default single-track TOC if the disc has
    /// none.
    pub fn insert_disc(&mut self, mut disc: Disc) {
        if disc.tracks.is_empty() {
            disc.tracks.push(DiscTrack {
                number: 1,
                start_lba: 0,
                audio: false,
            });
        }
        if disc.lead_out_lba == 0 {
            disc.lead_out_lba = disc.sector_count();
        }
        self.disc = Some(disc);
        self.motor_on = true;
    }

    /// Ejects the current disc, if any.
    pub fn eject(&mut self) {
        self.disc = None;
        self.reading = false;
        self.playing = false;
    }

    /// Returns `true` if a disc is present.
    #[must_use]
    pub fn has_disc(&self) -> bool {
        self.disc.is_some()
    }

    // ---- register access -------------------------------------------------

    /// Reads an 8-bit CD-ROM register. Some ports pop a FIFO, so this takes
    /// `&mut self`.
    pub fn read8(&mut self, phys: u32) -> u8 {
        match phys {
            0x1F80_1800 => self.status_byte(),
            0x1F80_1801 => self.response.pop_front().unwrap_or(0),
            0x1F80_1802 => self.data_fifo.pop_front().unwrap_or(0),
            0x1F80_1803 => match self.index {
                // Even index: interrupt enable register.
                0 | 2 => self.ie | 0xE0,
                // Odd index: interrupt flag register (low 3 bits = current INTn).
                _ => (self.flag & 0x07) | 0xE0,
            },
            _ => 0,
        }
    }

    /// Reads a 16-bit value. The CD-ROM registers are 8-bit; a wider read
    /// reflects the single addressed register mirrored across the access width
    /// (the byte-lane is replicated on the data bus), *not* a word composed from
    /// the four adjacent ports. The addressed port is read only once, so any
    /// FIFO-popping side effect happens a single time.
    pub fn read16(&mut self, phys: u32) -> u16 {
        let b = self.read8(phys);
        u16::from_le_bytes([b, b])
    }

    /// Reads a 32-bit value, mirroring the single addressed 8-bit register
    /// across all four byte lanes (see [`Cdrom::read16`]).
    pub fn read32(&mut self, phys: u32) -> u32 {
        let b = self.read8(phys);
        u32::from_le_bytes([b, b, b, b])
    }

    /// Pops four bytes from the data FIFO as a little-endian word (for DMA).
    pub fn read_data_word(&mut self) -> u32 {
        let mut b = [0u8; 4];
        for x in &mut b {
            *x = self.data_fifo.pop_front().unwrap_or(0);
        }
        u32::from_le_bytes(b)
    }

    /// Writes an 8-bit CD-ROM register.
    pub fn write8(&mut self, phys: u32, val: u8) {
        match phys {
            0x1F80_1800 => self.index = val & 0x03,
            0x1F80_1801 => match self.index {
                0 => self.run_command(val),
                // index3: CD Audio Volume Right-CD-Out → Right-SPU-In (RR).
                3 => self.vol_rr = val,
                // index 1/2: sound-map registers — accept & ignore.
                _ => {}
            },
            0x1F80_1802 => match self.index {
                0 => {
                    if self.params.len() < FIFO_CAP {
                        self.params.push_back(val);
                    }
                }
                1 => self.ie = val & 0x1F,
                // index2: CD Audio Volume Left-CD-Out → Left-SPU-In (LL).
                2 => self.vol_ll = val,
                // index3: CD Audio Volume Right-CD-Out → Left-SPU-In (RL).
                _ => self.vol_rl = val,
            },
            0x1F80_1803 => match self.index {
                0 => {
                    // Request register: BFRD (bit7) loads the data FIFO from the
                    // sector buffer; clearing it flushes the data FIFO.
                    if val & 0x80 != 0 {
                        self.data_fifo.clear();
                        self.data_fifo.extend(self.sector_buffer.iter().copied());
                    } else {
                        self.data_fifo.clear();
                    }
                }
                1 => {
                    // Interrupt flag register: writing bits 0-2 acknowledges the
                    // current interrupt; bit6 resets the parameter FIFO.
                    if val & 0x07 != 0 {
                        self.flag = 0;
                    }
                    if val & 0x40 != 0 {
                        self.params.clear();
                    }
                }
                // index2: CD Audio Volume Left-CD-Out → Right-SPU-In (LR).
                2 => self.vol_lr = val,
                // index3: apply-changes latch (bit5). We apply matrix writes
                // immediately, so commit is a no-op.
                _ => {}
            },
            _ => {}
        }
    }

    /// Writes a 16-bit value. As an 8-bit device, the CD-ROM latches each byte
    /// of a wider store into the *same* addressed register in ascending order
    /// (the memory controller issues repeated byte cycles to the one port); the
    /// high byte therefore lands last. This is not a write spread across the
    /// four adjacent ports — spreading it would spuriously latch the second byte
    /// as a command/parameter and leave BUSYSTS set.
    pub fn write16(&mut self, phys: u32, val: u16) {
        let b = val.to_le_bytes();
        self.write8(phys, b[0]);
        self.write8(phys, b[1]);
    }

    /// Writes a 32-bit value, latching all four bytes into the single addressed
    /// register in ascending order (see [`Cdrom::write16`]).
    pub fn write32(&mut self, phys: u32, val: u32) {
        for b in val.to_le_bytes() {
            self.write8(phys, b);
        }
    }

    /// The status/index register byte (`0x1F80_1800` read).
    fn status_byte(&self) -> u8 {
        let mut s = self.index & 0x03;
        // bit2 XA-ADPCM FIFO has data — always 0 (no XA engine).
        if self.params.is_empty() {
            s |= 0x08; // PRMEMPT: parameter FIFO empty
        }
        if self.params.len() < FIFO_CAP {
            s |= 0x10; // PRMWRDY: parameter FIFO not full
        }
        if !self.response.is_empty() {
            s |= 0x20; // RSLRRDY: response FIFO not empty
        }
        if !self.data_fifo.is_empty() {
            s |= 0x40; // DRQSTS: data FIFO has data
        }
        if self.busy {
            s |= 0x80; // BUSYSTS: command in progress
        }
        s
    }

    // ---- command execution ----------------------------------------------

    /// Current status byte reported in command responses.
    fn stat(&self) -> u8 {
        let mut s = 0u8;
        if self.disc.is_none() {
            s |= 0x10; // shell open / no disc
        }
        if self.motor_on {
            s |= 0x02;
        }
        if self.reading {
            s |= 0x20;
        }
        if self.seeking {
            s |= 0x40;
        }
        if self.playing {
            s |= 0x80;
        }
        s
    }

    /// `true` if Setmode selects double speed (bit7).
    fn double_speed(&self) -> bool {
        self.mode & 0x80 != 0
    }

    /// `true` if Setmode selects whole-sector delivery (bit5, 2340 bytes).
    fn whole_sector(&self) -> bool {
        self.mode & 0x20 != 0
    }

    /// Setmode bit0 (CDDA): allow CD-DA sectors to be routed to the SPU. Real
    /// drives require this before CD-DA feeds audio, so we gate emission on it.
    fn cdda_mode(&self) -> bool {
        self.mode & 0x01 != 0
    }

    /// Setmode bit1 (Autopause): stop CD-DA at the end of the current track.
    fn autopause(&self) -> bool {
        self.mode & 0x02 != 0
    }

    /// Setmode bit2 (Report): emit periodic INT1 position reports during CD-DA.
    fn report(&self) -> bool {
        self.mode & 0x04 != 0
    }

    /// Setmode bit6 (XA filter): restrict XA-ADPCM decode to the Setfilter
    /// file/channel.
    fn xa_filter(&self) -> bool {
        self.mode & 0x40 != 0
    }

    /// Sector read period for the current speed.
    fn sector_period(&self) -> i64 {
        if self.double_speed() {
            READ_PERIOD_DOUBLE
        } else {
            READ_PERIOD_SINGLE
        }
    }

    /// Enqueues a response for delayed delivery.
    fn push_resp(&mut self, delay: i64, int_kind: u8, bytes: Vec<u8>) {
        self.pending.push_back(PendingResp {
            delay,
            int_kind,
            bytes,
        });
    }

    /// Executes a command byte written to `0x1F80_1801` (index 0).
    fn run_command(&mut self, cmd: u8) {
        self.busy = true;
        let params: Vec<u8> = self.params.drain(..).collect();
        let p = |i: usize| params.get(i).copied().unwrap_or(0);

        match cmd {
            // Getstat.
            0x01 => self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]),
            // Setloc(amm, ass, asect) — BCD.
            0x02 => {
                self.seek_target = msf_bcd_to_lba(p(0), p(1), p(2));
                self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
            }
            // Play(track?). With a non-zero BCD track parameter, seek to that
            // track's start; otherwise resume at the Setloc target.
            0x03 => {
                self.reading = false;
                if p(0) != 0 {
                    let track = bcd_to_bin(p(0));
                    self.lba = self.track_start_lba(track);
                } else {
                    self.lba = self.seek_target;
                }
                self.playing = true;
                self.play_track = self.current_track();
                // CD-DA always plays at 1x (single speed) for correct pitch —
                // the play period ignores Setmode bit7 (see `tick_one`).
                self.read_timer = READ_PERIOD_SINGLE;
                self.reset_audio_decoders();
                self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
            }
            // ReadN / ReadS.
            0x06 | 0x1B => {
                self.lba = self.seek_target;
                self.reading = true;
                self.playing = false;
                self.read_timer = self.sector_period();
                self.reset_audio_decoders();
                self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
            }
            // MotorOn.
            0x07 => {
                self.motor_on = true;
                self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
                self.push_resp(SECOND_RESP_DELAY, 2, vec![self.stat()]);
            }
            // Stop.
            0x08 => {
                self.reading = false;
                self.playing = false;
                let ack = self.stat();
                self.motor_on = false;
                self.push_resp(FIRST_RESP_DELAY, 3, vec![ack]);
                self.push_resp(SECOND_RESP_DELAY, 2, vec![self.stat()]);
            }
            // Pause.
            0x09 => {
                let ack = self.stat();
                self.reading = false;
                self.playing = false;
                self.push_resp(FIRST_RESP_DELAY, 3, vec![ack]);
                self.push_resp(SECOND_RESP_DELAY, 2, vec![self.stat()]);
            }
            // Init.
            0x0A => {
                self.mode = 0;
                self.reading = false;
                self.playing = false;
                self.motor_on = true;
                self.params.clear();
                self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
                self.push_resp(SECOND_RESP_DELAY, 2, vec![self.stat()]);
            }
            // Mute / Demute.
            0x0B | 0x0C => {
                self.muted = cmd == 0x0B;
                self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
            }
            // Setfilter(file, channel).
            0x0D => {
                self.filter_file = p(0);
                self.filter_channel = p(1);
                self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
            }
            // Setmode(mode).
            0x0E => {
                self.mode = p(0);
                self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
            }
            // Getparam.
            0x0F => {
                let resp = vec![
                    self.stat(),
                    self.mode,
                    0,
                    self.filter_file,
                    self.filter_channel,
                ];
                self.push_resp(FIRST_RESP_DELAY, 3, resp);
            }
            // GetlocL.
            0x10 => {
                let h = self.locl_bytes();
                self.push_resp(FIRST_RESP_DELAY, 3, h.to_vec());
            }
            // GetlocP.
            0x11 => {
                let resp = self.locp_bytes();
                self.push_resp(FIRST_RESP_DELAY, 3, resp.to_vec());
            }
            // GetTN.
            0x13 => {
                let (first, last) = self.track_range();
                let resp = vec![self.stat(), bin_to_bcd(first), bin_to_bcd(last)];
                self.push_resp(FIRST_RESP_DELAY, 3, resp);
            }
            // GetTD(track) — BCD.
            0x14 => {
                let track = bcd_to_bin(p(0));
                let (mm, ss, _ff) = self.track_start_msf(track);
                let resp = vec![self.stat(), bin_to_bcd(mm), bin_to_bcd(ss)];
                self.push_resp(FIRST_RESP_DELAY, 3, resp);
            }
            // SeekL / SeekP.
            0x15 | 0x16 => {
                self.lba = self.seek_target;
                self.seeking = true;
                let ack = self.stat();
                self.seeking = false;
                self.push_resp(FIRST_RESP_DELAY, 3, vec![ack]);
                self.push_resp(SECOND_RESP_DELAY, 2, vec![self.stat()]);
            }
            // Test(subfn).
            0x19 => {
                if p(0) == 0x20 {
                    self.push_resp(FIRST_RESP_DELAY, 3, vec![0x94, 0x09, 0x19, 0xC0]);
                } else {
                    self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
                }
            }
            // GetID.
            0x1A => {
                self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
                if self.disc.is_some() {
                    self.push_resp(
                        SECOND_RESP_DELAY,
                        2,
                        vec![0x02, 0x00, 0x20, 0x00, 0x53, 0x43, 0x45, 0x41],
                    );
                } else {
                    self.push_resp(SECOND_RESP_DELAY, 5, vec![0x08, 0x40, 0, 0, 0, 0, 0, 0]);
                }
            }
            // ReadTOC.
            0x1E => {
                self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
                self.push_resp(SECOND_RESP_DELAY, 2, vec![self.stat()]);
            }
            // Unknown command.
            _ => {
                let err = self.stat() | 0x01;
                self.push_resp(FIRST_RESP_DELAY, 5, vec![err, 0x40]);
            }
        }
    }

    /// GetlocL response bytes (last-read sector header + subheader).
    fn locl_bytes(&self) -> [u8; 8] {
        if self.last_header != [0; 8] {
            self.last_header
        } else {
            let (mm, ss, ff) = lba_to_msf(self.lba);
            [
                bin_to_bcd(mm),
                bin_to_bcd(ss),
                bin_to_bcd(ff),
                self.mode,
                0,
                0,
                0,
                0,
            ]
        }
    }

    /// GetlocP response bytes (track/index + relative and absolute MSF).
    fn locp_bytes(&self) -> [u8; 8] {
        let track = self.current_track();
        let start = self.track_start_lba(track);
        let rel = self.lba.saturating_sub(start);
        let (amm, ass, aff) = lba_to_msf(self.lba);
        let (mm, ss, ff) = lba_to_msf_rel(rel);
        [
            bin_to_bcd(track),
            0x01,
            bin_to_bcd(mm),
            bin_to_bcd(ss),
            bin_to_bcd(ff),
            bin_to_bcd(amm),
            bin_to_bcd(ass),
            bin_to_bcd(aff),
        ]
    }

    /// First and last track numbers.
    fn track_range(&self) -> (u8, u8) {
        match &self.disc {
            Some(d) if !d.tracks.is_empty() => {
                let first = d.tracks.iter().map(|t| t.number).min().unwrap_or(1);
                let last = d.tracks.iter().map(|t| t.number).max().unwrap_or(1);
                (first, last)
            }
            _ => (1, 1),
        }
    }

    /// LBA where `track` starts (track 0 = lead-out).
    fn track_start_lba(&self, track: u8) -> u32 {
        match &self.disc {
            Some(d) => {
                if track == 0 {
                    d.lead_out_lba
                } else if let Some(t) = d.tracks.iter().find(|t| t.number == track) {
                    t.start_lba
                } else {
                    0
                }
            }
            None => 0,
        }
    }

    /// Start of `track` as an absolute MSF triple.
    fn track_start_msf(&self, track: u8) -> (u8, u8, u8) {
        lba_to_msf(self.track_start_lba(track))
    }

    /// Track number containing the current head position.
    fn current_track(&self) -> u8 {
        match &self.disc {
            Some(d) => {
                let mut cur = 1u8;
                for t in &d.tracks {
                    if self.lba >= t.start_lba {
                        cur = t.number;
                    }
                }
                cur
            }
            None => 1,
        }
    }

    /// Reads the current sector into the sector buffer and captures its header,
    /// then advances the head. Out-of-range / no-disc reads deliver zeros.
    fn read_current_sector(&mut self) {
        let whole = self.whole_sector();
        let (start, len) = if whole {
            (12usize, 2340usize)
        } else {
            (24usize, 2048usize)
        };
        let off = self.lba as usize * SECTOR_RAW;

        let mut buf = vec![0u8; len];
        let mut header = [0u8; 8];
        if let Some(d) = &self.disc {
            if off + start + len <= d.data.len() {
                buf.copy_from_slice(&d.data[off + start..off + start + len]);
            }
            // The header lives at raw offset 12 (4 header + first 4 subheader).
            if off + SECTOR_RAW <= d.data.len() {
                header.copy_from_slice(&d.data[off + 12..off + 20]);
            } else {
                let (mm, ss, ff) = lba_to_msf(self.lba);
                header = [
                    bin_to_bcd(mm),
                    bin_to_bcd(ss),
                    bin_to_bcd(ff),
                    self.mode,
                    0,
                    0,
                    0,
                    0,
                ];
            }
        }
        self.sector_buffer = buf;
        self.last_header = header;
        self.lba = self.lba.wrapping_add(1);
    }

    // ---- CD audio: XA-ADPCM + CD-DA -------------------------------------

    /// Resets the XA-ADPCM decode history and resampler state on a fresh
    /// Play/Read start (the decoded-audio queue is left intact for the drain).
    fn reset_audio_decoders(&mut self) {
        self.xa_hist_l = [0; 2];
        self.xa_hist_r = [0; 2];
        self.xa_resample_pos = 0;
        self.xa_last_l = 0;
        self.xa_last_r = 0;
    }

    /// Reads the four XA subheader bytes (file, channel, submode, coding-info)
    /// at raw sector offset 16..20. Returns `None` if there is no disc or the
    /// sector is out of range.
    fn raw_subheader(&self, off: usize) -> Option<[u8; 4]> {
        let d = self.disc.as_ref()?;
        if off + 20 <= d.data.len() {
            Some([
                d.data[off + 16],
                d.data[off + 17],
                d.data[off + 18],
                d.data[off + 19],
            ])
        } else {
            None
        }
    }

    /// Parses the XA subheader at raw offset `off` into structured fields.
    fn parse_xa(&self, off: usize) -> Option<XaInfo> {
        let sh = self.raw_subheader(off)?;
        let submode = sh[2];
        let coding = sh[3];
        Some(XaInfo {
            file: sh[0],
            channel: sh[1],
            audio: submode & 0x04 != 0,           // submode bit2 = Audio
            form2: submode & 0x20 != 0,           // submode bit5 = Form2
            stereo: coding & 0x03 == 1,           // coding bit0-1: 0=mono, 1=stereo
            rate18900: (coding >> 2) & 0x03 == 1, // coding bit2-3: 0=37800, 1=18900
            bits8: (coding >> 4) & 0x03 == 1,     // coding bit4-5: 0=4bit, 1=8bit
        })
    }

    /// Pushes one decoded CD-audio frame through the volume matrix into the
    /// capped output queue. `0x80` in a matrix cell is unity gain.
    fn push_cd_frame(&mut self, cd_l: i16, cd_r: i16) {
        let l = ((i32::from(cd_l) * i32::from(self.vol_ll)
            + i32::from(cd_r) * i32::from(self.vol_rl))
            >> 7)
            .clamp(-32768, 32767) as i16;
        let r = ((i32::from(cd_l) * i32::from(self.vol_lr)
            + i32::from(cd_r) * i32::from(self.vol_rr))
            >> 7)
            .clamp(-32768, 32767) as i16;
        self.cd_audio_push(l, r);
    }

    /// Appends a frame to the output queue, dropping the oldest at the cap.
    fn cd_audio_push(&mut self, l: i16, r: i16) {
        if self.cd_audio.len() >= CD_AUDIO_CAP {
            self.cd_audio.pop_front();
        }
        self.cd_audio.push_back((l, r));
    }

    /// Processes one sector while reading (ReadN/ReadS). XA-audio sectors are
    /// decoded to `cd_audio` and consumed (no CPU data delivery, no INT1); all
    /// other sectors take the normal data-delivery path with an INT1.
    fn process_read_sector(&mut self) {
        let off = self.lba as usize * SECTOR_RAW;
        if let Some(info) = self.parse_xa(off)
            && info.audio
            && info.form2
        {
            // XA-audio sector. When the XA filter (Setmode bit6) is on, only
            // decode sectors matching the Setfilter file/channel; others are
            // dropped entirely (no audio, no CPU data).
            let pass = !self.xa_filter()
                || (info.file == self.filter_file && info.channel == self.filter_channel);
            if pass && !self.muted {
                self.decode_xa_sector(off, info);
            }
            // XA-audio sectors are never delivered as CPU data — advance the
            // head and emit no INT1 (muted XA simply skips the push above).
            self.lba = self.lba.wrapping_add(1);
            return;
        }
        // Normal data sector: deliver bytes and raise INT1 (unchanged behavior).
        self.read_current_sector();
        let stat = self.stat();
        self.push_resp(0, 1, vec![stat]);
    }

    /// Decodes one XA-ADPCM sector (18 sound groups of 128 bytes at raw offset
    /// 24) into 44.1kHz stereo frames.
    ///
    /// Layout (verified against Nocash PSX-SPX "CDROM XA Audio ADPCM
    /// Compression"): each 128-byte group is a 16-byte header + 112 data bytes
    /// (28 words of 4). A "sound unit" `u` reads shift/filter parameter
    /// `header[4 + blk*2 + nibble]` and sample bytes `data[16 + blk + s*4]`,
    /// where for 4-bit `blk = u>>1`, `nibble = u&1` (so the parameter reduces to
    /// `header[4 + u]`) and for 8-bit `blk = u`, `nibble = 0` (parameter
    /// `header[4 + 2u]`). The sample is `(t << 12) >> shift` sign-extended, then
    /// filtered by `pred = (prev1*pos + prev2*neg + 32) >> 6` and clamped to
    /// i16. Stereo splits even units to the left channel and odd units to the
    /// right; mono duplicates to both. The nibble/param interleave is asserted
    /// against this documented layout by `xa_adpcm_decode_synthetic`, not
    /// against real silicon.
    fn decode_xa_sector(&mut self, off: usize, info: XaInfo) {
        // Copy the 2304-byte audio region out so we can mutate decode state
        // without holding an immutable borrow of the disc image.
        let audio: Vec<u8> = {
            let d = match &self.disc {
                Some(d) => d,
                None => return,
            };
            let start = off + 24;
            if start + 2304 > d.data.len() {
                return;
            }
            d.data[start..start + 2304].to_vec()
        };

        let mut src_l: Vec<i16> = Vec::new();
        let mut src_r: Vec<i16> = Vec::new();
        let n_units = if info.bits8 { 4 } else { 8 };

        for g in 0..18 {
            let group = &audio[g * 128..g * 128 + 128];
            let header = &group[0..16];
            let gdata = &group[16..128];
            for u in 0..n_units {
                let param = if info.bits8 {
                    header[4 + u * 2]
                } else {
                    header[4 + u]
                };
                // shift = 12 - (12 - shift); clamp to 12 so the arithmetic right
                // shift on i32 never overflows.
                let shift = u32::from(param & 0x0F).min(12);
                let filter = ((param >> 4) & 0x07).min(4) as usize;
                let f0 = XA_POS[filter];
                let f1 = XA_NEG[filter];

                let left = if info.stereo { u % 2 == 0 } else { true };
                let (mut prev1, mut prev2) = if left {
                    (self.xa_hist_l[0], self.xa_hist_l[1])
                } else {
                    (self.xa_hist_r[0], self.xa_hist_r[1])
                };

                for s in 0..28 {
                    let sample_in = if info.bits8 {
                        let byte = i32::from(gdata[u + s * 4] as i8);
                        (byte << 8) >> shift
                    } else {
                        let byte = gdata[(u >> 1) + s * 4];
                        let nib = (byte >> ((u & 1) * 4)) & 0x0F;
                        // Sign-extend the 4-bit nibble via a 16-bit left shift.
                        (i32::from(((u16::from(nib)) << 12) as i16)) >> shift
                    };
                    let pred = (prev1 * f0 + prev2 * f1 + 32) >> 6;
                    let out = (sample_in + pred).clamp(-32768, 32767);
                    prev2 = prev1;
                    prev1 = out;
                    if left {
                        src_l.push(out as i16);
                    } else {
                        src_r.push(out as i16);
                    }
                }

                if left {
                    self.xa_hist_l = [prev1, prev2];
                } else {
                    self.xa_hist_r = [prev1, prev2];
                }
            }
        }

        if !info.stereo {
            src_r = src_l.clone();
        }

        let rate: u32 = if info.rate18900 { 18900 } else { 37800 };
        self.xa_resample_push(&src_l, &src_r, rate);
    }

    /// Resamples decoded XA source samples to 44100 Hz by linear interpolation
    /// (src → 44100), carrying fractional phase and the last source sample per
    /// channel across sectors for continuity, then pushes each frame through the
    /// CD volume matrix.
    fn xa_resample_push(&mut self, src_l: &[i16], src_r: &[i16], rate: u32) {
        if src_l.is_empty() {
            return;
        }
        // Position is measured in source samples scaled by 44100 (integer, so
        // state stays `Eq`). Each output frame advances the source position by
        // `rate/44100`, i.e. `rate` in scaled units.
        const SCALE: i64 = 44_100;
        let step = i64::from(rate);
        let n = src_l.len() as i64;
        let mut x = self.xa_resample_pos;
        while x < n * SCALE {
            let i = (x / SCALE) as usize;
            let f = x % SCALE;
            let l0 = if i == 0 {
                i64::from(self.xa_last_l)
            } else {
                i64::from(src_l[i - 1])
            };
            let l1 = i64::from(src_l[i]);
            let r0 = if i == 0 {
                i64::from(self.xa_last_r)
            } else {
                i64::from(src_r[i - 1])
            };
            let r1 = i64::from(src_r[i]);
            let ol = (l0 + (l1 - l0) * f / SCALE).clamp(-32768, 32767) as i16;
            let or = (r0 + (r1 - r0) * f / SCALE).clamp(-32768, 32767) as i16;
            self.push_cd_frame(ol, or);
            x += step;
        }
        self.xa_resample_pos = x - n * SCALE;
        self.xa_last_l = src_l[src_l.len() - 1];
        self.xa_last_r = src_r[src_r.len() - 1];
    }

    /// Processes one CD-DA (audio-track) sector: 2352 raw bytes = 588 stereo
    /// i16 LE PCM frames at exactly 44.1kHz. Handles autopause at track end
    /// (Setmode bit1 → INT4) and periodic position reports (Setmode bit2 →
    /// INT1); report cadence and subq are approximate (once per sector).
    fn process_cdda_sector(&mut self) {
        let end = match &self.disc {
            Some(d) => {
                self.lba >= d.lead_out_lba || (self.lba as usize + 1) * SECTOR_RAW > d.data.len()
            }
            None => true,
        };
        let crossed = self.current_track() != self.play_track;
        if end || (self.autopause() && crossed) {
            self.playing = false;
            // Autopause reports completion with an INT4 (per Nocash).
            if self.autopause() && self.pending.is_empty() {
                let stat = self.stat();
                self.push_resp(0, 4, vec![stat]);
            }
            return;
        }

        // Gather 588 stereo PCM frames from the raw audio sector (offset 0).
        let frames: Vec<(i16, i16)> = {
            let d = match &self.disc {
                Some(d) => d,
                None => {
                    self.playing = false;
                    return;
                }
            };
            let off = self.lba as usize * SECTOR_RAW;
            (0..588)
                .map(|i| {
                    let b = off + i * 4;
                    let l = i16::from_le_bytes([d.data[b], d.data[b + 1]]);
                    let r = i16::from_le_bytes([d.data[b + 2], d.data[b + 3]]);
                    (l, r)
                })
                .collect()
        };

        // Emit audio only when Setmode bit0 (CDDA) routes CD-DA to the SPU.
        // Muted playback still advances the head/timer but pushes silence to
        // keep the SPU sample clock fed.
        let audible = self.cdda_mode();
        for (l, r) in frames {
            if !audible {
                continue;
            }
            if self.muted {
                self.cd_audio_push(0, 0);
            } else {
                self.push_cd_frame(l, r);
            }
        }

        if self.report() && self.pending.is_empty() {
            let locp = self.locp_bytes();
            self.push_resp(0, 1, locp.to_vec());
        }

        self.lba = self.lba.wrapping_add(1);
    }

    // ---- per-cycle tick --------------------------------------------------

    /// Advances the controller by `cycles` CPU cycles, delivering queued
    /// responses and raising [`IrqLine::CdRom`] when an enabled interrupt
    /// latches.
    pub fn tick(&mut self, cycles: u32, irq: &mut Irq) {
        // Idle fast path. `tick_one` only mutates state when the drive is
        // reading (advancing `read_timer` / `process_read_sector`), playing
        // (CD-DA sector cadence), or has a queued `pending` response (delay
        // countdown + latch). When none of those hold it is a pure no-op, and
        // nothing in `tick_one` starts a read/play or enqueues a response, so
        // the predicate stays true for the whole `cycles` window. Skipping the
        // per-cycle loop is therefore exactly equivalent to running it.
        if !self.reading && !self.playing && self.pending.is_empty() {
            return;
        }
        for _ in 0..cycles {
            self.tick_one(irq);
        }
    }

    /// Number of CPU cycles from the current state until the CD-ROM controller
    /// next performs an autonomous `I_STAT`-setting action (an INTn latch), for
    /// the lazy device scheduler. Returns `None` when the drive is idle.
    ///
    /// This is deliberately **conservative** — it may return a value smaller
    /// than the true next event, which only makes the scheduler catch the device
    /// up more often, never less:
    ///
    /// * Idle (`!reading && !playing && pending empty`) → `None`.
    /// * Producing CD audio (`playing`, or reading an XA-audio sector) → `Some(1)`
    ///   so the CD→SPU frame bridge runs per cycle and stays ordered with the
    ///   SPU's sample consumption (see [`Cdrom::is_cd_audio_active`]).
    /// * Otherwise → the smaller of the sector `read_timer` (while reading) and
    ///   the front pending response's `delay`, clamped to at least 1.
    #[must_use]
    pub fn cycles_to_next_event(&self) -> Option<u64> {
        if !self.reading && !self.playing && self.pending.is_empty() {
            return None;
        }
        if self.is_cd_audio_active() {
            return Some(1);
        }
        let mut off = i64::MAX;
        if self.reading {
            off = off.min(self.read_timer);
        }
        if let Some(front) = self.pending.front() {
            off = off.min(front.delay);
        }
        if off == i64::MAX {
            off = 1;
        }
        Some(off.max(1) as u64)
    }

    /// `true` when the drive is (or is about to be) decoding CD audio into the
    /// `cd_audio` queue: CD-DA playback, or reading a sector whose subheader
    /// marks it XA-audio (Form-2 Audio). Such windows must catch the CD→SPU
    /// bridge up cycle-by-cycle so decoded frames reach the SPU in the same
    /// order the naive per-instruction loop delivered them.
    pub(crate) fn is_cd_audio_active(&self) -> bool {
        if self.playing {
            return true;
        }
        if self.reading {
            let off = self.lba as usize * SECTOR_RAW;
            if let Some(info) = self.parse_xa(off) {
                return info.audio && info.form2;
            }
        }
        false
    }

    fn tick_one(&mut self, irq: &mut Irq) {
        // While reading, process a sector each read period: XA-audio sectors are
        // decoded to `cd_audio` (no CPU data / INT1); data sectors deliver bytes
        // and raise INT1 as before.
        if self.reading {
            if self.read_timer > 0 {
                self.read_timer -= 1;
            }
            if self.read_timer <= 0 {
                self.process_read_sector();
                self.read_timer = self.sector_period();
            }
        } else if self.playing {
            // CD-DA runs at a fixed single-speed period regardless of Setmode
            // bit7: 588 stereo frames per sector at 1x = exactly 44100 frames/s
            // (READ_PERIOD_SINGLE = 451_584 = 588 * 768), so no resampling is
            // needed for CD-DA.
            if self.read_timer > 0 {
                self.read_timer -= 1;
            }
            if self.read_timer <= 0 {
                self.process_cdda_sector();
                self.read_timer = READ_PERIOD_SINGLE;
            }
        }

        // Count down the front pending response.
        if let Some(front) = self.pending.front_mut()
            && front.delay > 0
        {
            front.delay -= 1;
        }

        // Latch it once its delay elapses and the previous interrupt is acked.
        let ready = self.flag == 0 && self.pending.front().is_some_and(|f| f.delay <= 0);
        if ready && let Some(resp) = self.pending.pop_front() {
            self.latch_response(resp, irq);
        }
    }

    /// Latches a response: publishes its bytes, sets the interrupt flag, clears
    /// busy, and raises the CD interrupt if enabled.
    fn latch_response(&mut self, resp: PendingResp, irq: &mut Irq) {
        self.flag = resp.int_kind & 0x07;
        self.response.clear();
        for b in resp.bytes {
            if self.response.len() < FIFO_CAP {
                self.response.push_back(b);
            }
        }
        // The first response of a command clears the busy flag.
        self.busy = false;
        if self.irq_asserted() {
            irq.set(IrqLine::CdRom);
        }
    }

    /// `true` when the latched interrupt is enabled and should assert the line.
    fn irq_asserted(&self) -> bool {
        (self.flag & self.ie & 0x1F) != 0
    }

    /// Test-only: seeds the sector buffer directly (bypassing a disc read).
    #[cfg(test)]
    pub(crate) fn set_sector_buffer_for_test(&mut self, bytes: Vec<u8>) {
        self.sector_buffer = bytes;
    }
}

// ---- BCD / MSF helpers ---------------------------------------------------

/// Converts a binary value (0-99) to two-digit packed BCD.
#[must_use]
pub fn bin_to_bcd(v: u8) -> u8 {
    ((v / 10) << 4) | (v % 10)
}

/// Converts a two-digit packed BCD value to binary.
#[must_use]
pub fn bcd_to_bin(v: u8) -> u8 {
    (v >> 4) * 10 + (v & 0x0F)
}

/// Converts an absolute BCD MSF address to a logical block address (accounting
/// for the 150-sector pregap).
#[must_use]
pub fn msf_bcd_to_lba(mm: u8, ss: u8, ff: u8) -> u32 {
    let m = u32::from(bcd_to_bin(mm));
    let s = u32::from(bcd_to_bin(ss));
    let f = u32::from(bcd_to_bin(ff));
    ((m * 60 + s) * 75 + f).saturating_sub(PREGAP)
}

/// Converts a logical block address to an absolute MSF triple (binary).
#[must_use]
pub fn lba_to_msf(lba: u32) -> (u8, u8, u8) {
    let total = lba + PREGAP;
    let mm = total / (60 * 75);
    let rem = total % (60 * 75);
    let ss = rem / 75;
    let ff = rem % 75;
    (mm as u8, ss as u8, ff as u8)
}

/// Converts a track-relative block offset to an MSF triple (no pregap).
#[must_use]
pub fn lba_to_msf_rel(lba: u32) -> (u8, u8, u8) {
    let mm = lba / (60 * 75);
    let rem = lba % (60 * 75);
    let ss = rem / 75;
    let ff = rem % 75;
    (mm as u8, ss as u8, ff as u8)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a disc of `n` sectors where sector `k`'s Mode-2 Form-1 user area
    /// (2048 bytes at raw offset 24) is filled with byte `k`. Bytes 12..20 carry
    /// a plausible header.
    fn make_disc(n: usize) -> Disc {
        let mut data = vec![0u8; n * SECTOR_RAW];
        for k in 0..n {
            let base = k * SECTOR_RAW;
            // Sync pattern (00 FF*10 00) at 0..12 left as-is except markers.
            let (mm, ss, ff) = lba_to_msf(k as u32);
            data[base + 12] = bin_to_bcd(mm);
            data[base + 13] = bin_to_bcd(ss);
            data[base + 14] = bin_to_bcd(ff);
            data[base + 15] = 0x02; // mode 2
            // Fill the whole 2340-byte payload region with byte k so both the
            // 2048 and 2340 deliveries carry the known pattern.
            for b in &mut data[base + 24..base + SECTOR_RAW] {
                *b = k as u8;
            }
        }
        Disc::from_bytes(data)
    }

    fn tick_until_int(cd: &mut Cdrom, irq: &mut Irq, max: i64) -> bool {
        for _ in 0..max {
            cd.tick(1, irq);
            if cd.flag != 0 {
                return true;
            }
        }
        false
    }

    #[test]
    fn cycles_to_next_event_predicts_first_response() {
        let mut cd = Cdrom::new();
        let mut irq = Irq::new();
        cd.ie = 0x1F;
        // Idle: no scheduled event.
        assert_eq!(cd.cycles_to_next_event(), None);

        // Getstat schedules an INT3 after FIRST_RESP_DELAY cycles.
        cd.write8(0x1F80_1801, 0x01);
        let n = cd
            .cycles_to_next_event()
            .expect("a response is now scheduled");
        assert_eq!(n, FIRST_RESP_DELAY as u64);

        // No INT latches strictly before the predicted cycle; it latches at it.
        for _ in 0..(n - 1) {
            cd.tick(1, &mut irq);
        }
        assert_eq!(cd.flag, 0, "no INT latched before the predicted cycle");
        cd.tick(1, &mut irq);
        assert_ne!(cd.flag, 0, "INT latched at the predicted cycle");
    }

    #[test]
    fn cycles_to_next_event_is_one_while_playing() {
        let mut cd = Cdrom::new();
        cd.insert_disc(make_disc(4));
        cd.playing = true;
        cd.read_timer = READ_PERIOD_SINGLE;
        // CD-DA playback produces audio frames, so the scheduler must catch the
        // CD→SPU bridge up every cycle.
        assert_eq!(cd.cycles_to_next_event(), Some(1));
    }

    #[test]
    fn status_reports_fifo_flags() {
        let mut cd = Cdrom::new();
        // Fresh: param FIFO empty (bit3), not full (bit4), no response (bit5=0),
        // no data (bit6=0), not busy (bit7=0).
        let s = cd.status_byte();
        assert_ne!(s & 0x08, 0, "PRMEMPT");
        assert_ne!(s & 0x10, 0, "PRMWRDY");
        assert_eq!(s & 0x20, 0, "RSLRRDY clear");
        assert_eq!(s & 0x40, 0, "DRQSTS clear");
        assert_eq!(s & 0x80, 0, "BUSYSTS clear");
        // Push a param → PRMEMPT clears.
        cd.write8(0x1F80_1802, 0xAA);
        assert_eq!(cd.status_byte() & 0x08, 0, "PRMEMPT clears after push");
    }

    #[test]
    fn param_fifo_caps_at_16() {
        let mut cd = Cdrom::new();
        for _ in 0..20 {
            cd.write8(0x1F80_1802, 0x11);
        }
        assert_eq!(cd.params.len(), FIFO_CAP);
        // PRMWRDY (bit4) clears when full.
        assert_eq!(cd.status_byte() & 0x10, 0, "PRMWRDY clear when full");
        // Bit6 reset (0x40 to index-1 flag reg) clears the param FIFO.
        cd.write8(0x1F80_1800, 0x01);
        cd.write8(0x1F80_1803, 0x40);
        assert!(cd.params.is_empty());
    }

    #[test]
    fn getstat_delivers_int3() {
        let mut cd = Cdrom::new();
        let mut irq = Irq::new();
        cd.ie = 0x1F;
        // Command Getstat (index 0).
        cd.write8(0x1F80_1801, 0x01);
        assert!(cd.busy, "busy set on command write");
        assert!(tick_until_int(&mut cd, &mut irq, 200_000), "INT delivered");
        assert_eq!(cd.flag, 3, "INT3 ack");
        assert!(!cd.busy, "busy cleared on first response");
        // Response FIFO holds the status byte (no disc → shell open | motor on).
        assert_eq!(
            cd.read8(0x1F80_1801),
            0x12,
            "no disc → 0x10 shell | 0x02 motor"
        );
    }

    #[test]
    fn ie_gates_the_irq_line() {
        let mut cd = Cdrom::new();
        let mut irq = Irq::new();
        cd.ie = 0; // interrupts disabled
        cd.write8(0x1F80_1801, 0x01);
        assert!(tick_until_int(&mut cd, &mut irq, 200_000));
        assert_eq!(cd.flag, 3);
        assert_eq!(
            irq.i_stat & (1 << IrqLine::CdRom.bit()),
            0,
            "line not raised"
        );

        // Now with IE set, a fresh command raises the line.
        cd.write8(0x1F80_1800, 0x01);
        cd.write8(0x1F80_1803, 0x07); // ack
        cd.ie = 0x1F;
        cd.write8(0x1F80_1800, 0x00);
        cd.write8(0x1F80_1801, 0x01);
        assert!(tick_until_int(&mut cd, &mut irq, 200_000));
        assert_ne!(irq.i_stat & (1 << IrqLine::CdRom.bit()), 0, "line raised");
    }

    #[test]
    fn two_phase_response_advances_on_ack() {
        let mut cd = Cdrom::new();
        let mut irq = Irq::new();
        cd.ie = 0x1F;
        // MotorOn → INT3 then INT2.
        cd.write8(0x1F80_1801, 0x07);
        assert!(tick_until_int(&mut cd, &mut irq, 200_000));
        assert_eq!(cd.flag, 3, "first is INT3");
        // Without acknowledging, INT2 cannot latch.
        for _ in 0..200_000 {
            cd.tick(1, &mut irq);
        }
        assert_eq!(cd.flag, 3, "still INT3 (not acked)");
        // Acknowledge → next tick window latches INT2.
        cd.write8(0x1F80_1800, 0x01);
        cd.write8(0x1F80_1803, 0x07);
        assert_eq!(cd.flag, 0, "acked");
        assert!(tick_until_int(&mut cd, &mut irq, 200_000));
        assert_eq!(cd.flag, 2, "second is INT2");
    }

    #[test]
    fn test_version_subfunction() {
        let mut cd = Cdrom::new();
        let mut irq = Irq::new();
        cd.write8(0x1F80_1802, 0x20); // param: subfn 0x20
        cd.write8(0x1F80_1801, 0x19); // Test
        assert!(tick_until_int(&mut cd, &mut irq, 200_000));
        assert_eq!(cd.flag, 3);
        assert_eq!(cd.read8(0x1F80_1801), 0x94);
        assert_eq!(cd.read8(0x1F80_1801), 0x09);
        assert_eq!(cd.read8(0x1F80_1801), 0x19);
        assert_eq!(cd.read8(0x1F80_1801), 0xC0);
    }

    #[test]
    fn getid_no_disc_is_int5() {
        let mut cd = Cdrom::new();
        let mut irq = Irq::new();
        cd.ie = 0x1F;
        cd.write8(0x1F80_1801, 0x1A); // GetID
        assert!(tick_until_int(&mut cd, &mut irq, 200_000));
        assert_eq!(cd.flag, 3, "ack INT3");
        // ack and advance to the error INT5.
        cd.write8(0x1F80_1800, 0x01);
        cd.write8(0x1F80_1803, 0x07);
        assert!(tick_until_int(&mut cd, &mut irq, 200_000));
        assert_eq!(cd.flag, 5, "no-disc INT5");
    }

    #[test]
    fn getid_with_disc_is_scea_int2() {
        let mut cd = Cdrom::new();
        let mut irq = Irq::new();
        cd.ie = 0x1F;
        cd.insert_disc(make_disc(4));
        cd.write8(0x1F80_1801, 0x1A);
        assert!(tick_until_int(&mut cd, &mut irq, 200_000));
        assert_eq!(cd.flag, 3);
        cd.write8(0x1F80_1800, 0x01);
        cd.write8(0x1F80_1803, 0x07);
        assert!(tick_until_int(&mut cd, &mut irq, 200_000));
        assert_eq!(cd.flag, 2, "INT2");
        // Skip stat/flags to the SCEA marker bytes.
        let bytes: Vec<u8> = (0..8).map(|_| cd.read8(0x1F80_1801)).collect();
        assert_eq!(&bytes[4..8], &[0x53, 0x43, 0x45, 0x41], "SCEA");
    }

    #[test]
    fn read_delivers_user_bytes_through_data_fifo() {
        let mut cd = Cdrom::new();
        let mut irq = Irq::new();
        cd.ie = 0x1F;
        cd.insert_disc(make_disc(4));
        // Setloc to sector 2 (absolute MSF of LBA 2 = 00:02:02 BCD).
        let (mm, ss, ff) = lba_to_msf(2);
        cd.write8(0x1F80_1802, bin_to_bcd(mm));
        cd.write8(0x1F80_1802, bin_to_bcd(ss));
        cd.write8(0x1F80_1802, bin_to_bcd(ff));
        cd.write8(0x1F80_1801, 0x02); // Setloc
        assert!(tick_until_int(&mut cd, &mut irq, 200_000));
        assert_eq!(cd.flag, 3);
        cd.write8(0x1F80_1800, 0x01);
        cd.write8(0x1F80_1803, 0x07); // ack

        // ReadN.
        cd.write8(0x1F80_1800, 0x00);
        cd.write8(0x1F80_1801, 0x06);
        // Ack the read's INT3.
        assert!(tick_until_int(&mut cd, &mut irq, 200_000));
        assert_eq!(cd.flag, 3, "read ack");
        cd.write8(0x1F80_1800, 0x01);
        cd.write8(0x1F80_1803, 0x07);
        // First data sector INT1.
        cd.write8(0x1F80_1800, 0x00);
        assert!(tick_until_int(&mut cd, &mut irq, 2_000_000), "INT1 data");
        assert_eq!(cd.flag, 1, "INT1 data ready");

        // Load the data FIFO (BFRD) and read the user bytes: sector 2 → byte 2.
        cd.write8(0x1F80_1800, 0x00);
        cd.write8(0x1F80_1803, 0x80); // Request BFRD
        assert_ne!(cd.status_byte() & 0x40, 0, "DRQSTS set");
        assert_eq!(cd.read8(0x1F80_1802), 2);
        assert_eq!(cd.read8(0x1F80_1802), 2);
        assert_eq!(cd.data_fifo.len(), 2048 - 2);
    }

    #[test]
    fn setmode_sector_size_changes_delivered_bytes() {
        let mut cd = Cdrom::new();
        cd.insert_disc(make_disc(2));
        // Whole-sector mode (bit5).
        cd.mode = 0x20;
        cd.lba = 1;
        cd.read_current_sector();
        assert_eq!(cd.sector_buffer.len(), 2340);
        // Default mode (2048).
        cd.mode = 0;
        cd.lba = 1;
        cd.read_current_sector();
        assert_eq!(cd.sector_buffer.len(), 2048);
        assert!(
            cd.sector_buffer.iter().all(|&b| b == 1),
            "sector 1 → byte 1"
        );
    }

    #[test]
    fn dma_word_pull_ordering() {
        let mut cd = Cdrom::new();
        // Sector buffer bytes 0,1,2,3,4,5,6,7 → words LE 0x03020100, 0x07060504.
        cd.sector_buffer = (0..8).collect();
        cd.write8(0x1F80_1803, 0x80); // BFRD load
        assert_eq!(cd.read_data_word(), 0x0302_0100);
        assert_eq!(cd.read_data_word(), 0x0706_0504);
        // Drained FIFO returns zero.
        assert_eq!(cd.read_data_word(), 0);
    }

    #[test]
    fn bcd_roundtrip() {
        for v in 0u8..=99 {
            assert_eq!(bcd_to_bin(bin_to_bcd(v)), v);
        }
        assert_eq!(msf_bcd_to_lba(0x00, 0x02, 0x00), 0, "MSF 00:02:00 → LBA 0");
    }

    // ---- CD audio: XA-ADPCM + CD-DA -------------------------------------

    /// Writes a 4-byte XA subheader (file, channel, submode, coding, and its
    /// duplicate copy at 20..24) into a raw sector at `base`.
    fn set_xa_subheader(data: &mut [u8], base: usize, file: u8, ch: u8, submode: u8, coding: u8) {
        for o in [16usize, 20] {
            data[base + o] = file;
            data[base + o + 1] = ch;
            data[base + o + 2] = submode;
            data[base + o + 3] = coding;
        }
    }

    #[test]
    fn xa_subheader_parsing() {
        // Audio (bit2) + Form2 (bit5) = 0x24; coding stereo|37800|4bit = 0x01.
        let mut data = vec![0u8; SECTOR_RAW];
        set_xa_subheader(&mut data, 0, 1, 2, 0x24, 0x01);
        let mut cd = Cdrom::new();
        cd.insert_disc(Disc::from_bytes(data));

        let info = cd.parse_xa(0).expect("subheader parses");
        assert_eq!(info.file, 1);
        assert_eq!(info.channel, 2);
        assert!(info.audio && info.form2, "audio + form2");
        assert!(info.stereo, "stereo");
        assert!(!info.rate18900, "37800 Hz");
        assert!(!info.bits8, "4-bit");

        // A non-audio submode (Data, bit3) is recognized as not-XA-audio.
        let mut d2 = vec![0u8; SECTOR_RAW];
        set_xa_subheader(&mut d2, 0, 0, 0, 0x08, 0x00);
        let mut cd2 = Cdrom::new();
        cd2.insert_disc(Disc::from_bytes(d2));
        let info2 = cd2.parse_xa(0).unwrap();
        assert!(!(info2.audio && info2.form2), "not an XA-audio sector");
    }

    #[test]
    fn xa_adpcm_decode_synthetic() {
        // filter=0 ⇒ pred=0, so each decoded sample is the closed-form
        // ((nibble << 12) as i16) >> shift for 4-bit, ((byte<<8) >> shift) for
        // 8-bit. History resets per Play/Read, so with filter 0 the output is
        // exactly the per-sample expansion.
        let shift: u32 = 4;

        // ---- 4-bit mono ----
        // Build a full sector: one group's unit 0 carries a constant nibble.
        // Unit 0 (4-bit) reads data[16 + 0 + s*4] low nibble; param header[4].
        let mut data = vec![0u8; SECTOR_RAW];
        set_xa_subheader(&mut data, 0, 0, 0, 0x24, 0x00); // audio+form2, mono/37800/4bit
        let audio = 24usize; // group 0 header at audio, data at audio+16
        data[audio + 4] = shift as u8; // param for unit 0: filter 0, shift 4
        // nibble value 0x5 in the low nibble of each of the 28 sample columns.
        for s in 0..28 {
            data[audio + 16 + s * 4] = 0x05;
        }

        let mut cd = Cdrom::new();
        cd.insert_disc(Disc::from_bytes(data));
        cd.reset_audio_decoders();
        let info = cd.parse_xa(0).unwrap();
        cd.decode_xa_sector(0, info);

        // The decoded source sample for nibble 0x5, shift 4:
        let expected = (i32::from(((0x5u16) << 12) as i16) >> shift) as i16;
        // Unit 0 fills the first 28 source samples with `expected` (the other 7
        // units read zero nibbles). The resampler seeds continuity from 0, so
        // frame 0 is the seed; steady-state frames inside unit 0's run equal the
        // constant source sample.
        let out = cd.take_cd_audio();
        assert!(out.len() > 5, "produced frames");
        assert_eq!(out[5].0, expected, "4-bit unit-0 sample (filter 0)");
        // Mono duplicates to both channels (unity matrix).
        assert_eq!(out[5].1, expected, "mono duplicated to right");

        // ---- 8-bit mono ----
        // Unit 0 (8-bit) reads full byte data[16 + 0 + s*4]; param header[4].
        let mut d8 = vec![0u8; SECTOR_RAW];
        set_xa_subheader(&mut d8, 0, 0, 0, 0x24, 0x10); // audio+form2, mono/37800/8bit
        d8[audio + 4] = shift as u8; // param unit 0
        let byte_val: u8 = 0x40;
        for s in 0..28 {
            d8[audio + 16 + s * 4] = byte_val;
        }
        let mut cd8 = Cdrom::new();
        cd8.insert_disc(Disc::from_bytes(d8));
        cd8.reset_audio_decoders();
        let info8 = cd8.parse_xa(0).unwrap();
        cd8.decode_xa_sector(0, info8);
        let expected8 = ((i32::from(byte_val as i8) << 8) >> shift) as i16;
        let out8 = cd8.take_cd_audio();
        assert!(out8.len() > 5);
        assert_eq!(out8[5].0, expected8, "8-bit unit-0 sample (filter 0)");
    }

    #[test]
    fn xa_filter_selects_channel() {
        // Setmode bit6 (XA filter) + Setfilter file=1, channel=2. A matching XA
        // sector decodes; a non-matching one is skipped (no audio, no data).
        let mut data = vec![0u8; 2 * SECTOR_RAW];
        // Sector 0: matching file/channel, 4-bit mono audio.
        set_xa_subheader(&mut data, 0, 1, 2, 0x24, 0x00);
        data[24 + 4] = 0x04; // param unit 0: filter 0, shift 4
        for s in 0..28 {
            data[24 + 16 + s * 4] = 0x05;
        }
        // Sector 1: non-matching channel.
        set_xa_subheader(&mut data, SECTOR_RAW, 1, 9, 0x24, 0x00);
        for s in 0..28 {
            data[SECTOR_RAW + 24 + 16 + s * 4] = 0x05;
        }

        let mut cd = Cdrom::new();
        cd.insert_disc(Disc::from_bytes(data));
        cd.mode = 0x40; // XA filter on
        cd.filter_file = 1;
        cd.filter_channel = 2;
        cd.reading = true;

        // Matching sector 0 → audio grows, no data delivered.
        cd.lba = 0;
        cd.process_read_sector();
        let after_match = cd.take_cd_audio();
        assert!(!after_match.is_empty(), "matching XA sector decodes audio");
        assert!(cd.sector_buffer.is_empty(), "no CPU data for XA sector");
        assert!(cd.pending.is_empty(), "no INT1 for XA sector");

        // Non-matching sector 1 → skipped: no audio, no data.
        cd.lba = 1;
        cd.process_read_sector();
        assert!(
            cd.take_cd_audio().is_empty(),
            "filtered-out XA sector produces no audio"
        );
        assert!(
            cd.pending.is_empty(),
            "filtered-out sector delivers no INT1"
        );
    }

    #[test]
    fn cdda_sector_delivers_pcm() {
        // Build a disc whose sector 0 is raw PCM (a constant per channel) on an
        // audio track, then Play and tick one play period.
        let sectors = 4;
        let mut data = vec![0u8; sectors * SECTOR_RAW];
        for i in 0..588 {
            let b = i * 4;
            data[b..b + 2].copy_from_slice(&1000i16.to_le_bytes());
            data[b + 2..b + 4].copy_from_slice(&(-2000i16).to_le_bytes());
        }
        let mut disc = Disc::from_bytes(data);
        disc.tracks = vec![DiscTrack {
            number: 1,
            start_lba: 0,
            audio: true,
        }];
        disc.lead_out_lba = sectors as u32;

        let mut cd = Cdrom::new();
        let mut irq = Irq::new();
        cd.insert_disc(disc);
        cd.mode = 0x01; // CDDA routing on
        // Play from LBA 0.
        cd.write8(0x1F80_1801, 0x03);
        assert!(cd.playing, "playing after Play");
        // Advance one full CD-DA period (single speed).
        cd.tick(READ_PERIOD_SINGLE as u32, &mut irq);

        let out = cd.take_cd_audio();
        assert_eq!(out.len(), 588, "588 frames per CD-DA sector");
        // Unity straight-through matrix (0x80): L=1000, R=-2000.
        assert_eq!(out[0], (1000, -2000), "PCM through unity matrix");
        assert_eq!(out[587], (1000, -2000));
    }

    #[test]
    fn cd_volume_matrix_applied() {
        // Swap-and-halve the matrix via the index-banked ports, then verify a
        // known CD-DA sector reflects it.
        let sectors = 2;
        let mut data = vec![0u8; sectors * SECTOR_RAW];
        for i in 0..588 {
            let b = i * 4;
            data[b..b + 2].copy_from_slice(&1000i16.to_le_bytes()); // L
            data[b + 2..b + 4].copy_from_slice(&500i16.to_le_bytes()); // R
        }
        let mut disc = Disc::from_bytes(data);
        disc.tracks = vec![DiscTrack {
            number: 1,
            start_lba: 0,
            audio: true,
        }];
        disc.lead_out_lba = sectors as u32;

        let mut cd = Cdrom::new();
        let mut irq = Irq::new();
        cd.insert_disc(disc);
        cd.mode = 0x01;

        // Program a cross-only matrix: LL=0, LR=0x80 (CD-L→SPU-R),
        // RL=0x80 (CD-R→SPU-L), RR=0. This swaps the channels at unity.
        cd.write8(0x1F80_1800, 0x02); // index 2
        cd.write8(0x1F80_1802, 0x00); // LL = 0
        cd.write8(0x1F80_1803, 0x80); // LR = 0x80
        cd.write8(0x1F80_1800, 0x03); // index 3
        cd.write8(0x1F80_1802, 0x80); // RL = 0x80
        cd.write8(0x1F80_1801, 0x00); // RR = 0
        cd.write8(0x1F80_1800, 0x00); // back to index 0

        cd.write8(0x1F80_1801, 0x03); // Play
        cd.tick(READ_PERIOD_SINGLE as u32, &mut irq);

        let out = cd.take_cd_audio();
        assert_eq!(out.len(), 588);
        // out_l = (cd_l*LL + cd_r*RL)>>7 = (0 + 500*128)>>7 = 500.
        // out_r = (cd_l*LR + cd_r*RR)>>7 = (1000*128 + 0)>>7 = 1000.
        assert_eq!(out[0], (500, 1000), "channels swapped by matrix");
    }

    #[test]
    fn mute_silences_cd_audio() {
        let sectors = 2;
        let mut data = vec![0u8; sectors * SECTOR_RAW];
        for i in 0..588 {
            let b = i * 4;
            data[b..b + 2].copy_from_slice(&1234i16.to_le_bytes());
            data[b + 2..b + 4].copy_from_slice(&5678i16.to_le_bytes());
        }
        let mut disc = Disc::from_bytes(data);
        disc.tracks = vec![DiscTrack {
            number: 1,
            start_lba: 0,
            audio: true,
        }];
        disc.lead_out_lba = sectors as u32;

        let mut cd = Cdrom::new();
        let mut irq = Irq::new();
        cd.insert_disc(disc);
        cd.mode = 0x01;
        cd.muted = true;
        cd.write8(0x1F80_1801, 0x03); // Play
        cd.tick(READ_PERIOD_SINGLE as u32, &mut irq);

        let out = cd.take_cd_audio();
        assert_eq!(out.len(), 588, "still emits frames while muted");
        assert!(out.iter().all(|&f| f == (0, 0)), "muted → silence");
    }

    #[test]
    fn serde_roundtrip() {
        let mut cd = Cdrom::new();
        cd.insert_disc(make_disc(2));
        cd.write8(0x1F80_1801, 0x01);
        let json = serde_json::to_string(&cd).unwrap();
        let back: Cdrom = serde_json::from_str(&json).unwrap();
        assert_eq!(cd, back);
    }
}
