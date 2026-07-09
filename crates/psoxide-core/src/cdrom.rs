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
        }
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
            0x1F80_1801 => {
                if self.index == 0 {
                    self.run_command(val);
                }
                // index 1/2/3: audio / sound-map registers — accept & ignore.
            }
            0x1F80_1802 => match self.index {
                0 => {
                    if self.params.len() < FIFO_CAP {
                        self.params.push_back(val);
                    }
                }
                1 => self.ie = val & 0x1F,
                // index 2/3: audio volume — accept & ignore.
                _ => {}
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
                // index 2/3: audio apply — accept & ignore.
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
            // Play.
            0x03 => {
                self.playing = true;
                self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]);
            }
            // ReadN / ReadS.
            0x06 | 0x1B => {
                self.lba = self.seek_target;
                self.reading = true;
                self.playing = false;
                self.read_timer = self.sector_period();
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
            0x0B | 0x0C => self.push_resp(FIRST_RESP_DELAY, 3, vec![self.stat()]),
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

    // ---- per-cycle tick --------------------------------------------------

    /// Advances the controller by `cycles` CPU cycles, delivering queued
    /// responses and raising [`IrqLine::CdRom`] when an enabled interrupt
    /// latches.
    pub fn tick(&mut self, cycles: u32, irq: &mut Irq) {
        for _ in 0..cycles {
            self.tick_one(irq);
        }
    }

    fn tick_one(&mut self, irq: &mut Irq) {
        // While reading, generate a per-sector INT1 when the read timer expires.
        if self.reading {
            if self.read_timer > 0 {
                self.read_timer -= 1;
            }
            if self.read_timer <= 0 {
                self.read_current_sector();
                let stat = self.stat();
                self.push_resp(0, 1, vec![stat]);
                self.read_timer = self.sector_period();
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
