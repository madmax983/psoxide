//! End-to-end CD-ROM integration test.
//!
//! Unlike the in-module unit tests in `psoxide-core/src/cdrom.rs` (which drive
//! the controller in isolation), this exercises the *assembled* core: the real
//! system bus routing to `0x1F80_1800..=0x1F80_1803`, the per-cycle
//! `Cdrom::tick` in the step loop, the CD interrupt landing on `I_STAT` bit 2,
//! and DMA channel 3 pulling a sector out of the data FIFO into main RAM — all
//! from a disc mounted through the CUE/BIN parser in [`psoxide_test_harness::disc`].
//!
//! No real BIOS is needed. The CPU runs a tiny self-looping program in RAM so
//! that each `StepCpu` advances (and ticks) the CD-ROM controller without the
//! program counter wandering into unmapped memory; interrupts stay masked
//! (`I_MASK` = 0), so the delivered CD interrupt only *sets* `I_STAT` bit 2 and
//! never diverts the looping CPU.

use std::path::PathBuf;

use psoxide_core::IrqLine;
use psoxide_test_harness::Harness;
use psoxide_test_harness::disc::{SECTOR_RAW, parse_cue};

// ---- CD-ROM register map (physical) --------------------------------------

const CD_STATUS: u32 = 0x1F80_1800; // write: index; read: status byte
const CD_1801: u32 = 0x1F80_1801; // write(idx0): command; read: response FIFO
const CD_1802: u32 = 0x1F80_1802; // write(idx0): param, (idx1): IE; read: data FIFO
const CD_1803: u32 = 0x1F80_1803; // write(idx0): request, (idx1): flag ack; read: IE/flag

// ---- DMA channel 3 (CD-ROM) register map ---------------------------------

const DMA3_MADR: u32 = 0x1F80_1080 + 3 * 0x10;
const DMA3_BCR: u32 = DMA3_MADR + 0x4;
const DMA3_CHCR: u32 = DMA3_MADR + 0x8;
const DMA_DICR: u32 = 0x1F80_10F4;

const I_STAT: u32 = 0x1F80_1070;

/// A synthetic sector's recognizable 4-byte magic (the 4th byte is the sector
/// number so each sector is uniquely tagged).
fn magic(k: u8) -> [u8; 4] {
    [0xAA, 0x55, 0xCD, k]
}

/// Writes a tiny `n`-sector MODE2/2352 BIN plus a matching single-track CUE
/// into `dir`, returning `(cue_path, bin_path)`.
///
/// Sector `k`'s 2048-byte user area (raw offset 24) begins with [`magic(k)`]
/// and is otherwise filled with byte `k`; the whole 2340-byte payload is filled
/// with `k` so whole-sector delivery also carries the pattern. Bytes 0..12 hold
/// a valid sync pattern and 12..16 a plausible BCD MSF + mode-2 header.
fn write_synthetic_disc(dir: &std::path::Path, n_sectors: usize) -> (PathBuf, PathBuf) {
    let mut data = vec![0u8; n_sectors * SECTOR_RAW];
    for k in 0..n_sectors {
        let base = k * SECTOR_RAW;
        // Sync: 00 FF*10 00.
        data[base] = 0x00;
        for b in &mut data[base + 1..base + 11] {
            *b = 0xFF;
        }
        data[base + 11] = 0x00;
        // Header: MSF (BCD) + mode 2. LBA k -> absolute MSF (k + 150 pregap).
        let total = k as u32 + 150;
        let mm = (total / (60 * 75)) as u8;
        let ss = ((total % (60 * 75)) / 75) as u8;
        let ff = (total % 75) as u8;
        let bcd = |v: u8| ((v / 10) << 4) | (v % 10);
        data[base + 12] = bcd(mm);
        data[base + 13] = bcd(ss);
        data[base + 14] = bcd(ff);
        data[base + 15] = 0x02;
        // Subheader (file, channel, submode=data, coding) — plausible, twice.
        data[base + 16] = 0x00;
        data[base + 17] = 0x00;
        data[base + 18] = 0x08;
        data[base + 19] = 0x00;
        data[base + 20..base + 24].copy_from_slice(&[0x00, 0x00, 0x08, 0x00]);
        // Payload: fill 24..2352 with k, then stamp the magic at the start.
        for b in &mut data[base + 24..base + SECTOR_RAW] {
            *b = k as u8;
        }
        data[base + 24..base + 28].copy_from_slice(&magic(k as u8));
    }

    let bin_path = dir.join("disc.bin");
    let cue_path = dir.join("disc.cue");
    std::fs::write(&bin_path, &data).expect("write bin");
    std::fs::write(
        &cue_path,
        "FILE \"disc.bin\" BINARY\r\n  TRACK 01 MODE2/2352\r\n    INDEX 01 00:00:00\r\n",
    )
    .expect("write cue");
    (cue_path, bin_path)
}

/// Writes a two-track (data + audio) synthetic disc, exercising AUDIO parsing
/// and multi-track INDEX offsets. Track 1 = `data_sectors` data sectors, track
/// 2 = 2 audio sectors starting immediately after.
fn write_two_track_disc(dir: &std::path::Path, data_sectors: usize) -> PathBuf {
    let audio_sectors = 2;
    let total = data_sectors + audio_sectors;
    let data = vec![0u8; total * SECTOR_RAW];
    let bin_path = dir.join("multi.bin");
    let cue_path = dir.join("multi.cue");
    std::fs::write(&bin_path, &data).expect("write bin");
    // INDEX 01 of track 2 is at the data-track's end (MSF of `data_sectors`).
    let mm = data_sectors as u32 / (60 * 75);
    let rem = data_sectors as u32 % (60 * 75);
    let ss = rem / 75;
    let ff = rem % 75;
    let cue = format!(
        "FILE \"multi.bin\" BINARY\n  TRACK 01 MODE2/2352\n    INDEX 01 00:00:00\n  \
         TRACK 02 AUDIO\n    INDEX 01 {mm:02}:{ss:02}:{ff:02}\n"
    );
    std::fs::write(&cue_path, cue).expect("write cue");
    cue_path
}

// ---- register-level drivers ----------------------------------------------

/// Puts the CPU in a tight self-loop in RAM so each `StepCpu` ticks the CD-ROM
/// controller without the PC leaving mapped memory.
fn spin_cpu(h: &mut Harness) {
    // 0x0000: j 0x0000 ; 0x0004: nop (delay slot) — loops forever at PC 0.
    h.load_program(&[0x0800_0000, 0x0000_0000]);
}

fn set_index(h: &mut Harness, index: u8) {
    h.core_mut().store8(CD_STATUS, index);
}

fn write_param(h: &mut Harness, val: u8) {
    set_index(h, 0);
    h.core_mut().store8(CD_1802, val);
}

fn send_command(h: &mut Harness, cmd: u8) {
    set_index(h, 0);
    h.core_mut().store8(CD_1801, cmd);
}

fn enable_cd_ints(h: &mut Harness) {
    set_index(h, 1);
    h.core_mut().store8(CD_1802, 0x1F); // IE = all five INT sources
}

/// Reads the current interrupt-flag number (INTn) from the flag register.
fn read_int_flag(h: &mut Harness) -> u8 {
    set_index(h, 1);
    h.core_mut().load8(CD_1803) & 0x07
}

/// Pops one response-FIFO byte (index-independent port).
fn read_response(h: &mut Harness) -> u8 {
    h.core_mut().load8(CD_1801)
}

/// Acknowledges the current CD interrupt (clears the flag) and clears
/// `I_STAT` bit 2 so the next interrupt is unambiguous.
fn ack_int(h: &mut Harness) {
    set_index(h, 1);
    h.core_mut().store8(CD_1803, 0x07);
    h.core_mut()
        .store32(I_STAT, !(1u32 << IrqLine::CdRom.bit()));
}

/// Steps the core until a CD interrupt latches (flag != 0), returning the INTn.
/// Panics with a clear message if none fires within `max_steps`.
fn tick_until_int(h: &mut Harness, max_steps: usize) -> u8 {
    for _ in 0..max_steps {
        h.run(1); // StepCpu ticks Cdrom by one cycle
        let flag = read_int_flag(h);
        if flag != 0 {
            return flag;
        }
    }
    panic!("CD interrupt did not fire within {max_steps} steps");
}

fn i_stat_cd_set(h: &mut Harness) -> bool {
    h.core_mut().load32(I_STAT) & (1u32 << IrqLine::CdRom.bit()) != 0
}

// Bounded step budgets. FIRST/SECOND response delays are 50_000 cycles; a
// single sector read is ~451_584 cycles at 1x, so the read path needs the
// larger budget. Double-speed mode (bit7) halves that.
const ACK_BUDGET: usize = 200_000;
const READ_BUDGET: usize = 1_000_000;

#[test]
fn cue_parser_single_track() {
    let dir = std::env::temp_dir().join(format!("psoxide-cd-cue-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (cue, _bin) = write_synthetic_disc(&dir, 8);

    let disc = parse_cue(&cue).expect("parse cue");
    assert_eq!(disc.sector_count(), 8);
    assert_eq!(disc.lead_out_lba, 8);
    assert_eq!(disc.tracks.len(), 1);
    assert_eq!(disc.tracks[0].number, 1);
    assert_eq!(disc.tracks[0].start_lba, 0);
    assert!(!disc.tracks[0].audio, "track 1 is data");

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn cue_parser_multi_track_with_audio() {
    let dir = std::env::temp_dir().join(format!("psoxide-cd-multi-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let cue = write_two_track_disc(&dir, 10);

    let disc = parse_cue(&cue).expect("parse cue");
    assert_eq!(disc.tracks.len(), 2);
    assert_eq!(disc.tracks[0].number, 1);
    assert!(!disc.tracks[0].audio, "track 1 data");
    assert_eq!(disc.tracks[0].start_lba, 0);
    assert_eq!(disc.tracks[1].number, 2);
    assert!(disc.tracks[1].audio, "track 2 audio");
    assert_eq!(disc.tracks[1].start_lba, 10, "audio track after 10 sectors");

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn cue_parser_rejects_missing_bin() {
    let dir = std::env::temp_dir().join(format!("psoxide-cd-bad-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let cue = dir.join("broken.cue");
    std::fs::write(&cue, "FILE \"nope.bin\" BINARY\n  TRACK 01 MODE2/2352\n").unwrap();
    let err = parse_cue(&cue).expect_err("missing bin must error, not panic");
    // It should be an I/O error naming the missing file.
    assert!(matches!(
        err,
        psoxide_test_harness::disc::DiscError::Io { .. }
    ));
    std::fs::remove_dir_all(&dir).ok();
}

/// The high-value end-to-end path: mount a synthetic disc through the CUE
/// parser, then drive Setmode/Setloc/ReadN through the real bus, deliver the
/// sector via the data FIFO *and* via DMA channel 3, and validate the interrupt
/// controller and the delivered bytes at every step.
#[test]
fn read_sector_through_data_fifo_and_dma() {
    let dir = std::env::temp_dir().join(format!("psoxide-cd-read-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (cue, _bin) = write_synthetic_disc(&dir, 8);
    let disc = parse_cue(&cue).expect("parse cue");

    let mut h = Harness::new();
    spin_cpu(&mut h);
    h.load_disc(disc);
    enable_cd_ints(&mut h);

    // --- Setmode: double speed (bit7), 2048-byte data sectors. ------------
    write_param(&mut h, 0x80);
    send_command(&mut h, 0x0E); // Setmode
    assert_eq!(tick_until_int(&mut h, ACK_BUDGET), 3, "Setmode acks INT3");
    ack_int(&mut h);

    // --- Setloc to sector 2 (absolute MSF of LBA 2 = 00:02:02 BCD). -------
    write_param(&mut h, 0x00); // mm
    write_param(&mut h, 0x02); // ss
    write_param(&mut h, 0x02); // ff
    send_command(&mut h, 0x02); // Setloc
    assert_eq!(tick_until_int(&mut h, ACK_BUDGET), 3, "Setloc acks INT3");
    ack_int(&mut h);

    // --- ReadN: INT3 ack, then the per-sector INT1 with data ready. -------
    send_command(&mut h, 0x06); // ReadN
    assert_eq!(tick_until_int(&mut h, ACK_BUDGET), 3, "ReadN acks INT3");
    ack_int(&mut h);
    assert_eq!(
        tick_until_int(&mut h, READ_BUDGET),
        1,
        "first sector delivers INT1"
    );
    assert!(
        i_stat_cd_set(&mut h),
        "the CD interrupt reached I_STAT bit 2 in the IRQ controller"
    );

    // --- Read the 2048 user bytes back via the data FIFO (BFRD). ----------
    set_index(&mut h, 0);
    h.core_mut().store8(CD_1803, 0x80); // Request: BFRD loads the data FIFO
    assert_ne!(
        h.core_mut().load8(CD_STATUS) & 0x40,
        0,
        "DRQSTS set: data FIFO has the sector"
    );
    let mut fifo = Vec::with_capacity(2048);
    for _ in 0..2048 {
        fifo.push(h.core_mut().load8(CD_1802));
    }
    assert_eq!(&fifo[0..4], &magic(2), "sector 2 magic through data FIFO");
    assert!(
        fifo[4..].iter().all(|&b| b == 2),
        "sector 2 payload byte == 2"
    );
    ack_int(&mut h);

    // --- Deliver the same sector via DMA channel 3 into RAM. --------------
    // Re-seek to sector 2 and re-read so a fresh sector sits in the buffer.
    write_param(&mut h, 0x00);
    write_param(&mut h, 0x02);
    write_param(&mut h, 0x02);
    send_command(&mut h, 0x02); // Setloc
    assert_eq!(tick_until_int(&mut h, ACK_BUDGET), 3);
    ack_int(&mut h);
    send_command(&mut h, 0x06); // ReadN
    assert_eq!(tick_until_int(&mut h, ACK_BUDGET), 3);
    ack_int(&mut h);
    assert_eq!(
        tick_until_int(&mut h, READ_BUDGET),
        1,
        "sector 2 INT1 again"
    );

    // Load the data FIFO, then program DMA ch3 (device->RAM, block sync).
    set_index(&mut h, 0);
    h.core_mut().store8(CD_1803, 0x80); // BFRD

    const DMA_TARGET: u32 = 0x0001_0000; // in main RAM
    h.core_mut().store32(DMA_DICR, (1 << 23) | (1 << (16 + 3))); // master + ch3 IRQ enable
    h.core_mut().store32(DMA3_MADR, DMA_TARGET);
    h.core_mut().store32(DMA3_BCR, 512); // 512 words = 2048 bytes, 1 block
    // CHCR: enable (bit24), sync mode 1 (block, bit9); direction device->RAM.
    h.core_mut().store32(DMA3_CHCR, (1 << 24) | (1 << 9));

    // The block transfer runs synchronously on the CHCR write. The first RAM
    // word is magic(2) little-endian; the rest are 0x02020202.
    let w0 = h.read_word(DMA_TARGET);
    assert_eq!(
        w0.to_le_bytes(),
        magic(2),
        "DMA landed sector 2 magic in RAM"
    );
    assert_eq!(
        h.read_word(DMA_TARGET + 4),
        0x0202_0202,
        "DMA payload word == 0x02020202"
    );
    assert_eq!(
        h.read_word(DMA_TARGET + 2044),
        0x0202_0202,
        "last DMA word of the 2048-byte sector"
    );
    // DMA completion raised the DMA interrupt (I_STAT bit 3).
    assert!(
        h.core_mut().load32(I_STAT) & (1u32 << IrqLine::Dma.bit()) != 0,
        "DMA completion raised I_STAT bit 3"
    );

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn getid_and_gettn_on_mounted_disc() {
    let dir = std::env::temp_dir().join(format!("psoxide-cd-id-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (cue, _bin) = write_synthetic_disc(&dir, 4);
    let disc = parse_cue(&cue).expect("parse cue");

    let mut h = Harness::new();
    spin_cpu(&mut h);
    h.load_disc(disc);
    enable_cd_ints(&mut h);

    // GetTN: first=1, last=1 for the single-track TOC.
    send_command(&mut h, 0x13);
    assert_eq!(tick_until_int(&mut h, ACK_BUDGET), 3, "GetTN acks INT3");
    let _stat = read_response(&mut h);
    assert_eq!(read_response(&mut h), 0x01, "GetTN first track = 1 (BCD)");
    assert_eq!(read_response(&mut h), 0x01, "GetTN last track = 1 (BCD)");
    ack_int(&mut h);

    // GetID: INT3 ack, then INT2 with the SCEA licence string.
    send_command(&mut h, 0x1A);
    assert_eq!(tick_until_int(&mut h, ACK_BUDGET), 3, "GetID acks INT3");
    ack_int(&mut h);
    assert_eq!(
        tick_until_int(&mut h, ACK_BUDGET),
        2,
        "GetID second response is INT2"
    );
    let id: Vec<u8> = (0..8).map(|_| read_response(&mut h)).collect();
    assert_eq!(&id[4..8], b"SCEA", "GetID reports an SCEA data disc");

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn getid_no_disc_is_int5() {
    let mut h = Harness::new();
    spin_cpu(&mut h);
    // No disc mounted (and eject for good measure).
    h.eject_disc();
    enable_cd_ints(&mut h);

    send_command(&mut h, 0x1A); // GetID
    assert_eq!(tick_until_int(&mut h, ACK_BUDGET), 3, "GetID acks INT3");
    ack_int(&mut h);
    assert_eq!(
        tick_until_int(&mut h, ACK_BUDGET),
        5,
        "no-disc GetID reports the INT5 error"
    );
}
