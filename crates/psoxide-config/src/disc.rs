//! CUE/BIN disc-image parsing for the frontend and test harness.
//!
//! `psoxide-core` is deliberately I/O-free: it accepts a fully-built
//! [`Disc`] through `Command::LoadDisc`. This module is the thin file-reading
//! layer that turns a `.cue` sheet (plus the `.bin` track files it references)
//! — or a bare `.bin` image — into that [`Disc`]. It is `std`-only, with no
//! external dependencies, and returns a [`DiscError`] rather than panicking on
//! malformed input.
//!
//! ## LBA / MSF convention
//!
//! A track's `INDEX 01 mm:ss:ff` time is converted to a logical block address
//! with `(mm * 60 + ss) * 75 + ff`, added to the sector offset of the track's
//! `FILE` within the concatenated image. No 2-second pregap is subtracted here:
//! `DiscTrack::start_lba` indexes directly into the raw sector image the same
//! way `cdrom.rs` maps a head position (`lba * SECTOR_RAW`). The controller's
//! Setloc applies the pregap when converting an *absolute* MSF address to an
//! LBA, so a data disc read at absolute `00:02:00` lands on `start_lba` 0.

use std::fmt;
use std::path::{Path, PathBuf};

use psoxide_core::{Disc, DiscTrack};

/// Raw bytes per CD sector (2352 = full Mode-2 raw frame).
pub const SECTOR_RAW: usize = 2352;

/// An error loading or parsing a disc image.
#[derive(Debug)]
pub enum DiscError {
    /// A referenced file could not be read.
    Io {
        /// The path that failed.
        path: PathBuf,
        /// The underlying I/O error.
        source: std::io::Error,
    },
    /// The CUE sheet (or BIN sizing) was malformed.
    Malformed(String),
}

impl fmt::Display for DiscError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, source } => {
                write!(f, "failed to read {}: {source}", path.display())
            }
            Self::Malformed(msg) => write!(f, "malformed cue/bin: {msg}"),
        }
    }
}

impl std::error::Error for DiscError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            Self::Malformed(_) => None,
        }
    }
}

/// Reads a file, wrapping any I/O error with its path.
fn read_file(path: &Path) -> Result<Vec<u8>, DiscError> {
    std::fs::read(path).map_err(|source| DiscError::Io {
        path: path.to_path_buf(),
        source,
    })
}

/// Loads a disc image from `path`.
///
/// A `.cue` extension parses the sheet (and its referenced BIN files); any
/// other extension is treated as a raw MODE2/2352 BIN with a single data
/// track.
///
/// # Errors
///
/// Returns [`DiscError`] if a file cannot be read or the CUE sheet is
/// malformed.
pub fn load_disc(path: &Path) -> Result<Disc, DiscError> {
    let is_cue = path
        .extension()
        .is_some_and(|e| e.eq_ignore_ascii_case("cue"));
    if is_cue {
        parse_cue(path)
    } else {
        Ok(disc_from_bin(read_file(path)?))
    }
}

/// Builds a single-data-track disc from a raw MODE2/2352 BIN image.
///
/// The table of contents is left implicit (track 1, data, LBA 0) — the
/// controller synthesizes it on insertion.
#[must_use]
pub fn disc_from_bin(data: Vec<u8>) -> Disc {
    Disc::from_bytes(data)
}

/// Parses a CUE sheet at `cue_path`, loading every `FILE` it references
/// (resolved relative to the sheet's directory) into one concatenated raw
/// image and building the track table from the `TRACK`/`INDEX 01` entries.
///
/// Recognized directives: `FILE "<name>" BINARY`, `TRACK nn MODE1/2352 |
/// MODE2/2352 | AUDIO`, and `INDEX 01 mm:ss:ff`. `INDEX 00` (pregap) and other
/// directives (`REM`, `CATALOG`, `PREGAP`, `FLAGS`, `POSTGAP`, …) are ignored.
///
/// # Errors
///
/// Returns [`DiscError`] if a BIN file cannot be read, a BIN length is not a
/// multiple of [`SECTOR_RAW`], or the sheet is structurally invalid (missing
/// `FILE`, an `INDEX` before its `TRACK`, a bad MSF, or no tracks).
pub fn parse_cue(cue_path: &Path) -> Result<Disc, DiscError> {
    let text = String::from_utf8_lossy(&read_file(cue_path)?).into_owned();
    let dir = cue_path.parent().unwrap_or_else(|| Path::new("."));

    let mut data: Vec<u8> = Vec::new();
    let mut tracks: Vec<DiscTrack> = Vec::new();
    // Sector offset of the current FILE within the concatenated image.
    let mut file_start_lba: u32 = 0;
    // The most recent `TRACK` line, awaiting its `INDEX 01`.
    let mut cur_track: Option<(u8, bool)> = None;
    let mut have_file = false;

    for (i, raw) in text.lines().enumerate() {
        let lineno = i + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        let mut it = line.split_whitespace();
        let keyword = it.next().unwrap_or("").to_ascii_uppercase();
        match keyword.as_str() {
            "FILE" => {
                let name = quoted_filename(line).ok_or_else(|| {
                    DiscError::Malformed(format!("line {lineno}: FILE missing quoted filename"))
                })?;
                let bin_path = dir.join(&name);
                let bytes = read_file(&bin_path)?;
                if bytes.is_empty() || bytes.len() % SECTOR_RAW != 0 {
                    return Err(DiscError::Malformed(format!(
                        "line {lineno}: BIN {} length {} is not a nonzero multiple of {SECTOR_RAW}",
                        bin_path.display(),
                        bytes.len()
                    )));
                }
                file_start_lba = (data.len() / SECTOR_RAW) as u32;
                data.extend_from_slice(&bytes);
                have_file = true;
            }
            "TRACK" => {
                let number = it
                    .next()
                    .and_then(|s| s.parse::<u8>().ok())
                    .ok_or_else(|| {
                        DiscError::Malformed(format!("line {lineno}: TRACK missing number"))
                    })?;
                let mode = it.next().ok_or_else(|| {
                    DiscError::Malformed(format!("line {lineno}: TRACK missing mode"))
                })?;
                let audio = mode.eq_ignore_ascii_case("AUDIO");
                cur_track = Some((number, audio));
            }
            "INDEX" => {
                let index = it
                    .next()
                    .and_then(|s| s.parse::<u8>().ok())
                    .ok_or_else(|| {
                        DiscError::Malformed(format!("line {lineno}: INDEX missing number"))
                    })?;
                let msf = it.next().ok_or_else(|| {
                    DiscError::Malformed(format!("line {lineno}: INDEX missing MSF"))
                })?;
                // Only INDEX 01 marks the track's start; INDEX 00 is pregap.
                if index != 1 {
                    continue;
                }
                let msf_lba = parse_msf(msf).ok_or_else(|| {
                    DiscError::Malformed(format!("line {lineno}: bad MSF {msf:?}"))
                })?;
                let (number, audio) = cur_track.ok_or_else(|| {
                    DiscError::Malformed(format!("line {lineno}: INDEX before TRACK"))
                })?;
                tracks.push(DiscTrack {
                    number,
                    start_lba: file_start_lba + msf_lba,
                    audio,
                });
            }
            // REM, CATALOG, PERFORMER, TITLE, PREGAP, POSTGAP, FLAGS, ISRC, …
            _ => {}
        }
    }

    if !have_file {
        return Err(DiscError::Malformed("no FILE directive".into()));
    }
    if tracks.is_empty() {
        return Err(DiscError::Malformed("no TRACK/INDEX 01 entries".into()));
    }
    tracks.sort_by_key(|t| t.number);
    let lead_out_lba = (data.len() / SECTOR_RAW) as u32;
    Ok(Disc {
        data,
        tracks,
        lead_out_lba,
    })
}

/// Extracts the filename from a `FILE "name" BINARY` line, honoring a quoted
/// name (which may contain spaces) or falling back to the second token.
fn quoted_filename(line: &str) -> Option<String> {
    if let Some(open) = line.find('"') {
        let rest = &line[open + 1..];
        let close = rest.find('"')?;
        return Some(rest[..close].to_string());
    }
    // Unquoted: FILE <name> BINARY
    line.split_whitespace().nth(1).map(str::to_string)
}

/// Parses an `mm:ss:ff` MSF address into a logical block count
/// `(mm * 60 + ss) * 75 + ff`. Frames must be < 75 and seconds < 60.
fn parse_msf(s: &str) -> Option<u32> {
    let mut parts = s.split(':');
    let mm: u32 = parts.next()?.parse().ok()?;
    let ss: u32 = parts.next()?.parse().ok()?;
    let ff: u32 = parts.next()?.parse().ok()?;
    if parts.next().is_some() || ss >= 60 || ff >= 75 {
        return None;
    }
    Some((mm * 60 + ss) * 75 + ff)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn msf_conversion() {
        assert_eq!(parse_msf("00:00:00"), Some(0));
        assert_eq!(parse_msf("00:02:00"), Some(150));
        assert_eq!(parse_msf("00:02:02"), Some(152));
        assert_eq!(parse_msf("01:00:00"), Some(60 * 75));
        assert_eq!(parse_msf("00:00:75"), None, "frame >= 75 rejected");
        assert_eq!(parse_msf("00:60:00"), None, "second >= 60 rejected");
        assert_eq!(parse_msf("1:2"), None, "too few fields rejected");
    }

    #[test]
    fn filename_quoting() {
        assert_eq!(
            quoted_filename("FILE \"game.bin\" BINARY").as_deref(),
            Some("game.bin")
        );
        assert_eq!(
            quoted_filename("FILE \"my game.bin\" BINARY").as_deref(),
            Some("my game.bin")
        );
        assert_eq!(
            quoted_filename("FILE track.bin BINARY").as_deref(),
            Some("track.bin")
        );
    }
}
