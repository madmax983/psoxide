//! Save-state file handling for the desktop frontend.
//!
//! A save state is the core's [`CoreSnapshot`] serialised to JSON (the same
//! serde representation the core round-trips in its own tests). States are
//! written next to the loaded content so they travel with the game: the file
//! name is `<stem>.ss<slot>`, where `<stem>` is the disc image's file stem
//! when a disc is mounted, otherwise the side-loaded EXE's, otherwise the
//! BIOS image's. Slots are `1..=9`.

use std::path::{Path, PathBuf};

use psoxide_core::CoreSnapshot;

/// Lowest selectable save-state slot.
pub const MIN_SLOT: u8 = 1;
/// Highest selectable save-state slot.
pub const MAX_SLOT: u8 = 9;

/// Builds the save-state path for `slot` given the base content path.
///
/// The base is the most specific loaded artefact (disc, else EXE, else BIOS).
/// The state lives beside it as `<stem>.ss<slot>`.
#[must_use]
pub fn slot_path(base: &Path, slot: u8) -> PathBuf {
    let stem = base
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "psoxide".to_string());
    let file_name = format!("{stem}.ss{slot}");
    match base.parent() {
        Some(dir) if !dir.as_os_str().is_empty() => dir.join(file_name),
        _ => PathBuf::from(file_name),
    }
}

/// Serialises a snapshot to a save-state file.
///
/// # Errors
///
/// Returns an error if serialisation or the file write fails.
pub fn write_slot(base: &Path, slot: u8, snapshot: &CoreSnapshot) -> Result<PathBuf, String> {
    if !(MIN_SLOT..=MAX_SLOT).contains(&slot) {
        return Err(format!(
            "save-state slot {slot} out of range {MIN_SLOT}..={MAX_SLOT}"
        ));
    }
    let path = slot_path(base, slot);
    let json =
        serde_json::to_vec(snapshot).map_err(|e| format!("failed to serialise save state: {e}"))?;
    std::fs::write(&path, json)
        .map_err(|e| format!("failed to write save state {}: {e}", path.display()))?;
    Ok(path)
}

/// Reads and deserialises a snapshot from a save-state file.
///
/// # Errors
///
/// Returns an error if the slot file is missing, unreadable, or not a valid
/// snapshot.
pub fn read_slot(base: &Path, slot: u8) -> Result<CoreSnapshot, String> {
    if !(MIN_SLOT..=MAX_SLOT).contains(&slot) {
        return Err(format!(
            "save-state slot {slot} out of range {MIN_SLOT}..={MAX_SLOT}"
        ));
    }
    let path = slot_path(base, slot);
    let bytes = std::fs::read(&path)
        .map_err(|e| format!("failed to read save state {}: {e}", path.display()))?;
    serde_json::from_slice(&bytes)
        .map_err(|e| format!("save state {} is invalid: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use psoxide_core::{Command, PsxCore};

    #[test]
    fn slot_path_beside_base() {
        let p = slot_path(Path::new("/games/crash.cue"), 3);
        assert_eq!(p, PathBuf::from("/games/crash.ss3"));
    }

    #[test]
    fn slot_path_no_parent() {
        let p = slot_path(Path::new("scph1001.bin"), 1);
        assert_eq!(p, PathBuf::from("scph1001.ss1"));
    }

    #[test]
    fn round_trip_through_files() {
        // Drive a tiny bit of state into the core, snapshot it, write+read the
        // slot file through the desktop's own path, and confirm the restored
        // core matches — the file-path save-state round trip.
        let dir = std::env::temp_dir();
        let base = dir.join(format!("psoxide-ss-test-{}.bin", std::process::id()));

        let mut core = PsxCore::new();
        core.store32(0x0000_0100, 0xDEAD_BEEF);
        let _ = core.execute(Command::SetControllerState {
            port: 0,
            buttons: 0x00F0,
        });
        let snap = core.save_state();

        let path = write_slot(&base, 4, &snap).unwrap();
        assert!(path.exists());
        let restored = read_slot(&base, 4).unwrap();
        let _ = std::fs::remove_file(&path);

        let mut other = PsxCore::new();
        other.load_state(&restored);
        assert_eq!(other.load32(0x0000_0100), 0xDEAD_BEEF);
        assert_eq!(snap, restored);
    }

    #[test]
    fn read_missing_slot_errors() {
        let base = std::env::temp_dir().join("psoxide-ss-does-not-exist.bin");
        assert!(read_slot(&base, 7).is_err());
    }
}
