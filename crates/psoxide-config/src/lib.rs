//! Configuration for psoxide.
//!
//! Loads settings from a TOML file (`psoxide.toml`) with sensible defaults.
//! The PlayStation requires a BIOS image, so [`DesktopConfig::bios_path`] is
//! the most important field.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

pub mod disc;

/// Top-level configuration.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PsxConfig {
    /// Desktop frontend settings.
    #[serde(default)]
    pub desktop: DesktopConfig,
    /// Keyboard bindings for runtime controls.
    #[serde(default)]
    pub keybindings: Keybindings,
    /// Named disc-image paths.
    #[serde(default)]
    pub discs: HashMap<String, String>,
    /// Named executable/ROM paths (alias of discs for side-loaded programs).
    #[serde(default)]
    pub roms: HashMap<String, String>,
}

/// Desktop frontend configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DesktopConfig {
    /// Path to the PlayStation BIOS image (512KB).
    #[serde(default)]
    pub bios_path: String,
    /// Window scale factor.
    #[serde(default = "default_scale")]
    pub window_scale: u32,
    /// Enable audio output (currently a silent stub).
    #[serde(default)]
    pub audio_enabled: bool,
    /// Start in fullscreen.
    #[serde(default)]
    pub fullscreen: bool,
    /// Last BIOS image path opened (remembered across runs).
    #[serde(default)]
    pub last_bios: String,
    /// Last disc image path mounted (remembered across runs).
    #[serde(default)]
    pub last_disc: String,
    /// Last memory-card image path used (remembered across runs).
    #[serde(default)]
    pub last_memcard: String,
}

impl Default for DesktopConfig {
    fn default() -> Self {
        Self {
            bios_path: String::new(),
            window_scale: default_scale(),
            audio_enabled: false,
            fullscreen: false,
            last_bios: String::new(),
            last_disc: String::new(),
            last_memcard: String::new(),
        }
    }
}

/// Keyboard bindings for runtime controls.
///
/// Each field holds a winit [`KeyCode`](https://docs.rs/winit) variant name
/// (e.g. `"KeyP"`, `"F5"`, `"Space"`, `"Equal"`). The desktop frontend parses
/// these strings and falls back to the built-in default when a name is
/// unrecognised, so a hand-edited config can never leave a control unbound.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Keybindings {
    /// Toggle pause/resume.
    #[serde(default = "default_pause")]
    pub pause: String,
    /// Advance one frame while paused.
    #[serde(default = "default_frame_step")]
    pub frame_step: String,
    /// Hold to fast-forward (uncap the frame pacer).
    #[serde(default = "default_fast_forward")]
    pub fast_forward: String,
    /// Reset the machine to the BIOS entry vector.
    #[serde(default = "default_reset")]
    pub reset: String,
    /// Toggle fullscreen.
    #[serde(default = "default_fullscreen")]
    pub fullscreen: String,
    /// Increase the window scale.
    #[serde(default = "default_scale_up")]
    pub scale_up: String,
    /// Decrease the window scale.
    #[serde(default = "default_scale_down")]
    pub scale_down: String,
    /// Save the current state to the active slot.
    #[serde(default = "default_save_state")]
    pub save_state: String,
    /// Load the active slot into the machine.
    #[serde(default = "default_load_state")]
    pub load_state: String,
}

impl Default for Keybindings {
    fn default() -> Self {
        Self {
            pause: default_pause(),
            frame_step: default_frame_step(),
            fast_forward: default_fast_forward(),
            reset: default_reset(),
            fullscreen: default_fullscreen(),
            scale_up: default_scale_up(),
            scale_down: default_scale_down(),
            save_state: default_save_state(),
            load_state: default_load_state(),
        }
    }
}

fn default_scale() -> u32 {
    2
}

fn default_pause() -> String {
    "KeyP".into()
}
fn default_frame_step() -> String {
    "KeyF".into()
}
fn default_fast_forward() -> String {
    "Space".into()
}
fn default_reset() -> String {
    "KeyR".into()
}
fn default_fullscreen() -> String {
    "F11".into()
}
fn default_scale_up() -> String {
    "Equal".into()
}
fn default_scale_down() -> String {
    "Minus".into()
}
fn default_save_state() -> String {
    "F5".into()
}
fn default_load_state() -> String {
    "F9".into()
}

impl PsxConfig {
    /// Loads config from a file, falling back to defaults if it is missing.
    ///
    /// # Errors
    ///
    /// Returns an error if the file exists but cannot be parsed.
    pub fn load(path: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        if path.exists() {
            let content = std::fs::read_to_string(path)?;
            let config: PsxConfig = toml::from_str(&content)?;
            Ok(config)
        } else {
            Ok(Self::default())
        }
    }

    /// Loads from the default path (`psoxide.toml`), or returns defaults.
    #[must_use]
    pub fn load_or_default() -> Self {
        Self::load(Path::new("psoxide.toml")).unwrap_or_default()
    }

    /// Serialises the config back to a TOML file.
    ///
    /// Used by the desktop frontend to persist last-used paths and the window
    /// scale on exit so the next launch remembers them.
    ///
    /// # Errors
    ///
    /// Returns an error if the config cannot be serialised or the file cannot
    /// be written.
    pub fn save(&self, path: &Path) -> Result<(), Box<dyn std::error::Error>> {
        let content = toml::to_string_pretty(self)?;
        std::fs::write(path, content)?;
        Ok(())
    }

    /// Resolves a disc path from a config name or a direct path.
    #[must_use]
    pub fn resolve_disc(&self, name_or_path: &str) -> PathBuf {
        self.discs
            .get(name_or_path)
            .map_or_else(|| PathBuf::from(name_or_path), PathBuf::from)
    }

    /// Resolves an executable/ROM path from a config name or a direct path.
    #[must_use]
    pub fn resolve_rom(&self, name_or_path: &str) -> PathBuf {
        self.roms
            .get(name_or_path)
            .map_or_else(|| PathBuf::from(name_or_path), PathBuf::from)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config() {
        let config = PsxConfig::default();
        assert_eq!(config.desktop.window_scale, 2);
        assert!(!config.desktop.audio_enabled);
        assert!(config.desktop.bios_path.is_empty());
    }

    #[test]
    fn parse_toml() {
        let toml = r#"
            [desktop]
            bios_path = "scph1001.bin"
            window_scale = 3
            audio_enabled = true

            [discs]
            crash = "/games/crash.cue"

            [roms]
            hello = "/exe/hello.exe"
        "#;
        let config: PsxConfig = toml::from_str(toml).unwrap();
        assert_eq!(config.desktop.bios_path, "scph1001.bin");
        assert_eq!(config.desktop.window_scale, 3);
        assert!(config.desktop.audio_enabled);
        assert_eq!(config.discs["crash"], "/games/crash.cue");
        assert_eq!(config.roms["hello"], "/exe/hello.exe");
    }

    #[test]
    fn default_keybindings() {
        let kb = Keybindings::default();
        assert_eq!(kb.pause, "KeyP");
        assert_eq!(kb.save_state, "F5");
        assert_eq!(kb.load_state, "F9");
        assert_eq!(kb.fast_forward, "Space");
    }

    #[test]
    fn save_load_round_trip() {
        let mut config = PsxConfig::default();
        config.desktop.window_scale = 4;
        config.desktop.fullscreen = true;
        config.desktop.last_disc = "/games/crash.cue".into();
        config.keybindings.pause = "KeyM".into();

        let dir = std::env::temp_dir();
        let path = dir.join(format!("psoxide-cfg-test-{}.toml", std::process::id()));
        config.save(&path).unwrap();
        let loaded = PsxConfig::load(&path).unwrap();
        let _ = std::fs::remove_file(&path);

        assert_eq!(loaded.desktop.window_scale, 4);
        assert!(loaded.desktop.fullscreen);
        assert_eq!(loaded.desktop.last_disc, "/games/crash.cue");
        assert_eq!(loaded.keybindings.pause, "KeyM");
    }

    #[test]
    fn partial_toml_fills_defaults() {
        // A config predating the keybindings/last-path fields must still load,
        // with the new fields filled from their defaults.
        let toml = r#"
            [desktop]
            window_scale = 3
        "#;
        let config: PsxConfig = toml::from_str(toml).unwrap();
        assert_eq!(config.desktop.window_scale, 3);
        assert!(!config.desktop.fullscreen);
        assert_eq!(config.keybindings.reset, "KeyR");
        assert_eq!(config.desktop.last_bios, "");
    }

    #[test]
    fn resolve_named_and_direct() {
        let mut config = PsxConfig::default();
        config
            .discs
            .insert("crash".into(), "/games/crash.cue".into());
        assert_eq!(
            config.resolve_disc("crash"),
            PathBuf::from("/games/crash.cue")
        );
        assert_eq!(config.resolve_disc("other.cue"), PathBuf::from("other.cue"));
    }
}
