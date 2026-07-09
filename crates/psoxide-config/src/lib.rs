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
}

impl Default for DesktopConfig {
    fn default() -> Self {
        Self {
            bios_path: String::new(),
            window_scale: default_scale(),
            audio_enabled: false,
        }
    }
}

fn default_scale() -> u32 {
    2
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
