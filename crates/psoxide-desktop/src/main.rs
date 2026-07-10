//! Psoxide desktop frontend.
//!
//! CLI-first PlayStation emulator skeleton.
//!
//! ```text
//! psoxide run scph1001.bin --scale 2
//! psoxide info scph1001.bin
//! ```
//!
//! The GPU is rendered via `framebuffer_rgba()` into a Pixels surface. Input is
//! driven by the keyboard and, when present, a gamepad (via gilrs), both on
//! controller port 0: the keyboard drives a digital pad, and a connected
//! gamepad attaches a DualShock (analog) pad with both sticks and L2/R2/L3/R3.
//! See [`pad_button_to_psx`] for the full mapping table. Audio is produced by
//! the core's SPU and played back through the host device via `rodio` (see the
//! [`audio`] module); if no audio device is available the emulator continues
//! silently.

mod audio;
mod savestate;

use std::fs;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use gilrs::{Axis, Button as PadButton, EventType, Gilrs};
use pixels::{Pixels, SurfaceTexture};
use psoxide_config::PsxConfig;
use psoxide_core::api::FRAMES_PER_SECOND;
use psoxide_core::{
    Button, Command, ControllerKind, CoreQuery, FRAME_HEIGHT, FRAME_WIDTH, PsxCore, QueryResult,
};
use winit::application::ApplicationHandler;
use winit::dpi::LogicalSize;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Fullscreen, Window, WindowId};

use crate::audio::AudioOutput;

/// Minimum / maximum integer window scale reachable with the rescale hotkeys.
const MIN_SCALE: u32 = 1;
const MAX_SCALE: u32 = 8;

#[derive(Parser)]
#[command(name = "psoxide", about = "Sony PlayStation emulator")]
struct Cli {
    #[command(subcommand)]
    command: CliCommand,
}

#[derive(Subcommand)]
enum CliCommand {
    /// Run with a BIOS image (and optionally side-load an EXE).
    Run {
        /// Path to the BIOS image (or a config name).
        bios: String,
        /// Optional PSX-EXE to side-load (currently a stub).
        #[arg(long)]
        exe: Option<PathBuf>,
        /// Optional disc image to mount: a `.cue` sheet (parsed with its BIN
        /// tracks) or a raw MODE2/2352 `.bin` (single data track).
        #[arg(long)]
        disc: Option<PathBuf>,
        /// Optional memory-card image file for slot 0 (128 KB). Created fresh
        /// (all-zero) if the path does not exist; flushed back on write + exit.
        #[arg(long)]
        memcard: Option<PathBuf>,
        /// Window scale factor (overrides the config's `window_scale`).
        #[arg(long)]
        scale: Option<u32>,
        /// Start in fullscreen (overrides the config's `fullscreen`).
        #[arg(long)]
        fullscreen: bool,
        /// Attach a Multitap (4-player adapter) on port 0. Player 1 (keyboard /
        /// gamepad) drives sub-slot A; sub-slots B/C/D can be driven by the core
        /// API (extra-gamepad mapping is a follow-up).
        #[arg(long)]
        multitap: bool,
        /// Config file path.
        #[arg(long, default_value = "psoxide.toml")]
        config: PathBuf,
    },
    /// Print information about a BIOS image.
    Info {
        /// Path to the BIOS image.
        bios: PathBuf,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        CliCommand::Run {
            bios,
            exe,
            disc,
            memcard,
            scale,
            fullscreen,
            multitap,
            config,
        } => cmd_run(
            &bios,
            exe.as_deref(),
            disc.as_deref(),
            memcard,
            scale,
            fullscreen,
            multitap,
            &config,
        ),
        CliCommand::Info { bios } => cmd_info(&bios),
    }
}

fn cmd_info(bios_path: &Path) -> Result<()> {
    let data = fs::read(bios_path)
        .with_context(|| format!("failed to read BIOS: {}", bios_path.display()))?;
    println!("BIOS: {}", bios_path.display());
    println!("Size: {} bytes ({} KB)", data.len(), data.len() / 1024);
    println!(
        "Expected: {} bytes ({} KB)",
        psoxide_core::BIOS_IMAGE_SIZE,
        psoxide_core::BIOS_IMAGE_SIZE / 1024
    );
    Ok(())
}

/// Memory-card image size in bytes (128 KB).
const MEMCARD_BYTES: usize = 128 * 1024;

#[allow(clippy::too_many_arguments)]
fn cmd_run(
    bios_name: &str,
    exe: Option<&Path>,
    disc: Option<&Path>,
    memcard: Option<PathBuf>,
    scale: Option<u32>,
    fullscreen: bool,
    multitap: bool,
    config_path: &Path,
) -> Result<()> {
    let mut config = PsxConfig::load(config_path).unwrap_or_default();
    // CLI flags override the persisted config; otherwise fall back to it.
    let scale = scale
        .unwrap_or(config.desktop.window_scale)
        .clamp(MIN_SCALE, MAX_SCALE);
    let fullscreen = fullscreen || config.desktop.fullscreen;
    let multitap = multitap || config.desktop.multitap;
    // Resolve the runtime keybindings once (config strings → winit KeyCodes,
    // falling back to the built-in default for any unrecognised name).
    let keys = Keys::from_config(&config.keybindings);

    let bios_path = config.resolve_disc(bios_name);
    let bios_data = fs::read(&bios_path)
        .with_context(|| format!("failed to read BIOS: {}", bios_path.display()))?;

    let mut core = PsxCore::new();
    core.execute(Command::LoadBios(bios_data))
        .map_err(|e| anyhow::anyhow!("failed to load BIOS: {e}"))?;

    // Base path for save-state files: the most specific loaded artefact.
    let save_base = disc
        .map(Path::to_path_buf)
        .or_else(|| exe.map(Path::to_path_buf))
        .unwrap_or_else(|| bios_path.clone());

    if let Some(exe_path) = exe {
        let exe_data = fs::read(exe_path)
            .with_context(|| format!("failed to read EXE: {}", exe_path.display()))?;
        core.execute(Command::LoadExe(exe_data))
            .map_err(|e| anyhow::anyhow!("failed to load EXE: {e}"))?;
    }

    if let Some(disc_path) = disc {
        let disc = psoxide_config::disc::load_disc(disc_path)
            .with_context(|| format!("failed to load disc: {}", disc_path.display()))?;
        core.execute(Command::LoadDisc(disc))
            .map_err(|e| anyhow::anyhow!("failed to mount disc: {e}"))?;
    }

    // Memory card in slot 0 (when `--memcard PATH` is given). If the file
    // exists, load it (padding/truncating to the 128 KB card size, warning if
    // the size is off); otherwise start from a fresh all-zero card that the
    // BIOS will format. Slot 1 (index 1) is left empty and always reports
    // "no card". The `PathBuf` is stashed in `App` so writes can be flushed
    // back (see the save-on-dirty policy in `RedrawRequested`/`CloseRequested`).
    if let Some(ref path) = memcard {
        let data = if path.exists() {
            let mut bytes = fs::read(path)
                .with_context(|| format!("failed to read memory card: {}", path.display()))?;
            if bytes.len() != MEMCARD_BYTES {
                eprintln!(
                    "Warning: memory card {} is {} bytes, expected {MEMCARD_BYTES}; padding/truncating",
                    path.display(),
                    bytes.len()
                );
                bytes.resize(MEMCARD_BYTES, 0);
            }
            bytes
        } else {
            vec![0u8; MEMCARD_BYTES]
        };
        core.execute(Command::InsertMemoryCard { slot: 0, data })
            .map_err(|e| anyhow::anyhow!("failed to insert memory card: {e}"))?;
    }

    // Gamepad input via gilrs (optional — keyboard still works without one).
    let gilrs = match Gilrs::new() {
        Ok(g) => Some(g),
        Err(e) => {
            eprintln!("Warning: gamepad support unavailable: {e}");
            None
        }
    };

    // Attach a Multitap on port 0 when requested. Player 1 (keyboard / the first
    // gamepad) drives sub-slot A; sub-slots B/C/D start empty and can be driven
    // via the core `SetMultitap*` API (mapping extra gilrs gamepads to B/C/D is
    // a follow-up — the current event loop aggregates all pads into port 0).
    if multitap {
        let _ = core.execute(Command::SetMultitap {
            port: 0,
            enabled: true,
        });
    }

    // If a gamepad is already connected at startup, attach a DualShock (analog)
    // pad to port 0 (or the tap's sub-slot A) so the sticks are live
    // immediately. Keyboard-only sessions keep the default digital pad (see the
    // input-mapping doc comment below).
    let mut analog_attached = false;
    if let Some(gilrs) = &gilrs
        && gilrs.gamepads().any(|(_, gp)| gp.is_connected())
    {
        if multitap {
            let _ = core.execute(Command::SetMultitapControllerType {
                port: 0,
                slot: 0,
                kind: ControllerKind::Analog,
            });
        } else {
            let _ = core.execute(Command::SetControllerType {
                port: 0,
                kind: ControllerKind::Analog,
            });
        }
        analog_attached = true;
    }

    // Remember the resolved paths + window settings so the next launch can
    // reuse them (persisted on exit).
    config.desktop.bios_path = bios_path.to_string_lossy().into_owned();
    config.desktop.last_bios = bios_path.to_string_lossy().into_owned();
    if let Some(d) = disc {
        config.desktop.last_disc = d.to_string_lossy().into_owned();
    }
    if let Some(m) = &memcard {
        config.desktop.last_memcard = m.to_string_lossy().into_owned();
    }
    config.desktop.window_scale = scale;
    config.desktop.fullscreen = fullscreen;
    config.desktop.multitap = multitap;

    print_controls_banner(&config.keybindings);

    let event_loop = EventLoop::new().context("failed to create event loop")?;
    event_loop.set_control_flow(ControlFlow::Poll);

    let frame_duration = Duration::from_secs_f64(1.0 / FRAMES_PER_SECOND as f64);
    let mut app = App {
        core,
        scale,
        buttons: 0,
        window: None,
        pixels: None,
        audio: AudioOutput::try_new(),
        gilrs,
        memcard_path: memcard,
        multitap,
        analog_attached,
        analog_lx: 0x80,
        analog_ly: 0x80,
        analog_rx: 0x80,
        analog_ry: 0x80,
        last_rumble: (0, 0),
        keys,
        config,
        config_path: config_path.to_path_buf(),
        save_base,
        paused: false,
        fast_forward: false,
        fullscreen,
        active_slot: savestate::MIN_SLOT,
        frame_duration,
        last_frame_time: None,
        fps_window_start: Instant::now(),
        fps_window_frames: 0,
        hud_fps: 0.0,
    };
    event_loop.run_app(&mut app).context("event loop error")?;
    Ok(())
}

/// Prints a one-time controls banner to the terminal at startup.
fn print_controls_banner(kb: &psoxide_config::Keybindings) {
    eprintln!("psoxide controls:");
    eprintln!("  {:<7} pause / resume", kb.pause);
    eprintln!("  {:<7} frame-step (while paused)", kb.frame_step);
    eprintln!("  {:<7} fast-forward (hold)", kb.fast_forward);
    eprintln!("  {:<7} reset", kb.reset);
    eprintln!("  {:<7} fullscreen", kb.fullscreen);
    eprintln!(
        "  {} / {}  window scale up / down",
        kb.scale_up, kb.scale_down
    );
    eprintln!("  1-9     select save-state slot");
    eprintln!("  {:<7} save state to active slot", kb.save_state);
    eprintln!("  {:<7} load state from active slot", kb.load_state);
    eprintln!("  Esc     quit");
}

struct App {
    core: PsxCore,
    scale: u32,
    buttons: u16,
    audio: Option<AudioOutput>,
    window: Option<Window>,
    pixels: Option<Pixels<'static>>,
    /// Gamepad input context (`None` when unavailable).
    gilrs: Option<Gilrs>,
    /// Slot-0 memory-card file to persist to (`None` when `--memcard` was not
    /// passed, in which case no card is inserted and none is written).
    memcard_path: Option<PathBuf>,
    /// Whether a Multitap is attached on port 0. When set, player-1 (keyboard /
    /// gamepad) input is routed to the tap's sub-slot A instead of the port's
    /// single pad.
    multitap: bool,
    /// Whether a DualShock (analog) pad has been attached to port 0 for the
    /// gamepad. Set once, on first gamepad presence/connect, to avoid re-sending
    /// `SetControllerType` every frame.
    analog_attached: bool,
    /// Latest analog-axis bytes (`0x80` = centre) for the gamepad's sticks, in
    /// PSX convention (Y already inverted vs gilrs). Left stick X/Y and right
    /// stick X/Y; pushed to the core on any axis change via
    /// [`Command::SetControllerSticks`].
    analog_lx: u8,
    analog_ly: u8,
    analog_rx: u8,
    analog_ry: u8,
    /// Last rumble-motor actuation (small, large) logged for port 0, so the
    /// log-only readback only fires on a change rather than every frame.
    last_rumble: (u8, u8),
    /// Resolved runtime-control keybindings.
    keys: Keys,
    /// Live config, persisted (with last-used paths + window settings) on exit.
    config: PsxConfig,
    /// Path the config is loaded from / saved back to.
    config_path: PathBuf,
    /// Base path for save-state slot files (disc, else EXE, else BIOS).
    save_base: PathBuf,
    /// Whether emulation stepping is paused (frontend-driven; the core's
    /// `StepFrame` does not itself honour a paused flag).
    paused: bool,
    /// Whether fast-forward is held (uncaps the frame pacer).
    fast_forward: bool,
    /// Whether the window is currently fullscreen.
    fullscreen: bool,
    /// Currently selected save-state slot (`1..=9`).
    active_slot: u8,
    /// Target wall-clock duration of one emulated frame at 1x speed.
    frame_duration: Duration,
    /// Instant the last frame finished, for pacing.
    last_frame_time: Option<Instant>,
    /// Start of the current ~1s HUD measurement window.
    fps_window_start: Instant,
    /// Frames rendered in the current HUD window.
    fps_window_frames: u32,
    /// Most recently measured frames-per-second (for the title-bar HUD).
    hud_fps: f64,
}

/// Runtime-control keybindings resolved to concrete winit key codes.
struct Keys {
    pause: KeyCode,
    frame_step: KeyCode,
    fast_forward: KeyCode,
    reset: KeyCode,
    fullscreen: KeyCode,
    scale_up: KeyCode,
    scale_down: KeyCode,
    save_state: KeyCode,
    load_state: KeyCode,
}

impl Keys {
    /// Resolves each configured binding, falling back to the built-in default
    /// name when the configured string does not name a known key.
    fn from_config(kb: &psoxide_config::Keybindings) -> Self {
        let d = psoxide_config::Keybindings::default();
        let resolve = |name: &str, fallback: &str| {
            parse_keycode(name)
                .or_else(|| parse_keycode(fallback))
                .unwrap_or(KeyCode::Escape)
        };
        Self {
            pause: resolve(&kb.pause, &d.pause),
            frame_step: resolve(&kb.frame_step, &d.frame_step),
            fast_forward: resolve(&kb.fast_forward, &d.fast_forward),
            reset: resolve(&kb.reset, &d.reset),
            fullscreen: resolve(&kb.fullscreen, &d.fullscreen),
            scale_up: resolve(&kb.scale_up, &d.scale_up),
            scale_down: resolve(&kb.scale_down, &d.scale_down),
            save_state: resolve(&kb.save_state, &d.save_state),
            load_state: resolve(&kb.load_state, &d.load_state),
        }
    }
}

/// Parses a winit [`KeyCode`] from its variant name (as written in the config).
///
/// Covers the letter/function/digit keys and the handful of punctuation keys
/// the default bindings use. Returns `None` for an unrecognised name so the
/// caller can fall back to a default.
fn parse_keycode(name: &str) -> Option<KeyCode> {
    Some(match name {
        "KeyA" => KeyCode::KeyA,
        "KeyB" => KeyCode::KeyB,
        "KeyC" => KeyCode::KeyC,
        "KeyD" => KeyCode::KeyD,
        "KeyE" => KeyCode::KeyE,
        "KeyF" => KeyCode::KeyF,
        "KeyG" => KeyCode::KeyG,
        "KeyH" => KeyCode::KeyH,
        "KeyI" => KeyCode::KeyI,
        "KeyJ" => KeyCode::KeyJ,
        "KeyK" => KeyCode::KeyK,
        "KeyL" => KeyCode::KeyL,
        "KeyM" => KeyCode::KeyM,
        "KeyN" => KeyCode::KeyN,
        "KeyO" => KeyCode::KeyO,
        "KeyP" => KeyCode::KeyP,
        "KeyQ" => KeyCode::KeyQ,
        "KeyR" => KeyCode::KeyR,
        "KeyS" => KeyCode::KeyS,
        "KeyT" => KeyCode::KeyT,
        "KeyU" => KeyCode::KeyU,
        "KeyV" => KeyCode::KeyV,
        "KeyW" => KeyCode::KeyW,
        "KeyX" => KeyCode::KeyX,
        "KeyY" => KeyCode::KeyY,
        "KeyZ" => KeyCode::KeyZ,
        "F1" => KeyCode::F1,
        "F2" => KeyCode::F2,
        "F3" => KeyCode::F3,
        "F4" => KeyCode::F4,
        "F5" => KeyCode::F5,
        "F6" => KeyCode::F6,
        "F7" => KeyCode::F7,
        "F8" => KeyCode::F8,
        "F9" => KeyCode::F9,
        "F10" => KeyCode::F10,
        "F11" => KeyCode::F11,
        "F12" => KeyCode::F12,
        "Space" => KeyCode::Space,
        "Tab" => KeyCode::Tab,
        "Enter" => KeyCode::Enter,
        "Equal" => KeyCode::Equal,
        "Minus" => KeyCode::Minus,
        "BracketLeft" => KeyCode::BracketLeft,
        "BracketRight" => KeyCode::BracketRight,
        "Backslash" => KeyCode::Backslash,
        "Period" => KeyCode::Period,
        "Comma" => KeyCode::Comma,
        "Backspace" => KeyCode::Backspace,
        _ => return None,
    })
}

/// Converts a gilrs analog-axis value (`f32` in `-1.0..=1.0`) to a PSX analog
/// axis byte (`0..=255`, `0x80` = centre): `0.0 -> 0x80`, `+1.0 -> 0xFF`,
/// `-1.0 -> 0x00`. Out-of-range inputs are clamped.
///
/// The caller is responsible for the PSX Y-axis inversion (gilrs up is `+1.0`,
/// PSX up is `0x00`): pass `-value` for the two Y axes.
fn axis_f32_to_u8(value: f32) -> u8 {
    let scaled = value.clamp(-1.0, 1.0) * 128.0 + 128.0;
    scaled.round().clamp(0.0, 255.0) as u8
}

impl App {
    /// Pushes the player-1 button bitfield to the core, routing to the tap's
    /// sub-slot A when a Multitap is attached on port 0, else the port's pad.
    fn send_player1_buttons(&mut self, buttons: u16) {
        let cmd = if self.multitap {
            Command::SetMultitapControllerState {
                port: 0,
                slot: 0,
                buttons,
            }
        } else {
            Command::SetControllerState { port: 0, buttons }
        };
        let _ = self.core.execute(cmd);
    }

    /// Pushes the player-1 analog-stick axes to the core, routing to the tap's
    /// sub-slot A when a Multitap is attached on port 0, else the port's pad.
    fn send_player1_sticks(&mut self, right: (u8, u8), left: (u8, u8)) {
        let cmd = if self.multitap {
            Command::SetMultitapControllerSticks {
                port: 0,
                slot: 0,
                right,
                left,
            }
        } else {
            Command::SetControllerSticks {
                port: 0,
                right,
                left,
            }
        };
        let _ = self.core.execute(cmd);
    }

    /// Attaches a DualShock (analog) pad for player 1, routing to the tap's
    /// sub-slot A when a Multitap is attached on port 0, else the port's pad.
    fn attach_player1_analog(&mut self) {
        let cmd = if self.multitap {
            Command::SetMultitapControllerType {
                port: 0,
                slot: 0,
                kind: ControllerKind::Analog,
            }
        } else {
            Command::SetControllerType {
                port: 0,
                kind: ControllerKind::Analog,
            }
        };
        let _ = self.core.execute(cmd);
    }

    /// Flushes the slot-0 memory card to its configured file if it is dirty.
    ///
    /// Persistence policy: flush on dirty each frame + on exit. Querying the
    /// card is cheap and only writes the file when the core reports unsaved
    /// changes, so an idle game does not thrash the disk. Only slot 0 is
    /// persisted; slot 1 is never populated by the desktop frontend.
    fn flush_memcard(&mut self) {
        let Some(path) = self.memcard_path.clone() else {
            return;
        };
        let QueryResult::MemoryCard {
            present,
            data,
            dirty,
        } = self.core.query(CoreQuery::MemoryCard { slot: 0 })
        else {
            return;
        };
        if !present || !dirty {
            return;
        }
        match fs::write(&path, &data) {
            Ok(()) => {
                let _ = self.core.execute(Command::ClearMemoryCardDirty { slot: 0 });
            }
            Err(e) => eprintln!(
                "Warning: failed to write memory card {}: {e}",
                path.display()
            ),
        }
    }

    /// Saves the current machine state to the active slot. Failures are logged,
    /// never fatal.
    fn save_state_slot(&mut self) {
        let snap = self.core.save_state();
        match savestate::write_slot(&self.save_base, self.active_slot, &snap) {
            Ok(path) => eprintln!(
                "Saved state to slot {} ({})",
                self.active_slot,
                path.display()
            ),
            Err(e) => eprintln!("Warning: {e}"),
        }
    }

    /// Loads the active slot into the machine. A missing or invalid slot is
    /// reported and otherwise ignored (the running game is left untouched).
    fn load_state_slot(&mut self) {
        match savestate::read_slot(&self.save_base, self.active_slot) {
            Ok(snap) => {
                self.core.load_state(&snap);
                eprintln!("Loaded state from slot {}", self.active_slot);
            }
            Err(e) => eprintln!("Warning: {e}"),
        }
    }

    /// Applies a new integer window scale (clamped) and resizes the window.
    fn set_scale(&mut self, scale: u32) {
        let scale = scale.clamp(MIN_SCALE, MAX_SCALE);
        if scale == self.scale {
            return;
        }
        self.scale = scale;
        if let Some(window) = self.window.as_ref() {
            let _ = window.request_inner_size(LogicalSize::new(
                FRAME_WIDTH as u32 * scale,
                FRAME_HEIGHT as u32 * scale,
            ));
        }
        eprintln!("Window scale: {scale}x");
    }

    /// Toggles fullscreen (borderless on the current monitor).
    fn toggle_fullscreen(&mut self) {
        self.fullscreen = !self.fullscreen;
        if let Some(window) = self.window.as_ref() {
            window.set_fullscreen(if self.fullscreen {
                Some(Fullscreen::Borderless(None))
            } else {
                None
            });
        }
    }

    /// Updates the title-bar HUD (fps, emulation speed, audio underruns) once
    /// per ~1s measurement window.
    fn update_hud(&mut self) {
        self.fps_window_frames += 1;
        let elapsed = self.fps_window_start.elapsed();
        if elapsed < Duration::from_millis(500) {
            return;
        }
        self.hud_fps = self.fps_window_frames as f64 / elapsed.as_secs_f64();
        self.fps_window_frames = 0;
        self.fps_window_start = Instant::now();

        let speed = self.hud_fps / FRAMES_PER_SECOND as f64 * 100.0;
        let underruns = self.audio.as_ref().map_or(0, AudioOutput::underruns);
        let status = if self.paused {
            " [paused]"
        } else if self.fast_forward {
            " [ff]"
        } else {
            ""
        };
        if let Some(window) = self.window.as_ref() {
            window.set_title(&format!(
                "psoxide — {:.0} fps  {:.0}%  slot {}  underruns {}{}",
                self.hud_fps, speed, self.active_slot, underruns, status
            ));
        }
    }

    /// Handles a runtime-control hotkey (edge-triggered on key-down). Returns
    /// `true` when the key was consumed as a control and must not also drive the
    /// pad. Fast-forward is handled by the caller (it needs both key edges).
    fn handle_hotkey(&mut self, key: KeyCode) -> bool {
        // Digit keys 1-9 select the active save-state slot.
        if let Some(slot) = digit_slot(key) {
            self.active_slot = slot;
            eprintln!("Save-state slot: {slot}");
            return true;
        }
        if key == self.keys.pause {
            self.paused = !self.paused;
            let _ = self.core.execute(if self.paused {
                Command::Pause
            } else {
                Command::Resume
            });
            eprintln!("{}", if self.paused { "Paused" } else { "Resumed" });
            true
        } else if key == self.keys.frame_step {
            if self.paused {
                // The core's StepFrame ignores the paused flag, so a single call
                // advances exactly one frame while we stay logically paused.
                let _ = self.core.execute(Command::StepFrame);
            }
            true
        } else if key == self.keys.reset {
            let _ = self.core.execute(Command::Reset);
            eprintln!("Reset");
            true
        } else if key == self.keys.fullscreen {
            self.toggle_fullscreen();
            true
        } else if key == self.keys.scale_up {
            self.set_scale(self.scale + 1);
            true
        } else if key == self.keys.scale_down {
            self.set_scale(self.scale.saturating_sub(1));
            true
        } else if key == self.keys.save_state {
            self.save_state_slot();
            true
        } else if key == self.keys.load_state {
            self.load_state_slot();
            true
        } else {
            false
        }
    }

    /// Persists the memory card and config on any exit path.
    fn on_exit(&mut self) {
        self.flush_memcard();
        self.config.desktop.window_scale = self.scale;
        self.config.desktop.fullscreen = self.fullscreen;
        if let Err(e) = self.config.save(&self.config_path) {
            eprintln!(
                "Warning: failed to write config {}: {e}",
                self.config_path.display()
            );
        }
        if let Some(audio) = self.audio.as_ref() {
            audio.shutdown();
        }
    }
}

/// Maps digit keys `1..=9` to a save-state slot number.
fn digit_slot(key: KeyCode) -> Option<u8> {
    Some(match key {
        KeyCode::Digit1 => 1,
        KeyCode::Digit2 => 2,
        KeyCode::Digit3 => 3,
        KeyCode::Digit4 => 4,
        KeyCode::Digit5 => 5,
        KeyCode::Digit6 => 6,
        KeyCode::Digit7 => 7,
        KeyCode::Digit8 => 8,
        KeyCode::Digit9 => 9,
        _ => return None,
    })
}

fn key_to_button(key: KeyCode) -> Option<Button> {
    match key {
        KeyCode::ArrowUp => Some(Button::Up),
        KeyCode::ArrowDown => Some(Button::Down),
        KeyCode::ArrowLeft => Some(Button::Left),
        KeyCode::ArrowRight => Some(Button::Right),
        KeyCode::KeyZ => Some(Button::Cross),
        KeyCode::KeyX => Some(Button::Circle),
        KeyCode::KeyA => Some(Button::Square),
        KeyCode::KeyS => Some(Button::Triangle),
        KeyCode::KeyQ => Some(Button::L1),
        KeyCode::KeyW => Some(Button::R1),
        KeyCode::Enter => Some(Button::Start),
        KeyCode::ShiftRight => Some(Button::Select),
        _ => None,
    }
}

/// # Desktop input mapping
///
/// The two host input devices map onto two different PSX pad kinds:
///
/// - **Keyboard = digital pad only.** Keyboard input drives the port-0 button
///   bitfield through [`Command::SetControllerState`]; a keyboard-only session
///   keeps the default (digital) pad and no analog sticks. See
///   [`key_to_button`].
/// - **Gamepad = DualShock (analog).** When a gilrs gamepad is present the
///   frontend attaches a [`ControllerKind::Analog`] pad to port 0, so both
///   sticks (and L3/R3) are live in addition to the digital buttons.
///
/// Gamepad button/axis table:
///
/// | gilrs input                    | PSX      |
/// |--------------------------------|----------|
/// | `DPadUp/Down/Left/Right`       | `Up/Down/Left/Right` |
/// | `South`                        | `Cross` (✕)   |
/// | `East`                         | `Circle` (○)  |
/// | `West`                         | `Square` (□)  |
/// | `North`                        | `Triangle` (△) |
/// | `LeftTrigger` (bumper)         | `L1`     |
/// | `RightTrigger` (bumper)        | `R1`     |
/// | `LeftTrigger2` (trigger)       | `L2`     |
/// | `RightTrigger2` (trigger)      | `R2`     |
/// | `LeftThumb` (stick click)      | `L3`     |
/// | `RightThumb` (stick click)     | `R3`     |
/// | `Start`                        | `Start`  |
/// | `Select`                       | `Select` |
/// | `LeftStickX/Y`                 | left analog stick  |
/// | `RightStickX/Y`                | right analog stick |
///
/// Analog axes: gilrs `f32` (`-1.0..=1.0`) → PSX byte (`0x80` centre) via
/// [`axis_f32_to_u8`]; the PSX Y axis is inverted vs gilrs (see the axis
/// handler in `about_to_wait`).
///
/// Returns `None` for buttons with no PSX equivalent.
fn pad_button_to_psx(button: PadButton) -> Option<Button> {
    Some(match button {
        PadButton::DPadUp => Button::Up,
        PadButton::DPadDown => Button::Down,
        PadButton::DPadLeft => Button::Left,
        PadButton::DPadRight => Button::Right,
        PadButton::South => Button::Cross,
        PadButton::East => Button::Circle,
        PadButton::West => Button::Square,
        PadButton::North => Button::Triangle,
        PadButton::LeftTrigger => Button::L1,
        PadButton::RightTrigger => Button::R1,
        PadButton::LeftTrigger2 => Button::L2,
        PadButton::RightTrigger2 => Button::R2,
        PadButton::LeftThumb => Button::L3,
        PadButton::RightThumb => Button::R3,
        PadButton::Start => Button::Start,
        PadButton::Select => Button::Select,
        _ => return None,
    })
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let size = LogicalSize::new(
            FRAME_WIDTH as u32 * self.scale,
            FRAME_HEIGHT as u32 * self.scale,
        );
        let mut attrs = Window::default_attributes()
            .with_title("psoxide")
            .with_inner_size(size)
            .with_min_inner_size(LogicalSize::new(FRAME_WIDTH as u32, FRAME_HEIGHT as u32));
        if self.fullscreen {
            attrs = attrs.with_fullscreen(Some(Fullscreen::Borderless(None)));
        }
        let window = event_loop
            .create_window(attrs)
            .expect("failed to create window");

        self.window = Some(window);
        let window_ref = self.window.as_ref().unwrap();
        let physical = window_ref.inner_size();
        let surface = SurfaceTexture::new(physical.width, physical.height, window_ref);
        let pixels = Pixels::new(FRAME_WIDTH as u32, FRAME_HEIGHT as u32, surface)
            .expect("failed to create pixel buffer");

        // SAFETY: `pixels` borrows `self.window`, which we keep alive for the
        // lifetime of `App` and never move or drop while `pixels` exists.
        self.pixels =
            Some(unsafe { std::mem::transmute::<pixels::Pixels<'_>, pixels::Pixels<'_>>(pixels) });
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => {
                // Flush the memory card + config one last time before exiting.
                self.on_exit();
                event_loop.exit();
            }
            WindowEvent::Resized(size) => {
                // Keep the pixel surface matched to the window (rescale +
                // fullscreen both land here); the 320x240 buffer is unchanged
                // and Pixels scales it up to the surface.
                if let Some(pixels) = self.pixels.as_mut() {
                    let _ = pixels.resize_surface(size.width.max(1), size.height.max(1));
                }
            }
            WindowEvent::KeyboardInput { event, .. } => {
                let pressed = event.state.is_pressed();
                if let PhysicalKey::Code(KeyCode::Escape) = event.physical_key {
                    if pressed {
                        self.on_exit();
                        event_loop.exit();
                    }
                    return;
                }
                if let PhysicalKey::Code(key) = event.physical_key {
                    // Runtime-control hotkeys are edge-triggered on key-down.
                    if pressed && self.handle_hotkey(key) {
                        return;
                    }
                    // Otherwise route to the digital pad.
                    if let Some(button) = key_to_button(key) {
                        if pressed {
                            self.buttons |= button.bit_mask();
                        } else {
                            self.buttons &= !button.bit_mask();
                        }
                        self.send_player1_buttons(self.buttons);
                    } else if key == self.keys.fast_forward {
                        // Fast-forward is a hold: track both edges.
                        self.fast_forward = pressed;
                    }
                }
            }
            WindowEvent::RedrawRequested => {
                // Advance the machine unless paused. When paused we still
                // re-render the last frame and drain audio so nothing backs up.
                if !self.paused {
                    let _ = self.core.execute(Command::StepFrame);
                }
                if let Some(pixels) = self.pixels.as_mut() {
                    let frame = self.core.framebuffer_rgba();
                    pixels.frame_mut().copy_from_slice(&frame);
                    let _ = pixels.render();
                }
                // Feed this frame's SPU output to the host audio device. Always
                // drain the core queue so it cannot grow unbounded even when
                // there is no audio device.
                let samples = self.core.drain_audio();
                if let Some(audio) = self.audio.as_mut() {
                    audio.queue(samples);
                }
                // Persistence policy: flush on dirty each frame + on exit. This
                // only touches the disk when the card was actually written.
                self.flush_memcard();
                self.update_hud();
                // The next redraw is requested from `about_to_wait`, after the
                // frame pacer has slept off the slack.
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        // Drain gamepad events into the port-0 button bitfield, mirroring the
        // keyboard path's whole-bitfield `SetControllerState` update.
        if let Some(gilrs) = &mut self.gilrs {
            let mut buttons_changed = false;
            let mut sticks_changed = false;
            let mut newly_connected = false;
            while let Some(gilrs::Event { event, .. }) = gilrs.next_event() {
                match event {
                    EventType::Connected => newly_connected = true,
                    EventType::ButtonPressed(b, _) | EventType::ButtonReleased(b, _) => {
                        let pressed = matches!(event, EventType::ButtonPressed(_, _));
                        if let Some(button) = pad_button_to_psx(b) {
                            if pressed {
                                self.buttons |= button.bit_mask();
                            } else {
                                self.buttons &= !button.bit_mask();
                            }
                            buttons_changed = true;
                        }
                    }
                    EventType::AxisChanged(axis, value, _) => {
                        // gilrs value is -1.0..=1.0; PSX Y axis is inverted vs
                        // gilrs (gilrs up = +1.0, PSX up = 0x00) so negate Y.
                        match axis {
                            Axis::LeftStickX => self.analog_lx = axis_f32_to_u8(value),
                            Axis::LeftStickY => self.analog_ly = axis_f32_to_u8(-value),
                            Axis::RightStickX => self.analog_rx = axis_f32_to_u8(value),
                            Axis::RightStickY => self.analog_ry = axis_f32_to_u8(-value),
                            _ => continue,
                        }
                        sticks_changed = true;
                    }
                    _ => {}
                }
            }

            // Attach a DualShock (analog) pad to port 0 on the first gamepad
            // connect, so hotplugged pads get live sticks too.
            if newly_connected && !self.analog_attached {
                self.attach_player1_analog();
                self.analog_attached = true;
            }
            if buttons_changed {
                self.send_player1_buttons(self.buttons);
            }
            if sticks_changed {
                self.send_player1_sticks(
                    (self.analog_rx, self.analog_ry),
                    (self.analog_lx, self.analog_ly),
                );
            }

            // Log-only rumble readback: report the analog pad's motor state when
            // it changes, so a future host rumble backend has a clear hook point
            // (the desktop has none yet, so actuation is logged, not played).
            if let QueryResult::ControllerRumble {
                present: true,
                small,
                large,
            } = self.core.query(CoreQuery::ControllerRumble { port: 0 })
                && (small, large) != self.last_rumble
            {
                self.last_rumble = (small, large);
                if small != 0 || large != 0 {
                    eprintln!(
                        "Rumble port 0: small={small:#04x} large={large:#04x} (no host backend; ignored)"
                    );
                }
            }
        }

        // Frame pacing: cap to ~60 fps at 1x speed by sleeping off the slack
        // since the previous frame. Fast-forward (and the paused re-render loop)
        // skip the sleep so they run as fast as the host allows.
        if !self.fast_forward
            && let Some(last) = self.last_frame_time
        {
            let elapsed = last.elapsed();
            if elapsed < self.frame_duration {
                std::thread::sleep(self.frame_duration - elapsed);
            }
        }
        self.last_frame_time = Some(Instant::now());

        if let Some(window) = self.window.as_ref() {
            window.request_redraw();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Keys, axis_f32_to_u8, digit_slot, parse_keycode};
    use psoxide_config::Keybindings;
    use winit::keyboard::KeyCode;

    #[test]
    fn parse_keycode_known_and_unknown() {
        assert_eq!(parse_keycode("KeyP"), Some(KeyCode::KeyP));
        assert_eq!(parse_keycode("F5"), Some(KeyCode::F5));
        assert_eq!(parse_keycode("Space"), Some(KeyCode::Space));
        assert_eq!(parse_keycode("Equal"), Some(KeyCode::Equal));
        assert_eq!(parse_keycode("NotAKey"), None);
    }

    #[test]
    fn keys_default_bindings_resolve() {
        let keys = Keys::from_config(&Keybindings::default());
        assert_eq!(keys.pause, KeyCode::KeyP);
        assert_eq!(keys.save_state, KeyCode::F5);
        assert_eq!(keys.load_state, KeyCode::F9);
        assert_eq!(keys.fast_forward, KeyCode::Space);
        assert_eq!(keys.reset, KeyCode::KeyR);
    }

    #[test]
    fn keys_unknown_binding_falls_back_to_default() {
        let kb = Keybindings {
            pause: "TotallyBogus".into(),
            ..Default::default()
        };
        let keys = Keys::from_config(&kb);
        // Unrecognised name → the default binding for that action is used.
        assert_eq!(keys.pause, KeyCode::KeyP);
    }

    #[test]
    fn keys_custom_binding_applies() {
        let kb = Keybindings {
            pause: "KeyM".into(),
            ..Default::default()
        };
        let keys = Keys::from_config(&kb);
        assert_eq!(keys.pause, KeyCode::KeyM);
    }

    #[test]
    fn digit_slot_maps_1_to_9() {
        assert_eq!(digit_slot(KeyCode::Digit1), Some(1));
        assert_eq!(digit_slot(KeyCode::Digit9), Some(9));
        assert_eq!(digit_slot(KeyCode::Digit0), None);
        assert_eq!(digit_slot(KeyCode::KeyA), None);
    }

    #[test]
    fn axis_centre_maps_to_0x80() {
        assert_eq!(axis_f32_to_u8(0.0), 0x80);
    }

    #[test]
    fn axis_full_positive_maps_to_0xff() {
        assert_eq!(axis_f32_to_u8(1.0), 0xFF);
    }

    #[test]
    fn axis_full_negative_maps_to_0x00() {
        assert_eq!(axis_f32_to_u8(-1.0), 0x00);
    }

    #[test]
    fn axis_out_of_range_is_clamped() {
        assert_eq!(axis_f32_to_u8(2.5), 0xFF);
        assert_eq!(axis_f32_to_u8(-2.5), 0x00);
    }
}
