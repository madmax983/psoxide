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

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use gilrs::{Axis, Button as PadButton, EventType, Gilrs};
use pixels::{Pixels, SurfaceTexture};
use psoxide_config::PsxConfig;
use psoxide_core::{
    Button, Command, ControllerKind, CoreQuery, FRAME_HEIGHT, FRAME_WIDTH, PsxCore, QueryResult,
};
use winit::application::ApplicationHandler;
use winit::dpi::LogicalSize;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

use crate::audio::AudioOutput;

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
        /// Window scale factor.
        #[arg(long, default_value = "2")]
        scale: u32,
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
            config,
        } => cmd_run(
            &bios,
            exe.as_deref(),
            disc.as_deref(),
            memcard,
            scale,
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

fn cmd_run(
    bios_name: &str,
    exe: Option<&Path>,
    disc: Option<&Path>,
    memcard: Option<PathBuf>,
    scale: u32,
    config_path: &Path,
) -> Result<()> {
    let config = PsxConfig::load(config_path).unwrap_or_default();
    let bios_path = config.resolve_disc(bios_name);
    let bios_data = fs::read(&bios_path)
        .with_context(|| format!("failed to read BIOS: {}", bios_path.display()))?;

    let mut core = PsxCore::new();
    core.execute(Command::LoadBios(bios_data))
        .map_err(|e| anyhow::anyhow!("failed to load BIOS: {e}"))?;

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

    // If a gamepad is already connected at startup, attach a DualShock (analog)
    // pad to port 0 so the sticks are live immediately. Keyboard-only sessions
    // keep the default digital pad (see the input-mapping doc comment below).
    let mut analog_attached = false;
    if let Some(gilrs) = &gilrs
        && gilrs.gamepads().any(|(_, gp)| gp.is_connected())
    {
        let _ = core.execute(Command::SetControllerType {
            port: 0,
            kind: ControllerKind::Analog,
        });
        analog_attached = true;
    }

    let event_loop = EventLoop::new().context("failed to create event loop")?;
    event_loop.set_control_flow(ControlFlow::Poll);

    let mut app = App {
        core,
        scale,
        buttons: 0,
        window: None,
        pixels: None,
        audio: AudioOutput::try_new(),
        gilrs,
        memcard_path: memcard,
        analog_attached,
        analog_lx: 0x80,
        analog_ly: 0x80,
        analog_rx: 0x80,
        analog_ry: 0x80,
        last_rumble: (0, 0),
    };
    event_loop.run_app(&mut app).context("event loop error")?;
    Ok(())
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
        let attrs = Window::default_attributes()
            .with_title("psoxide")
            .with_inner_size(size)
            .with_min_inner_size(LogicalSize::new(FRAME_WIDTH as u32, FRAME_HEIGHT as u32));
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
                // Flush the memory card one last time before exiting.
                self.flush_memcard();
                event_loop.exit();
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if let PhysicalKey::Code(KeyCode::Escape) = event.physical_key {
                    event_loop.exit();
                    return;
                }
                if let PhysicalKey::Code(key) = event.physical_key
                    && let Some(button) = key_to_button(key)
                {
                    if event.state.is_pressed() {
                        self.buttons |= button.bit_mask();
                    } else {
                        self.buttons &= !button.bit_mask();
                    }
                    let _ = self.core.execute(Command::SetControllerState {
                        port: 0,
                        buttons: self.buttons,
                    });
                }
            }
            WindowEvent::RedrawRequested => {
                let _ = self.core.execute(Command::StepFrame);
                if let Some(pixels) = self.pixels.as_mut() {
                    let frame = self.core.framebuffer_rgba();
                    pixels.frame_mut().copy_from_slice(&frame);
                    let _ = pixels.render();
                }
                // Feed this frame's SPU output to the host audio device. Always
                // drain the core queue so it cannot grow unbounded even when
                // there is no audio device.
                let samples = self.core.drain_audio();
                if let Some(audio) = self.audio.as_ref() {
                    audio.queue(samples);
                }
                // Persistence policy: flush on dirty each frame + on exit. This
                // only touches the disk when the card was actually written.
                self.flush_memcard();
                if let Some(window) = self.window.as_ref() {
                    window.request_redraw();
                }
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
                let _ = self.core.execute(Command::SetControllerType {
                    port: 0,
                    kind: ControllerKind::Analog,
                });
                self.analog_attached = true;
            }
            if buttons_changed {
                let _ = self.core.execute(Command::SetControllerState {
                    port: 0,
                    buttons: self.buttons,
                });
            }
            if sticks_changed {
                let _ = self.core.execute(Command::SetControllerSticks {
                    port: 0,
                    right: (self.analog_rx, self.analog_ry),
                    left: (self.analog_lx, self.analog_ly),
                });
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
    }
}

#[cfg(test)]
mod tests {
    use super::axis_f32_to_u8;

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
