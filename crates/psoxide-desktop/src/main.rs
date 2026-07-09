//! Psoxide desktop frontend.
//!
//! CLI-first PlayStation emulator skeleton.
//!
//! ```text
//! psoxide run scph1001.bin --scale 2
//! psoxide info scph1001.bin
//! ```
//!
//! The GPU is rendered via `framebuffer_rgba()` into a Pixels surface.
//! Input is driven by the keyboard and, when present, a gamepad (via gilrs);
//! both feed digital-pad state to controller port 0. Audio is a silent no-op
//! stub.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use gilrs::{Button as PadButton, EventType, Gilrs};
use pixels::{Pixels, SurfaceTexture};
use psoxide_config::PsxConfig;
use psoxide_core::{Button, Command, FRAME_HEIGHT, FRAME_WIDTH, PsxCore};
use winit::application::ApplicationHandler;
use winit::dpi::LogicalSize;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

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
            scale,
            config,
        } => cmd_run(&bios, exe.as_deref(), disc.as_deref(), scale, &config),
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

fn cmd_run(
    bios_name: &str,
    exe: Option<&Path>,
    disc: Option<&Path>,
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

    // Gamepad input via gilrs (optional — keyboard still works without one).
    let gilrs = match Gilrs::new() {
        Ok(g) => Some(g),
        Err(e) => {
            eprintln!("Warning: gamepad support unavailable: {e}");
            None
        }
    };

    let event_loop = EventLoop::new().context("failed to create event loop")?;
    event_loop.set_control_flow(ControlFlow::Poll);

    let mut app = App {
        core,
        scale,
        buttons: 0,
        window: None,
        pixels: None,
        gilrs,
    };
    event_loop.run_app(&mut app).context("event loop error")?;
    Ok(())
}

struct App {
    core: PsxCore,
    scale: u32,
    buttons: u16,
    window: Option<Window>,
    pixels: Option<Pixels<'static>>,
    /// Gamepad input context (`None` when unavailable).
    gilrs: Option<Gilrs>,
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

/// Maps a gilrs gamepad button to a PSX digital-pad button.
///
/// Standard SNES/PS-style layout:
/// - D-pad Up/Down/Left/Right → Up/Down/Left/Right
/// - South → Cross (✕), East → Circle (○), West → Square (□), North → Triangle (△)
/// - Left trigger (or bumper) → L1, Right trigger (or bumper) → R1
/// - Start → Start, Select → Select
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
        PadButton::LeftTrigger | PadButton::LeftTrigger2 => Button::L1,
        PadButton::RightTrigger | PadButton::RightTrigger2 => Button::R1,
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
            WindowEvent::CloseRequested => event_loop.exit(),
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
            let mut changed = false;
            while let Some(gilrs::Event { event, .. }) = gilrs.next_event() {
                let (pad_button, pressed) = match event {
                    EventType::ButtonPressed(b, _) => (b, true),
                    EventType::ButtonReleased(b, _) => (b, false),
                    _ => continue,
                };
                if let Some(button) = pad_button_to_psx(pad_button) {
                    if pressed {
                        self.buttons |= button.bit_mask();
                    } else {
                        self.buttons &= !button.bit_mask();
                    }
                    changed = true;
                }
            }
            if changed {
                let _ = self.core.execute(Command::SetControllerState {
                    port: 0,
                    buttons: self.buttons,
                });
            }
        }
    }
}
