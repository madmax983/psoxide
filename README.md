# psoxide

Sony PlayStation (PSX) emulator in Rust. Part of the oxide emulator family.

## Controls

### Keyboard (gameplay)

| Key         | PSX Button   |
| ----------- | ------------ |
| Arrow keys  | D-pad        |
| Z           | Cross (✕)    |
| X           | Circle (○)   |
| A           | Square (□)   |
| S           | Triangle (△) |
| Q           | L1           |
| W           | R1           |
| Enter       | Start        |
| Right Shift | Select       |

### Runtime controls

These control the emulator itself (not the game). The bindings are
configurable — see [Configuration](#configuration).

| Key       | Action                                   |
| --------- | ---------------------------------------- |
| `P`       | Pause / resume                           |
| `F`       | Frame-step (advance one frame while paused) |
| `Space`   | Fast-forward (hold — uncaps the frame pacer) |
| `R`       | Reset the machine to the BIOS entry vector |
| `F11`     | Toggle fullscreen                        |
| `=`       | Increase window scale                    |
| `-`       | Decrease window scale                    |
| `1`–`9`   | Select the active save-state slot        |
| `F5`      | Save state to the active slot            |
| `F9`      | Load state from the active slot          |
| `Esc`     | Quit (flushes the memory card + config)  |

Save states are written next to the loaded content as `<stem>.ss<slot>` (the
`<stem>` is the disc image's name, else the side-loaded EXE's, else the BIOS
image's). The slot defaults to `1`; press a number key to change it before
saving/loading.

### HUD

The window title shows a live HUD: frames-per-second, emulation speed (as a
percentage of real-time — 100% at 1x, higher while fast-forwarding), the active
save-state slot, and an audio-underrun counter. `[paused]` / `[ff]` markers show
the current run state.

### Gamepad

A gamepad is supported (when connected) via [gilrs](https://crates.io/crates/gilrs),
using the standard SNES/PS-style mapping: D-pad → D-pad, South/East/West/North →
✕/○/□/△, left/right triggers → L1/R1, and Start/Select → Start/Select. Keyboard
input still works when no gamepad is present.

## Configuration

Settings load from `psoxide.toml` (override the path with `--config`). Missing
files fall back to defaults, and the frontend writes the file back on exit to
remember the window scale, fullscreen state, and last-used BIOS/disc/memory-card
paths. CLI flags (`--scale`, `--fullscreen`) take precedence over the config.

Runtime-control keys are rebindable under `[keybindings]`, using
[winit `KeyCode`](https://docs.rs/winit) names:

```toml
[desktop]
window_scale = 2
fullscreen = false

[keybindings]
pause = "KeyP"
frame_step = "KeyF"
fast_forward = "Space"
reset = "KeyR"
fullscreen = "F11"
scale_up = "Equal"
scale_down = "Minus"
save_state = "F5"
load_state = "F9"
```

An unrecognised key name falls back to that action's built-in default, so a
hand-edited config can never leave a control unbound.
