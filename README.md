# psoxide

Sony PlayStation (PSX) emulator in Rust. Part of the oxide emulator family.

## Controls

### Keyboard

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
| Escape      | Quit         |

### Gamepad

A gamepad is supported (when connected) via [gilrs](https://crates.io/crates/gilrs),
using the standard SNES/PS-style mapping: D-pad → D-pad, South/East/West/North →
✕/○/□/△, left/right triggers → L1/R1, and Start/Select → Start/Select. Keyboard
input still works when no gamepad is present.
