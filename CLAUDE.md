# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a PlatformIO/Arduino project for a RoboCup rescue maze robot running on an **Arduino Giga R1 M7** (STM32H747, dual-core). The robot navigates a maze autonomously, detects floor colors and victims, handles ramps/stairs, and deploys rescue kits.

## Build & Flash Commands

```bash
# Build the project
pio run

# Build and upload to the board
pio run --target upload

# Clean build artifacts
pio run --target clean

# Open serial monitor
pio device monitor --baud 115200
```

There are no automated tests. After any major change, run `pio run` to verify the build succeeds. Full behavior verification requires flashing to hardware with `pio run --target upload`.

## Architecture

### Entry Point & State Machine (`src/main.cpp`)

The main loop runs two nested state machines:

- **`RobotState`** (menu/mode): `BOOT` → `RUN`, plus `SETTINGS`, `INFO_SENSOR`, `INFO_VISUAL`, `BT`, `ABOUT`, `CALIBRATION`, `CHECKPOINT`
- **`RunState`** (execution): `INITIAL` → `SETTILE` → `GET_INSTRUCTIONS` → `TURN`/`CHECK_DRIVE`/`DRIVE`/`RAMP`/`SCAN` → `SETTILE` (cyclic), plus `CHECKPOINT_RESET`, `ALIGN`, `END`

The **black button** (pin 49, ISR) starts a run or triggers a checkpoint reset. The **gray button** (pin 51, ISR) cycles drive speed modes via the UI.

`cyclicMainTask()` runs every loop iteration (ToF update, UI update, color sensor update).  
`cyclicRunTask()` runs only during `RUN` state (camera update, black tile handling, bumper handling, slow-speed logic).

### Library Structure (`lib/`)

Each library lives in `lib/<Name>/src/`. All share types from `CustomDatatypes`.

| Library | Purpose |
|---|---|
| `CustomDatatypes` | All shared enums/structs: `RobotState`, `RunState`, `ErrorCodes`, `TileType`, `Orientations`, `GyroData`, `PID_Coefficients`, etc. **Read this first.** |
| `Mapping` | A* pathfinding on a 3D tile grid (up to 256 tiles). Outputs `Instructionset` commands (turn/drive/ramp). Handles checkpoints and multi-level ramps. |
| `Driving` | High-level motion: PID wall-following, turn control, ramp detection/traversal, bumper avoidance. Depends on all sensors + `Mapping`. |
| `Motor` | Low-level `Motor` class and `Drivetrain` (two motors + encoder ISR). |
| `TofSensors` | Aggregates all ToF sensors (VL53L4CD for all 6 directional sensors, VL53L5CX 8×8 array for ramp detection). Returns wall bitmask for `Mapping`. Note: `TofVL6180X` class exists in the source but is not currently instantiated. |
| `ColorSensing` | AS7341 spectral sensor for floor type detection (`TileType`: checkpoint, blue, black, dangerZone). Stores calibration in FRAM via EEPROM. |
| `Gyro` | BNO055 IMU wrapper. Provides absolute/relative angles and `GetAngleFromOrientation()` for cardinal directions. |
| `Ejector` | Two servo-driven rescue kit dispensers (left pin 4, right pin 7). Tracks remaining kits in nibble-packed byte. |
| `UserInterface` | GigaDisplay 800×480 touchscreen, NeoPixel LEDs (18px, pin 8), buzzer (pin 58). Displays boot log, map, popups, and drive mode. |
| `Vcameras` | Serial communication (UART) with left/right cameras (Raspberry Pi or OpenMV) for victim detection. Triggers `Ejector` on detection. |
| `BLE` | Bluetooth LE interface (ArduinoBLE). |
| `SerialSetup` | Serial port initialization helpers. |

### Key Design Patterns

- **`ErrorCodes` enum** is used as a multipurpose return type across all modules — not just for errors but also for directional/state signals (`left`, `right`, `up`, `down`, `TURNED`, `CHECK_DRIVE`, etc.). Check `CustomDatatypes.h` before adding new values.
- **`#pragma region`** blocks guarded by `#ifdef _MSC_VER` are used throughout for Visual Studio code folding — they have no effect on compilation.
- **Pointer injection**: modules receive pointers to their dependencies via `Init()` calls in `main.cpp`. No global singletons except the objects declared at file scope in `main.cpp`.
- **Wall bitmask convention** (`SetTile`): bit 0 = front, bit 1 = right, bit 2 = behind, bit 3 = left (relative to robot orientation). `Mapping` converts to absolute before storing.

### Hardware Pin Reference (key pins)

| Pin | Use |
|---|---|
| 4 | Servo left ejector |
| 7 | Servo right ejector |
| 8 | NeoPixel LEDs |
| 40/42 | Camera left INT/RST |
| 41/43 | Camera right INT/RST |
| 45/47 | Bumper right/left |
| 49 | Button black (start/checkpoint) |
| 51 | Button gray (drive mode) |
| 56 | LED pin |
| 58 | Buzzer |
| Serial2 (D18) | Camera left UART |
| Serial3 (D19) | Camera right UART |
| Wire | I2C bus (ToF, Gyro, Color, EEPROM) |
| Wire1 | I2C bus (Display) |

### ToF Sensor Map

All 6 VL53L4CD sensors share the same default I2C address at power-on. `TofSensors::Init()` disables all via XSHUT, then brings them up one at a time to assign unique addresses. Knowing which XSHUT pin maps to which sensor is essential for debugging init failures.

| Sensor | Role | Type | XSHUT Pin | I2C Address |
|---|---|---|---|---|
| `leftFront` | Left side, front | VL53L4CD | A2 | 0x64 |
| `leftBack` | Left side, back | VL53L4CD | A5 | 0x68 |
| `rightFront` | Right side, front | VL53L4CD | A3 | 0x6C |
| `rightBack` | Right side, back | VL53L4CD | A4 | 0x70 |
| `front` | Front wall | VL53L4CD | A7 | 0x74 |
| `back` | Back wall | VL53L4CD | A6 | 0x78 |
| `front_x64` | Front ramp detection (8×8) | VL53L5CX | 32 | 0x46 |
| `back_x64` | Back ramp detection (8×8) | VL53L5CX | 26 | 0x47 |

## Agent Workflow Guidelines

When writing a **complete new class or library** (header + implementation), spawn a review agent afterward that runs `/check-style` and `fix-jsdoc` before committing.

When making **changes to a shared interface** (especially `CustomDatatypes.h`, or any `lib/` header that other modules include), spawn a second agent to check which other libraries are affected and whether they need updates. Cross-library ripple effects are easy to miss manually.

For **bug fixes, small tweaks, or iterative hardware debugging**, work directly — the agent overhead is not worth it.

Rule of thumb: if the change touches more than one library or introduces a new public interface, evaluate whether a Ruflo agent would catch something you'd otherwise miss.

## Known Issues (pending fixes)

- **ISR race condition** (`main.cpp`): `currentMenuState`, `currentRunState`, `_CHECKPOINT` are written in ISRs but not declared `volatile`. At higher optimization levels the compiler may cache them in registers and the main loop will never see ISR writes.
- **Ramp direction tautology** (`main.cpp`, `SCAN` state): `if(robot.GetCurrentRobotHeight() < robot.GetCurrentRobotHeight() + robot.GetRampHeight())` always reduces to `0 < GetRampHeight()` — `GetCurrentRobotHeight()` cancels out. Should be `if (robot.GetRampHeight() > 0)` / `< 0`.
- **Dead flag** (`main.cpp`): `_ROBOT_TURNING` is set in all four `GET_INSTRUCTIONS` turn cases but never read anywhere.
- **`ErrorCodes` god-enum** (`CustomDatatypes.h`): mixes error states, cardinal directions, popup severity, ejector state, and checkpoint flow control in one enum. Long-term maintenance risk.

## Driving Library — Pending Work

### Phase 5 — PID controller rewrite
Ziegler-Nichols is used **offline** to derive the P/I/D constants; those constants are then baked into the code as `static constexpr` values. The runtime loop uses a properly measured `dt` (actual elapsed milliseconds, not a fixed assumption). Anti-windup is included (integral clamped when output saturates). **Behavior will intentionally change — tune on hardware after implementation.**
