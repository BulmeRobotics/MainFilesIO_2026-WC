# MainFilesIO 2026

Firmware for the BulmeRobotics RoboCup Junior Rescue Maze robot (2025/26 season).
Forked to further implement the rules for the World Cup.

Runs on an **Arduino Giga R1 M7** (STM32H747 dual-core). The robot navigates a maze autonomously, detects floor colors and victims, handles ramps, and deploys rescue kits.

[![PlatformIO Build](https://github.com/BulmeRobotics/MainFilesIO_2026/actions/workflows/build.yml/badge.svg)](https://github.com/BulmeRobotics/MainFilesIO_2026/actions/workflows/build.yml)

## Build

Requires [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Flash to board
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor --baud 115200
```

## Architecture

The main loop runs two nested state machines:

- **RobotState** (menu/mode): `BOOT` → `RUN`, plus `SETTINGS`, `INFO_SENSOR`, `INFO_VISUAL`, `BT`
- **RunState** (execution): `INITIAL` → `SETTILE` → `GET_INSTRUCTIONS` → `TURN`/`CHECK_DRIVE`/`DRIVE`/`RAMP`/`SCAN` → `SETTILE` (cyclic)

## Libraries (`lib/`)

| Library | Purpose |
|---|---|
| `CustomDatatypes` | Shared enums/structs used across all modules |
| `Mapping` | A* pathfinding on a 3D tile grid, checkpoint handling |
| `Driving` | PID wall-following, turn control, ramp traversal, bumper avoidance |
| `Motor` | Low-level motor and drivetrain control with encoder ISR |
| `TofSensors` | VL6180X, VL53L4CD, VL53L5CX (8×8) aggregation; returns wall bitmask |
| `ColorSensing` | AS7341 spectral sensor for floor type detection (checkpoint, blue, black, danger zone) |
| `Gyro` | BNO055 IMU wrapper for absolute/relative heading |
| `Ejector` | Servo-driven rescue kit dispensers (left + right) |
| `UserInterface` | GigaDisplay 800×480 touchscreen, NeoPixel LEDs, buzzer |
| `Vcameras` | UART communication with victim-detection cameras, triggers ejector |
| `BLE` | Bluetooth LE interface |
| `SerialSetup` | Serial port initialization helpers |

## Hardware

**Board:** Arduino Giga R1 M7

| Pin | Use |
|---|---|
| 4 / 7 | Servo left / right ejector |
| 8 | NeoPixel LEDs (18px) |
| 45 / 47 | Bumper right / left |
| 49 | Black button (start / checkpoint reset) |
| 51 | Gray button (drive mode cycle) |
| 58 | Buzzer |
| Serial2 (D18) | Camera left UART |
| Serial3 (D19) | Camera right UART |
| Wire | I2C — ToF, Gyro, Color, EEPROM |
| Wire1 | I2C — Display |
