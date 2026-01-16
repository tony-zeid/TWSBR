# Two‑Wheeled Self‑Balancing Robot (TWSBR)

PID‑controlled, two‑wheeled self‑balancing robot built on Arduino Uno with an MPU6050 IMU, motor encoders, and optional Bluetooth for parameter updates. 

## Features
- Balance via cascaded PID controllers (position → balance → heading)
- MPU6050 sensor fusion (complementary filter) for roll/pitch/yaw
- Quadrature encoder support with interrupts per motor
- Modular drivers for motors, pins, messaging, and Bluetooth
- PlatformIO project for reproducible build and upload

## Hardware
- Arduino Uno (ATmega328P)
- 2 × DC gear motors with quadrature encoders
- Dual H‑bridge motor driver (e.g. L298N/L9110S/compatible)
- IMU: MPU6050 (I²C)
- Optional HC‑05/HC‑06 Bluetooth module
- 7–12V battery pack with suitable regulator for logic and motors

### Pinout (Arduino Uno)
- Motors (PWM/direction)
	- MOT1_A: D6, MOT1_B: D5
	- MOT2_A: D10, MOT2_B: D11
- Encoders (interrupt channels)
	- ENC1_A: D3, ENC1_B: D13
	- ENC2_A: D2, ENC2_B: D12
- Bluetooth (optional)
	- BT_RX_PIN: D7, BT_TX_PIN: D8
- I²C (MPU6050)
	- SDA: A4, SCL: A5

Refer to the constants in include/pins.h if you change wiring.

## Wiring Table
| Subsystem | Signal | Arduino Pin | Constant | Notes |
|---|---|---|---|---|
| Motor driver | Motor 1 A (PWM/dir) | D6 | MOT1_A | Connect to driver input for M1 phase A |
| Motor driver | Motor 1 B (PWM/dir) | D5 | MOT1_B | Connect to driver input for M1 phase B |
| Motor driver | Motor 2 A (PWM/dir) | D10 | MOT2_A | Connect to driver input for M2 phase A |
| Motor driver | Motor 2 B (PWM/dir) | D11 | MOT2_B | Connect to driver input for M2 phase B |
| Encoder 1 | Channel A (INT1) | D3 | ENC1_A | Attach to encoder A; interrupt on rising edge |
| Encoder 1 | Channel B | D13 | ENC1_B | Direction sense |
| Encoder 2 | Channel A (INT0) | D2 | ENC2_A | Attach to encoder A; interrupt on rising edge |
| Encoder 2 | Channel B | D12 | ENC2_B | Direction sense |
| Bluetooth (optional) | Module TX → Uno | D7 | BT_RX_PIN | HC‑05 TX to Uno D7 |
| Bluetooth (optional) | Module RX ← Uno | D8 | BT_TX_PIN | Use resistor divider for HC‑05 RX |
| IMU MPU6050 | SDA | A4 | — | I²C data |
| IMU MPU6050 | SCL | A5 | — | I²C clock |
| IMU MPU6050 | VCC | 3.3V/5V | — | Depends on breakout (often 5V tolerant) |
| IMU/Logic | GND | GND | — | Common ground for all modules |

## Project Structure
- include/
	- IMU_functions.h — IMU addresses and public IMU API
	- bluetooth.h — Bluetooth helpers (initialise/read)
	- command.h — command handling and parameter getters
	- control_sys.h — PID gains and control loop APIs
	- messages.h — serial/Bluetooth printing utilities
	- motor_driver.h — motor actuation API
	- pins.h — pin definitions and `setupPins()`
- src/
	- IMU_functions.cpp — IMU initialisation, sensitivity config, readout, error calibration
	- bluetooth.cpp — parameter getters and Bluetooth stubs
	- control_sys.cpp — PID controllers and cascade to motor outputs
	- main.cpp — application entrypoint, run modes, telemetry
	- messages.cpp — printing helpers
	- motor_driver.cpp — PWM and direction control with fail‑safes
	- pins.cpp — pin modes and interrupt attachment
- platformio.ini — board and framework configuration

## Build, Upload, Monitor
This project uses PlatformIO. In VS Code with the PlatformIO extension:

```bash
platformio run                 # Build
platformio run --target upload # Upload to Arduino Uno
platformio device monitor -b 19200 # Open serial monitor
```

Serial baud is 19200 (required for stable MPU6050 operation in this project). Bluetooth baud is 9600 (HC‑05/HC‑06 default).

## Command Interface
The firmware accepts simple newline‑terminated commands from either the Serial Monitor (19200 baud) or the Bluetooth link (9600 baud). Commands update run mode, debug mode, PID parameters, and telemetry rate at runtime.

### Commands
- Run mode
	- `mode 0` — measure only
	- `mode 1` — run controllers (no motor drive)
	- `mode 2` — full operation (drive motors)
- Debug/print mode
	- `debug 0` — compact output (roll pitch yaw, space‑separated)
	- `debug 1` — verbose output (all diagnostics and parameters)
- Telemetry rate
	- `rate <cycles>` — print every N cycles (applies to current debug mode independently)
- Position PID
	- `poskp <value>` — proportional gain
	- `poski <value>` — integral gain
	- `poskd <value>` — derivative gain
	- `posset <value>` — setpoint
- Balance PID
	- `balkp <value>` — proportional gain
	- `balki <value>` — integral gain
	- `balkd <value>` — derivative gain
	- `balset <value>` — setpoint
- Heading PID
	- `hdgkp <value>` — proportional gain
	- `hdgki <value>` — integral gain
	- `hdgkd <value>` — derivative gain
	- `hdgset <value>` — setpoint
- IMU Calibration
	- `cal` — calibrate IMU (robot upright on wheels)
	- `cal_inv` — calibrate IMU (robot upside-down on flat top)
- Status
	- `status` — print a simple OK message

**Note:** All parameter changes (PID gains, modes, rates) and calibration results are automatically saved to EEPROM and persist across reboots.

### Examples
```
# Serial Monitor – verbose with default rate (every 500 cycles):
debug 1
mode 2
balkp 2.0

# Serial Monitor – compact output for tuning (every 10 cycles):
debug 0
rate 10
mode 2

# Bluetooth – switch to verbose, adjust a parameter:
debug 1
poskp 0.5
rate 100
```

## Run Modes & Debug Modes
`runMode` controls algorithm behaviour:
- 0 — Measure only (reads IMU and prints diagnostics)
- 1 — Run controllers (compute control signals, do not drive motors)
- 2 — Drive motors (full operation with motor control)

`debugMode` controls telemetry output:
- 0 — **Compact**: prints roll, pitch, yaw only (space‑separated, one per line) – useful for live tuning via terminal or data logging
- 1 — **Verbose**: prints all parameters, control signals, motor outputs, and diagnostics – useful for development and debugging

Each debug mode has its own independent `dataRate` (cycles per print). Use `rate` command to tune telemetry frequency per mode.

## Control Parameters
PID gains live in include/control_sys.h:
- Balance (critical): `BAL_KP`, `BAL_KI`, `BAL_KD`, `BAL_SETPOINT`
- Position: `POS_KP`, `POS_KI`, `POS_KD`, `POS_SETPOINT`
- Heading: `HDG_KP`, `HDG_KI`, `HDG_KD`, `HDG_SETPOINT`

At runtime the firmware exposes read‑only parameter blocks via:
- `getPosParam()`, `getBalParam()`, `getHdgParam()`

Bluetooth hooks (`readBtSerial()`, `updateVar()`) are scaffolded for future use but disabled by default (SoftwareSerial lines commented). To experiment, enable the commented SoftwareSerial initialisation and message prints in src/main.cpp and src/messages.cpp.

## IMU Setup and Calibration
On startup the firmware:
1. `imuInitialise()` — wakes the IMU and resets it
2. `imuConfigSensitivity()` — sets accel/gyro full‑scale ranges (configurable via `ACCEL_FSR` and `GYRO_FSR` in `include/IMU_functions.h`)
3. Calibration is now done via command, not automatically on boot

### Manual Calibration Commands
**Normal calibration (robot upright on wheels):**
```
cal
```
Keep the robot completely still and level for 2 seconds. Errors are saved to EEPROM and persist across reboots.

**Upside-down calibration (robot on flat top):**
```
cal_inv
```
Place robot on its flat top surface, level and still. This inverts accelerometer error measurements so the robot works correctly when flipped right-side-up. After calibration, flip the robot and restart for normal operation.

### Configuring IMU Full-Scale Ranges
Edit `include/IMU_functions.h` to change sensor ranges:
- `ACCEL_FSR`: 0 (±2g), 1 (±4g), 2 (±8g), 3 (±16g)
- `GYRO_FSR`: 0 (±250°/s), 1 (±500°/s), 2 (±1000°/s), 3 (±2000°/s)

Divisors update automatically when you change these values.

## Code Conventions
- Functions/variables: lowerCamelCase (e.g. `runMode`, `setupPins`, `balanceControl`)
- Constants/macros: UPPER_SNAKE_CASE (e.g. `MOT1_A`, `IMU_GYRO_DATA`)
- Types: PascalCase

## Safety Notes
- Lift the wheels off the ground during first tests.
- Ensure power rails are stable; brown‑outs can cause runaway behaviour.
- The motor driver outputs are clamped to 0–255 PWM; direction pins clear before switching for safety.

## Memory Usage and Optimization Notes

**Current RAM Usage: ~86% (1762/2048 bytes)**

The Arduino Uno has only 2KB of RAM, and this project uses:
- **~450 bytes**: Arduino libraries (Serial, Wire/I²C, SoftwareSerial)
- **~200 bytes**: Global variables (control arrays, IMU state, motor state)
- **~100-150 bytes**: EEPROM library internal buffers
- **~1000 bytes**: Arduino core overhead (timers, interrupts, system buffers)
- **~286 bytes free**: Available for stack during runtime

### Recent Optimizations
Several RAM optimizations have been applied to reduce memory pressure:

1. **String → char array conversion**: Replaced `String` objects with fixed-size `char[64]` buffers for serial/Bluetooth input (~60-80 bytes saved)
2. **Bit-packed mode flags**: Packed `runMode`, `debugMode`, and `printData` into a single byte using bit fields (2 bytes saved)
3. **Local variables for controller outputs**: Changed `posOut`, `balOut`, `hdgOut` from static globals to local variables (12 bytes saved)

**Total RAM savings: ~95 bytes** (from 95.3% → 86.0% usage)

### IMU Configuration
The IMU full-scale ranges are now configurable at compile-time in `include/IMU_functions.h`:
- `ACCEL_FSR`: 0-3 (±2g, ±4g, ±8g, ±16g) - currently set to 2 (±8g)
- `GYRO_FSR`: 0-3 (±250°/s, ±500°/s, ±1000°/s, ±2000°/s) - currently set to 2 (±1000°/s)

Divisors are automatically calculated based on these settings.

### EEPROM Library Impact
The Arduino EEPROM library (`#include <EEPROM.h>`) adds significant RAM overhead (~100-150 bytes) for its internal buffering and state management. While convenient for parameter persistence, this contributes to overall RAM usage.

**Future Optimization:**
If additional features are needed, consider implementing direct EEPROM access using `eeprom_read_byte()` and `eeprom_write_byte()` from `<avr/eeprom.h>` instead of the Arduino EEPROM library. This can save ~50-100 bytes of RAM at the cost of slightly more complex code.

**Current Status:**
With 286 bytes of free stack space, there is adequate headroom for runtime operations. The control loops avoid deep recursion and large local variables, keeping stack usage minimal.

## Tuning the Control Loops

### Overview
The robot uses a cascaded control structure: **Position** → **Balance** → **Motors**. Tuning follows a bottom‑up approach starting with balance (most critical), then position and heading.

### Balance Loop (Critical)
The balance loop corrects tilt (pitch) to keep the robot upright. **Start here first.**

#### Phase 1: Controller Sanity Check (Mode 1, wheels lifted)
1. **Lift wheels off the ground** and hold robot level
2. Set `mode 1` (controllers run, motors disabled) and `debug 1` (verbose output)
3. Start with small gains: `balkp 1.0`, `balki 0.0`, `balkd 0.0`
4. **Tilt robot forward** and observe the **motor command output** in telemetry:
   - Controller should output positive PWM (trying to drive forward to correct)
   - Larger tilt → larger PWM (proportional response)
5. **Tilt robot backward** and verify opposite response (negative/reverse motor command)
6. If signs are wrong, swap motor direction pins or invert controller logic

#### Phase 2: Initial Balance Test (Mode 2, wheels on ground)
7. **Place robot on flat surface** with wheels on ground
8. Set `mode 2` (full operation, motors enabled)
9. **Hold robot upright** and release gently:
   - Robot should attempt to balance (may fall at first)
   - Observe for oscillation, divergence, or stability
10. If robot falls immediately:
    - Reduce `balkp` by 50% and retry
    - Verify motor directions are correct (both should spin same direction when tilting forward)
11. If robot oscillates slowly (growing):
    - Increase `balkp` by 20–50%
12. If robot oscillates rapidly (high frequency):
    - Reduce `balkp` or add small `balkd` (start 0.02)

#### Phase 3: Fine-Tuning
13. Once robot balances briefly (even 1–2 seconds), tune for stability:
    - Increase `balkp` until oscillation just begins, then back off 10–20%
    - Add `balki` (0.05–0.1) only if robot drifts slowly in one direction
    - Add `balkd` (0.02–0.05) if high-frequency vibration appears
14. Use compact telemetry for real-time feedback: `debug 0`, `rate 10`
15. Test by gently pushing robot forward/backward; it should recover smoothly

**Typical starting gains**: `balkp 2.0`, `balki 0.1`, `balkd 0.03` (highly robot-dependent)

### IMU Drift Issue
If IMU pitch/roll drifts after movement and doesn't return to zero:
- **Re-calibrate IMU** (keep robot level and still during startup)
- **Check complementary filter** (currently in `IMU_functions.cpp`, typically 0.98–0.99 gyro weight)
- **Gyro drift** is normal over time; re-calibration or magnetometer fusion helps

### Position Loop (Once Balance Works)
Position loop maintains forward/backward centering. The balance loop naturally provides position feedback via roll change.

1. Switch to `mode 2` with wheels still lifted (motor PWM will run but wheels won't turn much).
2. Observe whether the robot tends to "creep" forward or backward in pitch.
3. Adjust `poskp` (start ~0.5–1.0) and `poski` (~0.05) if systematic drift appears.
4. Most robots need minimal position tuning if balance loop is good.

### Heading Loop (Fine‑Tuning)
Heading loop corrects yaw. Usually requires minimal tuning for forward motion.

1. With the robot on a flat surface (wheels on ground), switch to `mode 2`.
2. Watch yaw drift; if the robot consistently turns left/right, adjust `hdgkp` and `hdgki` slightly.
3. Most setups work well with `hdgkp 1.0`, `hdgki 0.0`, `hdgkd 0.0`.

### Practical Workflow
```bash
# 1. Verify controller sign (mode 1, wheels lifted)
debug 1
mode 1
balkp 1.0
balki 0.0
balkd 0.0
# Tilt forward/backward and observe motor commands in telemetry

# 2. Initial balance test (mode 2, wheels on ground)
mode 2
# Hold upright and release; adjust balkp until stable

# 3. Switch to compact telemetry for live tuning
debug 0
rate 10

# 4. Fine-tune gains
balkp 2.0      # Adjust until oscillation just stops
balki 0.1      # Add if slow drift appears
balkd 0.03     # Add if high-frequency vibration

# 5. If IMU drifts, re-calibrate by restarting with robot level
# (or adjust complementary filter in IMU_functions.cpp)
```
rate 100
```

### Monitoring Telemetry
- **Compact mode** (`debug 0`): Three space‑separated floats per line: roll pitch yaw. Pipe to a file for post‑analysis.
  ```bash
  # Example: log data for 30 seconds at 19200 baud
  cat /dev/ttyUSB0 > data.txt &
  # ... run tuning commands ...
  # Ctrl+C to stop
  gnuplot -e "plot 'data.txt' u 2" # Plot pitch over time
  ```
- **Verbose mode** (`debug 1`): Full diagnostics; easier for real‑time observation but slower telemetry.

### If Robot Oscillates or Diverges
- **High‑frequency oscillation**: Reduce Kd or Kp; IMU noise may be coupling back.
- **Low‑frequency / growing oscillation**: Reduce Ki; integral term may be accumulating too fast.
- **Divergent (unstable)**: Cut all gains by half and restart. Check motor/encoder wiring.

### If Robot Falls Sideways
- Verify IMU calibration: run `mode 0` and confirm roll ≈ 0 when level.
- Check that motors spin in the correct directions (balance loop commands opposite motors for correction).
- Verify encoder counts are correct (see Troubleshooting).

## Troubleshooting
- No serial output: check the monitor baud is 19200.
- IMU values stuck or wild: verify SDA/SCL wiring and 5V↔3.3V compatibility of your module.
- Encoders count backwards: swap encoder B signal or invert in `inc1()/inc2()`.
- Motors spin but no balance: start tuning with balance Kp small (e.g. 2.0) and Ki/Kd near zero, then iterate.

## Acknowledgements
IMU routines adapted from Dejan Nedelkovski (howtomechatronics.com).
