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
- Status
	- `status` — print a simple OK message

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
2. `imuConfigSensitivity()` — sets accel/gyro full‑scale ranges
3. `imuCalculateError()` — samples stationary offsets for accel/gyro

Keep the robot completely still and level during initialisation. The measured errors are printed once.

## Code Conventions
- Functions/variables: lowerCamelCase (e.g. `runMode`, `setupPins`, `balanceControl`)
- Constants/macros: UPPER_SNAKE_CASE (e.g. `MOT1_A`, `IMU_GYRO_DATA`)
- Types: PascalCase

## Safety Notes
- Lift the wheels off the ground during first tests.
- Ensure power rails are stable; brown‑outs can cause runaway behaviour.
- The motor driver outputs are clamped to 0–255 PWM; direction pins clear before switching for safety.

## Tuning the Control Loops

### Overview
The robot uses a cascaded control structure: **Position** → **Balance** → **Motors**. Tuning follows a bottom‑up approach starting with balance (most critical), then position and heading.

### Balance Loop (Critical)
The balance loop corrects tilt (pitch) to keep the robot upright. **Start here first.**

1. **Lift wheels off the ground** and hold robot level in your hand.
2. Set `mode 1` (controllers run, but motors off) to test without movement.
3. Switch to compact telemetry: `debug 0` and `rate 10` to see roll/pitch at 10‑cycle intervals.
4. Start with very small Kp: `balkp 0.5`
5. Tilt robot gently forward/backward and watch the IMU pitch value respond:
   - Pitch should oscillate around zero as you release it (increasing Kp will dampen overshoot)
   - Aim for slow decay without oscillation
6. Increase Kp incrementally (e.g., 0.5 → 1.0 → 1.5 → 2.0) until you see stable response
7. Once Kp is tuned, add small Ki to eliminate steady‑state error:
   - Start with `balki 0.05` and increase if drift persists (e.g., 0.1, 0.15)
8. Fine‑tune Kd if oscillations appear:
   - `balkd 0.02` helps dampen high‑frequency noise from the IMU

**Typical starting gains**: `balkp 2.0`, `balki 0.1`, `balkd 0.03`

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
```
# 1. Start with balance loop (mode 1, no motor drive)
debug 1
mode 1
balkp 1.0

# 2. Switch to compact telemetry for faster feedback
debug 0
rate 10

# 3. Adjust gains live (robot in hand, level)
balkp 1.5
balki 0.1
balkd 0.02

# 4. Once stable (in hand), try with wheels on ground
mode 2

# 5. Collect verbose data if tuning feels rough
debug 1
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
