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
	- bluetooth.h — parameter accessors and Bluetooth helpers
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

Serial baud is 19200 (required for stable MPU6050 operation in this project).

## Run Modes
`runMode` in src/main.cpp controls behaviour:
- 0 — Measure only (reads IMU and prints diagnostics)
- 1 — Run controllers (compute control signals, do not drive motors)
- 2 — Drive motors (full operation)

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

## Troubleshooting
- No serial output: check the monitor baud is 19200.
- IMU values stuck or wild: verify SDA/SCL wiring and 5V↔3.3V compatibility of your module.
- Encoders count backwards: swap encoder B signal or invert in `inc1()/inc2()`.
- Motors spin but no balance: start tuning with balance Kp small (e.g. 2.0) and Ki/Kd near zero, then iterate.

## Acknowledgements
IMU routines adapted from Dejan Nedelkovski (howtomechatronics.com).
