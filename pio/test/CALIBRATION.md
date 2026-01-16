# Hardware Calibration Procedure

This guide covers calibrating the IMU, motor encoders, and motor directions for the TWSBR.

## IMU Calibration

The IMU (MPU6050) requires calibration to measure steady-state accelerometer and gyroscope offsets. These are subtracted from all readings to center the measurements around zero.

### Procedure

1. **Physical Setup**
   - Place the robot on a flat, level surface
   - Keep it completely **still and level** (±2° tilt acceptable)
   - Ensure nothing is vibrating or moving

2. **Power On & Boot**
   - Upload firmware: `platformio run -e uno --target upload`
   - Open Serial Monitor at **19200 baud**: `platformio device monitor -b 19200`
   - Wait for startup messages:
     ```
     [INIT] Serial
     [INIT] IMU Start
     [INIT] IMU Scale
     [INIT] IMU Config
     [INFO] IMU Errors
     AccErrX: <value>
     AccErrY: <value>
     GyrErrX: <value>
     GyrErrY: <value>
     GyrErrZ: <value>
     ```

3. **Verify Calibration**
   - Accelerometer errors (AccErrX, AccErrY) should be **< 0.5g** (ideally near zero)
   - Gyroscope errors (GyrErrX/Y/Z) should be **< 5 deg/s** (ideally near zero)
   - If errors are large, check:
     - Robot is truly level (use a bubble level if available)
     - No vibrations from nearby fans, motors, etc.
     - MPU6050 is rigidly mounted (not loose)

4. **Good Calibration Example**
   ```
   [INFO] IMU Errors
   AccErrX: 0.12
   AccErrY: -0.08
   GyrErrX: 1.2
   GyrErrY: -0.8
   GyrErrZ: 2.1
   ```

5. **If Calibration Fails**
   - Errors > 1g or > 10 deg/s suggest:
     - **Tilted placement**: Use a level; tilt introduces accel bias
     - **Vibration**: Move robot away from speakers, fans, running motors
     - **Sensor malfunction**: Verify I²C wiring (SDA/SCL) and 3.3V power
     - **Bad solder joint**: Check MPU6050 module for loose connections

## Encoder Calibration

Encoders measure motor speed and direction. Proper calibration ensures accurate position control.

### Procedure

1. **Pin Verification**
   - Verify encoder connections in `include/pins.h`:
     ```cpp
     #define ENC1_A D3  // Motor 1, Channel A
     #define ENC1_B D13 // Motor 1, Channel B
     #define ENC2_A D2  // Motor 2, Channel A
     #define ENC2_B D12 // Motor 2, Channel B
     ```
   - Check physical wiring to motor modules

2. **Direction Test**
   - Set robot in **mode 0** (measure only): `mode 0`
   - Watch encoder outputs in verbose mode: `debug 1` and `rate 100`
   - Manually spin motor 1 forward; verify encoder counts increase
   - Manually spin motor 1 backward; verify encoder counts decrease
   - Repeat for motor 2

3. **If Direction Is Reversed**
   - Swap encoder A and B channels in the motor module (physical rewiring)
   - OR invert in firmware by swapping the interrupt handlers (`inc1()` / `inc2()`) if needed

4. **If No Counts Register**
   - Check encoder power (usually 5V on motor module)
   - Verify I/O pins are not shorted or floating
   - Test with `digitalWrite()` and `digitalRead()` on the encoder pins
   - Ensure interrupt is attached correctly in `setupPins()`

## Motor Direction Calibration

Motors must spin in opposite directions for balance. If both spin the same way, the robot cannot correct tilt.

### Procedure

1. **Set Mode & Disable Motors**
   - Set `mode 1` (run controllers but don't drive motors yet)
   - Lift wheels off the ground

2. **Manual Direction Check**
   - Tilt the robot forward slightly in your hand
   - Observe the motor PWM signals (verbose telemetry shows motor commands)
   - Motor 1 should want to drive **forward** (pitch error is positive)
   - Motor 2 should want to drive **backward**
   - If reversed, swap motor A and B direction pins or reverse firmware logic

3. **Automated Test (Optional)**
   - Run hardware test: `platformio test -e uno -v`
   - Tests verify motor PWM can be applied without error

4. **If Motor Directions Are Wrong**
   - Physically swap the power leads (swap + and − terminals) on one motor, OR
   - Modify firmware to invert the PWM pins for that motor

## Control Parameter Calibration

Once hardware is calibrated, tune PID gains (see [README.md](../../README.md#tuning-the-control-loops) for detailed steps).

### Quick Start

1. **Balance Loop (Critical)**
   - Start with `balkp 0.5`, `balki 0.05`, `balkd 0.02`
   - Tilt robot gently; watch it oscillate and settle
   - Increase Kp until response is fast but not oscillating
   - Increase Ki slightly if drift persists
   - Increase Kd if high-frequency noise appears

2. **Position Loop**
   - Usually works well if balance is tuned
   - Adjust `poskp` and `poski` only if forward/backward creep is visible

3. **Heading Loop**
   - Minimal tuning; `hdgkp 1.0` often sufficient
   - Adjust if robot consistently turns left/right

## Testing & Verification

### Run Hardware Tests
```bash
# Compile and run motor/encoder/IMU tests on Arduino
platformio test -e uno -v
```

Tests will verify:
- Motor PWM and direction control
- Encoder pins are readable
- IMU initialization and calibration
- Calibration values are within bounds

### Soak Test
After calibration:
1. Set `mode 2` (full operation)
2. Lift wheels and let robot balance in your hand for 1–2 minutes
3. Monitor telemetry; observe for:
   - Smooth, stable response (no oscillation)
   - Consistent roll/pitch/yaw (no drift)
   - No watchdog resets or crashes

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Robot falls forward/backward immediately | Poor balance tuning or wrong motor direction | Increase Kp slowly; verify motors spin correctly |
| Robot oscillates and diverges | Kp too high or Ki accumulating | Reduce Kp by 50%; add anti-windup or cap Ki |
| Motors don't respond to commands | Motor driver not powered or pins misconfigured | Check 12V supply to motor driver; verify pin definitions |
| Encoders stuck at zero | Encoder pins not connected or interrupt not firing | Check physical wiring; use `digitalWrite`/`digitalRead` to test |
| IMU errors very large (>1g accel, >10 deg/s gyro) | Poor calibration or sensor malfunction | Re-calibrate with robot level; check I²C wiring |
| Yaw drifts continuously | Heading loop not tuned or gyro Z bias large | Increase `hdgki` slightly; re-calibrate IMU |

## Next Steps

After successful calibration:
1. **Field test** with wheels on ground (lift slightly to avoid sharp turns)
2. **Tuning refinement** using compact telemetry mode (`debug 0`, `rate 10`)
3. **Parameter persistence** — parameters are saved to EEPROM automatically (`saveParametersToEEPROM()`)
4. **Bluetooth control** (optional) — same commands work over HC-05 at 9600 baud
