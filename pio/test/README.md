# TWSBR Test Suite

This directory contains tests for the TWSBR firmware.

## Test Organization

All tests are in the `test/` folder with naming conventions:
- **test_\*.cpp** — Native PC tests (command parsing, control math, validation)
- **test_hw_\*.cpp** — Arduino Uno hardware tests (EEPROM, motors, encoders, IMU)
- **test_support.\*** — Shared test infrastructure for native tests

## Running Tests

### Native tests only (PC)
```bash
platformio test -e native -v
```

### Hardware tests only (requires Arduino Uno)
```bash
platformio test -e uno -v
```

### Specific test
```bash
platformio test -e uno -f test_hw_motors -v
platformio test -e native -f test_command -v
```

## Native Test Suite

Tests for command parsing, control logic, and parameter validation—run on PC without hardware.

### Test Files

- **test_command.cpp** — command parsing, mode changes, parameter updates
- **test_control.cpp** — PID math, motor clamping, numerical stability
- **test_validation.cpp** — bounds checking, realistic ranges
- **test_runner.cpp** — unified test entry point with setUp/tearDown
- **test_support.h/cpp** — shared mocks and helpers (Arduino types, String, command logic)

### Coverage

- **Command Parser** (~12 tests): mode, debug, rate, PID parameter updates, error handling
- **Control Logic** (~8 tests): proportional/integral/derivative response, motor output clamping
- **Parameter Validation** (~5 tests): gain ranges, setpoint bounds, mode validity

## Uno Test Suite

Tests for hardware integration and EEPROM persistence—run on Arduino Uno.

### Test Files

- **test_hw_eeprom.cpp** — EEPROM save/load, parameter persistence, validation
- **test_hw_motors.cpp** — motor control, PWM, direction, simultaneous drive
- **test_hw_encoders.cpp** — encoder pin initialization, state reading
- **test_hw_imu.cpp** — IMU initialization, sensitivity config, error calibration

### Coverage

- **EEPROM** (~11 tests): save/load runMode/debugMode/dataRate/PID gains, corruption recovery, roundtrip
- **Motors** (~7 tests): forward/backward, PWM range, simultaneous drive, stop
- **Encoders** (~2 tests): pin initialization, state reading
- **IMU** (~5 tests): initialization, sensitivity config, error calibration, data bounds

### Calibration

See [CALIBRATION.md](CALIBRATION.md) for comprehensive hardware calibration procedures:
- **IMU calibration**: level placement, error bounds, troubleshooting
- **Encoder calibration**: direction verification, pin testing
- **Motor direction**: balance verification, reversal procedures
- **PID tuning**: balance loop, position loop, heading loop
- **Verification**: hardware tests, soak testing

## Future Additions (ideas)
- Hardware smoke on `env:uno`: build/link and a tiny runtime check that default `runMode`/`debugMode` are sane.
- Compact-output contract: assert `debug 0` emits `roll pitch yaw\n` (spaces, trailing newline, no commas/brackets).
- Telemetry rate behavior: verify `rate` only updates the active `debugMode` slot and `printData` toggles correctly over multiple cycles.
- Command fuzzing: randomized long/short/malformed commands to ensure parser never corrupts state or crashes.
- Bluetooth/serial parity (hardware): commands over BT behave identically to serial, both verbose and compact.
- Soak test: multi-minute loop run (motors disabled or wheels lifted) to catch resets or memory creep.

## Example: Adding a New Test

```cpp
void test_new_feature(void) {
    // Arrange
    int input = 42;
    int expected = 84;
    
    // Act
    int result = double(input);
    
    // Assert
    TEST_ASSERT_EQUAL_INT(expected, result);
}
```

## Continuous Integration

To run tests as part of your build pipeline:
```bash
platformio ci --lib="./lib" --lib="./include" --project-conf=platformio.ini
```

## Troubleshooting

- **Test won't compile**: Ensure mocks for `Serial`, `printMsg`, and control functions match your actual signatures
- **Floating point assertions fail**: Use `TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, actual)` for comparisons
- **Extern variable errors**: Declare `extern` variables at the top of test files; ensure they match main.cpp definitions
