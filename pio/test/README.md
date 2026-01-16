# TWSBR Native Test Suite

This directory contains unit tests for the TWSBR firmware. Tests run on the native PC environment (not on the Arduino) using the Unity test framework.

## Test Files

- **test_command_parser.cpp**: Command parsing, mode changes, parameter updates, edge cases
- **test_control_logic.cpp**: PID calculations, motor output clamping, control symmetry
- **test_parameter_validation.cpp**: Bounds checking, realistic ranges, numerical stability

## Running Tests

### All tests
```bash
platformio test -e native
```

### Specific test file
```bash
platformio test -e native -f test_command_parser
```

### With verbose output
```bash
platformio test -e native -v
```

## Test Coverage

### Command Parser Tests (~35 tests)
- Mode transitions (valid 0, 1, 2; invalid inputs)
- Debug mode switching
- Telemetry rate setting per mode
- PID parameter updates (all 12 gains + setpoints)
- Status command
- Error handling (unknown commands, malformed input, whitespace)
- Case insensitivity
- Parameter bounds (zero, negative, very large values)

### Control Logic Tests (~20 tests)
- Proportional response to pitch/position errors
- Integral accumulation over time
- Derivative damping on rapid changes
- Motor output clamping (0–255 PWM range)
- Control sensitivity (Kp scaling)
- Symmetry (+ and − errors produce opposite outputs)

### Parameter Validation Tests (~30 tests)
- PID gain ranges (realistic tuning space)
- Setpoint bounds (position, balance, heading)
- IMU measurement ranges (roll, pitch, yaw)
- Telemetry rate bounds
- Motor PWM and direction pin ranges
- Mode/state bounds
- Numerical stability (no NaN/Inf)

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
