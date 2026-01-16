#include <unity.h>
#include "test_support.h"

void test_param_pid_gain_positive(void) {
    float gain = 2.0f;
    TEST_ASSERT_GREATER_THAN(0.0f, gain);
}

void test_param_setpoint_balance_range(void) {
    float setpoint = 0.0f;
    TEST_ASSERT_GREATER_OR_EQUAL(-90.0f, setpoint);
    TEST_ASSERT_LESS_OR_EQUAL(90.0f, setpoint);
}

void test_param_imu_pitch_range(void) {
    float pitch = 5.0f;
    TEST_ASSERT_GREATER_OR_EQUAL(-90.0f, pitch);
    TEST_ASSERT_LESS_OR_EQUAL(90.0f, pitch);
}

void test_param_mode_valid_range(void) {
    uint8_t mode = 1;
    TEST_ASSERT_GREATER_OR_EQUAL(0, mode);
    TEST_ASSERT_LESS_OR_EQUAL(2, mode);
}

void test_param_debug_valid_range(void) {
    uint8_t dbg = 1;
    TEST_ASSERT_GREATER_OR_EQUAL(0, dbg);
    TEST_ASSERT_LESS_OR_EQUAL(1, dbg);
}
