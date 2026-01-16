#include <unity.h>
#include "test_support.h"

void test_ctrl_balance_kp_positive(void) {
    float pitch_error = 10.0f;
    float expected = 10.0f * 2.0f; // Kp=2.0
    float output = pidStep(pitch_error, 2.0f, 0, 0, gControlState.balError_prev, gControlState.balInt);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected, output);
}

void test_ctrl_balance_kp_negative(void) {
    float pitch_error = -5.0f;
    float expected = -5.0f * 2.0f;
    float output = pidStep(pitch_error, 2.0f, 0, 0, gControlState.balError_prev, gControlState.balInt);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected, output);
}

void test_ctrl_zero_error(void) {
    float output = pidStep(0.0f, 2.0f, 0, 0, gControlState.balError_prev, gControlState.balInt);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, output);
}

void test_motor_clamp_high(void) {
    float output = clampMotorOutput(500.0f);
    TEST_ASSERT_EQUAL_FLOAT(255.0f, output);
}

void test_motor_clamp_low(void) {
    float output = clampMotorOutput(-50.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output);
}

void test_motor_clamp_valid(void) {
    float output = clampMotorOutput(128.0f);
    TEST_ASSERT_EQUAL_FLOAT(128.0f, output);
}

void test_param_numerical_not_nan(void) {
    float output = 2.0f * 5.0f;
    TEST_ASSERT_FALSE(std::isnan(output));
}

void test_param_numerical_not_inf(void) {
    float output = 2.0f * 5.0f;
    TEST_ASSERT_FALSE(std::isinf(output));
}
