#include <unity.h>
#include "test_support.h"

// Declarations of tests from other translation units
void test_cmd_mode_valid_0(void);
void test_cmd_mode_valid_1(void);
void test_cmd_mode_valid_2(void);
void test_cmd_debug_0(void);
void test_cmd_debug_1(void);
void test_cmd_rate_valid(void);
void test_cmd_poskp(void);
void test_cmd_balkp(void);
void test_cmd_balki(void);
void test_cmd_status(void);
void test_cmd_unknown(void);
void test_cmd_empty(void);

void test_ctrl_balance_kp_positive(void);
void test_ctrl_balance_kp_negative(void);
void test_ctrl_zero_error(void);
void test_motor_clamp_high(void);
void test_motor_clamp_low(void);
void test_motor_clamp_valid(void);
void test_param_numerical_not_nan(void);
void test_param_numerical_not_inf(void);

void test_param_pid_gain_positive(void);
void test_param_setpoint_balance_range(void);
void test_param_imu_pitch_range(void);
void test_param_mode_valid_range(void);
void test_param_debug_valid_range(void);

void setUp(void) {
    resetTestState();
}

void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();

    // Command tests
    RUN_TEST(test_cmd_mode_valid_0);
    RUN_TEST(test_cmd_mode_valid_1);
    RUN_TEST(test_cmd_mode_valid_2);
    RUN_TEST(test_cmd_debug_0);
    RUN_TEST(test_cmd_debug_1);
    RUN_TEST(test_cmd_rate_valid);
    RUN_TEST(test_cmd_poskp);
    RUN_TEST(test_cmd_balkp);
    RUN_TEST(test_cmd_balki);
    RUN_TEST(test_cmd_status);
    RUN_TEST(test_cmd_unknown);
    RUN_TEST(test_cmd_empty);

    // Control tests
    RUN_TEST(test_ctrl_balance_kp_positive);
    RUN_TEST(test_ctrl_balance_kp_negative);
    RUN_TEST(test_ctrl_zero_error);
    RUN_TEST(test_motor_clamp_high);
    RUN_TEST(test_motor_clamp_low);
    RUN_TEST(test_motor_clamp_valid);
    RUN_TEST(test_param_numerical_not_nan);
    RUN_TEST(test_param_numerical_not_inf);

    // Validation tests
    RUN_TEST(test_param_pid_gain_positive);
    RUN_TEST(test_param_setpoint_balance_range);
    RUN_TEST(test_param_imu_pitch_range);
    RUN_TEST(test_param_mode_valid_range);
    RUN_TEST(test_param_debug_valid_range);

    return UNITY_END();
}
