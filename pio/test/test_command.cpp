#include <unity.h>
#include "test_support.h"

// Command tests
void test_cmd_mode_valid_0(void) { handleCommandLine("mode 0"); TEST_ASSERT_EQUAL_UINT8(0, runMode); }
void test_cmd_mode_valid_1(void) { handleCommandLine("mode 1"); TEST_ASSERT_EQUAL_UINT8(1, runMode); }
void test_cmd_mode_valid_2(void) { handleCommandLine("mode 2"); TEST_ASSERT_EQUAL_UINT8(2, runMode); }

void test_cmd_debug_0(void) { handleCommandLine("debug 0"); TEST_ASSERT_EQUAL_UINT8(0, debugMode); }
void test_cmd_debug_1(void) { handleCommandLine("debug 1"); TEST_ASSERT_EQUAL_UINT8(1, debugMode); }

void test_cmd_rate_valid(void) {
    debugMode = 0;
    handleCommandLine("rate 100");
    TEST_ASSERT_EQUAL_UINT16(100, dataRatePerMode[0]);
}

void test_cmd_poskp(void) { handleCommandLine("poskp 1.5"); TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.5f, posControls[1]); }
void test_cmd_balkp(void) { handleCommandLine("balkp 3.0"); TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, balControls[1]); }
void test_cmd_balki(void) { handleCommandLine("balki 0.2"); TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.2f, balControls[2]); }

void test_cmd_status(void) { handleCommandLine("status"); TEST_PASS(); }
void test_cmd_unknown(void) { uint8_t before = runMode; handleCommandLine("invalid_cmd"); TEST_ASSERT_EQUAL_UINT8(before, runMode); }
void test_cmd_empty(void) { uint8_t before = runMode; handleCommandLine(""); TEST_ASSERT_EQUAL_UINT8(before, runMode); }
