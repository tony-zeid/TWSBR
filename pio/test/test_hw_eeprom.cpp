#ifdef ARDUINO

#include <unity.h>
#include <Arduino.h>
#include <EEPROM.h>

// Extern declarations from main.cpp and command.cpp
extern uint8_t runMode;
extern uint8_t debugMode;
extern uint16_t dataRatePerMode[2];
extern float posControls[4];
extern float balControls[4];
extern float hdgControls[4];

// Function declarations
void saveParametersToEEPROM();
void loadParametersFromEEPROM();
float *getPosParam();
float *getBalParam();
float *getHdgParam();

// Helper to clear EEPROM test region
void clearTestEEPROM(){
    for (int i = 0; i < 54; i++){
        EEPROM.write(i, 0xFF);
    }
}

void setUp(void){
    clearTestEEPROM();
    // Reset to defaults
    runMode = 2;
    debugMode = 0;
    dataRatePerMode[0] = 10;
    dataRatePerMode[1] = 500;
    posControls[0] = 0; posControls[1] = 1; posControls[2] = 0.05f; posControls[3] = 0.02f;
    balControls[0] = 0; balControls[1] = 2; balControls[2] = 0.1f; balControls[3] = 0.03f;
    hdgControls[0] = 0; hdgControls[1] = 1; hdgControls[2] = 0; hdgControls[3] = 0;
}

void tearDown(void){}

// Test: Save and load runMode
void test_eeprom_save_load_runmode(void){
    runMode = 1;
    saveParametersToEEPROM();
    
    runMode = 99;  // Corrupt
    loadParametersFromEEPROM();
    
    TEST_ASSERT_EQUAL_UINT8(1, runMode);
}

// Test: Save and load debugMode
void test_eeprom_save_load_debugmode(void){
    debugMode = 1;
    saveParametersToEEPROM();
    
    debugMode = 99;  // Corrupt
    loadParametersFromEEPROM();
    
    TEST_ASSERT_EQUAL_UINT8(1, debugMode);
}

// Test: debugMode validation (clamp to 0-1)
void test_eeprom_load_validates_debugmode(void){
    // Write invalid debugMode to EEPROM
    EEPROM.write(1, 5);  // Invalid: should be 0 or 1
    loadParametersFromEEPROM();
    
    TEST_ASSERT_LESS_OR_EQUAL(1, debugMode);
    TEST_ASSERT_GREATER_OR_EQUAL(0, debugMode);
}

// Test: runMode validation (clamp to 0-2)
void test_eeprom_load_validates_runmode(void){
    // Write invalid runMode to EEPROM
    EEPROM.write(0, 99);  // Invalid: should be 0-2
    loadParametersFromEEPROM();
    
    TEST_ASSERT_LESS_OR_EQUAL(2, runMode);
    TEST_ASSERT_GREATER_OR_EQUAL(0, runMode);
}

// Test: Save and load dataRatePerMode[0]
void test_eeprom_save_load_datarate_0(void){
    dataRatePerMode[0] = 123;
    saveParametersToEEPROM();
    
    dataRatePerMode[0] = 0;  // Corrupt
    loadParametersFromEEPROM();
    
    TEST_ASSERT_EQUAL_UINT16(123, dataRatePerMode[0]);
}

// Test: Save and load dataRatePerMode[1]
void test_eeprom_save_load_datarate_1(void){
    dataRatePerMode[1] = 456;
    saveParametersToEEPROM();
    
    dataRatePerMode[1] = 0;  // Corrupt
    loadParametersFromEEPROM();
    
    TEST_ASSERT_EQUAL_UINT16(456, dataRatePerMode[1]);
}

// Test: Save and load position PID gains
void test_eeprom_save_load_pos_gains(void){
    float *pos = getPosParam();
    pos[0] = 0.5f;
    pos[1] = 1.5f;
    pos[2] = 0.1f;
    pos[3] = 0.05f;
    
    saveParametersToEEPROM();
    
    // Corrupt
    pos[0] = 999.0f;
    pos[1] = 999.0f;
    pos[2] = 999.0f;
    pos[3] = 999.0f;
    
    loadParametersFromEEPROM();
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, pos[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.5f, pos[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f, pos[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.05f, pos[3]);
}

// Test: Save and load balance PID gains
void test_eeprom_save_load_bal_gains(void){
    float *bal = getBalParam();
    bal[0] = 1.0f;
    bal[1] = 2.5f;
    bal[2] = 0.2f;
    bal[3] = 0.04f;
    
    saveParametersToEEPROM();
    
    // Corrupt
    bal[0] = 888.0f;
    bal[1] = 888.0f;
    bal[2] = 888.0f;
    bal[3] = 888.0f;
    
    loadParametersFromEEPROM();
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, bal[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.5f, bal[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.2f, bal[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.04f, bal[3]);
}

// Test: Save and load heading PID gains
void test_eeprom_save_load_hdg_gains(void){
    float *hdg = getHdgParam();
    hdg[0] = 0.0f;
    hdg[1] = 0.8f;
    hdg[2] = 0.01f;
    hdg[3] = 0.1f;
    
    saveParametersToEEPROM();
    
    // Corrupt
    hdg[0] = 777.0f;
    hdg[1] = 777.0f;
    hdg[2] = 777.0f;
    hdg[3] = 777.0f;
    
    loadParametersFromEEPROM();
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, hdg[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.8f, hdg[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.01f, hdg[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f, hdg[3]);
}

// Test: Multiple saves don't corrupt data
void test_eeprom_multiple_saves(void){
    float *pos = getPosParam();
    
    // First save
    pos[1] = 1.1f;
    saveParametersToEEPROM();
    
    // Second save
    pos[1] = 2.2f;
    saveParametersToEEPROM();
    
    // Third save
    pos[1] = 3.3f;
    saveParametersToEEPROM();
    
    // Verify last value persists
    pos[1] = 999.0f;
    loadParametersFromEEPROM();
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.3f, pos[1]);
}

// Test: All parameters together
void test_eeprom_all_parameters_roundtrip(void){
    // Set all parameters
    runMode = 2;
    debugMode = 1;
    dataRatePerMode[0] = 100;
    dataRatePerMode[1] = 200;
    
    float *pos = getPosParam();
    float *bal = getBalParam();
    float *hdg = getHdgParam();
    
    pos[0] = 0.1f; pos[1] = 1.1f; pos[2] = 0.11f; pos[3] = 0.12f;
    bal[0] = 0.2f; bal[1] = 2.2f; bal[2] = 0.22f; bal[3] = 0.23f;
    hdg[0] = 0.3f; hdg[1] = 1.3f; hdg[2] = 0.32f; hdg[3] = 0.33f;
    
    saveParametersToEEPROM();
    
    // Corrupt all
    runMode = 0;
    debugMode = 0;
    dataRatePerMode[0] = 1;
    dataRatePerMode[1] = 1;
    pos[0] = pos[1] = pos[2] = pos[3] = 0;
    bal[0] = bal[1] = bal[2] = bal[3] = 0;
    hdg[0] = hdg[1] = hdg[2] = hdg[3] = 0;
    
    loadParametersFromEEPROM();
    
    // Verify all restored
    TEST_ASSERT_EQUAL_UINT8(2, runMode);
    TEST_ASSERT_EQUAL_UINT8(1, debugMode);
    TEST_ASSERT_EQUAL_UINT16(100, dataRatePerMode[0]);
    TEST_ASSERT_EQUAL_UINT16(200, dataRatePerMode[1]);
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f, pos[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.1f, pos[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.2f, bal[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.3f, hdg[1]);
}

// Required for Arduino test harness
void setup(){
    Serial.begin(9600);
    delay(2000);  // Wait for serial port to initialize
    UNITY_BEGIN();
}

void loop(){
    if (Unity.CurrentTestStatus == UNITY_FINISHED){
        UNITY_END();
        while (1);
    }
}
#endif // ARDUINO
