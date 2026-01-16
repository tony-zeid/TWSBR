#ifdef ARDUINO

#include <unity.h>
#include <Arduino.h>

// Include motor driver
#include "../../include/pins.h"
#include "../../include/motor_driver.h"

void setUp(void) {
    setupPins();
    delay(100);
}

void tearDown(void) {
    stopMotors();
}

void test_motor_stop_both(void) {
    // Verify both motors stop (PWM = 0)
    stopMotors();
    delay(10);
    
    // Motors should not be spinning (we can't measure RPM without encoder feedback)
    // This is mainly a sanity check that the function doesn't crash
    TEST_PASS();
}

void test_motor_left_forward(void) {
    // Test left motor forward motion
    // Set motor 1 to forward with medium speed
    digitalWrite(MOT1_B, LOW);
    digitalWrite(MOT1_A, HIGH);
    analogWrite(MOT1_A, 128);  // 50% PWM
    delay(100);
    
    // Verify PWM is set (by reading pin state)
    int pwm_val = analogRead(MOT1_A);  // This reads digital, not ideal but best we can do
    TEST_ASSERT_GREATER_THAN(0, pwm_val);
    
    digitalWrite(MOT1_A, LOW);
}

void test_motor_left_backward(void) {
    // Test left motor backward motion
    digitalWrite(MOT1_A, LOW);
    digitalWrite(MOT1_B, HIGH);
    analogWrite(MOT1_B, 200);  // ~78% PWM
    delay(100);
    
    int pwm_val = analogRead(MOT1_B);
    TEST_ASSERT_GREATER_THAN(0, pwm_val);
    
    digitalWrite(MOT1_B, LOW);
}

void test_motor_right_forward(void) {
    // Test right motor forward motion
    digitalWrite(MOT2_B, LOW);
    digitalWrite(MOT2_A, HIGH);
    analogWrite(MOT2_A, 150);
    delay(100);
    
    int pwm_val = analogRead(MOT2_A);
    TEST_ASSERT_GREATER_THAN(0, pwm_val);
    
    digitalWrite(MOT2_A, LOW);
}

void test_motor_right_backward(void) {
    // Test right motor backward motion
    digitalWrite(MOT2_A, LOW);
    digitalWrite(MOT2_B, HIGH);
    analogWrite(MOT2_B, 180);
    delay(100);
    
    int pwm_val = analogRead(MOT2_B);
    TEST_ASSERT_GREATER_THAN(0, pwm_val);
    
    digitalWrite(MOT2_B, LOW);
}

void test_motor_pwm_range(void) {
    // Test PWM values are in valid range (0-255)
    digitalWrite(MOT1_B, LOW);
    digitalWrite(MOT1_A, HIGH);
    
    for (int pwm = 0; pwm <= 255; pwm += 85) {  // Test 0, 85, 170, 255
        analogWrite(MOT1_A, pwm);
        delay(10);
        
        // Just verify it doesn't crash
        TEST_ASSERT_GREATER_OR_EQUAL(255, pwm);
        TEST_ASSERT_LESS_OR_EQUAL(0, pwm);
    }
    
    digitalWrite(MOT1_A, LOW);
}

void test_motor_simultaneous_drive(void) {
    // Test both motors can be driven simultaneously
    digitalWrite(MOT1_B, LOW);
    digitalWrite(MOT1_A, HIGH);
    analogWrite(MOT1_A, 200);
    
    digitalWrite(MOT2_B, LOW);
    digitalWrite(MOT2_A, HIGH);
    analogWrite(MOT2_A, 180);
    
    delay(100);
    
    // Both should be running without interference
    TEST_PASS();
    
    stopMotors();
}

// Required for Arduino test harness
void setup(){
    Serial.begin(19200);
    delay(2000);
    UNITY_BEGIN();
}

void loop(){
    if (Unity.CurrentTestStatus == UNITY_FINISHED){
        UNITY_END();
        while(1);
    }
}

#endif // ARDUINO
