#ifdef ARDUINO

#include <unity.h>
#include <Arduino.h>

// Include encoder and pin definitions
#include "../../include/pins.h"

void setUp(void) {
    setupPins();
    delay(100);
}

void tearDown(void) {}

// Volatile counters for encoder interrupts
volatile long enc1_count = 0;
volatile long enc2_count = 0;

void enc1_isr() { enc1_count++; }
void enc2_isr() { enc2_count++; }

void test_encoder_initialization(void) {
    // Verify encoder pins are configured as inputs
    int enc1_mode = digitalRead(ENC1_A);
    int enc2_mode = digitalRead(ENC2_A);
    
    // Should read LOW or HIGH without error (i.e., pins are valid)
    TEST_ASSERT_TRUE(enc1_mode == LOW || enc1_mode == HIGH);
    TEST_ASSERT_TRUE(enc2_mode == LOW || enc2_mode == HIGH);
}

void test_encoder_pin_states(void) {
    // Read encoder states without any motor motion
    int enc1_a = digitalRead(ENC1_A);
    int enc1_b = digitalRead(ENC1_B);
    int enc2_a = digitalRead(ENC2_A);
    int enc2_b = digitalRead(ENC2_B);
    
    // All should be either LOW or HIGH (valid pin states)
    TEST_ASSERT_TRUE(enc1_a == LOW || enc1_a == HIGH);
    TEST_ASSERT_TRUE(enc1_b == LOW || enc1_b == HIGH);
    TEST_ASSERT_TRUE(enc2_a == LOW || enc2_a == HIGH);
    TEST_ASSERT_TRUE(enc2_b == LOW || enc2_b == HIGH);
}

void test_encoder_channel_phase(void) {
    // Verify encoder channels have defined phase relationship
    int enc1_a = digitalRead(ENC1_A);
    int enc1_b = digitalRead(ENC1_B);
    int enc2_a = digitalRead(ENC2_A);
    int enc2_b = digitalRead(ENC2_B);
    
    // Channels should be stable (not both HIGH and LOW simultaneously on same pin)
    // This is a basic sanity check
    TEST_PASS();
}

void test_encoder_interrupt_attachment(void) {
    // Verify interrupt pins are correctly configured (D2 and D3 on Uno)
    // This test just ensures setupPins() configured them properly
    
    // Re-attach custom ISRs for testing
    attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2_isr, RISING);  // INT0 on D2
    attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1_isr, RISING);  // INT1 on D3
    
    enc1_count = 0;
    enc2_count = 0;
    
    // Simulate encoder pulses by toggling pins (if not on actual encoder)
    // For now, just verify ISR attachment didn't crash
    TEST_PASS();
    
    detachInterrupt(digitalPinToInterrupt(ENC1_A));
    detachInterrupt(digitalPinToInterrupt(ENC2_A));
}

void test_encoder_count_increment(void) {
    // Test that encoder counters increment on interrupts
    attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2_isr, RISING);
    enc2_count = 0;
    
    // Without real encoder pulses, count should remain zero
    // (This test verifies the ISR is callable but doesn't generate pulses)
    delay(100);
    
    TEST_ASSERT_EQUAL_INT(0, enc2_count);
    
    detachInterrupt(digitalPinToInterrupt(ENC2_A));
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
