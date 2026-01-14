#include "../include/motor_driver.h"

/*
// Motor control signal
mot1_spd = *(MOTptr + 0);
mot2_spd = *(MOTptr + 1);
mot1_dir = *(MOTptr + 2);
mot2_dir = *(MOTptr + 3);
*/

// Sets motor direction & PWM duty cycle
void run_motors(int *MOTptr_act){
    // Motor 1 run control
    switch (*(MOTptr_act + 2)){
        case 0:
        // Drive Motor1 Forward - clear reverse pin first (safety)
        analogWrite(mot1B, 0);
        analogWrite(mot1A, *(MOTptr_act + 0));    
        break;
        case 1:
        // Drive Motor1 Reverse - clear forward pin first (safety)
        analogWrite(mot1A, 0);
        analogWrite(mot1B, *(MOTptr_act + 0));
        break;
        default: 
        // Failsafe
        analogWrite(mot1A, 0);
        analogWrite(mot1B, 0);
        analogWrite(mot2A, 0);
        analogWrite(mot2B, 0);
        delay(20);
        print_msg("M1 DIRECTION FAULT!");
    }
    // Motor 2 run control
    switch (*(MOTptr_act + 3)){
        case 0:
        // Drive Motor2 Forward - clear reverse pin first (safety)
        analogWrite(mot2B, 0);
        analogWrite(mot2A, *(MOTptr_act + 1));    
        break;
        case 1:
        // Drive Motor2 Reverse - clear forward pin first (safety)
        analogWrite(mot2A, 0);
        analogWrite(mot2B, *(MOTptr_act + 1));   
        break;
        default: 
        // Failsafe
        analogWrite(mot2A, 0);
        analogWrite(mot2B, 0);
        analogWrite(mot1A, 0);
        analogWrite(mot1B, 0);
        delay(20);
        print_msg("M2 DIRECTION FAULT!");
    }
}

// Motor test function - spins each motor independently for diagnostics
void test_motors(){
    int pwm_speed = 150;  // Medium speed for testing
    int test_duration = 1000;  // 1 second per direction
    
    print_msg("==================================================");
    print_msg("[TEST] Motor Diagnostics Starting");
    print_msg("");
    
    // Test Motor 1 Forward
    print_msg("[TEST] Motor 1 - Forward");
    analogWrite(mot1B, 0);
    analogWrite(mot1A, pwm_speed);
    delay(test_duration);
    
    // Test Motor 1 Reverse
    print_msg("[TEST] Motor 1 - Reverse");
    analogWrite(mot1A, 0);
    analogWrite(mot1B, pwm_speed);
    delay(test_duration);
    
    // Stop Motor 1
    analogWrite(mot1A, 0);
    analogWrite(mot1B, 0);
    print_msg("[TEST] Motor 1 - Stop");
    delay(500);
    
    // Test Motor 2 Forward
    print_msg("[TEST] Motor 2 - Forward");
    analogWrite(mot2B, 0);
    analogWrite(mot2A, pwm_speed);
    delay(test_duration);
    
    // Test Motor 2 Reverse
    print_msg("[TEST] Motor 2 - Reverse");
    analogWrite(mot2A, 0);
    analogWrite(mot2B, pwm_speed);
    delay(test_duration);
    
    // Stop Motor 2
    analogWrite(mot2A, 0);
    analogWrite(mot2B, 0);
    print_msg("[TEST] Motor 2 - Stop");
    delay(500);
    
    // All motors idle
    print_msg("");
    print_msg("[TEST] Motor Diagnostics Complete");
    print_msg("==================================================");
}