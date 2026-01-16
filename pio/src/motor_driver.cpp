#include "../include/motor_driver.h"

/*
// Motor control signal
mot1_spd = *(MOTptr + 0);
mot2_spd = *(MOTptr + 1);
mot1_dir = *(MOTptr + 2);
mot2_dir = *(MOTptr + 3);
*/

// Sets motor direction & PWM duty cycle
void runMotors(int *motorActuationPtr){
    // Motor 1 run control
    switch (*(motorActuationPtr + 2)){
        case 0:
        // Drive Motor1 Forward - clear reverse pin first (safety)
        analogWrite(MOT1_B, 0);
        analogWrite(MOT1_A, *(motorActuationPtr + 0));    
        break;
        case 1:
        // Drive Motor1 Reverse - clear forward pin first (safety)
        analogWrite(MOT1_A, 0);
        analogWrite(MOT1_B, *(motorActuationPtr + 0));
        break;
        default: 
        // Failsafe
        analogWrite(MOT1_A, 0);
        analogWrite(MOT1_B, 0);
        analogWrite(MOT2_A, 0);
        analogWrite(MOT2_B, 0);
        delay(20);
        printMsg("M1 DIRECTION FAULT!");
    }
    // Motor 2 run control
    switch (*(motorActuationPtr + 3)){
        case 0:
        // Drive Motor2 Forward - clear reverse pin first (safety)
        analogWrite(MOT2_B, 0);
        analogWrite(MOT2_A, *(motorActuationPtr + 1));    
        break;
        case 1:
        // Drive Motor2 Reverse - clear forward pin first (safety)
        analogWrite(MOT2_A, 0);
        analogWrite(MOT2_B, *(motorActuationPtr + 1));   
        break;
        default: 
        // Failsafe
        analogWrite(MOT2_A, 0);
        analogWrite(MOT2_B, 0);
        analogWrite(MOT1_A, 0);
        analogWrite(MOT1_B, 0);
        delay(20);
        printMsg("M2 DIRECTION FAULT!");
    }
}

// Motor test function - spins each motor independently for diagnostics
void testMotors(){
    int pwm_speed = 150;  // Medium speed for testing
    int test_duration = 1000;  // 1 second per direction
    
    printMsg("==================================================");
    printMsg("[TEST] Motor Diagnostics Starting");
    printMsg("");
    
    // Test Motor 1 Forward
    printMsg("[TEST] Motor 1 - Forward");
    analogWrite(MOT1_B, 0);
    analogWrite(MOT1_A, pwm_speed);
    delay(test_duration);
    
    // Test Motor 1 Reverse
    printMsg("[TEST] Motor 1 - Reverse");
    analogWrite(MOT1_A, 0);
    analogWrite(MOT1_B, pwm_speed);
    delay(test_duration);
    
    // Stop Motor 1
    analogWrite(MOT1_A, 0);
    analogWrite(MOT1_B, 0);
    printMsg("[TEST] Motor 1 - Stop");
    delay(500);
    
    // Test Motor 2 Forward
    printMsg("[TEST] Motor 2 - Forward");
    analogWrite(MOT2_B, 0);
    analogWrite(MOT2_A, pwm_speed);
    delay(test_duration);
    
    // Test Motor 2 Reverse
    printMsg("[TEST] Motor 2 - Reverse");
    analogWrite(MOT2_A, 0);
    analogWrite(MOT2_B, pwm_speed);
    delay(test_duration);
    
    // Stop Motor 2
    analogWrite(MOT2_A, 0);
    analogWrite(MOT2_B, 0);
    printMsg("[TEST] Motor 2 - Stop");
    delay(500);
    
    // All motors idle
    printMsg("");
    printMsg("[TEST] Motor Diagnostics Complete");
    printMsg("==================================================");
}