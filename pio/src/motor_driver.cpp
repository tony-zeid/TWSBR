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
        // Drive Motor1 Forward
        analogWrite(mot1A, *(MOTptr_act + 0));    
        break;
        case 1:
        // Drive Motor1 Reverse
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
        // Drive Motor2 Forward
        analogWrite(mot2A, *(MOTptr_act + 1));    
        break;
        case 1:
        // Drive Motor2 Reverse
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