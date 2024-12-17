#include "motor_driver.h"

// Motor deadzone compensation
int mot_DZ = 50;

/*
// Motor control signal
mot1_spd = *(MOTptr + 0);
mot2_spd = *(MOTptr + 1);
mot1_dir = *(MOTptr + 2);
mot2_dir = *(MOTptr + 3);
*/

// Sets motor direction & PWM duty cycle
void run_motors(int *MOTptr){
    switch (*(MOTptr + 2)){
        case 0:
        // Drive Motor1 Forward
        analogWrite(mot1A, *(MOTptr + 0) + mot_DZ);    
        print_msg("Motor1 Forward");
        break;
        case 1:
        // Drive Motor1 Reverse
        analogWrite(mot1B, *(MOTptr + 0) + mot_DZ);
        print_msg("Motor1 Reverse");
        break;
        default: 
        // Failsafe
        analogWrite(mot1A, 0);
        analogWrite(mot1B, 0);
        analogWrite(mot2A, 0);
        analogWrite(mot2B, 0);
        print_msg("M1 DIRECTION FAULT!");
    }

    switch (*(MOTptr + 3)){
        case 0:
        // Drive Motor2 Forward
        analogWrite(mot2A, *(MOTptr + 1) + mot_DZ);    
        print_msg("Motor2 Forward");
        break;
        case 1:
        // Drive Motor2 Reverse
        analogWrite(mot2B, *(MOTptr + 1) + mot_DZ);   
        print_msg("Motor2 Reverse"); 
        break;
        default: 
        // Failsafe
        analogWrite(mot1A, 0);
        analogWrite(mot1B, 0);
        analogWrite(mot2A, 0);
        analogWrite(mot2B, 0);
        print_msg("M2 DIRECTION FAULT!");
    }
}