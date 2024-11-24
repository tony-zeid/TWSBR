#include "motor_driver.h"

// Motor deadzone compensation
int mot_DZ = 60;

void run_motors(int *MOTptr){
    /*
    // Retreive actuation signal
    int mot1_spd = *(MOTptr + 0);
    int mot2_spd = *(MOTptr + 1);
    int mot1_dir = *(MOTptr + 2);
    int mot2_dir = *(MOTptr + 3);
    */

    switch (*(MOTptr + 2)){
        case 0:
        // Drive Motor1 Forward
        analogWrite(mot1A, *(MOTptr + 0) + mot_DZ);    
        break;
        case 1:
        // Drive Motor1 Reverse
        analogWrite(mot1B, *(MOTptr + 0) + mot_DZ);
        break;
        default: 
        // Failsafe
        analogWrite(mot1A, 0);
        analogWrite(mot1B, 0);
        analogWrite(mot2A, 0);
        analogWrite(mot2B, 0);
        print_msg("DIRECTION FAULT!");
    }

    switch (*(MOTptr + 3)){
        case 0:
        // Drive Motor2 Forward
        analogWrite(mot2A, *(MOTptr + 1) + mot_DZ);    
        break;
        case 1:
        // Drive Motor2 Reverse
        analogWrite(mot2B, *(MOTptr + 1) + mot_DZ);    
        break;
        default: 
        // Failsafe
        analogWrite(mot1A, 0);
        analogWrite(mot1B, 0);
        analogWrite(mot2A, 0);
        analogWrite(mot2B, 0);
        print_msg("DIRECTION FAULT!");
    }
}