#include "bluetooth.h"

// Control parameters
static float controls[5] = {0};

float *get_param(){
    controls[0] = 0; // Setpoint Angle
    controls[1] = 0; // Setpoint Poisiton           // Reserved for PID cascade, consider Yaw / steering?
    controls[2] = 40; // Proportional Gain kP
    controls[3] = 0.01; // Integral Gain kI
    controls[4] = 0.001; // Derivative Gain kD
    return controls;
}

void read_BTSerial(){
    // BT Communications
}

void update_var(/*string command*/){
    // Update commands
}