#include "../include/bluetooth.h"

// Control parameters
static float pos_controls[4] = {0}; // Position
static float bal_controls[4] = {0}; // Balance
static float hdg_controls[4] = {0}; // Heading

float *get_pos_param(){
    pos_controls[0] = 0;    // Setpoint Position
    pos_controls[1] = 1;    // Proportional Gain kP
    pos_controls[2] = 0;    // Integral Gain kI
    pos_controls[3] = 0;    // Derivative Gain kD
    return pos_controls;
}

float *get_bal_param(){
    bal_controls[0] = 0;    // Setpoint Pitch
    bal_controls[1] = 1;    // Proportional Gain kP
    bal_controls[2] = 0;    // Integral Gain kI
    bal_controls[3] = 0;    // Derivative Gain kD
    return bal_controls;
}

float *get_hdg_param(){
    hdg_controls[0] = 0;    // Setpoint Heading
    hdg_controls[1] = 1;    // Proportional Gain kP
    hdg_controls[2] = 0;    // Integral Gain kI
    hdg_controls[3] = 0;    // Derivative Gain kD
    return hdg_controls;
}

void read_BTSerial(){
    // Communicate with remote Bluetooth device
}   

void update_var(/*string command*/){
    // Update parameters based on Bluetooth command
}