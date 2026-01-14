#include "../include/bluetooth.h"
#include "../include/control_sys.h"

// Control parameters
static float pos_controls[4] = {0}; // Position
static float bal_controls[4] = {0}; // Balance
static float hdg_controls[4] = {0}; // Heading

float *get_pos_param(){
    pos_controls[0] = POS_SETPOINT;  // Setpoint Position
    pos_controls[1] = POS_KP;        // Proportional Gain kP
    pos_controls[2] = POS_KI;        // Integral Gain kI
    pos_controls[3] = POS_KD;        // Derivative Gain kD
    return pos_controls;
}

float *get_bal_param(){
    bal_controls[0] = BAL_SETPOINT;  // Setpoint Pitch
    bal_controls[1] = BAL_KP;        // Proportional Gain kP
    bal_controls[2] = BAL_KI;        // Integral Gain kI
    bal_controls[3] = BAL_KD;        // Derivative Gain kD
    return bal_controls;
}

float *get_hdg_param(){
    hdg_controls[0] = HDG_SETPOINT;  // Setpoint Heading
    hdg_controls[1] = HDG_KP;        // Proportional Gain kP
    hdg_controls[2] = HDG_KI;        // Integral Gain kI
    hdg_controls[3] = HDG_KD;        // Derivative Gain kD
    return hdg_controls;
}

void read_BTSerial(){
    // Communicate with remote Bluetooth device
}   

void update_var(/*string command*/){
    // Update parameters based on Bluetooth command
}