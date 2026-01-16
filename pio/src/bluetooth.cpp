#include "../include/bluetooth.h"
#include "../include/control_sys.h"

// Control parameters
static float posControls[4] = {0}; // Position
static float balControls[4] = {0}; // Balance
static float hdgControls[4] = {0}; // Heading

float *getPosParam(){
    posControls[0] = POS_SETPOINT;  // Setpoint Position
    posControls[1] = POS_KP;        // Proportional Gain kP
    posControls[2] = POS_KI;        // Integral Gain kI
    posControls[3] = POS_KD;        // Derivative Gain kD
    return posControls;
}

float *getBalParam(){
    balControls[0] = BAL_SETPOINT;  // Setpoint Pitch
    balControls[1] = BAL_KP;        // Proportional Gain kP
    balControls[2] = BAL_KI;        // Integral Gain kI
    balControls[3] = BAL_KD;        // Derivative Gain kD
    return balControls;
}

float *getHdgParam(){
    hdgControls[0] = HDG_SETPOINT;  // Setpoint Heading
    hdgControls[1] = HDG_KP;        // Proportional Gain kP
    hdgControls[2] = HDG_KI;        // Integral Gain kI
    hdgControls[3] = HDG_KD;        // Derivative Gain kD
    return hdgControls;
}

void readBtSerial(){
    // Communicate with remote Bluetooth device
}   

void updateVar(/*string command*/){
    // Update parameters based on Bluetooth command
}