#ifndef CONTROL_SYS_H
#define CONTROL_SYS_H

    #include <Arduino.h>
    #include "pins.h"
    #include "messages.h"
    
    /***************************************************************/
    // PID Gain Constants - Tunable parameters
    /***************************************************************/
    
    // BALANCE CONTROLLER GAINS (most critical for stability)
    // Recommended starting values: Kp=2.0, Ki=0.1, Kd=0.5
    #define BAL_KP      1.0f   // Proportional gain - increase for faster response
    #define BAL_KI      0.0f   // Integral gain - prevents steady-state error
    #define BAL_KD      0.0f   // Derivative gain - damping, reduces oscillation
    #define BAL_SETPOINT 0.0f  // Desired pitch angle (degrees)
    
    // POSITION CONTROLLER GAINS (forward/backward movement)
    // Recommended starting values: Kp=0.5, Ki=0.05, Kd=0.2
    #define POS_KP      0.0f   // Proportional gain
    #define POS_KI      0.0f   // Integral gain
    #define POS_KD      0.0f   // Derivative gain
    #define POS_SETPOINT 0.0f  // Desired velocity (m/s or arbitrary units)
    
    // HEADING CONTROLLER GAINS (yaw rotation)
    // Recommended starting values: Kp=1.0, Ki=0.05, Kd=0.3
    #define HDG_KP      0.0f   // Proportional gain
    #define HDG_KI      0.0f   // Integral gain
    #define HDG_KD      0.0f   // Derivative gain
    #define HDG_SETPOINT 0.0f  // Desired heading (degrees)
    
    /***************************************************************/
    
    // Individual PID controllers for cascade
    float position_control(float *IMUptr, float *POSptr_param, bool printData);
    float balance_control(float *IMUptr, float *BALpos_param, float POS_out, bool printData);
    float heading_control(float *IMUptr, float *HDGptr_param, bool printData);

    // Runs PID cascade control system
    int *cascade_control(float *IMUptr, float BAL_out, float HDG_out, bool printData);

    // Inrements motor position upon interrupt
    void inc1();    // Motor 1
    void inc2();    // Motor 2
    
#endif