#ifndef CONTROL_SYS_H
#define CONTROL_SYS_H

    #include <Arduino.h>
    
    #include "pins.h"
    #include "messages.h"
    
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