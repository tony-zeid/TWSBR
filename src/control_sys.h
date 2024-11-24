#ifndef CONTROL_SYS_H
#define CONTROL_SYS_H

    #include <Arduino.h>
    
    #include "pins.h"

    int *PID_control(float *IMUptr, float *CONptr, int printOut);

    void inc1();

    void inc2();
    
#endif