#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

    #include <Arduino.h>
    #include "pins.h"
    #include "messages.h"

    // Sets motor direction & PWM duty cycle
    void runMotors(int *motorActuationPtr);

    // Test function for motor diagnostics - spins each motor independently
    void testMotors();

#endif