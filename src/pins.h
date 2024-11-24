#ifndef PINS_H
#define PINS_H

    #include <Arduino.h>

    #include "control_sys.h"

    // Motors
    #define     mot1A       6
    #define     mot1B       5
    #define     mot2A       10
    #define     mot2B       11

    // Encoders
    #define     enp1A       2
    #define     enp1B       12
    #define     enp2A       3
    #define     enp2B       13

    // Bluetooth
    #define     rxPin       7
    #define     txPin       8

    void setup_pins();

#endif