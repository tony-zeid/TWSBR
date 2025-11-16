#ifndef PINS_H
#define PINS_H

    #include <Arduino.h>
    #include "control_sys.h"

    // Motors
    #define     mot1A       6       // Motor 1, phase A
    #define     mot1B       5       // Motor 1, phase B
    #define     mot2A       10      // Motor 2, phase A
    #define     mot2B       11      // Motor 2, phase B

    // Encoders
    #define     enp1A       3       // Motor 1 encoder, phase A
    #define     enp1B       13      // Motor 1 encoder, phase B
    #define     enp2A       2       // Motor 2 encoder, phase A  
    #define     enp2B       12      // Motor 2 encoder, phase B

    // Bluetooth
    #define     rxPin       7       // Bluetooth RX
    #define     txPin       8       // Bluetooth TX

    // Setup I/O pin modes and interrupts
    void setup_pins();

#endif