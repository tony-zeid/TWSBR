#ifndef PINS_H
#define PINS_H

    #include <Arduino.h>
    #include "control_sys.h"

    // Motors
    #define     MOT1_A      6       // Motor 1, phase A
    #define     MOT1_B      5       // Motor 1, phase B
    #define     MOT2_A      10      // Motor 2, phase A
    #define     MOT2_B      11      // Motor 2, phase B

    // Encoders
    #define     ENC1_A      3       // Motor 1 encoder, phase A
    #define     ENC1_B      13      // Motor 1 encoder, phase B
    #define     ENC2_A      2       // Motor 2 encoder, phase A  
    #define     ENC2_B      12      // Motor 2 encoder, phase B

    // Bluetooth
    #define     BT_RX_PIN   7       // Bluetooth RX
    #define     BT_TX_PIN   8       // Bluetooth TX

    // Setup I/O pin modes and interrupts
    void setupPins();

#endif