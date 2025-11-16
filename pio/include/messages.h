#ifndef MESSAGES_H
#define MESSAGES_H

    #include <Arduino.h>
    #include <SoftwareSerial.h>
    #include "pins.h"

    // Prints messages to serial monitor & Bluetooth
    void print_msg(char message[]);

    // Prints integer variables to serial monitor & Bluetooth
    void print_int(char variable[], int *VARptr, byte offset);
    
    // Prints float variables to serial monitor & Bluetooth
    void print_float(char variable[], float *VARptr, byte offset);

#endif