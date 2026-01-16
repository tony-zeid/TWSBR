#ifndef MESSAGES_H
#define MESSAGES_H

    #include <Arduino.h>
    #include <SoftwareSerial.h>
    #include "pins.h"

    // Prints messages to serial monitor & Bluetooth
    void printMsg(const char message[]);

    // Prints integer variables to serial monitor & Bluetooth
    void printInt(const char variable[], int *varPtr, byte offset);
    
    // Prints float variables to serial monitor & Bluetooth
    void printFloat(const char variable[], float *varPtr, byte offset);

#endif