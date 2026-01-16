#ifndef BLUETOOTH_H
#define BLUETOOTH_H

    #include <Arduino.h>
    #include <Wire.h>
    #include <SoftwareSerial.h>
    #include <string.h>
    #include "pins.h"

    // Passes control parameters only
    float *getPosParam();
    float *getBalParam();
    float *getHdgParam();

    void readBtSerial();
    // Communicates with remote device
    // Calls updateVar() passing command

    void updateVar();
    // Interprets command and updates variable

#endif