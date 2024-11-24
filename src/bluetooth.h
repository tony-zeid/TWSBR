#ifndef BLUETOOTH_H
#define BLUETOOTH_H

    #include <Arduino.h>
    #include <Wire.h>
    #include <SoftwareSerial.h>
    #include <string.h>

    #include "pins.h"

    float *get_param();
    // Passes control parameters only

    void read_BTSerial();
    // Communicates with remote device
    // Calls update_var() passing command

    void update_var();
    // Interprets command and updates variable

#endif