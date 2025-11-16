#ifndef BLUETOOTH_H
#define BLUETOOTH_H

    #include <Arduino.h>
    #include <Wire.h>
    #include <SoftwareSerial.h>
    #include <string.h>
    #include "pins.h"

    // Passes control parameters only
    float *get_pos_param();
    float *get_bal_param();
    float *get_hdg_param();

    void read_BTSerial();
    // Communicates with remote device
    // Calls update_var() passing command

    void update_var();
    // Interprets command and updates variable

#endif