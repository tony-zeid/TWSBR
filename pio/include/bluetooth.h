#ifndef BLUETOOTH_H
#define BLUETOOTH_H

    #include <Arduino.h>
    #include <Wire.h>
    #include <SoftwareSerial.h>
    #include <string.h>
    #include "pins.h"

    // Initialise Bluetooth serial
    void initBtSerial();

    // Poll Bluetooth for incoming commands and process them
    void readBtSerial();

#endif