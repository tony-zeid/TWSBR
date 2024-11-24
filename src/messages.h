#ifndef MESSAGES_H
#define MESSAGES_H

    #include <Arduino.h>
    #include <SoftwareSerial.h>
    
    #include "pins.h"

    void print_msg(char message[]);

    void print_int(char variable[], int *VARptr, byte offset);
    
    void print_float(char variable[], float *VARptr, byte offset);

#endif