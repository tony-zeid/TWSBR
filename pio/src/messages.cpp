#include "../include/messages.h"

// Messages formatted for Arduino serial plotter

// Bluetooth features temporarily disabled

// Toggle  messages
bool msgs_on = 1;
bool BTmsgs_on = 0;

// Prints messages to serial monitor & Bluetooth
void print_msg(const char message[]){
    if(msgs_on == 1){
        Serial.println(message);
    }
    if(BTmsgs_on == 1){
        //BTSerial.println(message);
    }
}

// Prints integer variables to serial monitor & Bluetooth
void print_int(const char variable[], int *VARptr, byte offset){
    if(msgs_on == 1){
        Serial.print(variable); 
        Serial.print(VARptr[offset]);
        Serial.print(",");
    }
    if(BTmsgs_on == 1){
        //BTSerial.print(variable); 
        //BTSerial.print(VARptr[offset]); 
        //BTSerial.print(",");
    }
}

// Prints float variables to serial monitor & Bluetooth
void print_float(const char variable[], float *VARptr, byte offset){
    if(msgs_on == 1){
        Serial.print(variable); 
        Serial.print(VARptr[offset]); 
        Serial.print(",");
    }
    
    if(BTmsgs_on == 1){
        //BTSerial.print(variable); 
        //BTSerial.print(VARptr[offset]); 
        //BTSerial.print(",");
    }
}