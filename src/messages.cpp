#include "../include/messages.h"

// Messages formatted for Arduino serial plotter

// Bluetooth features temporarily disabled due to issues with 
// defining SoftwareSerial in multiple locations 

// Toggle Bluetooth messages
bool BTdebug_on = 0;

// Prints messages to serial monitor & Bluetooth
void print_msg(char message[]){
    Serial.println(message);

    if(BTdebug_on == 1){
        //BTSerial.println(message);
    }
}

// Prints integer variables to serial monitor & Bluetooth
void print_int(char variable[], int *VARptr, byte offset){
    Serial.print(variable); 
    Serial.print(VARptr[offset]);
    Serial.print(",");
    
    if(BTdebug_on == 1){
        //BTSerial.print(variable); 
        //BTSerial.print(VARptr[offset]); 
        //BTSerial.print(",");
    }
}

// Prints float variables to serial monitor & Bluetooth
void print_float(char variable[], float *VARptr, byte offset){
    Serial.print(variable); 
    Serial.print(VARptr[offset]); 
    Serial.print(",");
    
    if(BTdebug_on == 1){
        //BTSerial.print(variable); 
        //BTSerial.print(VARptr[offset]); 
        //BTSerial.print(",");
    }
}