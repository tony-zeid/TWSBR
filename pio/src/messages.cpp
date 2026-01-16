#include "../include/messages.h"

// Messages formatted for Arduino serial plotter

// Bluetooth features temporarily disabled

// Toggle  messages
bool messagesOn = 1;
bool btMessagesOn = 0;

// Prints messages to serial monitor & Bluetooth
void printMsg(const char message[]){
    if(messagesOn == 1){
        Serial.println(message);
    }
    if(btMessagesOn == 1){
        //BTSerial.println(message);
    }
}

// Prints integer variables to serial monitor & Bluetooth
void printInt(const char variable[], int *varPtr, byte offset){
    if(messagesOn == 1){
        Serial.print(variable); 
        Serial.print(varPtr[offset]);
        Serial.print(",");
    }
    if(btMessagesOn == 1){
        //BTSerial.print(variable); 
        //BTSerial.print(varPtr[offset]); 
        //BTSerial.print(",");
    }
}

// Prints float variables to serial monitor & Bluetooth
void printFloat(const char variable[], float *varPtr, byte offset){
    if(messagesOn == 1){
        Serial.print(variable); 
        Serial.print(varPtr[offset]); 
        Serial.print(",");
    }
    
    if(btMessagesOn == 1){
        //BTSerial.print(variable); 
        //BTSerial.print(varPtr[offset]); 
        //BTSerial.print(",");
    }
}