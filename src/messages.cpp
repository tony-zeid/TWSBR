#include "messages.h"

// Messages formatted for Arduino serial plotter

// Bluetooth features temporarily disabled due to issues with 
// defining SoftwareSerial in multiple locations 

// Toggle debug messages and pause time
bool debug_on = 0;
bool BTdebug_on = 0;
bool msgPause = 1;          
int pauseTime = 100;
// Toggle data monitoring
bool monitor_on = 0;
bool BTmonitor_on = 0;
// Data message speed set in main with dataRate

void print_msg(char message[]){
    if(debug_on == 1){
        Serial.println(message);
        delay(pauseTime * msgPause);
    }
    if(BTdebug_on == 1){
        //BTSerial.println(message);
        delay(pauseTime * msgPause);
    }
}

void print_int(char variable[], int *VARptr, byte offset){
    if(monitor_on == 1){
        Serial.print(variable); Serial.print(*(VARptr + offset)); Serial.print(",");
    }
    if(BTmonitor_on == 1){
        //BTSerial.print(variable); BTSerial.print(*(VARptr + offset)); BTSerial.print(",");
    }
}

void print_float(char variable[], float *VARptr, byte offset){
    if(monitor_on == 1){
        Serial.print(variable); Serial.print(*(VARptr + offset)); Serial.print(",");
    }
    if(BTmonitor_on == 1){
        //BTSerial.print(variable); BTSerial.print(*(VARptr + offset)); BTSerial.print(",");
    }
}