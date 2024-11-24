#include "pins.h"

void setup_pins(){
    // Motors
    pinMode(mot1A, OUTPUT); 
    pinMode(mot1B, OUTPUT);
    pinMode(mot2A, OUTPUT);
    pinMode(mot2B, OUTPUT);
    
    // Encoders
    pinMode(enp1A, INPUT);   
    pinMode(enp1B, INPUT);
    pinMode(enp2A, INPUT);
    pinMode(enp2B, INPUT);
    
    // Bluetooth
    pinMode(rxPin, INPUT);   
    pinMode(txPin, OUTPUT); 

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(enp1A), inc1, RISING);
    attachInterrupt(digitalPinToInterrupt(enp2A), inc2, RISING);
}