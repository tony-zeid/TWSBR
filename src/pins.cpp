#include "../include/pins.h"

void setup_pins(){
    // Motors
    pinMode(mot1A, OUTPUT); // Motor 1, phase A
    pinMode(mot1B, OUTPUT); // Motor 1, phase B
    pinMode(mot2A, OUTPUT); // Motor 2, phase A
    pinMode(mot2B, OUTPUT); // Motor 2, phase B
    
    // Encoders
    pinMode(enp1A, INPUT);  // Motor 1 encoder, phase A
    pinMode(enp1B, INPUT);  // Motor 1 encoder, phase B
    pinMode(enp2A, INPUT);  // Motor 2 encoder, phase A
    pinMode(enp2B, INPUT);  // Motor 2 encoder, phase B
    
    // Bluetooth
    pinMode(rxPin, INPUT);  // Bluetooth RX
    pinMode(txPin, OUTPUT); // Bluetooth TX

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(enp1A), inc1, RISING);    // Motor 1 encoder
    attachInterrupt(digitalPinToInterrupt(enp2A), inc2, RISING);    // Motor 2 encoder
}