#include "../include/pins.h"

void setupPins(){
    // Motors
    pinMode(MOT1_A, OUTPUT); // Motor 1, phase A
    pinMode(MOT1_B, OUTPUT); // Motor 1, phase B
    pinMode(MOT2_A, OUTPUT); // Motor 2, phase A
    pinMode(MOT2_B, OUTPUT); // Motor 2, phase B
    
    // Encoders
    pinMode(ENC1_A, INPUT);  // Motor 1 encoder, phase A
    pinMode(ENC1_B, INPUT);  // Motor 1 encoder, phase B
    pinMode(ENC2_A, INPUT);  // Motor 2 encoder, phase A
    pinMode(ENC2_B, INPUT);  // Motor 2 encoder, phase B
    
    // Bluetooth
    pinMode(BT_RX_PIN, INPUT);  // Bluetooth RX
    pinMode(BT_TX_PIN, OUTPUT); // Bluetooth TX

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(ENC1_A), inc1, RISING);    // Motor 1 encoder
    attachInterrupt(digitalPinToInterrupt(ENC2_A), inc2, RISING);    // Motor 2 encoder
}