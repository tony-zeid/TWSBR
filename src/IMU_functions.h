#ifndef IMU_FUNCTIONS_H
#define IMU_FUNCTIONS_H

    #include <Arduino.h>
    #include <Wire.h>
    #include <math.h>

    // MPU Address 
    #define     MPU         0x68

    void initialise_IMU();

    void configure_IMU_sens();

    float *calculate_IMU_error();

    float *read_IMU_data(float *ERRptr);

#endif