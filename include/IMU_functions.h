#ifndef IMU_FUNCTIONS_H
#define IMU_FUNCTIONS_H

    #include <Arduino.h>
    #include <Wire.h>
    #include <math.h>

    // MPU6050 I2C Address 
    #define     MPU         0x68

    // Initialisation routines
    void initialise_IMU();          // Initialise I2C comms and reset IMU 
    void configure_IMU_sens();      // Configures sensitivity of Accel and Gyro
    float *calculate_IMU_error();   // Measure steady-state errors for compensation

    // Update IMU measurements, returns RPYT
    float *read_IMU_data();

#endif