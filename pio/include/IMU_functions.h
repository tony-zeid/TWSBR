#ifndef IMU_FUNCTIONS_H
#define IMU_FUNCTIONS_H

    #include <Arduino.h>
    #include <Wire.h>
    #include <math.h>

    // MPU6050 Addresses
    #define     IMU_addr        0x68    // I2C Address
    #define     IMU_reset       0x6B    // IMU reset 
    #define     IMU_accel_conf  0x1C    // IMU accel config
    #define     IMU_gyro_conf   0x1B    // IMU gyro config 
    #define     IMU_accel_data  0x3B    // IMU accel data begin
    #define     IMU_gyro_data   0x43    // IMU gyro data begin

    // Initialisation routines
    void IMU_initialise();          // Initialise I2C comms and reset IMU 
    void IMU_sens_config();         // Configures sensitivity of Accel and Gyro
    float *IMU_error_calc();        // Measure steady-state errors for compensation

    // Update IMU measurements, returns RPYT
    float *IMU_read_data();

#endif