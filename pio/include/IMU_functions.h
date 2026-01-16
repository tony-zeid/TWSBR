#ifndef IMU_FUNCTIONS_H
#define IMU_FUNCTIONS_H

    #include <Arduino.h>
    #include <Wire.h>
    #include <math.h>

    // MPU6050 Addresses
    #define     IMU_ADDR        0x68    // I2C Address
    #define     IMU_RESET       0x6B    // IMU reset 
    #define     IMU_ACCEL_CONF  0x1C    // IMU accel config
    #define     IMU_GYRO_CONF   0x1B    // IMU gyro config 
    #define     IMU_ACCEL_DATA  0x3B    // IMU accel data begin
    #define     IMU_GYRO_DATA   0x43    // IMU gyro data begin

    // Initialisation routines
    void imuInitialise();           // Initialise I2C comms and reset IMU 
    void imuConfigSensitivity();    // Configures sensitivity of Accel and Gyro
    float *imuCalculateError();     // Measure steady-state errors for compensation

    // Update IMU measurements, returns RPYT
    float *imuReadData();

#endif