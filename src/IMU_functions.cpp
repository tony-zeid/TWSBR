/*
  IMU Functions adapted from tutorial by Dejan Nedelkovski at https://howtomechatronics.com
*/

#include "IMU_functions.h"

// Time markers
float currentTime = 0;
//float previousTime = 0;      
//float elapsedTime = 0;

// Angle measurements
float GyrAngX = 0;
float GyrAngY = 0;
//float AccAngX = 0;
//float AccAngY = 0;

float *read_IMU_data(float *ERRptr){
    // Output array
    // Roll, Pitch, Yaw, Time
    static float RPYT[4] = {0};
    // === Read acceleromter data === //
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    float AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    float AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    float AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
    // Calculating Roll and Pitch from the accelerometer data
    //AccAngX = ((atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - *ERRptr); // See the calculate_IMU_error()custom function for more details
    //AccAngY = ((atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - *(ERRptr + 1));

    // === Update timestamps === //
    float previousTime = currentTime; // Previous time is stored before the actual time read
    currentTime = millis(); // Current time actual time read
    float elapsedTime = (currentTime - previousTime) / 1000; // Converts back to seconds for calculation
    RPYT[3] = currentTime;

    // === Read gyroscope data === //
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
    float GyrX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    float GyrY = (Wire.read() << 8 | Wire.read()) / 131.0;
    float GyrZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    // Correct the outputs with the calculated error values
    GyrX = GyrX - *(ERRptr + 2);
    GyrY = GyrY - *(ERRptr + 3);
    GyrZ = GyrZ - *(ERRptr + 4);
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    GyrAngX = GyrAngX + GyrX * elapsedTime; // deg/s * s = deg
    GyrAngY = GyrAngY + GyrY * elapsedTime;
    RPYT[2] =  RPYT[2] + GyrZ * elapsedTime;

    // Complementary filter - combine acceleromter and gyro angle values
    RPYT[0] = 0.96 * GyrAngX + 0.04 * ((atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - *ERRptr);
    RPYT[1] = 0.96 * GyrAngY + 0.04 * ((atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - *(ERRptr + 1));

    // Drift correction suggested in comments. Acts as LPF?!
    //GyrAngX = RPYT[0];
    //GyrAngY = RPYT[1];

    return RPYT;
}

void initialise_IMU(){
    Wire.begin();                      // Initialize comunication
    Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);                  // Talk to the register 6B
    Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);        // End the transmission
}

void configure_IMU_sens(){
    // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);                  // Talk to the ACCEL_CONFIG register (1C hex)
    Wire.write(0x10);                  // Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(true);

    // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
    Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);
}

float  *calculate_IMU_error() {  
    // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. 
    // From here we will get the error values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
    /*
    //Error values
    float AccErrX = 0;
    float AccErrY = 0;
    float GyrErrX = 0;
    float GyrErrY = 0;
    float GyrErrZ = 0;
    */
    // Array to return error values
    // AccErrX, AccErrY, GyrErrX, GyrErrY, GyrErrZ
    static float errors[5] = {0};

    // Read accelerometer values 200 times
    for(int i = 0; i < 200; i++){
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        float AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
        float AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
        float AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
        // Sum all readings
        errors[0] = errors[0] + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
        errors[1] = errors[1] + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    }
    //Divide the sum by 200 to get the error value
    errors[0] = errors[0] / 200;
    errors[1] = errors[1] / 200;

    // Read gyro values 200 times
    for(int i = 0; i < 200; i++){
        Wire.beginTransmission(MPU);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        float GyrX = Wire.read() << 8 | Wire.read();
        float GyrY = Wire.read() << 8 | Wire.read();
        float GyrZ = Wire.read() << 8 | Wire.read();
        // Sum all readings
        errors[2] = errors[2] + (GyrX / 131.0);
        errors[3] = errors[3] + (GyrY / 131.0);
        errors[4] = errors[4] + (GyrZ / 131.0);
    }
    // Divide the sum by 200 to get the error value
    errors[2] = errors[2] / 200;
    errors[3] = errors[3] / 200;
    errors[4] = errors[4] / 200;

    // Returns pointer to start of array
    return errors;
}