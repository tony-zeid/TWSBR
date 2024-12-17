/*
  IMU Functions adapted from tutorial by Dejan Nedelkovski at https://howtomechatronics.com
*/

#include "../include/IMU_functions.h"

// Time markers
float currentTime = 0;
float previousTime = 0;      
float elapsedTime = 0;

// Angle measurements
float GyrAngX = 0;
float GyrAngY = 0;
float AccAngX = 0;
float AccAngY = 0;

// Array to pass error values
// AccErrX, AccErrY, GyrErrX, GyrErrY, GyrErrZ
static float errors[5] = {0};

// Output array
// Roll, Pitch, Yaw, Time
static float RPYT[5] = {0};

// Update IMU measurements, returns RPYT
float *read_IMU_data(){
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
    float AccAngX = ((atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - errors[0]); 
    float AccAngY = ((atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - errors[1]);

    // === Update timestamps === //
    previousTime = currentTime; // Previous time is stored before the actual time read
    currentTime = millis(); // Current time actual time read
    elapsedTime = (currentTime - previousTime); // Calculate elapsed time
    RPYT[3] = currentTime;
    RPYT[4] = elapsedTime;
    elapsedTime = elapsedTime / 1000; // Converts back to seconds for calculation

    // === Read gyroscope data === //
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
    float GyrX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    float GyrY = (Wire.read() << 8 | Wire.read()) / 131.0;
    float GyrZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    // Correct the outputs with the calculated error values
    GyrX = GyrX - errors[2];
    GyrY = GyrY - errors[3];
    GyrZ = GyrZ - errors[4];
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    GyrAngX = GyrAngX + GyrX * elapsedTime; // deg/s * s = deg
    GyrAngY = GyrAngY + GyrY * elapsedTime;
    RPYT[2] =  RPYT[2] + GyrZ * elapsedTime;

    // Complementary filter - combine acceleromter and gyro angle values
    RPYT[0] = 0.96 * GyrAngX + 0.04 * AccAngX;
    RPYT[1] = 0.96 * GyrAngY + 0.04 * AccAngY;

    // Drift correction suggested in comments. Acts as LPF?!
    //GyrAngX = RPYT[0];
    //GyrAngY = RPYT[1];

    return RPYT;
}

// Initialise I2C comms and reset IMU 
void initialise_IMU(){
    Wire.begin();                      // Initialize comunication
    Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);                  // Talk to the register 6B
    Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);        // End the transmission
}

// Configures sensitivity of Accel and Gyro
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

    delay(20);
}

// Measure steady-state errors for compensation
float  *calculate_IMU_error() {  
    // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. 
    // From here we will get the error values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values

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
    errors[0] = errors[0] / 200;    // AccErrX
    errors[1] = errors[1] / 200;    // AccErrY

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
    errors[2] = errors[2] / 200;    // GyrErrX
    errors[3] = errors[3] / 200;    // GyrErrY
    errors[4] = errors[4] / 200;    // GyrErrZ

    // Returns pointer to start of array
    return errors;
}