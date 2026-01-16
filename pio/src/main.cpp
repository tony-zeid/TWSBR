/*******************************************************************************
 *                                                                             
 *                Two-Wheeled Self-Balancing Robot (TWSBR)
 *                     Project using Arduing Uno              
 *
 ******************************************************************************/

// Standard libraries
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <string.h>

// Project libraries
#include "../include/pins.h"           // Defines pinout and sets pin modes
#include "../include/messages.h"       // Prints messages & data to serial & BT
#include "../include/IMU_functions.h"  // Configure & read from IMU
#include "../include/control_sys.h"    // Runs control system calculation
#include "../include/motor_driver.h"   // Sets motor direction & PWM duty cycle
#include "../include/bluetooth.h"      // Bluetooth I/O
#include "../include/command.h"        // Command handling & parameters

// Set running mode
uint8_t runMode = 2;
// 0 - Measure only
// 1 - Run controller
// 2 - Drive motors

// Debug/print mode
uint8_t debugMode = 1;
// 0 - Compact: roll, pitch, yaw only (space-separated)
// 1 - Verbose: all telemetry and diagnostics

// Program counter and data output rates per debug mode
int loopCount = 0;    // Increments on each loop
uint16_t dataRatePerMode[2] = {10, 500};  // dataRatePerMode[debugMode]
#define dataRate (dataRatePerMode[debugMode])
uint8_t printData = 0;

// Pointer to IMU error array
float *errPtr;              

// Setup Bluetooth serial
//SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

// Initialisation routines
void setup() {
    // Setup I/O pin modes and interrupts
    setupPins();
    //test_motors();    // Uncomment to test (run mode 0 only!)

    // Initialise serial and Bluetooth communications
    Serial.begin(19200);        // 19200 required for MPU6050
    printMsg("");
    printMsg("[INIT] Serial");

    initBtSerial();
    
    // Initialise I2C comms and reset IMU 
    imuInitialise();
    printMsg("[INIT] IMU Start");

    // Configures sensitivity of Accel and Gyro
    imuConfigSensitivity();
    delay(20);                                                      // Delay for stability (min 20ms)
    printMsg("[INIT] IMU Scale");

    // Measure steady-state errors for compensation
    errPtr = imuCalculateError();
    printMsg("[INIT] IMU Config");

    // Print error values to serial monitor and Bluetooth
    if(1){
        printMsg("[INFO] IMU Errors");
        printFloat("AccErrX:", errPtr, 0);
        printFloat("AccErrY:", errPtr, 1);
        printFloat("GyrErrX:", errPtr, 2);
        printFloat("GyrErrY:", errPtr, 3);
        printFloat("GyrErrZ:", errPtr, 4);
        printMsg("");
    }

    // End-of-setup message
    printMsg("==================================================");
    printMsg("[STATUS] Robot Ready");
    delay(50);                                                      // Delay for stability (min 20ms)
}

// Main Control Cycle
void loop() {
    // Determines whether to print this cycle
    printData = (loopCount % dataRate == 0);

    // Process Bluetooth commands if any
    readBtSerial();

    if(runMode == 0 || runMode == 1 || runMode == 2){   
        // Get current control parameters
        float *posParamPtr = getPosParam();    // Position
        float *balParamPtr = getBalParam();    // Balance
        float *hdgParamPtr = getHdgParam();    // Heading                                                             

        // Update IMU measurements, returns RPYT
        float *imuPtr = imuReadData();                                                            // Read IMU 

        // Compact mode: print only Roll, Pitch, Yaw (space-separated)
        if(debugMode == 0){
            if(printData){
                Serial.print(imuPtr[0]);
                Serial.print(" ");
                Serial.print(imuPtr[1]);
                Serial.print(" ");
                Serial.println(imuPtr[2]);
            }
        }
        // Verbose mode: print full diagnostics
        else if(debugMode == 1){
            if(printData){
                printMsg("==================================================");
                printInt("Count:", &loopCount, 0);
                int timeNow = millis();
                printInt("Time:", &timeNow, 0);   
                printMsg("");
                printMsg("[INFO] Control parameters");
                // Position control parameters
                printFloat("SetPos:", posParamPtr, 0); 
                printFloat("PosKP:", posParamPtr, 1);
                printFloat("PosKI:", posParamPtr, 2);
                printFloat("PosKD:", posParamPtr, 3);
                printMsg("");
                // Balance control parameters
                printFloat("SetBal:", balParamPtr, 0);        
                printFloat("BalKP:", balParamPtr, 1);
                printFloat("BalKI:", balParamPtr, 2);
                printFloat("BalKD:", balParamPtr, 3);
                printMsg("");
                // Heading control parameters
                printFloat("SetHdg:", hdgParamPtr, 0);       
                printFloat("HdgKP:", hdgParamPtr, 1);
                printFloat("HdgKI:", hdgParamPtr, 2);
                printFloat("HdgKD:", hdgParamPtr, 3);
                printMsg("");
                printMsg("[INFO] IMU measurements");       
                // IMU measurement data
                printFloat("Time:", imuPtr, 3);   
                printFloat("Tdif:", imuPtr, 4);
                printMsg("");
                printFloat("Roll:", imuPtr, 0);                
                printFloat("Pitch:", imuPtr, 1);
                printFloat("Yaw:", imuPtr, 2);
                printMsg("");
            }
        }

        // Send RPYT to controller for motor drive signal (verbose debug mode)                                      
        if((runMode == 1 || runMode == 2) && debugMode == 1){
            float posOut = positionControl(imuPtr, posParamPtr, printData);                      // P/B/H controllers    
            float balOut = balanceControl(imuPtr, balParamPtr, posOut, printData);
            float hdgOut = headingControl(imuPtr, hdgParamPtr, printData);

            if(printData){
                printMsg("[INFO] Control signals");
                printFloat("POScon:", &posOut, 0);          // Position control output
                printFloat("BALcon:", &balOut, 0);          // Balance control output
                printFloat("HDGcon:", &hdgOut, 0);          // Heading control output
                printMsg("");
            }     

            int *motorActuationPtr = cascadeControl(imuPtr, balOut, hdgOut, printData);                 // Cascade controller

            // Print actuation signal values to serial monitor and Bluetooth
            if(printData){
                printMsg("[INFO] Motor output");
                printInt("M1Spd:", motorActuationPtr, 0);     
                printInt("M1Dir:", motorActuationPtr, 2);        
                printMsg("");
                printInt("M2Spd:", motorActuationPtr, 1);
                printInt("M2Dir:", motorActuationPtr, 3);
                printMsg("");
            }     
            
            // Send actuation signal to motors
            if(runMode == 2){
                runMotors(motorActuationPtr);                                                             // Run Motors
                if(printData) printMsg("[STATUS] Motors running");
            }
        }
        // Compact mode: compute control but don't print verbose output
        else if((runMode == 1 || runMode == 2) && debugMode == 0){
            float posOut = positionControl(imuPtr, posParamPtr, false);                           // Compute silently
            float balOut = balanceControl(imuPtr, balParamPtr, posOut, false);
            float hdgOut = headingControl(imuPtr, hdgParamPtr, false);
            int *motorActuationPtr = cascadeControl(imuPtr, balOut, hdgOut, false);
            if(runMode == 2){
                runMotors(motorActuationPtr);
            }
        }
    }

    // Message for run mode error
    else if (runMode != 0 && runMode != 1 && runMode != 2){
        if(printData && debugMode == 1) printMsg("[ERROR] Invalid run mode");
    }

    // Increment program counter
    ++loopCount;

    // Also process any commands typed into Serial Monitor
    processSerialCommands();
}