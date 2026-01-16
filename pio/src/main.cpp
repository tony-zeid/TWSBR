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

// Packed mode bits (saves 2 bytes vs separate uint8_t variables)
// Bits 0-1: runMode (0-2)
// Bit 2: debugMode (0-1)
// Bit 3: printData (0-1)
uint8_t modeBits = 0x02;  // runMode=2, debugMode=0, printData=0
#define runMode       ((modeBits) & 0x03)
#define setRunMode(m) (modeBits = (modeBits & 0xFC) | ((m) & 0x03))
#define debugMode     (((modeBits) >> 2) & 0x01)
#define setDebugMode(d) (modeBits = (modeBits & 0xFB) | (((d) & 0x01) << 2))
#define printData     (((modeBits) >> 3) & 0x01)
#define setPrintData(p) (modeBits = (modeBits & 0xF7) | (((p) & 0x01) << 3))

// Program counter and data output rates per debug mode
int loopCount = 0;    // Increments on each loop
uint16_t dataRatePerMode[2] = {10, 500};  // dataRatePerMode[debugMode]
#define dataRate (dataRatePerMode[debugMode])

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

    // Load saved parameters from EEPROM
    loadParametersFromEEPROM();

    initBtSerial();
    
    // Initialise I2C comms and reset IMU 
    imuInitialise();
    printMsg("[INIT] IMU Start");

    // Configures sensitivity of Accel and Gyro
    imuConfigSensitivity();
    delay(20);                                                      // Delay for stability (min 20ms)
    printMsg("[INIT] IMU Scale");

    // NOTE: Calibration now done via 'cal' or 'cal_inv' commands, not on boot
    // This saves RAM and allows choosing normal or upside-down calibration
    printMsg("[INIT] IMU Ready");
    printMsg("[INFO] Use 'cal' or 'cal_inv' command to calibrate");

    // End-of-setup message
    printMsg("==================================================");
    printMsg("[STATUS] Robot Ready");
    delay(50);                                                      // Delay for stability (min 20ms)
}

// Main Control Cycle
void loop() {
    // Determines whether to print this cycle
    if (loopCount % dataRate == 0) {
        modeBits |= 0x08;  // Set printData bit
    } else {
        modeBits &= 0xF7;  // Clear printData bit
    }

    // Process Bluetooth commands if any
    readBtSerial();

    if((modeBits & 0x03) == 0 || (modeBits & 0x03) == 1 || (modeBits & 0x03) == 2) {   
        // Get current control parameters
        float *posParamPtr = getPosParam();    // Position
        float *balParamPtr = getBalParam();    // Balance
        float *hdgParamPtr = getHdgParam();    // Heading                                                             

        // Update IMU measurements, returns RPYT
        float *imuPtr = imuReadData();
        
        // SAFETY CUTOFF: Check tilt angle for safety
        // If robot tilts beyond safe limits, cut all motor outputs
        float pitch = imuPtr[1];  // Pitch angle
        float roll = imuPtr[0];   // Roll angle
        if (abs(pitch) > TILT_SAFETY_LIMIT || abs(roll) > TILT_SAFETY_LIMIT) {
            // Robot has fallen or being picked up - stop motors immediately
            int safeMotors[4] = {0, 0, 0, 0};  // All motors off
            runMotors(safeMotors);
            
            if (printData) {
                printMsg("[SAFETY CUTOFF] Tilt limit exceeded - motors stopped");
            }
            
            loopCount++;
            return;  // Skip control loop this cycle
        }

        // Continue with normal control if within safe limits
        // Compute control outputs for both compact and verbose modes
        float posOut = 0, balOut = 0, hdgOut = 0;
        int *motorActuationPtr = nullptr;
        
        if(runMode == 1 || runMode == 2){
            posOut = positionControl(imuPtr, posParamPtr, false);
            balOut = balanceControl(imuPtr, balParamPtr, posOut, false);
            hdgOut = headingControl(imuPtr, hdgParamPtr, false);
            motorActuationPtr = cascadeControl(imuPtr, balOut, hdgOut, false);
        }

        // Compact mode: print Roll, Pitch, Yaw, M1_speed, M2_speed, Enc1_pos, Enc2_pos (space-separated)
        if(debugMode == 0){
            if(printData){
                int *encPos = getMotorPositions();
                Serial.print(imuPtr[0]);    // Roll
                Serial.print(" ");
                Serial.print(imuPtr[1]);    // Pitch
                Serial.print(" ");
                Serial.print(imuPtr[2]);    // Yaw
                Serial.print(" ");
                Serial.print(motorActuationPtr ? motorActuationPtr[0] : 0);  // M1 speed
                Serial.print(" ");
                Serial.print(motorActuationPtr ? motorActuationPtr[1] : 0);  // M2 speed
                Serial.print(" ");
                Serial.print(encPos[0]);  // Encoder 1 position
                Serial.print(" ");
                Serial.println(encPos[1]);  // Encoder 2 position
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
            // Re-compute with verbose printing enabled for verbose mode
            posOut = positionControl(imuPtr, posParamPtr, printData);                      // P/B/H controllers    
            balOut = balanceControl(imuPtr, balParamPtr, posOut, printData);
            hdgOut = headingControl(imuPtr, hdgParamPtr, printData);

            if(printData){
                printMsg("[INFO] Control signals");
                printFloat("POScon:", &posOut, 0);          // Position control output
                printFloat("BALcon:", &balOut, 0);          // Balance control output
                printFloat("HDGcon:", &hdgOut, 0);          // Heading control output
                printMsg("");
            }     

            motorActuationPtr = cascadeControl(imuPtr, balOut, hdgOut, printData);                 // Cascade controller

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
