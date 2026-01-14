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
#include "../include/bluetooth.h"      // Receives parameter updates

// Set running mode
uint8_t run_mode = 1;
// 0 - Measure only
// 1 - Run controller
// 2 - Drive motors

// Program counter and data output rate
int c = 0;    // Increments on each loop
uint16_t dataRate = 500;   // Prints data every n cycles
uint8_t printData = 0; // = (c % dataRate == 0);

// Pointer to IMU error array
float *ERRptr;              

// Setup Bluetooth serial
//SoftwareSerial BTSerial(rxPin, txPin);

// Initialisation routines
void setup() {
    // Setup I/O pin modes and interrupts
    setup_pins();
    //test_motors();    // Uncomment to test (run mode 0 only!)

    // Initialise serial and Bluetooth communications
    Serial.begin(19200);        // 19200 required for MPU6050
    print_msg("");
    print_msg("[INIT] Serial");

    //BTSerial.begin(9600);     // 9600 required for HC-05 
    //print_msg("[INIT] Bluetooth");
    
    // Initialise I2C comms and reset IMU 
    IMU_initialise();
    print_msg("[INIT] IMU Start");

    // Configures sensitivity of Accel and Gyro
    IMU_sens_config();
    delay(20);                                                      // Delay for stability (min 20ms)
    print_msg("[INIT] IMU Scale");

    // Measure steady-state errors for compensation
    ERRptr = IMU_error_calc();
    print_msg("[INIT] IMU Config");

    // Print error values to serial monitor and Bluetooth
    if(1){
        print_msg("[INFO] IMU Errors");
        print_float("AccErrX:", ERRptr, 0);
        print_float("AccErrY:", ERRptr, 1);
        print_float("GyrErrX:", ERRptr, 2);
        print_float("GyrErrY:", ERRptr, 3);
        print_float("GyrErrZ:", ERRptr, 4);
        print_msg("");
    }

    // End-of-setup message
    print_msg("==================================================");
    print_msg("[STATUS] Robot Ready");
    delay(50);                                                      // Delay for stability (min 20ms)
}

// Main Control Cycle
void loop() {
    // Determines whether to print this cycle
    printData = (c % dataRate == 0);

    // Print count and time
    if(printData){
        print_msg("==================================================");
        print_int("Count:", &c, 0);
        int timenow = millis();
        print_int("Time:", &timenow, 0);   
        print_msg("");
    }

    if(run_mode == 0 || run_mode == 1 || run_mode == 2){   
        // Receive control parameters (Bluetooth)
        float *POSptr_param = get_pos_param();    // Position                                       // Read parameters
        float *BALptr_param = get_bal_param();    // Balance
        float *HDGptr_param = get_hdg_param();    // Heading                                                             

        // Print control parameters to serial monitor and Bluetooth
        if(printData){
            print_msg("[INFO] Control parameters");
            // Position control parameters
            print_float("SetPos:", POSptr_param, 0); 
            print_float("PosKP:", POSptr_param, 1);
            print_float("PosKI:", POSptr_param, 2);
            print_float("PosKD:", POSptr_param, 3);
            print_msg("");
            // Balance control parameters
            print_float("SetBal:", BALptr_param, 0);        
            print_float("BalKP:", BALptr_param, 1);
            print_float("BalKI:", BALptr_param, 2);
            print_float("BalKD:", BALptr_param, 3);
            print_msg("");
            // Heading control parameters
            print_float("SetHdg:", HDGptr_param, 0);       
            print_float("HdgKP:", HDGptr_param, 1);
            print_float("HdgKI:", HDGptr_param, 2);
            print_float("HdgKD:", HDGptr_param, 3);
            print_msg("");
        }

        // Update IMU measurements, returns RPYT
        float *IMUptr = IMU_read_data();                                                            // Read IMU 

        // Print RPYT values to serial monitor and Bluetooth
        if(printData){
            print_msg("[INFO] IMU measurements");       
            // IMU measurement data
            print_float("Time:", IMUptr, 3);   
            print_float("Tdif:", IMUptr, 4);
            print_msg("");
            print_float("Roll:", IMUptr, 0);                
            print_float("Pitch:", IMUptr, 1);
            print_float("Yaw:", IMUptr, 2);
            print_msg("");
        }

        // Send RPYT to controller for motor drive signal                                      
        if(run_mode == 1 || run_mode == 2){
            float POS_out = position_control(IMUptr, POSptr_param, printData);                      // P/B/H contrrollers    
            float BAL_out = balance_control(IMUptr, BALptr_param, POS_out, printData);
            float HDG_out = heading_control(IMUptr, HDGptr_param, printData);

            if(printData){
                print_msg("[INFO] Control signals");
                print_float("POScon:", &POS_out, 0);          // Position control output
                print_float("BALcon:", &BAL_out, 0);          // Balance control output
                print_float("HDGcon:", &HDG_out, 0);          // Heading control output
                print_msg("");
            }     

            int *MOTptr_act = cascade_control(IMUptr, BAL_out, HDG_out, printData);                 // Cascade controller

            // Print actuation signal values to serial monitor and Bluetooth
            if(printData){
                print_msg("[INFO] Motor output");
                print_int("M1Spd:", MOTptr_act, 0);     
                print_int("M1Dir:", MOTptr_act, 2);        
                print_msg("");
                print_int("M2Spd:", MOTptr_act, 1);
                print_int("M2Dir:", MOTptr_act, 3);
                print_msg("");
            }     
            
            // Send actuation signal to motors
            if(run_mode == 2){
                run_motors(MOTptr_act);                                                             // Run Motors
                if(printData) print_msg("[STATUS] Motors running");
            }
        }
    }

    // Message for run mode error
    else if (run_mode != 0 && run_mode != 1 && run_mode != 2){
        if(printData) print_msg("[ERROR] Invalid run mode");
    }

    // Increment program counter
    ++c;
}