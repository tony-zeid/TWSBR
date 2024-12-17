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

// User-defined libraries
#include "../include/pins.h"           // Defines pinout and sets pin modes
#include "../include/messages.h"       // Prints messages & data to serial & BT
#include "../include/IMU_functions.h"  // Configure & read from IMU
#include "../include/control_sys.h"    // Runs control system calculation
#include "../include/motor_driver.h"   // Sets motor direction & PWM duty cycle
#include "../include/bluetooth.h"      // Receives parameter updates

// Set running mode
byte run_mode = 2;
// 0 - Measure only
// 1 - Run controller
// 2 - Drive motors

// Program counter and data rate
int c = 0;    // Increments on each loop
int dataRate = 500;   // Prints data every n cycles
bool printData = 0;

// Pointer to IMU error array
float *ERRptr;              

// Setup Bluetooth serial
//SoftwareSerial BTSerial(rxPin, txPin);

// Initialisation routines
void setup() {
    // Setup I/O pin modes and interrupts
    setup_pins();

    // Initialise serial and Bluetooth communications
    Serial.begin(19200);    // 19200 required for MPU6050
    //BTSerial.begin(9600);   // 9600 required for HC-05 
    print_msg("Communications initialised");

    // Initialise I2C comms and reset IMU 
    initialise_IMU();
    print_msg("IMU initialisation complete");

    // Configures sensitivity of Accel and Gyro
    configure_IMU_sens();
    print_msg("IMU sensitivity configured");

    // Measure steady-state errors for compensation
    ERRptr = calculate_IMU_error();
    print_msg("IMU errors calculated");

    // Print error values to serial monitor and Bluetooth
    print_float("AccErrX:", ERRptr, 0);
    print_float("AccErrY:", ERRptr, 1);
    print_float("GyrErrX:", ERRptr, 2);
    print_float("GyrErrY:", ERRptr, 3);
    print_float("GyrErrZ:", ERRptr, 4);
    print_msg("");

    // Checkpoint message
    print_msg("Initialisation complete - Robot ready!");

    // Required for stability (min 20ms)
    delay(50);
}

// Main Control Cycle
void loop() { 
    // Determines whether to print this cycle
    printData = (c % dataRate == 0);

    // Update IMU measurements, returns RPYT
    float *IMUptr = read_IMU_data();                                                            // Read IMU 

    // Print RPYT values to serial monitor and Bluetooth
    if(printData){
        print_msg("IMU measurements recieved");         // Count and timings 
        print_int("Count:", &c, 0);
        print_float("Time:", IMUptr, 3);   
        print_float("Tdif:", IMUptr, 4 );
        print_msg("");
        print_float("Roll:", IMUptr, 0);                // IMU measurement data
        print_float("Pitch:", IMUptr, 1);
        print_float("Yaw:", IMUptr, 2);
        print_msg("");
    }

    // Receive control parameters (Bluetooth)
    float *POSptr_param = get_pos_param();    // Position                                       // Read parameters
    float *BALptr_param = get_bal_param();    // Balance
    float *HDGptr_param = get_hdg_param();    // Heading                                                             

    // Print control parameters to serial monitor and Bluetooth
    if(printData){
        print_msg("Control parameters recieved");
        print_float("SetPos:", POSptr_param, 0);        // Position control parameters
        print_float("PosKP:", POSptr_param, 1);
        print_float("PosKI:", POSptr_param, 2);
        print_float("PosKD:", POSptr_param, 3);
        print_msg("");
    }
    if(printData){
        print_float("SetBal:", BALptr_param, 0);        // Balance control parameters
        print_float("BalKP:", BALptr_param, 1);
        print_float("BalKI:", BALptr_param, 2);
        print_float("BalKD:", BALptr_param, 3);
        print_msg("");
    }
    if(printData){
        print_float("SetHdg:", HDGptr_param, 0);        // Heading control parameters
        print_float("HdgKP:", HDGptr_param, 1);
        print_float("HdgKI:", HDGptr_param, 2);
        print_float("HdgKD:", HDGptr_param, 3);
        print_msg("");
    }

    // Send RPYT to controller for motor drive signal                                      
    if(run_mode == 1 || run_mode == 2){
        float POS_out = position_control(IMUptr, POSptr_param, printData);                      // P/B/H contrrollers    
        float BAL_out = balance_control(IMUptr, BALptr_param, POS_out, printData);
        float HDG_out = heading_control(IMUptr, HDGptr_param, printData);

       if(printData){
            print_msg("Control signals calculated");
            print_float("POScon:", &POS_out, 0);          // Position control output
            print_float("BALcon:", &BAL_out, 0);          // Balance control output
            print_float("HDGcon:", &HDG_out, 0);          // Heading control output
            print_msg("");
        }     

        int *MOTptr = cascade_control(IMUptr, BAL_out, HDG_out, printData);                     // Cascade controller

        // Print actuation signal values to serial monitor and Bluetooth
        if(printData){
            print_msg("Actuation signal calculated");
            print_int("M1Spd:", MOTptr, 0);             
            print_int("M2Spd:", MOTptr, 1);
            print_int("M1Dir:", MOTptr, 2);
            print_int("M2Dir:", MOTptr, 3);
            print_int("M1Pos:", MOTptr, 4);
            print_int("M2Pos:", MOTptr, 5);
            print_msg("");
        }     
        
        // Send actuation signal to motors
        if(run_mode == 2){
            run_motors(MOTptr);                                                                 // Run Motors
            if(printData) print_msg("Motor drive running");
        }
    }
    // Disable motors if run mode invalid
    else if(run_mode < 0 || run_mode > 2){
        run_mode = 0;
        if (printData) print_msg("RUN MODE FAULT!");
    }

    // Print line break
    if(printData) print_msg("====================");

    // Increment program counter
    c++;
}