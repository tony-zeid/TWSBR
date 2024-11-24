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
#include "pins.h"
#include "messages.h"
#include "IMU_functions.h"
#include "control_sys.h"
#include "motor_driver.h"
#include "bluetooth.h"

// Set running mode
byte run_mode = 2;
// 0 - Measure only
// 1 - Run controller
// 2 - Drive motors

// Program counter and data rate
unsigned long c = 0;
int dataRate = 1;

// Pointer to IMU error array
float *ERRptr;              

// Setup Bluetooth and pins
//SoftwareSerial BTSerial(rxPin, txPin);

// Initialisation routines
void setup() {
    // Setup I/O pin modes and interrupts
    setup_pins();

    // Initialise serial and Bluetooth communications
    Serial.begin(19200);    // 19200 required for MPU6050
    //BTSerial.begin(9600);   // 9600 required for HC-05 
    print_msg("Communications initialised");

    // Send reset command to IMU
    initialise_IMU();
    print_msg("IMU initialisation complete");

    // Configure IMU sentitivity range
    configure_IMU_sens();
    print_msg("IMU sensitivity configured");

    // Measure IMU steady-state error 
    ERRptr = calculate_IMU_error();
    print_msg("IMU errors calculated");

    // Print error values to serial monitor and Bluetooth
    print_float("AccErrX:", ERRptr, 0);
    print_float("AccErrY:", ERRptr, 1);
    print_float("GyrErrX:", ERRptr, 2);
    print_float("GyrErrY:", ERRptr, 3);
    print_float("GyrErrZ:", ERRptr, 4);
    Serial.print("\n");

    // Checkpoint message
    print_msg("Initialisation complete - Robot ready!");

    // Required for stability (min 50ms)
    delay(50);
}

// Main control cycle
void loop() { 
    // Retrieve IMU measurements
    float *IMUptr = read_IMU_data(ERRptr);                                                 // Read IMU
    print_msg("IMU measurements recieved"); 

    // Print RPYT values to serial monitor and Bluetooth
    if(c % dataRate == 0){
        print_float("Time:", IMUptr, 3);
        print_float("Roll:", IMUptr, 0);
        print_float("Pitch:", IMUptr, 1);
        print_float("Yaw:", IMUptr, 2);
        //Serial.print("\n");
    }

    // Receive control parameters (Bluetooth)
    float *CONptr = get_param();                                                           // Read parameters
    print_msg("Control parameters recieved");

    // Print control parameters to serial monitor and Bluetooth
    if(c % dataRate == 0){
        print_float("SetAng:", CONptr, 0);
        print_float("SetPos:", CONptr, 1);
        print_float("kP:", CONptr, 2);
        print_float("kI:", CONptr, 3);
        print_float("kD:", CONptr, 4);
        //Serial.print("\n");
    }

    // Send RPYT to controller for motor drive signal
    if(run_mode == 1 || run_mode == 2){
        int *MOTptr = PID_control(IMUptr, CONptr, c % dataRate);                           // Run Control              
        print_msg("Control signal calculated");

        // Print actuation signal values to serial monitor and Bluetooth
        if(c % dataRate == 0){
            print_int("M1Spd:", MOTptr, 0);
            print_int("M2Spd:", MOTptr, 1);
            print_int("M1Dir:", MOTptr, 2);
            print_int("M2Dir:", MOTptr, 3);
            print_int("M1Pos:", MOTptr, 4);
            print_int("M2Pos:", MOTptr, 5);
            //Serial.print("\n");
        }     
        
        // Send actuation signal to motors
        if(run_mode == 2){
            run_motors(MOTptr);                                                            // Run Motors
            print_msg("Motor drive running");
        }
    }
    // Disable motors if run mode invalid
    else if(run_mode < 0 || run_mode > 2){
        run_mode = 0;
        print_msg("RUN MODE FAULT!");
    }
    // Increment program counter
    c++;
}
