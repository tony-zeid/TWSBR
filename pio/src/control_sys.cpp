#include "../include/control_sys.h"

/***************************************************************/

// Motors previous position
long mot1_prv_pos = 0;
long mot2_prv_pos = 0;

// Motor position and velocity measurement
// mot1_pos, mot2_pos, (m1_vel, m2_vel)
static int MOTptr_meas[2] = {0};

/***************************************************************/

// Individual controller outputs
static float POS_out = 0;     // Position control output
static float BAL_out = 0;     // Balance control output
static float HDG_out = 0;     // Headaing control output

// Actuation signal array
// mot1_spd, mot2_spd, mot1_dir, mot2_dir
static int MOTptr_act[4] = {0};

// PID position controller                                                                      // Position 
float position_control(float *IMUptr, float *POSptr_param, bool printData){
    // IMUptr
    // roll, pitch, yaw, timestamp, elapsed_time
    // POSptr_param
    // setPos, posKP, posKI, posKD
    POS_out = 33;
    return POS_out;
}
// PID balance controller                                                                       // Balance
float balance_control(float *IMUptr, float *BALptr_param, float POS_out, bool printData){
    // IMUptr
    // roll, pitch, yaw, timestamp, elapsed_time
    // BALptr_param
    // setBal, balKP, balKI, balKD
    BAL_out = 55;
    return BAL_out;
}
// PID heading controller                                                                       // Heading
float heading_control(float *IMUptr, float *HDGptr_param, bool printData){       
    // IMUptr
    // roll, pitch, yaw, timestamp, elapsed_time
    // HDGptr_param
    // setHdg, hdgKP, hdgKI, hdgKD
    HDG_out = 77;
    return HDG_out;
}

// Combined cascade control system                                                              // Combined
int *cascade_control(float *IMUptr, float BAL_out, float HDG_out, bool printData){
    // IMUptr
    // roll, pitch, yaw, timestamp, elapsed_time
    // MOTptr
    // mot1_spd, mot2_spd, mot1_dir, mot2_dir, mot1_pos, mot2_pos

    /***************************************************************/
    // Motor setpoints
    int mot1_vel_set = 0;     
    int mot2_vel_set = 1000;    

    // Motors travel since last cycle
    int mot1_trv = MOTptr_meas[0] - mot1_prv_pos;    
    int mot2_trv = MOTptr_meas[1] - mot2_prv_pos;    

    // Update previous position
    mot1_prv_pos = MOTptr_meas[0];   
    mot2_prv_pos = MOTptr_meas[1];

    // Calculate velocity, arbitrary units
    float mot1_vel_meas = mot1_trv / (IMUptr[4] / 5000);  
    float mot2_vel_meas = mot2_trv / (IMUptr[4] / 5000);


    // Simple proportional control for testing //

    // Calculate error signal
    int mot1_error = mot1_vel_set - mot1_vel_meas;
    int mot2_error = mot2_vel_set - mot2_vel_meas;

    // Calculate output velocity
    int mot1_vel_out = mot1_error;
    int mot2_vel_out = mot2_error;

    /************************************************************** */

    // Apply output clamping
    if(mot1_vel_out > 250) mot1_vel_out = 250;
    else if(mot1_vel_out < -250) mot1_vel_out = -250;
    if(mot2_vel_out > 250) mot2_vel_out = 250;
    else if(mot2_vel_out < -250) mot2_vel_out = -250;

    // Set speed and direction from calculated velocity
    // mot1_spd, mot2_spd, mot1_dir, mot2_dir, mot1_pos, mot2_pos
    // Motor 1 reverse
    if(mot1_vel_out < 0){
        MOTptr_act[0] = -mot1_vel_out; 
        MOTptr_act[2] = 1;
    }
    // Motor 1 forward
    else{
        MOTptr_act[0] = mot1_vel_out;
        MOTptr_act[2] = 0;
    }
    // Motor 2 reverse
    if(mot2_vel_out < 0){
        MOTptr_act[1] = -mot2_vel_out;
        MOTptr_act[3] = 1;
    }
    // Motor 2 forward
    else{
        MOTptr_act[1] = mot2_vel_out;
        MOTptr_act[3] = 0;
    }

    // Print variables used in calculation
    if(printData){
        print_float("M1_meas:", &mot1_vel_meas, 0);  
        print_int("M1_set:", &mot1_vel_set, 0);   
        print_int("M1_out:", &mot1_vel_out, 0);
        print_msg("");
        print_float("M2_meas:", &mot2_vel_meas, 0);
        print_int("M2_set:", &mot2_vel_set, 0);
        print_int("M2_out:", &mot2_vel_out, 0);
        print_msg("");
    }
    
    return MOTptr_act;
}

// Motor 1 Interrupts
void inc1(){
    int ens1B = digitalRead(enp1B);
    if(ens1B > 0){
        MOTptr_meas[0]--;
    }
    else{
        MOTptr_meas[0]++;
    }
}
// Motor 2 Interrupts
void inc2(){
    int ens2B = digitalRead(enp2B);
    if(ens2B > 0){
        MOTptr_meas[1]++;
    }
    else{
        MOTptr_meas[1]--;
    }
}