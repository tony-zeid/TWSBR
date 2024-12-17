#include "../include/control_sys.h"

// Individual controller outputs
static float POS_out = 0;     // Position control output
static float BAL_out = 0;     // Balance control output
static float HDG_out = 0;     // Headaing control output

// Actuation signal array
// mot1_spd, mot2_spd, mot1_dir, mot2_dir, mot1_pos, mot2_pos
static int MOTptr[6] = {0};

// Motor 1 Interrupts
void inc1(){
    int ens1B = digitalRead(enp1B);
    if(ens1B > 0){
        MOTptr[4]--;
    }
    else{
        MOTptr[4]++;
    }
}
// Motor 2 Interrupts
void inc2(){
    int ens2B = digitalRead(enp2B);
    if(ens2B > 0){
        MOTptr[5]++;
    }
    else{
        MOTptr[5]--;
    }
}

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
        
    // Calculate heading control signal

    // IMUptr
    // roll, pitch, yaw, timestamp, elapsed_time

    // HDGptr_param
    // setHdg, hdgKP, hdgKI, hdgKD

    HDG_out = 77;

    return HDG_out;
}

// Combined cascade control system                                                              // Combined
int *cascade_control(float *IMUptr, float BAL_out, float HDG_out, bool printData){

    // Combine controller outputs to form actuation signal

    // IMUptr
    // roll, pitch, yaw, timestamp, elapsed_time

    // MOTptr
    // mot1_spd, mot2_spd, mot1_dir, mot2_dir, mot1_pos, mot2_pos

    int mot1_vel = 0;
    int mot2_vel = 0;

    // Set speed and direction from calculated velocity
    // Motor 1 reverse
    if(mot1_vel < 0){
        MOTptr[0] = -mot1_vel; 
        MOTptr[2] = 1;
    }
    // Motor 1 forward
    else{
        MOTptr[0] = mot1_vel;
        MOTptr[2] = 0;
    }
    // Motor 2 reverse
    if(mot2_vel < 0){
        MOTptr[1] = -mot1_vel;
        MOTptr[3] = 1;
    }
    // Motor 2 forward
    else{
        MOTptr[1] = mot1_vel;
        MOTptr[3] = 0;
    }

    return MOTptr;
}