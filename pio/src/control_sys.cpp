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
static float HDG_out = 0;     // Heading control output

// Actuation signal array
// mot1_spd, mot2_spd, mot1_dir, mot2_dir
static int MOTptr_act[4] = {0};

/***************************************************************/
// PID state variables for each controller
static float pos_error_prev = 0;      // Previous position error (for derivative)
static float pos_integral = 0;        // Accumulated position error (for integral)

static float bal_error_prev = 0;      // Previous balance error
static float bal_integral = 0;        // Accumulated balance error

static float hdg_error_prev = 0;      // Previous heading error
static float hdg_integral = 0;        // Accumulated heading error

// Anti-windup limits for integral terms
#define POS_INTEGRAL_MAX 100.0f
#define BAL_INTEGRAL_MAX 50.0f
#define HDG_INTEGRAL_MAX 100.0f

/***************************************************************/
// PID position controller - controls forward/backward movement setpoint
float position_control(float *IMUptr, float *POSptr_param, bool printData){
    // IMUptr: roll, pitch, yaw, timestamp, elapsed_time (milliseconds)
    // POSptr_param: setPos, posKP, posKI, posKD
    
    float dt = IMUptr[4] / 1000.0f;  // Convert ms to seconds
    if(dt <= 0) dt = 0.001f;          // Prevent division by zero
    
    float setPos = POSptr_param[0];
    float Kp = POSptr_param[1];
    float Ki = POSptr_param[2];
    float Kd = POSptr_param[3];
    
    // Position error (difference from setpoint)
    float pos_error = setPos - 0;  // setPos is velocity target (0 for balance point)
    
    // Proportional term
    float pos_P = Kp * pos_error;
    
    // Integral term with anti-windup
    pos_integral += pos_error * dt;
    if(pos_integral > POS_INTEGRAL_MAX) pos_integral = POS_INTEGRAL_MAX;
    if(pos_integral < -POS_INTEGRAL_MAX) pos_integral = -POS_INTEGRAL_MAX;
    float pos_I = Ki * pos_integral;
    
    // Derivative term
    float pos_D = 0;
    if(dt > 0) pos_D = Kd * (pos_error - pos_error_prev) / dt;
    pos_error_prev = pos_error;
    
    // Sum all terms
    POS_out = pos_P + pos_I + pos_D;
    
    return POS_out;
}

// PID balance controller - core stabilisation controller
float balance_control(float *IMUptr, float *BALptr_param, float POS_out, bool printData){
    // IMUptr: roll, pitch, yaw, timestamp, elapsed_time
    // BALptr_param: setBal, balKP, balKI, balKD
    // POS_out: balance offset from position controller
    
    float dt = IMUptr[4] / 1000.0f;  // Convert ms to seconds
    if(dt <= 0) dt = 0.001f;
    
    float setBal = BALptr_param[0];
    float Kp = BALptr_param[1];
    float Ki = BALptr_param[2];
    float Kd = BALptr_param[3];
    
    // Pitch angle is the balance error (want pitch near 0)
    // Add position offset to create forward/backward lean
    float pitch = IMUptr[1];  // Current pitch angle
    float bal_setpoint = setBal + POS_out;  // Add position adjustment
    float bal_error = bal_setpoint - pitch;
    
    // Proportional term
    float bal_P = Kp * bal_error;
    
    // Integral term with anti-windup
    bal_integral += bal_error * dt;
    if(bal_integral > BAL_INTEGRAL_MAX) bal_integral = BAL_INTEGRAL_MAX;
    if(bal_integral < -BAL_INTEGRAL_MAX) bal_integral = -BAL_INTEGRAL_MAX;
    float bal_I = Ki * bal_integral;
    
    // Derivative term (also adds gyro feedback for stability)
    float bal_D = Kd * (bal_error - bal_error_prev) / dt;
    bal_error_prev = bal_error;
    
    // Sum all terms
    BAL_out = bal_P + bal_I + bal_D;
    
    return BAL_out;
}

// PID heading controller - yaw/rotation control
float heading_control(float *IMUptr, float *HDGptr_param, bool printData){
    // IMUptr: roll, pitch, yaw, timestamp, elapsed_time
    // HDGptr_param: setHdg, hdgKP, hdgKI, hdgKD
    
    float dt = IMUptr[4] / 1000.0f;  // Convert ms to seconds
    if(dt <= 0) dt = 0.001f;
    
    float setHdg = HDGptr_param[0];
    float Kp = HDGptr_param[1];
    float Ki = HDGptr_param[2];
    float Kd = HDGptr_param[3];
    
    // Heading error (difference from desired yaw)
    float yaw = IMUptr[2];  // Current yaw angle
    float hdg_error = setHdg - yaw;
    
    // Proportional term
    float hdg_P = Kp * hdg_error;
    
    // Integral term with anti-windup
    hdg_integral += hdg_error * dt;
    if(hdg_integral > HDG_INTEGRAL_MAX) hdg_integral = HDG_INTEGRAL_MAX;
    if(hdg_integral < -HDG_INTEGRAL_MAX) hdg_integral = -HDG_INTEGRAL_MAX;
    float hdg_I = Ki * hdg_integral;
    
    // Derivative term
    float hdg_D = Kd * (hdg_error - hdg_error_prev) / dt;
    hdg_error_prev = hdg_error;
    
    // Sum all terms
    HDG_out = hdg_P + hdg_I + hdg_D;
    
    return HDG_out;
}

// Combined cascade control system                                                              // Combined
int *cascade_control(float *IMUptr, float BAL_out, float HDG_out, bool printData){
    // IMUptr: roll, pitch, yaw, timestamp, elapsed_time
    // BAL_out: balance controller output (pitch correction)
    // HDG_out: heading controller output (yaw correction)

    /***************************************************************/
    // Motor setpoints based on balance and heading outputs
    // Balance output directly drives forward/backward
    // Heading output creates differential thrust for rotation
    
    float mot1_vel_set = BAL_out - HDG_out;  // Left motor: balance minus turn correction
    float mot2_vel_set = BAL_out + HDG_out;  // Right motor: balance plus turn correction

    // Motors travel since last cycle
    int mot1_trv = MOTptr_meas[0] - mot1_prv_pos;    
    int mot2_trv = MOTptr_meas[1] - mot2_prv_pos;    

    // Update previous position
    mot1_prv_pos = MOTptr_meas[0];   
    mot2_prv_pos = MOTptr_meas[1];

    // Calculate velocity, arbitrary units
    // Prevent division by near-zero values for numerical stability
    float timeScale = (IMUptr[4] > 1.0f) ? (IMUptr[4] / 5000.0f) : 0.0001f;
    float mot1_vel_meas = (timeScale > 0) ? (mot1_trv / timeScale) : 0.0f;  
    float mot2_vel_meas = (timeScale > 0) ? (mot2_trv / timeScale) : 0.0f;

    // Simple proportional control for inner velocity loop
    // Calculate error signal
    int mot1_error = mot1_vel_set - mot1_vel_meas;
    int mot2_error = mot2_vel_set - mot2_vel_meas;

    // Calculate output velocity with proportional gain
    int mot1_vel_out = mot1_error * 1;  // Velocity loop gain = 1 (can be tuned)
    int mot2_vel_out = mot2_error * 1;

    /************************************************************** */

    // Apply output clamping to PWM limits (0-255)
    if(mot1_vel_out > 250) mot1_vel_out = 250;
    else if(mot1_vel_out < -250) mot1_vel_out = -250;
    if(mot2_vel_out > 250) mot2_vel_out = 250;
    else if(mot2_vel_out < -250) mot2_vel_out = -250;

    // Set speed and direction from calculated velocity
    // mot1_spd, mot2_spd, mot1_dir, mot2_dir
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
        print_int("M1_set:", (int*)&mot1_vel_set, 0);   
        print_int("M1_out:", &mot1_vel_out, 0);
        print_msg("");
        print_float("M2_meas:", &mot2_vel_meas, 0);
        print_int("M2_set:", (int*)&mot2_vel_set, 0);
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