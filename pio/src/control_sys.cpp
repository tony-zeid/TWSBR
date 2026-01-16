#include "../include/control_sys.h"

/***************************************************************/

// Motors previous position
long mot1PrevPos = 0;
long mot2PrevPos = 0;

// Motor position and velocity measurement
// mot1_pos, mot2_pos, (m1_vel, m2_vel)
static int motMeas[2] = {0};

/***************************************************************/

// Actuation signal array
// mot1_spd, mot2_spd, mot1_dir, mot2_dir
static int motAct[4] = {0};

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
float positionControl(float *imuPtr, float *posParamPtr, bool printData){
    // IMUptr: roll, pitch, yaw, timestamp, elapsed_time (milliseconds)
    // POSptr_param: setPos, posKP, posKI, posKD
    
    // Controller outputs (local variables, not static)
    
    float dt = imuPtr[4] / 1000.0f;  // Convert ms to seconds
    if(dt <= 0) dt = 0.001f;          // Prevent division by zero
    
    float setPos = posParamPtr[0];
    float Kp = posParamPtr[1];
    float Ki = posParamPtr[2];
    float Kd = posParamPtr[3];
    
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
    
    // Sum all terms and return
    float posOut = pos_P + pos_I + pos_D;
    
    return posOut;
}

// PID balance controller - core stabilisation controller
float balanceControl(float *imuPtr, float *balParamPtr, float posOut, bool printData){
    // IMUptr: roll, pitch, yaw, timestamp, elapsed_time
    // BALptr_param: setBal, balKP, balKI, balKD
    // POS_out: balance offset from position controller
    
    float dt = imuPtr[4] / 1000.0f;  // Convert ms to seconds
    if(dt <= 0) dt = 0.001f;
    
    float setBal = balParamPtr[0];
    float Kp = balParamPtr[1];
    float Ki = balParamPtr[2];
    float Kd = balParamPtr[3];
    
    // Pitch angle is the balance error (want pitch near 0)
    // Add position offset to create forward/backward lean
    float pitch = imuPtr[1];  // Current pitch angle
    float bal_setpoint = setBal + posOut;  // Add position adjustment
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
    
    // Sum all terms and return
    float balOut = bal_P + bal_I + bal_D;
    
    return balOut;
}

// PID heading controller - yaw/rotation control
float headingControl(float *imuPtr, float *hdgParamPtr, bool printData){
    // IMUptr: roll, pitch, yaw, timestamp, elapsed_time
    // HDGptr_param: setHdg, hdgKP, hdgKI, hdgKD
    
    float dt = imuPtr[4] / 1000.0f;  // Convert ms to seconds
    if(dt <= 0) dt = 0.001f;
    
    float setHdg = hdgParamPtr[0];
    float Kp = hdgParamPtr[1];
    float Ki = hdgParamPtr[2];
    float Kd = hdgParamPtr[3];
    
    // Heading error (difference from desired yaw)
    float yaw = imuPtr[2];  // Current yaw angle
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
    
    // Sum all terms and return
    float hdgOut = hdg_P + hdg_I + hdg_D;
    
    return hdgOut;
}

// Combined cascade control system                                                              // Combined
int *cascadeControl(float *imuPtr, float balOut, float hdgOut, bool printData){
    // IMUptr: roll, pitch, yaw, timestamp, elapsed_time
    // BAL_out: balance controller output (pitch correction)
    // HDG_out: heading controller output (yaw correction)

    /***************************************************************/
    // Motor setpoints based on balance and heading outputs
    // Balance output directly drives forward/backward
    // Heading output creates differential thrust for rotation
    
    float mot1VelocitySet = balOut - hdgOut;  // Left motor: balance minus turn correction
    float mot2VelocitySet = balOut + hdgOut;  // Right motor: balance plus turn correction

    // Motors travel since last cycle
    int mot1Travel = motMeas[0] - mot1PrevPos;    
    int mot2Travel = motMeas[1] - mot2PrevPos;    

    // Update previous position
    mot1PrevPos = motMeas[0];   
    mot2PrevPos = motMeas[1];

    // Calculate velocity, arbitrary units
    // Prevent division by near-zero values for numerical stability
    float timeScale = (imuPtr[4] > 1.0f) ? (imuPtr[4] / 5000.0f) : 0.0001f;
    float mot1VelocityMeas = (timeScale > 0) ? (mot1Travel / timeScale) : 0.0f;  
    float mot2VelocityMeas = (timeScale > 0) ? (mot2Travel / timeScale) : 0.0f;

    // Simple proportional control for inner velocity loop
    // Calculate error signal
    int mot1Error = mot1VelocitySet - mot1VelocityMeas;
    int mot2Error = mot2VelocitySet - mot2VelocityMeas;

    // Calculate output velocity with proportional gain
    int mot1VelocityOut = mot1Error * 1;  // Velocity loop gain = 1 (can be tuned)
    int mot2VelocityOut = mot2Error * 1;

    /************************************************************** */

    // Apply output clamping to PWM limits (0-255)
    if(mot1VelocityOut > 250) mot1VelocityOut = 250;
    else if(mot1VelocityOut < -250) mot1VelocityOut = -250;
    if(mot2VelocityOut > 250) mot2VelocityOut = 250;
    else if(mot2VelocityOut < -250) mot2VelocityOut = -250;

    // Set speed and direction from calculated velocity
    // mot1_spd, mot2_spd, mot1_dir, mot2_dir
    // Motor 1 reverse
    if(mot1VelocityOut < 0){
        motAct[0] = -mot1VelocityOut; 
        motAct[2] = 1;
    }
    // Motor 1 forward
    else{
        motAct[0] = mot1VelocityOut;
        motAct[2] = 0;
    }
    // Motor 2 reverse
    if(mot2VelocityOut < 0){
        motAct[1] = -mot2VelocityOut;
        motAct[3] = 1;
    }
    // Motor 2 forward
    else{
        motAct[1] = mot2VelocityOut;
        motAct[3] = 0;
    }

    // Print variables used in calculation
    if(printData){
        printFloat("M1_meas:", &mot1VelocityMeas, 0);  
        printInt("M1_set:", (int*)&mot1VelocitySet, 0);   
        printInt("M1_out:", &mot1VelocityOut, 0);
        printMsg("");
        printFloat("M2_meas:", &mot2VelocityMeas, 0);
        printInt("M2_set:", (int*)&mot2VelocitySet, 0);
        printInt("M2_out:", &mot2VelocityOut, 0);
        printMsg("");
    }
    
    return motAct;
}

// Motor 1 Interrupts
void inc1(){
    int ens1B = digitalRead(ENC1_B);
    if(ens1B > 0){
        motMeas[0]--;
    }
    else{
        motMeas[0]++;
    }
}
// Motor 2 Interrupts
void inc2(){
    int ens2B = digitalRead(ENC2_B);
    if(ens2B > 0){
        motMeas[1]++;
    }
    else{
        motMeas[1]--;
    }
}

// Get current encoder positions
int *getMotorPositions(){
    return motMeas;
}