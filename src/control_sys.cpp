#include "control_sys.h"

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
        MOTptr[4]++;
    }
    else{
        MOTptr[4]--;
    }
}
// Motor 2 Interrupts
void inc2(){
    int ens2B = digitalRead(enp2B);
    if(ens2B > 0){
        MOTptr[5]--;
    }
    else{
        MOTptr[5]++;
    }
}

// PID position controller
float position_control(float *IMUptr, float *POSptr_param, bool printData){

    return POS_out;
}

// PID balance controller
float balance_control(float *IMUptr, float *BALptr_param, float POS_out, bool printData){
    
    return BAL_out;
}

// PID heading controller
float heading_control(float *IMUptr, float *HDGptr_param, bool printData){

    return HDG_out;
}

// Combined cascade control system
int *cascade_control(float *IMUptr, float BAL_out, float HDG_out, bool printData){

    return MOTptr;
}









//  Old version - simple PID
/****************************************************


// Variables used in control calcualtions
float Tlast = 0;
float prvERR = 0;
float prvPitch = 0;
float conINT = 0;
float conDER = 0;
float tau = 0.01;
//float act_DZ = 10;

// Integrator anti-windup and output clamping
int maxINT = 150;
int maxOUT = 190;

// Motor velocity
float mot1_vel = 0;
float mot2_vel = 0;


int *PID_control(float *IMUptr, float *CONptr, int printOut){
    
    // IMU measurements and sample time
    //float roll = *(IMUptr + 0);
    //float pitch = *(IMUptr + 1);
    //float yaw = *(IMUptr + 2);
    //float Ts = *(IMUptr + 3);

    // Control setpoints and parameters
    //float setAng = *(CONptr + 0);
    //float setPos = *(CONptr + 1);
    //float kP = *(CONptr + 2);
    //float kI = *(CONptr + 3);
    //float kD = *(CONptr + 4);
    

    // Calculate dT and update marker
    float dT = *(IMUptr + 3) - Tlast;
    Tlast = *(IMUptr + 3);
    //Serial.print("dT:"); Serial.print(dT); Serial.print(",");

    // ================= PID Control System ================= //

    // Error = SetAng - Pitch
    float conERR = *(CONptr + 0) - *(IMUptr + 1); 

    // Proportional term
    float conPRO = *(CONptr + 2) * conERR;

    // Integral term 
    conINT += (*(CONptr + 3) * 0.5f * dT * (conERR + prvERR));

    // Apply anti-windup clamping to integrator
    if(conINT > maxINT){
        conINT = maxINT;
    } else if(conINT < -maxINT){
        conINT = -maxINT;
    }

    // Derivative term on measurement with low pass filter     - blows up to infinity!
    //conDER = ( conDER * (2 * tau - dT) - 2 * (*(CONptr + 4)) * (*(IMUptr + 1) - prvPitch)) / (2 * tau * dT);

    // Increment previous values
    prvERR = conERR;
    prvPitch = *(IMUptr + 1);

    // Sum for output signal
    float conOUT = conPRO + conINT + conDER;

    // Apply clamping to limit output for PWM (max 255 inc mot_DZ)
    if (conOUT > maxOUT){
        conOUT = maxOUT;
    } else if (conOUT < -maxOUT){
        conOUT = -maxOUT;
    }

    // ====================================================== //

    // Print internal control variables
    if(printOut == 0 && 1){ // printOut = c % dataRate
        //Serial.print("dT:"); Serial.print(dT); Serial.print(",");
        Serial.print("conPRO:"); Serial.print(conPRO); Serial.print(",");
        Serial.print("conINT:"); Serial.print(conINT); Serial.print(",");
        Serial.print("conDER:"); Serial.print(conDER); Serial.print(","); 
        Serial.print("conOUT:"); Serial.print(conOUT); Serial.print(",");
        // To keep serial monitor scaling constant
        Serial.print("MAX:"); Serial.print(150); Serial.print(",");
        Serial.print("MIN:"); Serial.print(-150); Serial.print(",");
        Serial.print("ZERO:"); Serial.print(0); Serial.print(",");
        Serial.print("\n");
    }
 
    // Write control output to PWM driver
    mot1_vel = conOUT;
    mot2_vel = mot1_vel; // Simple balance motion only for now

    // Set speed and direction from calculated velocity
    // Motor 1 reverse
    if(mot1_vel < 0){
        act_MOT[0] = -mot1_vel; 
        act_MOT[2] = 0;
    }
    // Motor 1 forward
    else{
        act_MOT[0] = mot1_vel;
        act_MOT[2] = 1;
    }
    // Motor 2 reverse
    if(mot2_vel < 0){
        act_MOT[1] = -mot2_vel;
        act_MOT[3] = 0;
    }
    // Motor 2 forward
    else{
        act_MOT[1] = mot2_vel;
        act_MOT[3] = 1;
    }

    return act_MOT;
}



************************************************************/