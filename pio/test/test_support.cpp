#include "test_support.h"

// Global state definitions
uint8_t runMode = 2;
uint8_t debugMode = 1;
uint16_t dataRatePerMode[2] = {500, 500};
float posControls[4] = {0, 1, 0.05f, 0.02f};
float balControls[4] = {0, 2, 0.1f, 0.03f};
float hdgControls[4] = {0, 1, 0, 0};
ControlState gControlState{};

// Mocks (no-op for native tests)
void printMsg(const char msg[]){ (void)msg; }
void printFloat(const char label[], float *varPtr, byte offset){ (void)label; (void)varPtr; (void)offset; }
void printInt(const char label[], int *varPtr, byte offset){ (void)label; (void)varPtr; (void)offset; }

// Helpers
bool parseNumberFloat(const String &arg, float &value) {
    if (arg.length() == 0) return false;
    bool hasDigit = false;
    for (size_t i = 0; i < arg.length(); i++) {
        char c = arg.buffer[i];
        if (!(std::isdigit(static_cast<unsigned char>(c)) || c=='-' || c=='+' || c=='.')) {
            return false;
        }
        if (std::isdigit(static_cast<unsigned char>(c))) hasDigit = true;
    }
    if (!hasDigit) return false;
    value = arg.toFloat();
    return true;
}

bool parseNumberInt(const String &arg, int &value) {
    if (arg.length() == 0) return false;
    bool neg = (arg.buffer[0] == '-');
    for (size_t i = neg ? 1 : 0; i < arg.length(); i++) {
        if (!std::isdigit(static_cast<unsigned char>(arg.buffer[i]))) return false;
    }
    value = arg.toInt();
    return true;
}

void setParam(float *arr, int index, const String &arg, const char *label){
    float v = 0.0f;
    if (!parseNumberFloat(arg, v)){
        printMsg("Invalid number");
        return;
    }
    arr[index] = v;
    printFloat(label, arr, static_cast<byte>(index));
    printMsg("");
}

void handleCommandLine(const String &line){
    String trimmed = line;
    trimmed.trim();
    if (trimmed.length() == 0) return;

    int sp = trimmed.indexOf(' ');
    String cmd = (sp == -1) ? trimmed : trimmed.substring(0, sp);
    String arg = (sp == -1) ? String() : trimmed.substring(sp + 1);
    cmd.toLowerCase();

    if (cmd == "status"){
        printMsg("Status OK");
        return;
    }

    if (cmd == "mode" || cmd == "run"){
        int m = 0;
        if (!parseNumberInt(arg, m)){
            printMsg("Provide 0, 1 or 2");
            return;
        }
        if (m < 0 || m > 2){
            printMsg("Mode must be 0..2");
            return;
        }
        runMode = static_cast<uint8_t>(m);
        printMsg("Mode updated");
        return;
    }

    if (cmd == "debug"){
        int d = 0;
        if (!parseNumberInt(arg, d)){
            printMsg("Provide 0 (compact) or 1 (verbose)");
            return;
        }
        if (d < 0 || d > 1){
            printMsg("Debug must be 0 or 1");
            return;
        }
        debugMode = static_cast<uint8_t>(d);
        printMsg("Debug mode updated");
        return;
    }

    if (cmd == "rate"){
        int r = 0;
        if (!parseNumberInt(arg, r)){
            printMsg("Provide rate (cycles)");
            return;
        }
        if (r <= 0){
            printMsg("Rate must be positive");
            return;
        }
        dataRatePerMode[debugMode] = static_cast<uint16_t>(r);
        printMsg("Data rate updated");
        return;
    }

    if (cmd == "poskp") { setParam(posControls, 1, arg, "PosKP:"); return; }
    if (cmd == "poski") { setParam(posControls, 2, arg, "PosKI:"); return; }
    if (cmd == "poskd") { setParam(posControls, 3, arg, "PosKD:"); return; }
    if (cmd == "posset") { setParam(posControls, 0, arg, "SetPos:"); return; }

    if (cmd == "balkp") { setParam(balControls, 1, arg, "BalKP:"); return; }
    if (cmd == "balki") { setParam(balControls, 2, arg, "BalKI:"); return; }
    if (cmd == "balkd") { setParam(balControls, 3, arg, "BalKD:"); return; }
    if (cmd == "balset") { setParam(balControls, 0, arg, "SetBal:"); return; }

    if (cmd == "hdgkp") { setParam(hdgControls, 1, arg, "HdgKP:"); return; }
    if (cmd == "hdgki") { setParam(hdgControls, 2, arg, "HdgKI:"); return; }
    if (cmd == "hdgkd") { setParam(hdgControls, 3, arg, "HdgKD:"); return; }
    if (cmd == "hdgset") { setParam(hdgControls, 0, arg, "SetHdg:"); return; }

    printMsg("Unknown command");
}

// Control helpers
float pidStep(float error, float kp, float ki, float kd, float &error_prev, float &integral) {
    integral += error;
    float derivative = error - error_prev;
    error_prev = error;
    return kp * error + ki * integral + kd * derivative;
}

float clampMotorOutput(float value) {
    if (value > 255.0f) return 255.0f;
    if (value < 0.0f) return 0.0f;
    return value;
}

void resetTestState(){
    runMode = 2;
    debugMode = 1;
    dataRatePerMode[0] = 500;
    dataRatePerMode[1] = 500;
    posControls[0] = 0; posControls[1] = 1; posControls[2] = 0.05f; posControls[3] = 0.02f;
    balControls[0] = 0; balControls[1] = 2; balControls[2] = 0.1f; balControls[3] = 0.03f;
    hdgControls[0] = 0; hdgControls[1] = 1; hdgControls[2] = 0; hdgControls[3] = 0;
    gControlState = ControlState();
}

float *getPosParam(){ return posControls; }
float *getBalParam(){ return balControls; }
float *getHdgParam(){ return hdgControls; }
