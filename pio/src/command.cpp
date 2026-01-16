#include "../include/command.h"
#include "../include/messages.h"
#include "../include/control_sys.h"

// runMode is defined in main.cpp
extern uint8_t runMode;
extern uint8_t debugMode;
extern uint16_t dataRatePerMode[2];

namespace {
String serialInput;
bool serialLineComplete = false;

// Persistent, editable control parameter arrays
float posControls[4] = {POS_SETPOINT, POS_KP, POS_KI, POS_KD};
float balControls[4] = {BAL_SETPOINT, BAL_KP, BAL_KI, BAL_KD};
float hdgControls[4] = {HDG_SETPOINT, HDG_KP, HDG_KI, HDG_KD};

bool parseNumberFloat(const String &arg, float &value) {
    if (arg.length() == 0) return false;
    bool hasDigit = false;
    for (unsigned int i = 0; i < arg.length(); i++) {
        char c = arg[i];
        if (!(isDigit(c) || c=='-' || c=='+' || c=='.')) {
            return false;
        }
        if (isDigit(c)) hasDigit = true;
    }
    if (!hasDigit) return false;
    value = arg.toFloat();
    return true;
}

bool parseNumberInt(const String &arg, int &value) {
    if (arg.length() == 0) return false;
    bool neg = (arg[0] == '-');
    for (unsigned int i = neg ? 1 : 0; i < arg.length(); i++) {
        if (!isDigit(arg[i])) return false;
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
    printFloat(label, arr, index);
    printMsg("");
}
}

void commandInit(){
    serialInput.reserve(128);
}

void handleCommandLine(const String &line){
    String trimmed = line;
    trimmed.trim();
    if (trimmed.length() == 0) return;

    int sp = trimmed.indexOf(' ');
    String cmd = (sp == -1) ? trimmed : trimmed.substring(0, sp);
    String arg = (sp == -1) ? "" : trimmed.substring(sp + 1);
    cmd.toLowerCase();

    if (cmd == "status"){
        printMsg("Status OK");
        return;
    }

    // Run mode
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
        runMode = (uint8_t)m;
        printMsg("Mode updated");
        return;
    }

    // Debug mode (0=compact, 1=verbose)
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
        debugMode = (uint8_t)d;
        printMsg("Debug mode updated");
        return;
    }

    // Data rate for current debug mode
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
        dataRatePerMode[debugMode] = (uint16_t)r;
        printMsg("Data rate updated");
        return;
    }

    // Position
    if (cmd == "poskp") { setParam(posControls, 1, arg, "PosKP:"); return; }
    if (cmd == "poski") { setParam(posControls, 2, arg, "PosKI:"); return; }
    if (cmd == "poskd") { setParam(posControls, 3, arg, "PosKD:"); return; }
    if (cmd == "posset") { setParam(posControls, 0, arg, "SetPos:"); return; }

    // Balance
    if (cmd == "balkp") { setParam(balControls, 1, arg, "BalKP:"); return; }
    if (cmd == "balki") { setParam(balControls, 2, arg, "BalKI:"); return; }
    if (cmd == "balkd") { setParam(balControls, 3, arg, "BalKD:"); return; }
    if (cmd == "balset") { setParam(balControls, 0, arg, "SetBal:"); return; }

    // Heading
    if (cmd == "hdgkp") { setParam(hdgControls, 1, arg, "HdgKP:"); return; }
    if (cmd == "hdgki") { setParam(hdgControls, 2, arg, "HdgKI:"); return; }
    if (cmd == "hdgkd") { setParam(hdgControls, 3, arg, "HdgKD:"); return; }
    if (cmd == "hdgset") { setParam(hdgControls, 0, arg, "SetHdg:"); return; }

    printMsg("Unknown command");
}

void processSerialCommands(){
    while (Serial.available()){
        char ch = (char)Serial.read();
        if (ch == '\n'){
            serialLineComplete = true;
            break;
        } else {
            serialInput += ch;
        }
    }
    if (serialLineComplete){
        handleCommandLine(serialInput);
        serialInput = "";
        serialLineComplete = false;
    }
}

float *getPosParam(){ return posControls; }
float *getBalParam(){ return balControls; }
float *getHdgParam(){ return hdgControls; }
