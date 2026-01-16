#include "../include/command.h"
#include "../include/messages.h"
#include "../include/control_sys.h"
#include <avr/eeprom.h>  // Direct AVR EEPROM access (saves ~100 bytes RAM vs Arduino EEPROM library)

// modeBits and related externs defined in main.cpp
extern uint8_t modeBits;
extern uint16_t dataRatePerMode[2];
extern float *errPtr;  // Pointer to IMU error array

namespace {
char serialInput[64];  // Fixed char array instead of String (saves ~64 bytes)
uint8_t serialInputIdx = 0;
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
    // No initialization needed for char array
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
        modeBits = (modeBits & 0xFC) | (m & 0x03);  // Update runMode bits
        saveParametersToEEPROM();
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
        modeBits = (modeBits & 0xFB) | ((d & 0x01) << 2);  // Update debugMode bit
        saveParametersToEEPROM();
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
        uint8_t dm = (modeBits >> 2) & 0x01;  // Extract debugMode from modeBits
        dataRatePerMode[dm] = (uint16_t)r;
        saveParametersToEEPROM();
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
        if (ch == '\n' || ch == '\r'){
            if (serialInputIdx > 0){
                serialInput[serialInputIdx] = '\0';  // Null terminate
                serialLineComplete = true;
            }
            break;
        } else if (serialInputIdx < 63) {
            serialInput[serialInputIdx++] = ch;
        }
    }
    if (serialLineComplete){
        handleCommandLine(String(serialInput));  // Convert char array to String for parsing
        serialInputIdx = 0;
        serialLineComplete = false;
    }
}

float *getPosParam(){ return posControls; }
float *getBalParam(){ return balControls; }
float *getHdgParam(){ return hdgControls; }

// EEPROM persistence (~54 bytes total)
// Layout: runMode(1) | debugMode(1) | dataRatePerMode[2](4) | posControls[4](16) | balControls[4](16) | hdgControls[4](16)
#define EEPROM_START_ADDR 0

void saveParametersToEEPROM(){
    uint16_t addr = EEPROM_START_ADDR;
    
    // Save modeBits (packed runMode, debugMode, printData)
    eeprom_write_byte((uint8_t*)addr++, modeBits);
    
    // Save dataRatePerMode[2]
    for (int i = 0; i < 2; i++){
        uint16_t val = dataRatePerMode[i];
        eeprom_write_byte((uint8_t*)addr++, (uint8_t)(val & 0xFF));
        eeprom_write_byte((uint8_t*)addr++, (uint8_t)((val >> 8) & 0xFF));
    }
    
    // Save parameter arrays (3 arrays × 4 floats each)
    float *arrays[3] = {posControls, balControls, hdgControls};
    for (int a = 0; a < 3; a++){
        for (int i = 0; i < 4; i++){
            float fval = arrays[a][i];
            uint8_t *bytes = (uint8_t *)&fval;
            for (int j = 0; j < 4; j++){
                eeprom_write_byte((uint8_t*)addr++, bytes[j]);
            }
        }
    }
}

void loadParametersFromEEPROM(){
    uint16_t addr = EEPROM_START_ADDR;
    
    // Load modeBits (packed runMode, debugMode, printData)
    modeBits = eeprom_read_byte((uint8_t*)addr++);
    
    // Validate modeBits (runMode must be 0-2, debugMode must be 0-1)
    uint8_t rm = modeBits & 0x03;
    uint8_t dm = (modeBits >> 2) & 0x01;
    if (rm > 2) modeBits = (modeBits & 0xFC) | 2;  // Set runMode to 2
    if (dm > 1) modeBits = (modeBits & 0xFB) | 0;  // Set debugMode to 0
    
    // Load dataRatePerMode[2]
    for (int i = 0; i < 2; i++){
        uint8_t lo = eeprom_read_byte((uint8_t*)addr++);
        uint8_t hi = eeprom_read_byte((uint8_t*)addr++);
        dataRatePerMode[i] = (uint16_t)(lo | (hi << 8));
    }
    
    // Load parameter arrays
    float *arrays[3] = {posControls, balControls, hdgControls};
    for (int a = 0; a < 3; a++){
        for (int i = 0; i < 4; i++){
            uint8_t bytes[4];
            for (int j = 0; j < 4; j++){
                bytes[j] = eeprom_read_byte((uint8_t*)addr++);
            }
            memcpy(&arrays[a][i], bytes, 4);
        }
    }
}
