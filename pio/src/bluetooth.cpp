#include "../include/bluetooth.h"
#include "../include/control_sys.h"
#include "../include/messages.h"
#include "../include/command.h"

// Bluetooth serial instance
SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

namespace {
char btInputString[64];  // Fixed char array instead of String (saves ~64 bytes)
uint8_t btInputIdx = 0;
bool btStringComplete = false;
}

void initBtSerial(){
    BTSerial.begin(9600);
    setBtMessagesEnabled(true);
    printMsg("[INIT] Bluetooth");
}

void readBtSerial(){
    while (BTSerial.available()){
        char ch = (char)BTSerial.read();
        if (ch == '\n' || ch == '\r') {
            if (btInputIdx > 0){
                btInputString[btInputIdx] = '\0';  // Null terminate
                btStringComplete = true;
            }
            break;
        } else if (btInputIdx < 63) {
            btInputString[btInputIdx++] = ch;
        }
    }

    if (btStringComplete){
        handleCommandLine(String(btInputString));  // Convert char array to String for parsing
        btInputIdx = 0;
        btStringComplete = false;
    }
}   