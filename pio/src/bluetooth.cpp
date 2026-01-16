#include "../include/bluetooth.h"
#include "../include/control_sys.h"
#include "../include/messages.h"
#include "../include/command.h"

// Bluetooth serial instance
SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

namespace {
String btInputString = "";
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
        if (ch == '\n') {
            btStringComplete = true;
            break;
        } else {
            btInputString += ch;
        }
    }

    if (btStringComplete){
        btInputString.trim();
        handleCommandLine(btInputString);

        btInputString = "";
        btStringComplete = false;
    }
}   