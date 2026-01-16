#ifndef COMMAND_H
#define COMMAND_H

#include <Arduino.h>

// Initialise command handling (reserves buffers, etc.)
void commandInit();

// Process a full command line (without trailing newline)
void handleCommandLine(const String &line);

// Poll Serial for commands (newline-terminated)
void processSerialCommands();

// Access to mutable controller parameter arrays
float *getPosParam();
float *getBalParam();
float *getHdgParam();

#endif
