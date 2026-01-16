#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <cctype>

// Arduino type alias for compatibility
typedef uint8_t byte;

// Minimal Arduino-like String replacement for tests
class String {
public:
    char buffer[256];
    size_t len;

    String() : buffer{0}, len(0) {}
    String(const char *s) {
        std::strncpy(buffer, s, sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0';
        len = std::strlen(buffer);
    }

    size_t length() const { return len; }

    void trim() {
        while (len > 0 && (buffer[0] == ' ' || buffer[0] == '\t')) {
            std::memmove(buffer, buffer + 1, len);
            len--;
        }
        while (len > 0 && (buffer[len - 1] == ' ' || buffer[len - 1] == '\t')) {
            len--;
        }
        buffer[len] = '\0';
    }

    int indexOf(char c) const {
        for (size_t i = 0; i < len; i++) {
            if (buffer[i] == c) return static_cast<int>(i);
        }
        return -1;
    }

    String substring(int start, int end) const {
        String result;
        if (start < 0) start = 0;
        if (end > static_cast<int>(len)) end = static_cast<int>(len);
        if (start >= end) return result;
        size_t n = static_cast<size_t>(end - start);
        std::strncpy(result.buffer, buffer + start, n);
        result.buffer[n] = '\0';
        result.len = n;
        return result;
    }

    String substring(int start) const { return substring(start, static_cast<int>(len)); }

    void toLowerCase() {
        for (size_t i = 0; i < len; i++) {
            buffer[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(buffer[i])));
        }
    }

    float toFloat() const { return std::atof(buffer); }
    int toInt() const { return std::atoi(buffer); }

    bool operator==(const char *s) const { return std::strcmp(buffer, s) == 0; }
};

// Global test state
extern uint8_t runMode;
extern uint8_t debugMode;
extern uint16_t dataRatePerMode[2];
extern float posControls[4];
extern float balControls[4];
extern float hdgControls[4];

// Mocks for firmware printing
void printMsg(const char msg[]);
void printFloat(const char label[], float *varPtr, byte offset);
void printInt(const char label[], int *varPtr, byte offset);

// Command helpers
bool parseNumberFloat(const String &arg, float &value);
bool parseNumberInt(const String &arg, int &value);
void setParam(float *arr, int index, const String &arg, const char *label);
void handleCommandLine(const String &line);

// Control helpers
struct ControlState {
    float balError_prev = 0.0f;
    float balInt = 0.0f;
    float posError_prev = 0.0f;
    float posInt = 0.0f;
};

extern ControlState gControlState;
void resetTestState();

float pidStep(float error, float kp, float ki, float kd, float &error_prev, float &integral);
float clampMotorOutput(float value);

// Parameter getters
float *getPosParam();
float *getBalParam();
float *getHdgParam();
