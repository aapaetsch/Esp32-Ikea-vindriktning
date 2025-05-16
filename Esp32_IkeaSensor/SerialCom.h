#pragma once
#include <SoftwareSerial.h>

struct particleSensorState_t {
    uint16_t avgPM25 = 0;
    uint16_t measurements[5] = {0, 0, 0, 0, 0};
    uint8_t measurementIdx = 0;
    bool valid = false;
};

namespace SerialCom {
    // Define the UART pins for SoftwareSerial
    constexpr static const int PIN_UART_RX = 1; // Adjust as per your wiring
    constexpr static const int PIN_UART_TX = 16; // Adjust as per your wiring

    extern uint8_t serialRxBuf[255];
    extern uint8_t rxBufIdx;
    extern SoftwareSerial sensorSerial;

    void setup();
    void clearRxBuf();
    void parseState(particleSensorState_t& state);    bool isValidHeader();
    bool isValidChecksum();
} // namespace SerialCom
