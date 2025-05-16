#include "SerialCom.h"
#include <Arduino.h>

namespace SerialCom {
    uint8_t serialRxBuf[255];
    uint8_t rxBufIdx = 0;
    SoftwareSerial sensorSerial(PIN_UART_RX, PIN_UART_TX);

    void setup() {
        // Begin SoftwareSerial communication with the sensor
        sensorSerial.begin(9600);
        delay(2000); // Wait for the sensor to initialize
    }

    void clearRxBuf() {
        memset(serialRxBuf, 0, sizeof(serialRxBuf));
        rxBufIdx = 0;
    }

    void parseState(particleSensorState_t& state) {
        // PM2.5 = DF3*256 + DF4
        const uint16_t pm25 = (serialRxBuf[5] << 8) | serialRxBuf[6];
        
        #ifdef WEBLOG
        WEBLOG("25: %d", pm25);
        #endif

        // Store the measurement
        state.measurements[state.measurementIdx] = pm25;

        state.measurementIdx = (state.measurementIdx + 1) % 5;

        if (state.measurementIdx == 0) {
            uint32_t sumPM25 = 0;
            for (uint8_t i = 0; i < 5; ++i) {
                sumPM25 += state.measurements[i];
            }

            state.avgPM25 = sumPM25 / 5;
            state.valid = true;
        }

        clearRxBuf();
    }

    bool isValidHeader() {
        bool headerValid = serialRxBuf[0] == 0x16 && serialRxBuf[1] == 0x11 && serialRxBuf[2] == 0x0B;

        #ifdef WEBLOG
        if (!headerValid) {
            WEBLOG("Invalid header received.");
        }
        #endif

        return headerValid;
    }

    bool isValidChecksum() {
        if (rxBufIdx == 0) return false; // Prevent underflow

        uint8_t checksum = 0;

        // Calculate checksum over all bytes except the last one
        for (uint8_t i = 0; i < rxBufIdx - 1; i++) {
            checksum += serialRxBuf[i];
        }

        checksum = (256 - checksum) % 256; // Two's complement

        #ifdef WEBLOG
        if (checksum != serialRxBuf[rxBufIdx - 1]) {
            WEBLOG("Invalid checksum. Calculated: 0x%02X, Received: 0x%02X", checksum, serialRxBuf[rxBufIdx - 1]);
            return false;
        }
        #endif

        return true;
    }
} // namespace SerialCom
