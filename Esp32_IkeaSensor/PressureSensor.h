#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <HomeSpan.h>

// --- Eve-compatible custom service/characteristic for pressure ---
CUSTOM_SERV(AtmosphericPressureSensor, E863F00A-079E-48FF-8F27-9C2605A29F52);
CUSTOM_CHAR(AtmosphericPressure, E863F10F-079E-48FF-8F27-9C2605A29F52, PR+EV, FLOAT, 1013, 100, 13000, false);

struct PressureSensor : Service::AtmosphericPressureSensor {
  Characteristic::AtmosphericPressure pressure;
  float* avgPressurePtr;
  
  PressureSensor(float* pressureValue) : Service::AtmosphericPressureSensor() {
    Serial.println("Configuring Air Pressure Sensor");
    avgPressurePtr = pressureValue;
  }  
  
  void loop() override {
    float avgPressure = *avgPressurePtr;
    
    // Log pressure value periodically regardless of validity
    static unsigned long lastStatusLog = 0;
    if (millis() - lastStatusLog > 60000) { // Log status every minute
      Serial.printf("PressureSensor: Current pressure value is %.1f hPa\n", avgPressure);
      lastStatusLog = millis();
    }
    
    if (!isnan(avgPressure) && avgPressure > 700 && avgPressure < 1200) {
      pressure.setVal(avgPressure);  // HomeKit uses hPa
      
      // Log occasional successful updates
      static unsigned long lastSuccessLog = 0;
      if (millis() - lastSuccessLog > 300000) { // Log success every 5 minutes
        Serial.printf("PressureSensor: Successfully updated HomeKit with pressure: %.1f hPa\n", avgPressure);
        lastSuccessLog = millis();
      }
    } else {
      // Log errors occasionally to avoid spamming
      static unsigned long lastErrorLog = 0;
      if (millis() - lastErrorLog > 30000) { // Log at most every 30 seconds
        if (isnan(avgPressure)) {
          Serial.println("PressureSensor ERROR: avgPressure is NaN");
        } else if (avgPressure <= 100 || avgPressure >= 13000) {
          Serial.printf("PressureSensor ERROR: avgPressure out of range: %.1f hPa\n", avgPressure);
        }
        lastErrorLog = millis();
      }
    }
  }
};

#endif // PRESSURE_SENSOR_H
