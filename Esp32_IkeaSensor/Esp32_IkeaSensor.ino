/*
  AirQualityHomeSpan.ino

  ESP32-S3 Zero Arduino sketch integrating:
    - IKEA Vindriktning (Cubic PM1006 PM2.5 sensor via UART)
    - ENS160 + AHT21 gas/temperature/humidity combo (I2C)
    - BMP280 + AHT21 combo (I2C2)
    - 128×32 I2C OLED display
  into Apple HomeKit using HomeSpan
*/
#include <HomeSpan.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include "PressureSensor.h"
#include "SerialCom.h"
#define VERS "0.1.7"  // Increment version for ens repin

// === Pin definitions ===
#define PM_RX_PIN    1   // RX2 receives PM1006 TX
#define PM_TX_PIN    16   // TX2 (unused)
#define OLED_RESET   -1
#define I2C_SDA      8   // I²C SDA (ENS160, AHT21)
#define I2C_SCL      7 // I²C SCL
#define I2C2_SDA     5   // I²C2 SDA (BMP280, AHT21, OLED)
#define I2C2_SCL     6   // I2C2 SCL
#define QRID "000-00-000" // QR code ID for pairing
#define PAIRING_CODE "00000000" // Pairing code for HomeKit

// === DFRobot ENS160 AQI definitions ===
#define ENS160_AQI_EXCELLENT 1
#define ENS160_AQI_GOOD      2
#define ENS160_AQI_MODERATE  3
#define ENS160_AQI_POOR      4
#define ENS160_AQI_UNHEALTHY 5

// === Warm‑up ===
#define ENS_WARMUP_MS 180000  // ENS160 warm‑up duration (ms)
#define ENS_SLEEP_PERIOD_MS (15 * 60 * 1000) // 15 minutes sleep time
#define ENS_READINGS_BEFORE_SLEEP 60 // Number of readings before sleep cycle
unsigned long ensStartTime = 0;
bool ensWarmed = false;
bool ensSleeping = false;
unsigned long ensSleepStartTime = 0;
int ensReadingCount = 0;

// === Polling intervals (ms) ===
unsigned long aht1PollInterval = 5000;   // ENS160 board AHT21
unsigned long aht2PollInterval = 5000;   // BMP280 board AHT21
unsigned long pm25PollInterval = 5000;   // PM1006w
unsigned long bmpPollInterval  = 5000;   // BMP280
unsigned long ensPollInterval  = 3000;   // ENS160 gas sensor (separate from AHT21)

unsigned long lastAHT1Poll = 0;
unsigned long lastAHT2Poll = 0;
unsigned long lastPM25Poll = 0;
unsigned long lastBMPPoll  = 0;
unsigned long lastENSPoll  = 0;

// --- Cached sensor values ---
float lastTemp1 = 0, lastHum1 = 0;
float lastTemp2 = 0, lastHum2 = 0;
float lastPM25 = 0;
float lastPressure = 0;
float lastAltitude = 0;  // Last altitude reading in meters
float co2PeakLevel = 400.0; // Track highest CO2 reading since reset

// === Status Variables ===
bool wifiConnected = false;          // WiFi connection status
bool homeKitPaired = false;          // HomeKit pairing status

// === Objects ===
// Using standard Wire libraries for I2C buses
// Wire = I2C0, Wire1 = I2C1

DFRobot_ENS160_I2C ens(&Wire, 0x53); // DFRobot ENS160 on default address (Wire)
Adafruit_AHTX0   aht; // For ENS160 board (Wire)
Adafruit_AHTX0   aht2; // For BMP280 board (Wire1)

particleSensorState_t pmState;
Adafruit_SSD1306 display(128, 32, &Wire1, OLED_RESET); // OLED on Wire1 bus
Adafruit_BMP280 bmp(&Wire1); // Will use Wire1

// === Rolling buffers for averaging last 20 readings ===
const uint8_t AVG_SAMPLES = 20;
float   temp1Buf[AVG_SAMPLES] = {0};
float   temp2Buf[AVG_SAMPLES] = {0};
float   hum1Buf[AVG_SAMPLES]  = {0};
float   hum2Buf[AVG_SAMPLES]  = {0};
float   tempBuf[AVG_SAMPLES] = {0};
float   humBuf[AVG_SAMPLES]  = {0};
uint16_t co2Buf[AVG_SAMPLES] = {0};
uint16_t tvocBuf[AVG_SAMPLES] = {0};
uint8_t  aqiBuf[AVG_SAMPLES] = {0};
uint16_t aqi500Buf[AVG_SAMPLES] = {0};
float   pm25Buf[AVG_SAMPLES] = {0};
float   pressureBuf[AVG_SAMPLES] = {0};
// Individual buffer indices for each sensor type
uint8_t tempHumBufIndex = 0;   // For temperature and humidity
uint8_t pm25BufIndex = 0;      // For PM2.5 readings
uint8_t bmpBufIndex = 0;       // For BMP280 (pressure/altitude)
uint8_t ensBufIndex = 0;       // For ENS160 (CO2, TVOC, AQI)
float   temp1Sum = 0, temp2Sum = 0, hum1Sum = 0, hum2Sum = 0;
float   tempSum = 0, humSum = 0, pm25Sum = 0, pressureSum = 0;
uint32_t co2Sum = 0, tvocSum = 0;
uint16_t aqiSum = 0, aqi500Sum = 0;

// --- Rolling averages for each sensor ---
float avgTemp1 = 0, avgHum1 = 0, avgTemp2 = 0, avgHum2 = 0;
float avgTemp = 0, avgHum = 0;
float avgPM25 = 0, avgPressure = 0, avgAltitude = 0;
float avgCO2 = 0, avgTVOC_ppb = 0, avgTVOC_ugm3 = 0;
uint8_t avgAQI_raw = 0, avgAQI = 0;
uint16_t avgAQI500 = 0;

// === HomeSpan characteristic pointers ===
SpanCharacteristic *pTempChar;
SpanCharacteristic *pHumChar;
SpanCharacteristic *pCO2Char;
SpanCharacteristic *pCO2DetectedChar;
SpanCharacteristic *pCO2PeakLevelChar; // CO2 peak level characteristic
SpanCharacteristic *pTVOCChar;
SpanCharacteristic *pAQIChar;
SpanCharacteristic *pPM25Char;
// Add pointers for individual sensors
SpanCharacteristic *pTemp1Char;
SpanCharacteristic *pHum1Char;
SpanCharacteristic *pTemp2Char;
SpanCharacteristic *pHum2Char;

// PressureSensor is now defined in PressureSensor.h

// --- Function Prototypes ---
void initHomeKit();
void initSensors();
void handleENS160(float temp, float hum, uint16_t &co2, uint16_t &tvoc, uint8_t &aqi, uint16_t &aqi500);
float updateRollingAverages_AHT(float *buf, float &sum, float value);
void updateRollingAverages_PM25(float pm25);
void updateRollingAverages_BMP(float pressure, float altitude);
void updateRollingAverages_ENS160(uint16_t co2, uint16_t tvoc, uint8_t aqi, uint16_t aqi500);
void updateRollingAverages_Combined();
void publishToHomeKit(float avgTemp, float avgHum, float avgCO2, bool co2Detected, float avgTVOC_ugm3, float avgPM25, float avgPressure, uint8_t avgAQI);
void updateOLED(float avgTemp, float avgHum, float avgCO2, bool co2Detected, float avgPM25, float avgTVOC_ugm3, uint8_t avgAQI, uint16_t avgAQI500, float avgPressure, float avgAltitude);
void printAvgArraysToWeblog();

// --- Setup and loop functions ---

void setup() {
  // Initialize serial communication first for debugging
  Serial.begin(115200);
  Serial.println("Starting Esp32_IkeaSensor v" VERS "...");
  
  // Initialize HomeKit and sensors
  initHomeKit();
  WEBLOG("\n\n==== STARTING ESP32-S3-Zero Environmental Monitor ====\n");
  initSensors();
}

void loop() {
  unsigned long now = millis();
  
  // Process HomeSpan events
  homeSpan.poll();
  
  // Check if it's time to poll the ENS160 board AHT21 sensor
  if (now - lastAHT1Poll >= aht1PollInterval) {
    sensors_event_t humidity, temp;
    
    // Read AHT21 on I2C bus 1
    if (aht.getEvent(&humidity, &temp)) {
      // Update last temp and humidity values
      lastTemp1 = temp.temperature;
      lastHum1 = humidity.relative_humidity;
      
      // Update rolling averages
      avgTemp1 = updateRollingAverages_AHT(temp1Buf, temp1Sum, lastTemp1);
      avgHum1 = updateRollingAverages_AHT(hum1Buf, hum1Sum, lastHum1);
      
      // Update combined temperature and humidity averages
      updateRollingAverages_Combined();
      
      // Read ENS160 sensor (CO2, VOC, AQI) with fresh temperature/humidity values
      uint16_t co2, tvoc;
      uint8_t aqi;
      uint16_t aqi500;
      
      // Pass temperature and humidity for compensation
      handleENS160(lastTemp1, lastHum1, co2, tvoc, aqi, aqi500);
      
      // Update rolling averages for ENS160 readings
      updateRollingAverages_ENS160(co2, tvoc, aqi, aqi500);
      
      // Debug output
      WEBLOG("AHT21 (ENS160 board): T=%.1f°C, H=%.1f%%\n", lastTemp1, lastHum1);
      WEBLOG("ENS160: CO2=%uppm, TVOC=%uppb, AQI=%u/5\n", co2, tvoc, aqi);
    } else {
      WEBLOG("ERROR: Failed to read from AHT21 (ENS160 board)\n");
    }
    
    lastAHT1Poll = now;
  }
  
  // Separate polling for ENS160 sensor using the latest temp/humidity values
  // This allows more frequent ENS160 updates without waiting for fresh AHT21 readings
  if (now - lastENSPoll >= ensPollInterval) {
    // Only proceed if we have valid temperature and humidity values
    if (!isnan(lastTemp1) && !isnan(lastHum1)) {
      uint16_t co2, tvoc;
      uint8_t aqi;
      uint16_t aqi500;
      
      // Use the most recent temperature/humidity values for compensation
      handleENS160(lastTemp1, lastHum1, co2, tvoc, aqi, aqi500);
      
      // Update rolling averages for ENS160 readings
      updateRollingAverages_ENS160(co2, tvoc, aqi, aqi500);
      
      // Check if values are out of reasonable range, which might indicate sensor issues
      static unsigned long lastENSRescan = 0;
      if ((co2 < 300 || co2 > 5000 || tvoc > 10000) && millis() - lastENSRescan > 300000) { // Check every 5 minutes
        WEBLOG("ENS160 reported out-of-range values (CO2: %u, TVOC: %u), rescanning I2C bus...\n", co2, tvoc);
        lastENSRescan = millis();
        scanI2CBus(Wire, "Wire (I2C bus 0)");
        tryStartEns(); // Try to reinitialize
      }
      
      // Debug output
      WEBLOG("ENS160: CO2=%uppm, TVOC=%uppb, AQI=%u/5\n", co2, tvoc, aqi);
    }
    
    lastENSPoll = now;
  }
  
  // Check if it's time to poll the BMP280 board AHT21 sensor
  if (now - lastAHT2Poll >= aht2PollInterval) {
    sensors_event_t humidity, temp;
    
    // Read AHT21 on I2C bus 2
    if (aht2.getEvent(&humidity, &temp)) {
      // Update last values
      lastTemp2 = temp.temperature;
      lastHum2 = humidity.relative_humidity;
      
      // Update rolling averages
      avgTemp2 = updateRollingAverages_AHT(temp2Buf, temp2Sum, lastTemp2);
      avgHum2 = updateRollingAverages_AHT(hum2Buf, hum2Sum, lastHum2);
      
      // Update combined temperature and humidity averages
      updateRollingAverages_Combined();
      
      // Debug output
      WEBLOG("AHT21 (BMP board): T=%.1f°C, H=%.1f%%\n", lastTemp2, lastHum2);
    } else {
      WEBLOG("ERROR: Failed to read from AHT21 (BMP board)\n");
    }
    
    lastAHT2Poll = now;
  }
  
  // Check if it's time to poll the BMP280 sensor
  if (now - lastBMPPoll >= bmpPollInterval) {
    // Read pressure and altitude from BMP280
    float pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa
    float altitude = bmp.readAltitude(1013.25); // Using standard sea level pressure
    
    // Update last values
    lastPressure = pressure;
    lastAltitude = altitude;
    
    // Update rolling averages
    updateRollingAverages_BMP(pressure, altitude);
    
    lastBMPPoll = now;
  }
  
  // Check if it's time to poll the PM1006 sensor
  if (now - lastPM25Poll >= pm25PollInterval) {
    // Check if data is available from the PM sensor
    while (SerialCom::sensorSerial.available()) {
      uint8_t b = SerialCom::sensorSerial.read();
      
      // Store the byte in the buffer
      SerialCom::serialRxBuf[SerialCom::rxBufIdx++] = b;
      
      // Message is complete when we receive 20 bytes
      if (SerialCom::rxBufIdx == 20) {
        if (SerialCom::isValidHeader() && SerialCom::isValidChecksum()) {
          SerialCom::parseState(pmState);
          
          if (pmState.valid) {
            // Update PM2.5 value
            lastPM25 = pmState.avgPM25;
            
            // Update rolling average
            updateRollingAverages_PM25(lastPM25);
            
            WEBLOG("PM2.5: %.1f µg/m³\n", lastPM25);
          }
        } else {
          // Invalid message, clear buffer
          SerialCom::clearRxBuf();
        }
      }
    }
    
    lastPM25Poll = now;
  }
  
  // Calculate whether CO2 is over threshold for detection
  bool co2Detected = avgCO2 > 1000.0; // Threshold for CO2 detection (1000 ppm)
  
  // Update HomeKit with the latest sensor data
  publishToHomeKit(avgTemp2, avgHum2, avgCO2, co2Detected, avgTVOC_ugm3, avgPM25, avgPressure, avgAQI);
  
  // Update OLED display with latest data
  static unsigned long lastOLEDUpdate = 0;
  if (now - lastOLEDUpdate >= 1000) { // Update display once per second
    updateOLED(avgTemp2, avgHum2, avgCO2, co2Detected, avgPM25, avgTVOC_ugm3, avgAQI, avgAQI500, avgPressure, avgAltitude);
    lastOLEDUpdate = now;
  }
}

// --- Function Definitions ---
void initHomeKit() {
  // Set up HomeSpan configuration before calling begin()
  homeSpan.setControlPin(0);             // Disable control pin (0 = disabled)
  homeSpan.enableOTA();                  // Enable over-the-air updates
  // homeSpan.enableAutoStartAP();          // Enable auto-start of setup access point
  
  // Set a custom 8-digit pairing code (format: XXX-XX-XXX)
  homeSpan.setPairingCode(PAIRING_CODE); 
  homeSpan.setQRID(QRID);
  
  // Set a custom name for the setup access point
  // homeSpan.setApSSID("EnvMonitor-Setup");
  // homeSpan.setApPassword("envmonitor123"); // Optional: access point password
  
  // Initialize HomeSpan with device category and name
  homeSpan.begin(Category::Sensors, "EnvMonitor");
  homeSpan.enableWebLog(500);
  WEBLOG("HomeSpan initialized with WebLog and OTA enabled\n");
  WEBLOG("HomeKit pairing code: %s\n", QRID);
  
  new SpanAccessory();
    new Service::AccessoryInformation();
      new Characteristic::Manufacturer("ALP");
      new Characteristic::Model("Ikea Vikdriktning+");
      new Characteristic::SerialNumber("0001");
      new Characteristic::Identify();
      new Characteristic::FirmwareRevision (VERS);
  // Combined average
  new Service::TemperatureSensor();
  pTempChar = new Characteristic::CurrentTemperature();
  new Service::HumiditySensor();
  pHumChar = new Characteristic::CurrentRelativeHumidity();
  // AHT1
  new Service::TemperatureSensor();
  pTemp1Char = new Characteristic::CurrentTemperature();
  new Service::HumiditySensor();
  pHum1Char = new Characteristic::CurrentRelativeHumidity();
  // AHT2
  new Service::TemperatureSensor();
  pTemp2Char = new Characteristic::CurrentTemperature();
  new Service::HumiditySensor();
  pHum2Char = new Characteristic::CurrentRelativeHumidity();
  new Service::CarbonDioxideSensor();
  pCO2Char = new Characteristic::CarbonDioxideLevel();
  pCO2DetectedChar = new Characteristic::CarbonDioxideDetected(0);
  pCO2PeakLevelChar = new Characteristic::CarbonDioxidePeakLevel(400.0);
  new Service::AirQualitySensor();
  pAQIChar = new Characteristic::AirQuality();
  pPM25Char = new Characteristic::PM25Density(0);
  // VOCDensity expects values in micrograms per cubic meter (μg/m³)
  pTVOCChar = new Characteristic::VOCDensity(0);
  new PressureSensor(&avgPressure); // Eve-compatible pressure sensor
  WEBLOG("HomeKit services configured\n");
}

// Scan I2C bus for devices
void scanI2CBus(TwoWire &wire, const char* busName) {
  WEBLOG("Scanning %s for devices...\n", busName);
  byte error, address;
  int deviceCount = 0;
  
  for(address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    error = wire.endTransmission();
    
    if (error == 0) {
      WEBLOG("- I2C device found at address 0x%02X\n", address);
      deviceCount++;
    }
    else if (error == 4) {
      WEBLOG("- Unknown error at address 0x%02X\n", address);
    }
  }
  
  if (deviceCount == 0) {
    WEBLOG("No I2C devices found on %s\n", busName);
  } else {
    WEBLOG("Found %d device(s) on %s\n", deviceCount, busName);
  }
}

void tryStartEns() {
  WEBLOG("Attempting to initialize ENS160 gas sensor on address 0x53...\n");
  
  // Try direct I2C communication first to test the bus
  Wire.beginTransmission(0x53); // ENS160 address
  byte error = Wire.endTransmission();
  
  // Interpret the I2C error code
  switch (error) {
    case 0:
      WEBLOG("  I2C communication with ENS160 successful\n");
      break;
    case 1:
      WEBLOG("  I2C ERROR: Data too long for buffer\n");
      break;
    case 2:
      WEBLOG("  I2C ERROR: Address NACK - device not responding at 0x53\n");
      WEBLOG("  Check wiring and pull-up resistors\n");
      break;
    case 3:
      WEBLOG("  I2C ERROR: Data NACK - device refused data\n");
      break;
    case 4:
      WEBLOG("  I2C ERROR: Unknown error\n");
      break;
  }
  
  if (ens.begin() != 0) {
    WEBLOG("ERROR: ENS160 init failed. Please check connections and restart.\n");
  } else {
    WEBLOG("ENS160 initialized, warming up for %lu ms\n", ENS_WARMUP_MS);
    ensStartTime = millis();
    ens.setPWRMode(ENS160_STANDARD_MODE);
  }
}

void initSensors() {
  SerialCom::setup();
  WEBLOG("PM1006 UART initialized using SerialCom's SoftwareSerial\n");
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // 100kHz for better stability
  
  WEBLOG("Wire (I2C bus 0) initialized on pins SDA=%d, SCL=%d with internal pull-ups\n", I2C_SDA, I2C_SCL);
  scanI2CBus(Wire, "Wire (I2C bus 0)");
  
  tryStartEns(); // Start ENS160 sensor
  
  // Configure and check sensor status
  uint8_t status = ens.getENS160Status();
  WEBLOG("ENS160 initial status: 0x%02X\n", status);


  if (!aht.begin(&Wire)) {
    WEBLOG("ERROR: AHT21 (ENS160 board) init failed.\n");
  } else {
    WEBLOG("AHT21 (ENS160 board) initialized\n");
  }
  
  Wire1.begin(I2C2_SDA, I2C2_SCL);
  Wire1.setClock(100000); // 100kHz for better stability
  WEBLOG("Wire1 (I2C bus 1) initialized on pins SDA=%d, SCL=%d\n", I2C2_SDA, I2C2_SCL);
  scanI2CBus(Wire1, "Wire1 (I2C bus 1)");
  
  // Init variables (for initial averaging)
  for (int i = 0; i < AVG_SAMPLES; i++) {
    tempBuf[i] = 0.0; // 20°C
    humBuf[i]  = 0.0; // 50% RH
    temp1Buf[i] = 0.0;
    temp2Buf[i] = 0.0;
    hum1Buf[i]  = 0.0;
    hum2Buf[i]  = 0.0;
    pm25Buf[i] = 0.0;
    pressureBuf[i] = 0.0; // 1013 hPa (sea level)
    co2Buf[i]  = 400; // 400 ppm (outdoor level)
    tvocBuf[i] = 0;
    aqiBuf[i]  = 1; // 1 = good
    aqi500Buf[i] = 10; // Good AQI
  }
  
  // Initial altitude (not averaged)
  lastAltitude = 0.0;  // Sea level as default
  avgAltitude = lastAltitude;
  
  // Initialize peak CO2 level with standard outdoor value
  co2PeakLevel = 0.0;
  
  // Calculate initial sums for averaging
  for (int i = 0; i < AVG_SAMPLES; i++) {
    temp1Sum += temp1Buf[i];
    temp2Sum += temp2Buf[i];
    hum1Sum += hum1Buf[i];
    hum2Sum += hum2Buf[i];
    tempSum += tempBuf[i];
    humSum += humBuf[i];
    pm25Sum += pm25Buf[i];
    pressureSum += pressureBuf[i];
    co2Sum += co2Buf[i];
    tvocSum += tvocBuf[i];
    aqiSum += aqiBuf[i];
    aqi500Sum += aqi500Buf[i];
  }
  
  // Set initial averages
  avgTemp1 = temp1Sum / AVG_SAMPLES;
  avgTemp2 = temp2Sum / AVG_SAMPLES;
  avgHum1 = hum1Sum / AVG_SAMPLES;
  avgHum2 = hum2Sum / AVG_SAMPLES;
  avgTemp = tempSum / AVG_SAMPLES;
  avgHum = humSum  / AVG_SAMPLES;
  avgPM25 = pm25Sum / AVG_SAMPLES;
  avgPressure = pressureSum / AVG_SAMPLES;
  // Altitude is not averaged - already set from lastAltitude
  avgCO2 = co2Sum / AVG_SAMPLES;
  avgTVOC_ppb = tvocSum / AVG_SAMPLES;
  avgAQI = aqiSum / AVG_SAMPLES;
  avgAQI500 = aqi500Sum / AVG_SAMPLES;
  
  // Try to detect BMP280 on Wire1 bus
  WEBLOG("Attempting to initialize BMP280 pressure sensor on Wire1 (address 0x76)...\n");
  if (!bmp.begin(0x76)) {  // Wire1 already passed in constructor
    WEBLOG("ERROR: BMP280 init failed. Check wiring or try alternate address (0x77).\n");
    // Try alternate address
    WEBLOG("Attempting BMP280 with alternate address (0x77)...\n");
    if (!bmp.begin(0x77)) {
      WEBLOG("ERROR: BMP280 init failed with both addresses. Check wiring and connections.\n");
    } else {
      WEBLOG("BMP280 initialized at alternate address 0x77\n");
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_500);
    }
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                   Adafruit_BMP280::SAMPLING_X2,
                   Adafruit_BMP280::SAMPLING_X16,
                   Adafruit_BMP280::FILTER_X16,
                   Adafruit_BMP280::STANDBY_MS_500);
    WEBLOG("BMP280 initialized at address 0x76 successfully\n");
  }
  if (!aht2.begin(&Wire1)) {  // Use Wire1 for AHT21 on BMP280 board
    WEBLOG("ERROR: AHT21 (BMP board) init failed.\n");
  } else {
    WEBLOG("AHT21 (BMP board) initialized\n");
  }
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {  // Use explicit parameters for Wire1
    WEBLOG("ERROR: OLED init failed.\n");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setRotation(2);  // Rotate display 180 degrees (flip)
    WEBLOG("OLED initialized on Wire1 with 180-degree rotation\n");
  }
}

void handleENS160(float temp, float hum, uint16_t &co2, uint16_t &tvoc, uint8_t &aqi, uint16_t &aqi500) {
  // Check if ENS160 is currently in sleep mode
  if (ensSleeping) {
    unsigned long elapsed = millis() - ensSleepStartTime;
    if (elapsed >= ENS_SLEEP_PERIOD_MS) {
      // Time to wake up the sensor
      WEBLOG("ENS160 sleep period complete. Waking up sensor.\n");
      ens.setPWRMode(ENS160_STANDARD_MODE); // Wake up the sensor
      ensStartTime = millis();
      ensSleeping = false;
      ensWarmed = false;
      ensReadingCount = 0;
      
      // Use default values while warming up again
      co2 = co2Buf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      tvoc = tvocBuf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      aqi = aqiBuf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      aqi500 = aqi500Buf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      return;
    } else {
      // Still in sleep mode, use previous readings
      static unsigned long lastSleepLog = 0;
      if (millis() - lastSleepLog > 60000) { // Log once per minute during sleep
        unsigned long sleepLeft = (ENS_SLEEP_PERIOD_MS - elapsed) / 1000;
        WEBLOG("ENS160 sleeping, %lu seconds left until wake-up\n", sleepLeft);
        lastSleepLog = millis();
      }
      
      co2 = co2Buf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      tvoc = tvocBuf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      aqi = aqiBuf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      aqi500 = aqi500Buf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      return;
    }
  }

  // Check if ENS160 is in error state or not ready
  int status = ens.getENS160Status();
  
  // Decode and log status if there's an issue
  if (status != 0) {
    static unsigned long lastStatusLog = 0;
    static unsigned long lastRescan = 0;
    
    // Log status message periodically if there's an issue
    if (millis() - lastStatusLog > 10000) { // Log every 10 seconds if there's an issue
      lastStatusLog = millis();
      WEBLOG("ENS160 status: 0x%02X - ", status);
      
      // Interpret status bits
      if (status & 0x01) WEBLOG("Running | ");
      if (status & 0x02) WEBLOG("STATERR | ");
      if (status & 0x04) WEBLOG("DATAERR | ");
      if (status & 0x08) WEBLOG("BOOTING | ");
      if (status & 0x10) WEBLOG("BUSY | ");
      if (status & 0x80) WEBLOG("NEWGPR | ");
      WEBLOG("\n");
    }
  }
  
  // Handle warmup state
  if (!ensWarmed) {
    unsigned long elapsed = millis() - ensStartTime;
    if (elapsed >= ENS_WARMUP_MS && status == 0) {
      ensWarmed = true;
      WEBLOG("ENS160 warm-up complete. Starting measurements.\n");
    } else {
      // Check if elapsed time is less than warmup time to avoid overflow
      if (elapsed < ENS_WARMUP_MS) {
        unsigned long left = (ENS_WARMUP_MS - elapsed) / 1000;
        WEBLOG("ENS160 warming up, %lus left\n", left);
      } else {
        // We've exceeded warmup time but status isn't 0
        WEBLOG("ENS160 warmup time exceeded but sensor not ready (status: 0x%02X)\n", status);
      }
    }
  }

  if (!isnan(temp) && !isnan(hum)) {
      // Just call the method without trying to capture a return value
      ens.setTempAndHum(temp, hum);
  }
  
  if (ensWarmed && status == 0) {
    // Check if we should enter sleep mode after collecting enough readings
    if (ensReadingCount >= ENS_READINGS_BEFORE_SLEEP) {
      // Put sensor to sleep to save power and extend lifetime
      WEBLOG("ENS160 collected %d readings, entering sleep mode for %lu minutes...\n", 
             ensReadingCount, ENS_SLEEP_PERIOD_MS / 60000);
      ens.setPWRMode(ENS160_SLEEP_MODE);
      ensSleeping = true;
      ensSleepStartTime = millis();
      
      // Get last readings from the buffer
      co2 = co2Buf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      tvoc = tvocBuf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      aqi = aqiBuf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      aqi500 = aqi500Buf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      return;
    }
    
    co2 = ens.getECO2();
    tvoc = ens.getTVOC();
    aqi = ens.getAQI();
    
    // Increment reading count when we get a successful reading
    ensReadingCount++;
    
    // Update CO2 peak level if current average is higher
    if (avgCO2 > co2PeakLevel && avgCO2 <= 10000) {  // Sanity check to avoid unreasonable values
      co2PeakLevel = avgCO2;
      WEBLOG("New CO2 peak level: %.1f ppm\n", co2PeakLevel);
    }
    
    // Generate AQI500 value (0-500 scale) since DFRobot doesn't provide this
    // Using EPA-like formula based on CO2 and TVOC levels
    uint16_t co2_contribution = map(constrain(co2, 400, 5000), 400, 5000, 0, 350);
    uint16_t tvoc_contribution = map(constrain(tvoc, 0, 2000), 0, 2000, 0, 150);
    aqi500 = co2_contribution + tvoc_contribution;
    aqi500 = constrain(aqi500, 0, 500);
    
    // Log debug info more frequently
    static unsigned long lastDebugLog = 0;
    if (millis() - lastDebugLog > 20000) { // Every 20 seconds
      // Convert TVOC from ppb to μg/m³ for logging (using standard conversion factor of 4.57)
      float tvoc_ugm3 = tvoc * 4.57;
      WEBLOG("ENS160: Comp T=%.1f H=%.1f | CO2=%uppm, TVOC=%uppb (%.1f µg/m³), AQI=%u/5, AQI500=%u/500\n", 
             temp, hum, co2, tvoc, tvoc_ugm3, aqi, aqi500); // Log compensation values used
      
      // Add additional debug info about indices
      WEBLOG("ENS160 buffer index: %d/%d\n", ensBufIndex, AVG_SAMPLES-1);
      lastDebugLog = millis();
    }
    
    // Validate readings against reasonable ranges
    if (co2 == 0 || tvoc == 0 || aqi == 0 || co2 < 400 || co2 > 8000 || tvoc > 60000) {
      WEBLOG("WARNING: ENS160 readings out of normal range: CO2=%uppm, TVOC=%uppb, AQI=%u\n", co2, tvoc, aqi);
      // Use previous values from buffer if current values are suspicious
      co2 = co2Buf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      tvoc = tvocBuf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      aqi = aqiBuf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
      aqi500 = aqi500Buf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
    }
  } else {
    // Either not warmed up yet or sensor in error state, use previous values
    co2 = co2Buf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
    tvoc = tvocBuf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
    aqi = aqiBuf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
    aqi500 = aqi500Buf[(ensBufIndex + AVG_SAMPLES - 1) % AVG_SAMPLES];
    
    if (!ensWarmed) {
      // Log that we are still warming up
      static unsigned long lastWarmupLog = 0;
      if (millis() - lastWarmupLog > 5000) { // Log every 5 seconds during warmup
         WEBLOG("ENS160 still warming up, using previous values.\n");
         lastWarmupLog = millis();
      }
    } else if (status != 0) {
      // Sensor is in error state
      static unsigned long lastErrorLog = 0;
      if (millis() - lastErrorLog > 10000) { // Log every 10 seconds
        WEBLOG("WARNING: ENS160 in error state (0x%02X). Using previous values.\n", status);
        lastErrorLog = millis();
      }
    }
  }
}

float updateRollingAverages_AHT(float *buf, float &sum, float value) {
  // Skip update if value is 0 (likely error)
  if (value == 0) {
    WEBLOG("Skipping AHT update with zero value\n");
    return sum / AVG_SAMPLES;
  }
  
  sum -= buf[tempHumBufIndex];
  buf[tempHumBufIndex] = value;
  sum += value;
  return sum / AVG_SAMPLES;
}

void updateRollingAverages_PM25(float pm25) {
  // Skip update if value is 0 (likely error)
  if (pm25 == 0) {
    WEBLOG("Skipping PM25 update with zero value\n");
    return;
  }
  
  pm25Sum -= pm25Buf[pm25BufIndex];
  pm25Buf[pm25BufIndex] = pm25;
  pm25Sum += pm25;
  avgPM25 = pm25Sum / AVG_SAMPLES;
  // Increment PM2.5-specific buffer index
  pm25BufIndex = (pm25BufIndex + 1) % AVG_SAMPLES;
}

void updateRollingAverages_BMP(float pressure, float altitude) {
  // Debug pressure value
  static unsigned long lastPressureDebug = 0;
  if (millis() - lastPressureDebug > 10000) {  // Debug every 10 seconds
    WEBLOG("BMP280: Current pressure=%.1f hPa, Alt=%.1f m\n", pressure, altitude);
    lastPressureDebug = millis();
  }
  
  // Validate pressure reading
  if (isnan(pressure) || pressure == 0 || pressure < 700 || pressure > 1200) {
    WEBLOG("WARNING: Invalid pressure reading: %.1f hPa. Using last valid reading.\n", pressure);
    // Don't update the buffer with invalid readings
    return;
  }
  
  // Update pressure rolling average
  pressureSum -= pressureBuf[bmpBufIndex];
  pressureBuf[bmpBufIndex] = pressure;
  pressureSum += pressure;
  avgPressure = pressureSum / AVG_SAMPLES;
  
  // Use immediate altitude value instead of averaging
  avgAltitude = altitude;  // Direct altitude reading, no averaging
  
  WEBLOG("Updated values: pressure=%.1f hPa (avg), altitude=%.1f m (direct)\n", avgPressure, avgAltitude);
  
  // Increment BMP-specific buffer index
  bmpBufIndex = (bmpBufIndex + 1) % AVG_SAMPLES;
}

void updateRollingAverages_ENS160(uint16_t co2, uint16_t tvoc, uint8_t aqi, uint16_t aqi500) {
  // Skip update if any value is 0 (likely error)
  if (co2 == 0 || tvoc == 0 || aqi == 0) {
    WEBLOG("Skipping ENS160 update with zero value(s): CO2=%u, TVOC=%u, AQI=%u\n", co2, tvoc, aqi);
    return;
  }
  
  co2Sum  -= co2Buf[ensBufIndex];
  tvocSum -= tvocBuf[ensBufIndex];
  aqiSum  -= aqiBuf[ensBufIndex];
  aqi500Sum -= aqi500Buf[ensBufIndex];
  co2Buf[ensBufIndex]  = co2;
  tvocBuf[ensBufIndex] = tvoc;
  aqiBuf[ensBufIndex]  = aqi;
  aqi500Buf[ensBufIndex] = aqi500;
  co2Sum  += co2;
  tvocSum += tvoc;
  aqiSum  += aqi;
  aqi500Sum += aqi500;
  avgCO2  = (float)co2Sum / AVG_SAMPLES;
  avgTVOC_ppb = (float)tvocSum / AVG_SAMPLES;
  avgAQI_raw = round((float)aqiSum / AVG_SAMPLES);
  avgAQI500 = aqi500Sum / AVG_SAMPLES;
  avgCO2 = constrain(avgCO2, 0.0, 100000.0);
  
  // DFRobot ENS160 library AQI range is same as our desired output (1-5)
  // 1=Excellent, 2=Good, 3=Moderate, 4=Poor, 5=Unhealthy
  avgAQI = constrain(avgAQI_raw, 1, 5);
  
  // TVOC conversion from ppb to μg/m³ 
  // Standard conversion factor of 4.57 for mixed VOCs
  // This conversion is required because:
  // - ENS160 sensor reports VOC in parts per billion (ppb)
  // - HomeKit expects VOC in micrograms per cubic meter (μg/m³)
  // - The conversion factor assumes a typical molecular weight distribution of VOCs
  avgTVOC_ugm3 = constrain(avgTVOC_ppb * 4.57, 0.0, 1000.0);
  
  // Increment ENS-specific buffer index
  ensBufIndex = (ensBufIndex + 1) % AVG_SAMPLES;
}

void updateRollingAverages_Combined() {
  // Only update combined temp/hum when either AHT1 or AHT2 is polled
  float temp = (lastTemp1 + lastTemp2) / 2.0;
  float hum  = (lastHum1 + lastHum2) / 2.0;
  
  // Skip update if either value is 0 (likely error)
  if (temp == 0 || hum == 0) {
    WEBLOG("Skipping combined temp/hum update with zero value(s): T=%.1f, H=%.1f\n", temp, hum);
    return;
  }
  
  tempSum -= tempBuf[tempHumBufIndex];
  humSum  -= humBuf[tempHumBufIndex];
  tempBuf[tempHumBufIndex] = temp;
  humBuf[tempHumBufIndex]  = hum;
  tempSum += temp;
  humSum  += hum;
  avgTemp = tempSum / AVG_SAMPLES;
  avgHum  = humSum  / AVG_SAMPLES;
  avgTemp = constrain(avgTemp, -40.0, 100.0);
  avgHum = constrain(avgHum, 0.0, 100.0);
  
  // Increment temperature/humidity buffer index
  tempHumBufIndex = (tempHumBufIndex + 1) % AVG_SAMPLES;
}

void publishToHomeKit(float avgTemp, float avgHum, float avgCO2, bool co2Detected, float avgTVOC_ugm3, float avgPM25, float avgPressure, uint8_t avgAQI) {
  // Clamp and check for NaN before publishing
  if (isnan(avgTemp) || avgTemp < 0.0 || avgTemp > 100.0) avgTemp = 0.0;
  if (isnan(avgHum) || avgHum < 0.0 || avgHum > 100.0) avgHum = 0.0;
  if (isnan(avgCO2) || avgCO2 < 0.0 || avgCO2 > 100000.0) avgCO2 = 0.0;
  if (isnan(avgTVOC_ugm3) || avgTVOC_ugm3 < 0.0 || avgTVOC_ugm3 > 1000.0) avgTVOC_ugm3 = 0.0;
  if (isnan(avgPM25) || avgPM25 < 0.0 || avgPM25 > 1000.0) avgPM25 = 0.0;
  if (isnan(avgPressure) || avgPressure < 700.0 || avgPressure > 1200.0) avgPressure = 1013.0;
  if (isnan(avgAQI) || avgAQI < 1 || avgAQI > 5) avgAQI = 1;
  float t1 = avgTemp1, h1 = avgHum1, t2 = avgTemp2, h2 = avgHum2;
  if (isnan(t1) || t1 < 0.0 || t1 > 100.0) t1 = 0.0;
  if (isnan(h1) || h1 < 0.0 || h1 > 100.0) h1 = 0.0;
  if (isnan(t2) || t2 < 0.0 || t2 > 100.0) t2 = 0.0;
  if (isnan(h2) || h2 < 0.0 || h2 > 100.0) h2 = 0.0;
  pTempChar->setVal(avgTemp);
  pHumChar->setVal(avgHum);
  pTemp1Char->setVal(t1);
  pHum1Char->setVal(h1);
  pTemp2Char->setVal(t2);
  pHum2Char->setVal(h2);
  pCO2Char->setVal(avgCO2);
  pCO2DetectedChar->setVal(co2Detected ? 1 : 0);
  pCO2PeakLevelChar->setVal(co2PeakLevel);
  pTVOCChar->setVal(avgTVOC_ugm3);
  pPM25Char->setVal(avgPM25);
  pAQIChar->setVal(avgAQI);
}

void updateOLED(float avgTemp, float avgHum, float avgCO2, bool co2Detected, float avgPM25, float avgTVOC_ugm3, uint8_t avgAQI, uint16_t avgAQI500, float avgPressure, float avgAltitude) {
  static unsigned long lastPM25Debug = 0;
  unsigned long now = millis();
  
  if (now - lastPM25Debug > 10000) {
    WEBLOG("OLED Debug: PM2.5 value = %.1f µg/m³\n", avgPM25);
    lastPM25Debug = now;
  }
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("T:%.1fC H:%.1f%%\n", avgTemp, avgHum);
  display.printf("CO2:%4.0f %s\n", avgCO2, co2Detected ? "!" : " ");
  
  if (isnan(avgPM25)) {
    display.printf("PM2.5:ERR TV:%4.0f\n", avgTVOC_ugm3);
    WEBLOG("ERROR: PM2.5 value is NaN\n");
  } else {
    display.printf("PM:%-5.1f TV:%-4.0f\n", avgPM25, avgTVOC_ugm3);
  }
  
  // Alternate bottom row every 3 seconds
  if (((millis() / 3000) % 2) == 0) {
    display.printf("AQI:%d/5 AQI500:%d", avgAQI, avgAQI500);
  } else {
    display.printf("P:%.1fhPa Alt:%.0fm", avgPressure, avgAltitude);
  }
  display.display();
}

void printAvgArraysToWeblog() {
  WEBLOG("Current Altitude (m): %.1f (not averaged)\n", lastAltitude);
  
  WEBLOG("\n==== CURRENT AVERAGES ====\n");
  WEBLOG("Avg Temps: T1=%.1f T2=%.1f Combined=%.1f\n", avgTemp1, avgTemp2, avgTemp);
  WEBLOG("Avg Humidity: H1=%.1f H2=%.1f Combined=%.1f\n", avgHum1, avgHum2, avgHum);
  WEBLOG("Avg CO2=%.1f ppm (Peak=%.1f ppm), TVOC=%d ppb (%.1f µg/m³), AQI=%d/5, AQI500=%d/500\n", 
         avgCO2, co2PeakLevel, (int)avgTVOC_ppb, avgTVOC_ugm3, avgAQI, avgAQI500);
  WEBLOG("Avg PM2.5=%.1f µg/m³, Pressure=%.1f hPa\n", avgPM25, avgPressure);
  WEBLOG("Current Altitude=%.1f m (not averaged)\n", avgAltitude);
  
  WEBLOG("\n==== SUMS FOR VERIFICATION ====\n");
  WEBLOG("Sums: temp1=%.1f temp2=%.1f hum1=%.1f hum2=%.1f\n", temp1Sum, temp2Sum, hum1Sum, hum2Sum);
  WEBLOG("Sums: temp=%.1f hum=%.1f pm25=%.1f pressure=%.1f\n", tempSum, humSum, pm25Sum, pressureSum);
  WEBLOG("Sums: co2=%d tvoc=%d aqi=%d aqi500=%d\n", co2Sum, tvocSum, aqiSum, aqi500Sum);
  
  WEBLOG("\n==== BUFFER INDICES ====\n");
  WEBLOG("Current indices: Temp/Hum=%d, PM2.5=%d, BMP=%d, ENS=%d (of %d max)\n", 
         tempHumBufIndex, pm25BufIndex, bmpBufIndex, ensBufIndex, AVG_SAMPLES-1);
  
  WEBLOG("\nPM2.5 Buffer Contents: ");
  for (int i = 0; i < AVG_SAMPLES; i++) {
    WEBLOG("%.1f ", pm25Buf[i]);
    if (i % 5 == 4) WEBLOG("| ");
  }
  WEBLOG("\n");
  
  WEBLOG("==== END OF ARRAYS ====\n\n");
}
