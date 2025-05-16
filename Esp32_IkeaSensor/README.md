# Ikea Vindriktning+ ESP32-S3 Environmental Monitor with HomeKit

This project transforms the IKEA Vindriktning air quality sensor into a comprehensive environmental monitoring system with Apple HomeKit integration. The system combines multiple sensors to measure air quality, temperature, humidity, pressure, and volatile organic compounds, all accessible through the Apple Home app.

![HomeKit Integration](https://github.com/yourusername/Esp32_IkeaSensor/raw/main/images/homekit-screenshot.jpg)

## Features

- **Multiple Environmental Measurements**:
  - PM2.5 particulate matter (from IKEA Vindriktning)
  - Temperature and humidity (dual AHT21 sensors)
  - Barometric pressure (BMP280)
  - CO2 equivalent levels (ENS160)
  - TVOC (Total Volatile Organic Compounds) (ENS160)
  - Air Quality Index (1-5 scale and 0-500 scale)

- **Smart Energy Management**:
  - Sensor sleep/wake cycles to extend component lifespan
  - 15-minute sleep periods after collecting 60 readings
  - 3-minute warmup cycles for sensor stabilization
  - Intelligent handling of sensor errors and abnormal readings

- **Advanced Data Processing**:
  - Rolling averages for all sensor readings (20-sample window)
  - Automatic filtering of anomalous readings
  - Protection against zero values skewing averages
  - Peak CO2 level tracking

- **Comprehensive Display**:
  - Real-time readings on integrated OLED display
  - Alternating information screens
  - HomeKit integration for viewing on Apple devices

## Hardware

- **Microcontroller**: ESP32-S3 Zero
- **Sensors**:
  - IKEA Vindriktning (Cubic PM1006 PM2.5 sensor) connected via UART
  - ENS160 gas sensor + AHT21 temperature/humidity (I2C bus 0)
  - BMP280 pressure sensor + AHT21 temperature/humidity (I2C bus 1)
- **Display**: 0.91" 128×32 I2C OLED display
- **Connectivity**: WiFi for HomeKit integration

### Pin Configuration

| Component          | Connection Type | Pins                 |
|--------------------|----------------|----------------------|
| PM1006 (Vindriktning) | UART        | RX: 1, TX: 13        |
| ENS160 + AHT21     | I2C Bus 0      | SDA: 3, SCL: 2       |
| BMP280 + AHT21     | I2C Bus 1      | SDA: 5, SCL: 6       |
| OLED Display       | I2C Bus 1      | SDA: 5, SCL: 6       |

## Setup Instructions

### Hardware Assembly

1. **IKEA Vindriktning Modification**:
   - Open the Vindriktning sensor case
   - Connect the ESP32-S3 to the PM1006 sensor's UART interface
   - Power the ESP32-S3 from the Vindriktning's 5V supply

2. **Sensor Connections**:
   - Connect ENS160+AHT21 to I2C bus 0 (SDA: 3, SCL: 2)
   - Connect BMP280+AHT21 to I2C bus 1 (SDA: 5, SCL: 6)
   - Connect OLED display to I2C bus 1 (SDA: 5, SCL: 6)

3. **Power Supply**:
   - The system can be powered via USB-C or directly from the Vindriktning's power supply

### Software Setup

1. **Required Libraries**:
   - HomeSpan (for HomeKit integration)
   - Adafruit_SSD1306
   - Adafruit_GFX
   - DFRobot_ENS160
   - Adafruit_AHTX0
   - Adafruit_BMP280
   - Adafruit_Sensor
   - ESP SoftwareSerial

2. **Arduino IDE Configuration**:
   - Board: "ESP32S3 Dev Module" or "WAVESHARE_ESP32_S3_ZERO"
   - USB CDC On Boot: Enabled
   - CPU Frequency: 240MHz
   - Flash Mode: QIO 80MHz
   - Partition Scheme: Minimal SPIFFS

3. **HomeKit Pairing**:
   - Default pairing code: 466-37-726
   - Access the device through the Apple Home app using this code

## Usage

After powering on the device:

1. The system will automatically initialize all sensors and display readings on the OLED screen.
2. Connect to the ESP32 with the HomeKit pairing code through the Apple Home app.
3. Monitor all environmental parameters through either the OLED display or Apple Home app.

### HomeKit Integration

The following HomeKit services are provided:
- Temperature Sensor (combined average and individual sensors)
- Humidity Sensor (combined average and individual sensors)
- Carbon Dioxide Sensor (with detection threshold at 1000 ppm)
- Air Quality Sensor (with PM2.5 and VOC measurements)
- Custom Eve-compatible Pressure Sensor

### OLED Display Information

The OLED display shows:
- Temperature and humidity
- CO2 levels (with alert indicator for high levels)
- PM2.5 particulate matter concentration
- TVOC levels
- Alternating screens with:
  - Air Quality Index (AQI scales)
  - Barometric pressure and altitude

## Troubleshooting

### Common Issues

1. **I2C Communication Errors**:
   - Check wiring connections
   - Ensure proper pull-up resistors (4.7kΩ recommended)
   - The code uses internal pull-ups which may not be sufficient for longer wires

2. **Sensor Not Detected**:
   - Verify sensor I2C addresses (ENS160: 0x53, BMP280: 0x76/0x77, AHT21: 0x38)
   - Check voltage levels (sensors require 3.3V)

3. **Unreliable Readings**:
   - Allow proper warm-up time (3 minutes for ENS160)
   - Keep sensors away from direct heat sources

## Version History

- **v0.1.7** - Improved ENS160 pin configuration and reliability
  - Added internal pull-up resistors for I2C bus 0
  - Enhanced error diagnostics for I2C communication
  - Added protection against zero values in sensor readings

- **v0.1.6** - Added ENS160 sensor sleep/wake cycle
  - Implemented 15-minute sleep periods after 60 readings
  - Added 3-minute warmup after each wake-up
  - Improved buffer handling during sleep periods

- **v0.1.5** - Improved sensor data averaging
  - Implemented 20-sample rolling averages for all sensors
  - Added validation and filtering of anomalous readings
  - Fixed overflow in warmup timer calculation

- **v0.1.4** - Enhanced HomeKit integration
  - Added individual sensor tiles in HomeKit
  - Implemented Eve-compatible pressure sensor
  - Added CO2 peak level tracking

- **v0.1.3** - Fixed display issues and UART communication
  - Improved PM2.5 sensor data parsing
  - Added error handling for sensor communication
  - Fixed OLED display rotation

- **v0.1.2** - Separated I2C buses and added secondary sensors
  - Added BMP280 pressure sensor on separate I2C bus
  - Added second AHT21 for redundant temperature/humidity

- **v0.1.1** - Added ENS160 gas sensor integration
  - Implemented CO2 and TVOC measurements
  - Added Air Quality Index calculations
  - Created dual-scale AQI reporting (1-5 and 0-500)

- **v0.1.0** - Initial release
  - Basic integration with IKEA Vindriktning
  - HomeKit connectivity
  - OLED display support

## Technical Details

### Buffer Management

The system uses a sophisticated buffer management system to ensure stable readings:

- 20-sample rolling buffers for all sensor data types
- Separate indices for different sensor types (temperature/humidity, PM2.5, pressure, gas)
- Automatic filtering of anomalous values
- Protection against zero values affecting averages

### Energy and Sensor Management

To extend sensor life and reduce power consumption, the ENS160 gas sensor operates on a cycle:
1. Active period collecting 60 readings
2. Sleep period of 15 minutes
3. Warmup period of 3 minutes
4. Repeat

### Data Conversion

- TVOC readings are converted from ppb to μg/m³ using a standard conversion factor of 4.57
- CO2 readings are in ppm (parts per million)
- PM2.5 readings are in μg/m³
- Pressure readings are in hPa (hectopascals)

## Credits

- [HomeSpan](https://github.com/HomeSpan/HomeSpan) for HomeKit integration
- [Adafruit](https://github.com/adafruit) for sensor libraries
- [DFRobot](https://github.com/DFRobot/DFRobot_ENS160) for ENS160 library
- [IKEA](https://www.ikea.com) for the Vindriktning base design

## License

This project is licensed under the MIT License - see the LICENSE file for details.
