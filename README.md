# FAULT-DIAGONISE-AND-WIND-TURBINE-MONITORING-SYSTEM
IoT-based Fire Detection &amp; Suppression System using ESP8266. Monitors temperature, smoke, and vibration, tracks GPS location, displays status on LCD, sends data to cloud, and activates FM-200 suppression via relay. Powered by 12V battery &amp; wind turbine for continuous operation.
# ESP8266 Fire Detection & Suppression System

This project uses an ESP8266 (NodeMCU) to monitor **temperature**, **smoke**, and **vibration** levels.  
If fire is detected, it activates an FM-200 fire suppression system via relay, sends alerts to the cloud, and logs GPS coordinates.

## Features
- Temperature sensing (DHT22)
- Smoke detection (MQ-2)
- Vibration detection
- GPS tracking
- Cloud logging via Wi-Fi
- LCD display for real-time status
- Relay control for FM-200 suppression

## Block Diagram
Block Diagram.jpg


## Hardware Required
- ESP8266 NodeMCU
- DHT22 temperature sensor
- MQ-2 smoke sensor
- Vibration sensor module
- GPS module (NEO-6M)
- Relay module
- I2C 16x2 LCD
- 12V battery
- FM-200 fire suppression unit

## Setup
1. Install Arduino IDE & ESP8266 board support.
2. Install required libraries:
   - LiquidCrystal I2C
   - DHT sensor library
   - TinyGPS++
   - ArduinoJson

