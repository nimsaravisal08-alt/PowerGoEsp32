# PowerGO – ESP32-S3 Hardware Controller Firmware

This repository contains the embedded C++/Arduino firmware for the **PowerGO Automated Power Bank Vending Machine**. The firmware runs on an **ESP32-S3** microcontroller, orchestrating sensor telemetry, motorized dispensing trays, and bidirectional real-time cloud communication with Google Firebase.

---

## ⚡ System Architecture

- **Microcontroller:** ESP32-S3-WROOM-1
- **Cloud Backend:** Google Cloud Firestore (REST API & Real-time Telemetry)
- **Wi-Fi Connectivity:** 2.4 GHz IEEE 802.11 b/g/n
- **Power Bank Detection:** Optical Break-Beam Sensors (Active LOW)
- **Current & Power Monitoring:** Dual INA219 Sensors (I2C Addresses: `0x41` for Slot 1, `0x40` for Slot 2)
- **Actuation:** 2x DC Motor Drives (Motorized slide trays)
- **Visual Feedback:** Dual Bi-Color Status Indicators (Green = Charging/Ready, Red = Slot Empty)

---

## 📌 Hardware Pinout Configuration

| Component | Pin (ESP32-S3) | Description |
|---|---|---|
| **I2C SDA** | GPIO 8 | Shared I2C Data (INA219 sensors) |
| **I2C SCL** | GPIO 9 | Shared I2C Clock (INA219 sensors) |
| **Slot 1 IR Sensor** | GPIO 4 | Break-beam receiver (LOW = present, HIGH = empty) |
| **Slot 2 IR Sensor** | GPIO 5 | Break-beam receiver (LOW = present, HIGH = empty) |
| **Slot 1 Motor IN1 / IN2** | GPIO 15 / 16 | Forward / Reverse motorized tray dispensing |
| **Slot 2 Motor IN1 / IN2** | GPIO 17 / 18 | Forward / Reverse motorized tray dispensing |
| **Slot 1 Green LED** | GPIO 1 | Power bank present & charging |
| **Slot 1 Red LED** | GPIO 2 | Slot empty |
| **Slot 2 Green LED** | GPIO 41 | Power bank present & charging |
| **Slot 2 Red LED** | GPIO 42 | Slot empty |

---

## 🚀 Key Features

1. **Autonomous Cloud Telemetry:** Synchronizes slot status, bus voltage, charging current (mA), power (mW), and optical presence to Firestore every 4 seconds.
2. **Dynamic 30-Second Operation Windows:** Manages motor extension and auto-retraction timeouts for rental collection and return.
3. **Continuous 5-Second Return Verification:** Enforces a 5-second continuous IR beam break window to eliminate false return confirmations.
4. **Resilient Network Handling:** Automatic Wi-Fi reconnection, timestamp synchronization, and non-blocking command execution.

---

## 🛠️ Build & Flashing Instructions

1. Install **Arduino IDE** (v2.0 or higher).
2. Install the **ESP32 Board Package** (`esp32` by Espressif Systems v3.x).
3. Install required libraries:
   - `Adafruit INA219`
   - `ArduinoJson` (v6 or v7)
4. Select Board: **ESP32S3 Dev Module**.
5. Set Flash Size to **8MB/16MB** with `OPI PSRAM` enabled if applicable.
6. Compile and upload `esp32_firmware.ino` via USB-CDC.
