/*
 * ============================================================
 * POWERBANK VENDING MACHINE - ESP32-S3
 * Two ULN2003 stepper motors, IR beam sensors, Firebase REST
 * ============================================================
 */

#include <Adafruit_INA219.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <time.h>
#include <esp_system.h>

// ============================================================
// CONFIGURATION
// ============================================================

const char *WIFI_SSID = "SKS 2.5";
const char *WIFI_PASSWORD = "D4803B5A";

const char *FIREBASE_PROJECT_ID = "powerbank-fc675";
const char *FIREBASE_API_KEY = "AIzaSyCb7ZePe7pCrrZ3foKtGhPQhyqQVxfoUAg";
const char *STATION_ID = "main_station";

// ============================================================
// PIN CONFIGURATION
// ============================================================

#define SDA_PIN 8
#define SCL_PIN 9

#define SLOT1_BEAM_PIN 7
#define SLOT2_BEAM_PIN 6

// SLOT STATUS LEDs
#define SLOT1_RED_PIN   37
#define SLOT1_GREEN_PIN 35
#define SLOT2_RED_PIN   38
#define SLOT2_GREEN_PIN 36

#define M1_IN1 10
#define M1_IN2 11
#define M1_IN3 12
#define M1_IN4 13

#define M2_IN1 14
#define M2_IN2 15
#define M2_IN3 16
#define M2_IN4 17

const unsigned long FORWARD_TIME_MS = 11000;
const unsigned long BACKWARD_TIME_MS = 12000;
const int MOTOR_STEP_DELAY_MS = 3;

// ============================================================
// MOTOR SEQUENCE
// ============================================================

const int motorSeq[8][4] = {{1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0},
                            {0, 1, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 1},
                            {0, 0, 0, 1}, {1, 0, 0, 1}};

// Forward declaration for timestamp generation
String getIsoTimestampString();

// ============================================================
// INA219 SENSORS (Single Source of Truth: Slot 1 -> 0x41, Slot 2 -> 0x40)
// ============================================================

Adafruit_INA219 sensor1(0x41); // Slot 1 mapped to 0x41
Adafruit_INA219 sensor2(0x40); // Slot 2 mapped to 0x40

#define ina219_slot1 sensor1
#define ina219_slot2 sensor2

bool sensor1OK = false;
bool sensor2OK = false;
#define ina219Slot1OK sensor1OK
#define ina219Slot2OK sensor2OK

struct MeasurementSnapshot {
  unsigned long cycleNumber;
  String timestamp;

  // Slot 1 (INA219 0x41)
  bool slot1Connected;
  bool slot1Valid;
  float slot1Current_mA;
  float slot1BusVoltage_V;
  float slot1Power_mW;
  float slot1ShuntVoltage_mV;

  // Slot 2 (INA219 0x40)
  bool slot2Connected;
  bool slot2Valid;
  float slot2Current_mA;
  float slot2BusVoltage_V;
  float slot2Power_mW;
  float slot2ShuntVoltage_mV;
};

MeasurementSnapshot liveSnapshot = {0, "UPTIME-0", false, false, 0.0f, 0.0f, 0.0f, 0.0f, false, false, 0.0f, 0.0f, 0.0f, 0.0f};
unsigned long measurementCycleCount = 0;

// ── Stable Charging Detection Configuration (Hysteresis & Debounce) ──────────
// Genuine charging draw is typically 200–1000+ mA (e.g. 242.4 mA).
// Idle/empty noise floor is typically 0.0–5.0 mA (e.g. 0.20 mA).
const float CHARGING_THRESHOLD_START_MA = 80.0f; // Must exceed 80 mA to enter CHARGING state
const float CHARGING_THRESHOLD_STOP_MA  = 40.0f; // Must fall below 40 mA to exit CHARGING state
const int   CHARGING_DEBOUNCE_SAMPLES   = 3;     // Require 3 consecutive cycles (3s) to change state

struct SlotChargingState {
  bool isCharging;
  int  consecutiveHighCount;
  int  consecutiveLowCount;
};

SlotChargingState slot1ChargeState = {false, 0, 0};
SlotChargingState slot2ChargeState = {false, 0, 0};

// Forward declaration of helpers
void printResetReason();
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
bool updateSlotCharging(SlotChargingState &detector, bool isPresent, bool sensorOK, float current_mA);

// ============================================================
// TIMING
// ============================================================

const unsigned long SENSOR_MONITOR_INTERVAL_MS = 1000;
const unsigned long FIREBASE_UPDATE_INTERVAL_MS = 4000;
const unsigned long COMMAND_POLL_INTERVAL_MS = 2000;
const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;

// Rental auto-close timer
const unsigned long RENTAL_AUTO_CLOSE_MS = 30000;

unsigned long lastSensorMonitorMs = 0;
unsigned long lastFirebaseUpdateMs = 0;
unsigned long lastCommandPollMs = 0;
unsigned long lastWiFiRetryMs = 0;

// Active rental tray timer
bool rentalTrayOpen = false;
int rentalOpenSlotId = 0;
unsigned long rentalOpenedAtMs = 0;
String slotTrayState[2] = {"closed", "closed"};
String slotOpenedAt[2] = {"", ""};

// ============================================================
// SLOT CONFIGURATION
// ============================================================

struct SlotConfig {
  int slotId;
  uint8_t beamPin;
};

const SlotConfig SLOTS[] = {{1, SLOT1_BEAM_PIN}, {2, SLOT2_BEAM_PIN}};

const int SLOT_COUNT = sizeof(SLOTS) / sizeof(SLOTS[0]);

bool motorBusy = false;

// ============================================================
// MOTOR OUTPUT HELPERS
// ============================================================

void setMotor1(int a, int b, int c, int d) {
  digitalWrite(M1_IN1, a);
  digitalWrite(M1_IN2, b);
  digitalWrite(M1_IN3, c);
  digitalWrite(M1_IN4, d);
}

void setMotor2(int a, int b, int c, int d) {
  digitalWrite(M2_IN1, a);
  digitalWrite(M2_IN2, b);
  digitalWrite(M2_IN3, c);
  digitalWrite(M2_IN4, d);
}

void stopMotor1() { setMotor1(LOW, LOW, LOW, LOW); }

void stopMotor2() { setMotor2(LOW, LOW, LOW, LOW); }

void setMotorSlot(int slotId, int a, int b, int c, int d) {
  if (slotId == 1) {
    setMotor1(a, b, c, d);
  } else if (slotId == 2) {
    setMotor2(a, b, c, d);
  }
}

void stopMotorSlot(int slotId) {
  if (slotId == 1) {
    stopMotor1();
  } else if (slotId == 2) {
    stopMotor2();
  }
}

// ============================================================
// MOTOR MOVEMENT
// ============================================================

bool stepMotorForward(int slotId) {
  if (slotId != 1 && slotId != 2) {
    Serial.println("[MOTOR] Invalid slot ID.");
    return false;
  }

  if (motorBusy) {
    Serial.println("[MOTOR] Busy; refusing overlapping command.");
    return false;
  }

  motorBusy = true;
  const unsigned long started = millis();

  Serial.printf("[MOTOR] Slot %d FORWARD for %lu ms\n", slotId,
                FORWARD_TIME_MS);

  while (millis() - started < FORWARD_TIME_MS) {
    for (int i = 0; i < 8; i++) {
      if (millis() - started >= FORWARD_TIME_MS)
        break;

      setMotorSlot(slotId, motorSeq[i][0], motorSeq[i][1], motorSeq[i][2],
                   motorSeq[i][3]);
      delay(MOTOR_STEP_DELAY_MS);
    }
  }

  stopMotorSlot(slotId);
  motorBusy = false;

  Serial.printf("[MOTOR] Slot %d FORWARD complete\n", slotId);
  return true;
}

bool stepMotorBackward(int slotId) {
  if (slotId != 1 && slotId != 2) {
    Serial.println("[MOTOR] Invalid slot ID.");
    return false;
  }

  if (motorBusy) {
    Serial.println("[MOTOR] Busy; refusing overlapping command.");
    return false;
  }

  motorBusy = true;
  const unsigned long started = millis();

  Serial.printf("[MOTOR] Slot %d BACKWARD for %lu ms\n", slotId,
                BACKWARD_TIME_MS);

  while (millis() - started < BACKWARD_TIME_MS) {
    for (int i = 7; i >= 0; i--) {
      if (millis() - started >= BACKWARD_TIME_MS)
        break;

      setMotorSlot(slotId, motorSeq[i][0], motorSeq[i][1], motorSeq[i][2],
                   motorSeq[i][3]);
      delay(MOTOR_STEP_DELAY_MS);
    }
  }

  stopMotorSlot(slotId);
  motorBusy = false;

  Serial.printf("[MOTOR] Slot %d BACKWARD complete\n", slotId);
  return true;
}

bool openTray(int slotId) {
  Serial.printf("[MOTOR] Opening Slot %d\n", slotId);
  return stepMotorForward(slotId);
}

bool closeTray(int slotId) {
  Serial.printf("[MOTOR] Closing Slot %d\n", slotId);
  return stepMotorBackward(slotId);
}

// ============================================================
// IR BREAK-BEAM SENSOR
// LOW = beam broken = power bank present
// ============================================================

bool readPowerBankPresent(uint8_t pin) {
  int brokenCount = 0;

  for (int i = 0; i < 5; i++) {
    if (digitalRead(pin) == LOW) {
      brokenCount++;
    }
    delay(10);
  }

  return brokenCount >= 3;
}

// ============================================================
// INA219 DEDICATED MEASUREMENT & LIVE DIAGNOSTIC MONITOR
// Standalone logic: Slot 1 -> 0x41 (sensor1), Slot 2 -> 0x40 (sensor2)
// Runs every 1 second independently of WiFi & Firebase.
// ============================================================

bool checkI2CAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// ── Stable Charging State Evaluator with Direction, Presence & Debounce ───────
bool updateSlotCharging(SlotChargingState &detector, bool isPresent, bool sensorOK, float current_mA) {
  // 1. If powerbank is physically absent or sensor has error, CANNOT be charging!
  if (!isPresent || !sensorOK) {
    detector.isCharging = false;
    detector.consecutiveHighCount = 0;
    detector.consecutiveLowCount = 0;
    return false;
  }

  // 2. Genuine charging current must flow in forward direction (positive into battery)
  if (current_mA <= 0.0f) {
    current_mA = 0.0f;
  }

  // 3. State machine with hysteresis and consecutive sample filtering
  if (!detector.isCharging) {
    // Currently NOT CHARGING: must exceed start threshold for 3 consecutive 1s cycles
    if (current_mA >= CHARGING_THRESHOLD_START_MA) {
      detector.consecutiveHighCount++;
      detector.consecutiveLowCount = 0;
      if (detector.consecutiveHighCount >= CHARGING_DEBOUNCE_SAMPLES) {
        detector.isCharging = true;
      }
    } else {
      detector.consecutiveHighCount = 0;
    }
  } else {
    // Currently CHARGING: must fall below stop threshold for 3 consecutive 1s cycles
    if (current_mA < CHARGING_THRESHOLD_STOP_MA) {
      detector.consecutiveLowCount++;
      detector.consecutiveHighCount = 0;
      if (detector.consecutiveLowCount >= CHARGING_DEBOUNCE_SAMPLES) {
        detector.isCharging = false;
      }
    } else {
      detector.consecutiveLowCount = 0;
    }
  }

  return detector.isCharging;
}

void readAndMonitorSensors() {
  measurementCycleCount++;
  liveSnapshot.cycleNumber = measurementCycleCount;
  liveSnapshot.timestamp = getIsoTimestampString();

  // ── Measure Slot 1 (INA219 at 0x41) ───────────────────────
  if (!sensor1OK) {
    sensor1OK = sensor1.begin();
    if (sensor1OK) sensor1.setCalibration_32V_2A();
  }

  if (sensor1OK) {
    float v1 = sensor1.getBusVoltage_V();
    float s1 = sensor1.getShuntVoltage_mV();
    float i1 = sensor1.getCurrent_mA();
    float p1 = sensor1.getPower_mW();

    // Detect disconnection or I2C communication fault
    if (isnan(v1) || isnan(s1) || isnan(i1) || (v1 == 0.0f && s1 == 0.0f && i1 == 0.0f && !checkI2CAddress(0x41))) {
      sensor1OK = false;
      liveSnapshot.slot1Connected = false;
      liveSnapshot.slot1Valid = false;
    } else {
      liveSnapshot.slot1Connected = true;
      liveSnapshot.slot1Valid = true;

      // Direct shunt resistor fallback (0.1 ohm shunt: 1 mV = 10 mA)
      float calcI1 = fabsf(s1) * 10.0f;
      float finalI1 = fabsf(i1);
      if (finalI1 < 0.1f && calcI1 >= 0.1f) {
        sensor1.setCalibration_32V_2A();
        finalI1 = calcI1;
      }

      liveSnapshot.slot1Current_mA = finalI1;
      liveSnapshot.slot1BusVoltage_V = v1;
      liveSnapshot.slot1Power_mW = (finalI1 > 0.1f && p1 <= 0.0f) ? (v1 * finalI1) : fabsf(p1);
      liveSnapshot.slot1ShuntVoltage_mV = s1;
    }
  } else {
    liveSnapshot.slot1Connected = false;
    liveSnapshot.slot1Valid = false;
  }

  // ── Measure Slot 2 (INA219 at 0x40) ───────────────────────
  if (!sensor2OK) {
    sensor2OK = sensor2.begin();
    if (sensor2OK) sensor2.setCalibration_32V_2A();
  }

  if (sensor2OK) {
    float v2 = sensor2.getBusVoltage_V();
    float s2 = sensor2.getShuntVoltage_mV();
    float i2 = sensor2.getCurrent_mA();
    float p2 = sensor2.getPower_mW();

    // Detect disconnection or I2C communication fault
    if (isnan(v2) || isnan(s2) || isnan(i2) || (v2 == 0.0f && s2 == 0.0f && i2 == 0.0f && !checkI2CAddress(0x40))) {
      sensor2OK = false;
      liveSnapshot.slot2Connected = false;
      liveSnapshot.slot2Valid = false;
    } else {
      liveSnapshot.slot2Connected = true;
      liveSnapshot.slot2Valid = true;

      // Direct shunt resistor fallback (0.1 ohm shunt: 1 mV = 10 mA)
      float calcI2 = fabsf(s2) * 10.0f;
      float finalI2 = fabsf(i2);
      if (finalI2 < 0.1f && calcI2 >= 0.1f) {
        sensor2.setCalibration_32V_2A();
        finalI2 = calcI2;
      }

      liveSnapshot.slot2Current_mA = finalI2;
      liveSnapshot.slot2BusVoltage_V = v2;
      liveSnapshot.slot2Power_mW = (finalI2 > 0.1f && p2 <= 0.0f) ? (v2 * finalI2) : fabsf(p2);
      liveSnapshot.slot2ShuntVoltage_mV = s2;
    }
  } else {
    liveSnapshot.slot2Connected = false;
    liveSnapshot.slot2Valid = false;
  }

  // ── Break-Beam Presence & Stable Charging Evaluation ───────
  bool pb1Present = readPowerBankPresent(SLOTS[0].beamPin);
  bool pb2Present = readPowerBankPresent(SLOTS[1].beamPin);

  bool ch1 = updateSlotCharging(slot1ChargeState, pb1Present,
                                liveSnapshot.slot1Connected && liveSnapshot.slot1Valid,
                                liveSnapshot.slot1Current_mA);
  bool ch2 = updateSlotCharging(slot2ChargeState, pb2Present,
                                liveSnapshot.slot2Connected && liveSnapshot.slot2Valid,
                                liveSnapshot.slot2Current_mA);

  // ── Dedicated Serial Output (Requirements 7, 8 & 10) ────────
  Serial.println();
  Serial.println("==================================================");
  Serial.printf("  DUAL INA219 LIVE MONITOR — CYCLE #%lu\n", liveSnapshot.cycleNumber);
  Serial.printf("  Timestamp: %s\n", liveSnapshot.timestamp.c_str());
  Serial.println("==================================================");
  Serial.println();

  Serial.println("SLOT 1 - INA219 0x41");
  Serial.printf("Presence       : %s\n", pb1Present ? "POWERBANK PRESENT" : "POWERBANK ABSENT");
  if (!liveSnapshot.slot1Connected) {
    Serial.println("Sensor Status  : SENSOR ERROR");
    Serial.println("Raw Current    : N/A");
    Serial.println("Bus Voltage    : N/A");
    Serial.println("Power          : N/A");
    Serial.println("Charging State : SENSOR ERROR");
  } else if (!liveSnapshot.slot1Valid) {
    Serial.println("Sensor Status  : READING ERROR");
    Serial.println("Raw Current    : N/A");
    Serial.println("Bus Voltage    : N/A");
    Serial.println("Power          : N/A");
    Serial.println("Charging State : READING ERROR");
  } else {
    Serial.println("Sensor Status  : CONNECTED");
    Serial.printf("Raw Current    : %.2f mA\n", liveSnapshot.slot1Current_mA);
    Serial.printf("Bus Voltage    : %.2f V\n", liveSnapshot.slot1BusVoltage_V);
    Serial.printf("Power          : %.2f mW\n", liveSnapshot.slot1Power_mW);
    Serial.printf("Shunt Voltage  : %.2f mV\n", liveSnapshot.slot1ShuntVoltage_mV);
    Serial.printf("Charging State : %s\n", ch1 ? "CHARGING" : "NOT CHARGING");
  }

  Serial.println();
  Serial.println("--------------------------------------------------");
  Serial.println();

  Serial.println("SLOT 2 - INA219 0x40");
  Serial.printf("Presence       : %s\n", pb2Present ? "POWERBANK PRESENT" : "POWERBANK ABSENT");
  if (!liveSnapshot.slot2Connected) {
    Serial.println("Sensor Status  : SENSOR ERROR");
    Serial.println("Raw Current    : N/A");
    Serial.println("Bus Voltage    : N/A");
    Serial.println("Power          : N/A");
    Serial.println("Charging State : SENSOR ERROR");
  } else if (!liveSnapshot.slot2Valid) {
    Serial.println("Sensor Status  : READING ERROR");
    Serial.println("Raw Current    : N/A");
    Serial.println("Bus Voltage    : N/A");
    Serial.println("Power          : N/A");
    Serial.println("Charging State : READING ERROR");
  } else {
    Serial.println("Sensor Status  : CONNECTED");
    Serial.printf("Raw Current    : %.2f mA\n", liveSnapshot.slot2Current_mA);
    Serial.printf("Bus Voltage    : %.2f V\n", liveSnapshot.slot2BusVoltage_V);
    Serial.printf("Power          : %.2f mW\n", liveSnapshot.slot2Power_mW);
    Serial.printf("Shunt Voltage  : %.2f mV\n", liveSnapshot.slot2ShuntVoltage_mV);
    Serial.printf("Charging State : %s\n", ch2 ? "CHARGING" : "NOT CHARGING");
  }

  Serial.println();
  Serial.println("==================================================");
  Serial.println();
}

float readCurrent_mA(int slotId) {
  if (slotId == 1) {
    if (!liveSnapshot.slot1Connected || !liveSnapshot.slot1Valid)
      return 0.0f;
    return liveSnapshot.slot1Current_mA;
  } else if (slotId == 2) {
    if (!liveSnapshot.slot2Connected || !liveSnapshot.slot2Valid)
      return 0.0f;
    return liveSnapshot.slot2Current_mA;
  }
  return 0.0f;
}

// ============================================================
// ISO TIMESTAMP UTILITY
// ============================================================

String getIsoTimestampString() {
  time_t now;
  time(&now);
  struct tm timeinfo;

  if (now > 100000 && gmtime_r(&now, &timeinfo)) {
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(buf);
  }

  return "UPTIME-" + String(millis() / 1000);
}

// ============================================================
// RESET REASON DIAGNOSTICS
// Decodes hardware brownouts, watchdogs, and reboots
// ============================================================

void printResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.println("\n--------------------------------------------------");
  Serial.print("[RESET REASON] ESP32-S3 Boot Reason: ");
  switch (reason) {
    case ESP_RST_POWERON:   Serial.println("POWER-ON RESET"); break;
    case ESP_RST_EXT:       Serial.println("EXTERNAL PIN RESET"); break;
    case ESP_RST_SW:        Serial.println("SOFTWARE RESET"); break;
    case ESP_RST_PANIC:     Serial.println("CRASH / PANIC RESET"); break;
    case ESP_RST_INT_WDT:   Serial.println("INTERRUPT WATCHDOG RESET"); break;
    case ESP_RST_TASK_WDT:  Serial.println("TASK WATCHDOG RESET"); break;
    case ESP_RST_WDT:       Serial.println("OTHER WATCHDOG RESET"); break;
    case ESP_RST_DEEPSLEEP: Serial.println("WAKE FROM DEEP SLEEP"); break;
    case ESP_RST_BROWNOUT:  Serial.println("BROWNOUT DETECTED (Power supply dropped below 2.8V)"); break;
    case ESP_RST_SDIO:      Serial.println("SDIO RESET"); break;
    default:                Serial.printf("UNKNOWN (%d)\n", reason); break;
  }
  if (reason == ESP_RST_BROWNOUT) {
    Serial.println(">>> WARNING: 5V/3.3V rail dipped! Ensure stepper motors and ESP32 have a common ground and separate power decoupling cap.");
  }
  Serial.println("--------------------------------------------------\n");
}

// ============================================================
// WIFI EVENT HANDLER (Detailed Disconnect Reason Logging)
// ============================================================

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("[WiFi Event] Station Mode Started");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WiFi Event] Connected to AP");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WiFi Event] Got IP: %s\n", IPAddress(info.got_ip.ip_info.ip.addr).toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
      uint8_t reason = info.wifi_sta_disconnected.reason;
      Serial.printf("[WiFi Event] Disconnected from AP. Reason code: %d ", reason);
      switch (reason) {
        case 200: Serial.println("(BEACON_TIMEOUT: AP stopped responding or RF sleep issue)"); break;
        case 201: Serial.println("(NO_AP_FOUND: SSID not seen in air)"); break;
        case 202: Serial.println("(AUTH_FAIL: Password or auth mode rejected)"); break;
        case 204: Serial.println("(HANDSHAKE_TIMEOUT)"); break;
        default:  Serial.println("(OTHER)"); break;
      }
      break;
    }
    default:
      break;
  }
}

// ============================================================
// WIFI MANAGEMENT
// ============================================================

void connectWiFi() {
  Serial.print("[WiFi] Connecting to: ");
  Serial.println(WIFI_SSID);

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWiFiEvent);

  // Disable ESP32 modem sleep to prevent random WiFi beacon dropouts
  WiFi.setSleep(false);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) {
    Serial.print(".");
    delay(400);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Initial connection SUCCESSFUL. IP: %s\n",
                  WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WiFi] Initial connection pending; background auto-reconnect active.");
  }
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED)
    return;

  unsigned long now = millis();
  if (now - lastWiFiRetryMs < WIFI_RETRY_INTERVAL_MS)
    return;

  lastWiFiRetryMs = now;

  Serial.println("[WiFi] Connection down. Triggering non-blocking reconnect...");
  WiFi.reconnect();
}

// ============================================================
// FIRESTORE COMMAND STATUS UPDATE
// ============================================================

void updateCommandStatus(const String &commandId, const String &status) {
  if (WiFi.status() != WL_CONNECTED)
    return;

  String url = String("https://firestore.googleapis.com/v1/projects/") +
               FIREBASE_PROJECT_ID +
               "/databases/(default)/documents/commands/" + commandId +
               "?updateMask.fieldPaths=status&key=" + FIREBASE_API_KEY;

  StaticJsonDocument<256> patchDoc;
  patchDoc["fields"]["status"]["stringValue"] = status;

  String body;
  serializeJson(patchDoc, body);

  HTTPClient http;
  http.setTimeout(3500);
  if (!http.begin(url))
    return;

  http.addHeader("Content-Type", "application/json");
  int code = http.PATCH(body);

  if (code == 200) {
    Serial.printf("[COMMAND] %s -> %s\n", commandId.c_str(), status.c_str());
  } else {
    Serial.printf("[COMMAND] Status update failed: HTTP %d\n", code);
    Serial.println(http.getString());
  }

  http.end();
}

void updateCommandReturnResult(const String &commandId, const String &status,
                               const String &returnStatus,
                               const String &trayStatus) {
  if (WiFi.status() != WL_CONNECTED)
    return;

  String url = String("https://firestore.googleapis.com/v1/projects/") +
               FIREBASE_PROJECT_ID +
               "/databases/(default)/documents/commands/" + commandId +
               "?updateMask.fieldPaths=status&updateMask.fieldPaths="
               "returnStatus&updateMask.fieldPaths=trayStatus&updateMask."
               "fieldPaths=completedAt&key=" +
               FIREBASE_API_KEY;

  StaticJsonDocument<512> patchDoc;
  patchDoc["fields"]["status"]["stringValue"] = status;
  patchDoc["fields"]["returnStatus"]["stringValue"] = returnStatus;
  patchDoc["fields"]["trayStatus"]["stringValue"] = trayStatus;
  patchDoc["fields"]["completedAt"]["stringValue"] = getIsoTimestampString();

  String body;
  serializeJson(patchDoc, body);

  HTTPClient http;
  http.setTimeout(3500);
  if (!http.begin(url))
    return;

  http.addHeader("Content-Type", "application/json");
  http.PATCH(body);
  http.end();
}

void updateSlotReturnStateInStation(int slotId, const String &trayState,
                                    const String &returnStatus,
                                    int returnCountdown = -1,
                                    int presentOverride = -1) {
  if (WiFi.status() != WL_CONNECTED)
    return;

  String url = String("https://firestore.googleapis.com/v1/projects/") +
               FIREBASE_PROJECT_ID +
               "/databases/(default)/documents/stations/" + STATION_ID +
               "?key=" + FIREBASE_API_KEY;

  HTTPClient http;
  http.setTimeout(3500);
  if (!http.begin(url))
    return;

  int httpCode = http.GET();

  if (httpCode != 200) {
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  DynamicJsonDocument doc(16384);
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.printf("[Firebase] Return state JSON error: %s\n", error.c_str());
    return;
  }

  JsonArray slots =
      doc["fields"]["slots"]["arrayValue"]["values"].as<JsonArray>();
  if (slots.isNull())
    return;

  for (JsonObject slotObj : slots) {
    JsonObject fields = slotObj["mapValue"]["fields"];
    int id = fields["id"]["integerValue"].as<String>().toInt();

    if (id != slotId)
      continue;

    if (slotId >= 1 && slotId <= 2) {
      if (trayState.length() > 0) {
        slotTrayState[slotId - 1] = trayState;
        if (trayState == "open") {
          slotOpenedAt[slotId - 1] = getIsoTimestampString();
        } else if (trayState == "closed") {
          slotOpenedAt[slotId - 1] = "";
        }
      }
    }

    if (trayState.length() > 0) {
      fields["trayState"]["stringValue"] = trayState;

      if (trayState == "open") {
        fields["openedAt"]["stringValue"] = getIsoTimestampString();
      } else if (trayState == "closed") {
        fields.remove("openedAt");
      }
    }

    if (returnStatus.length() > 0) {
      fields["returnStatus"]["stringValue"] = returnStatus;
    }

    if (returnCountdown >= 0) {
      fields["returnCountdown"]["integerValue"] = String(returnCountdown);
    }

    if (presentOverride == 1) {
      fields["present"]["booleanValue"] = true;
      fields["powerbankPresent"]["booleanValue"] = true;
      fields["beamSensor"]["stringValue"] = "BROKEN";
      fields["charging"]["booleanValue"] = true;
      fields["chargingState"]["stringValue"] = "CHARGING";
      fields["battery"]["integerValue"] = "10";
      fields["available"]["booleanValue"] = false;
    } else if (presentOverride == 0) {
      fields["present"]["booleanValue"] = false;
      fields["powerbankPresent"]["booleanValue"] = false;
      fields["beamSensor"]["stringValue"] = "CLEAR";
      fields["charging"]["booleanValue"] = false;
      fields["chargingState"]["stringValue"] = "NOT PRESENT";
      fields["battery"]["integerValue"] = "0";
      fields["available"]["booleanValue"] = false;
    }

    break;
  }

  String patchUrl = String("https://firestore.googleapis.com/v1/projects/") +
                    FIREBASE_PROJECT_ID +
                    "/databases/(default)/documents/stations/" + STATION_ID +
                    "?updateMask.fieldPaths=slots&key=" + FIREBASE_API_KEY;

  String patchPayload;
  serializeJson(doc, patchPayload);

  HTTPClient patchHttp;
  patchHttp.setTimeout(3500);
  if (!patchHttp.begin(patchUrl))
    return;

  patchHttp.addHeader("Content-Type", "application/json");
  patchHttp.PATCH(patchPayload);
  patchHttp.end();
}

// ============================================================
// POWER BANK RETURN SEQUENCE (UNCHANGED)
// ============================================================

bool executeReturnSequence(int slotId, const String &commandId,
                           const String &userId) {
  if (slotId != 1 && slotId != 2)
    return false;

  if (motorBusy) {
    Serial.println("[RETURN] Motor is currently busy; command refused.");
    updateCommandReturnResult(commandId, "failed", "NOT_CONFIRMED", "busy");
    return false;
  }

  uint8_t beamPin = (slotId == 1) ? SLOT1_BEAM_PIN : SLOT2_BEAM_PIN;

  Serial.println();
  Serial.println("==================================================");
  Serial.printf("[RETURN] INITIATING RETURN SEQUENCE FOR SLOT %d\n", slotId);
  Serial.printf("[RETURN] IR Beam GPIO: %d | User ID: %s\n", beamPin,
                userId.c_str());
  Serial.println("==================================================");

  updateSlotReturnStateInStation(slotId, "opening", "PENDING", 30);
  updateCommandReturnResult(commandId, "processing", "PENDING", "opening");

  bool openOk = openTray(slotId);

  if (!openOk) {
    Serial.println("[RETURN] Motor open failed!");
    updateCommandReturnResult(commandId, "failed", "NOT_CONFIRMED", "error");
    updateSlotReturnStateInStation(slotId, "closed", "NOT_CONFIRMED", 0);
    return false;
  }

  Serial.println();
  Serial.printf("[RETURN] Tray %d fully opened.\n", slotId);
  Serial.println("[RETURN] >>> Please place your power bank inside the opened "
                 "slot. You have 30 seconds. <<<");
  Serial.println(
      "[RETURN] Continuous requirement: IR beam must remain BROKEN >= 5s.");

  updateSlotReturnStateInStation(slotId, "open", "AWAITING_INSERTION", 30);
  updateCommandReturnResult(commandId, "processing", "AWAITING_INSERTION",
                            "open");

  unsigned long windowStartMs = millis();
  unsigned long continuousBrokenStartMs = 0;
  bool returnConfirmed = false;
  unsigned long lastSyncMs = millis();

  while (millis() - windowStartMs < 30000) {
    unsigned long now = millis();
    maintainWiFi();

    bool isBroken = (digitalRead(beamPin) == LOW);

    if (isBroken) {
      if (continuousBrokenStartMs == 0) {
        continuousBrokenStartMs = now;
        Serial.printf(
            "[RETURN] Slot %d: Beam BROKEN. Starting continuous 5s timer...\n",
            slotId);
      } else if (!returnConfirmed && (now - continuousBrokenStartMs >= 5000)) {
        returnConfirmed = true;
        Serial.printf("\n[RETURN] Slot %d: Continuous 5s beam break verified "
                      "-> RETURN CONFIRMED!\n",
                      slotId);
        updateSlotReturnStateInStation(
            slotId, "open", "CONFIRMED",
            max(0, 30 - (int)((now - windowStartMs) / 1000)));
      }
    } else {
      if (continuousBrokenStartMs != 0 && !returnConfirmed) {
        Serial.printf("[RETURN] Slot %d: Beam CLEARED before 5s. Continuous "
                      "timer reset.\n",
                      slotId);
        continuousBrokenStartMs = 0;
      }
    }

    if (now - lastSyncMs >= 2000) {
      lastSyncMs = now;
      int remainingSec = max(0, 30 - (int)((now - windowStartMs) / 1000));
      updateSlotReturnStateInStation(
          slotId, "open", returnConfirmed ? "CONFIRMED" : "AWAITING_INSERTION",
          remainingSec);
    }

    delay(50);
  }

  Serial.println();
  Serial.printf("[RETURN] 30-second countdown ended. Result: %s\n",
                returnConfirmed ? "CONFIRMED" : "NOT_CONFIRMED");

  String finalStatus = returnConfirmed ? "CONFIRMED" : "NOT_CONFIRMED";

  updateSlotReturnStateInStation(slotId, "closing", finalStatus, 0);
  updateCommandReturnResult(commandId, "processing", finalStatus, "closing");

  closeTray(slotId);

  Serial.printf("[RETURN] Tray %d closed.\n", slotId);

  int presentOverride = returnConfirmed ? 1 : 0;

  updateSlotReturnStateInStation(slotId, "closed", finalStatus, 0,
                                 presentOverride);
  updateCommandReturnResult(commandId, "completed", finalStatus, "closed");

  Serial.println("[RETURN] Return process completed.");
  Serial.println("==================================================\n");

  return returnConfirmed;
}

// ============================================================
// FIRESTORE COMMAND POLLING
// ============================================================

void pollCommands() {
  if (WiFi.status() != WL_CONNECTED || motorBusy)
    return;

  String url =
      String("https://firestore.googleapis.com/v1/projects/") +
      FIREBASE_PROJECT_ID +
      "/databases/(default)/documents:runQuery?key=" + FIREBASE_API_KEY;

  StaticJsonDocument<512> queryDoc;

  JsonObject structuredQuery = queryDoc.createNestedObject("structuredQuery");
  JsonArray from = structuredQuery.createNestedArray("from");
  from.createNestedObject()["collectionId"] = "commands";

  JsonObject filter =
      structuredQuery.createNestedObject("where").createNestedObject(
          "fieldFilter");

  filter["field"]["fieldPath"] = "status";
  filter["op"] = "EQUAL";
  filter["value"]["stringValue"] = "pending";

  structuredQuery["limit"] = 5;

  String requestBody;
  serializeJson(queryDoc, requestBody);

  HTTPClient http;
  http.setTimeout(3500);

  if (!http.begin(url))
    return;

  http.addHeader("Content-Type", "application/json");

  int httpCode = http.POST(requestBody);

  if (httpCode != 200) {
    Serial.printf("[Firebase] Command poll failed: HTTP %d\n", httpCode);
    Serial.println(http.getString());
    http.end();
    return;
  }

  String response = http.getString();
  http.end();

  DynamicJsonDocument responseDoc(8192);
  DeserializationError error = deserializeJson(responseDoc, response);

  if (error) {
    Serial.printf("[Firebase] Command JSON error: %s\n", error.c_str());
    return;
  }

  JsonArray results = responseDoc.as<JsonArray>();

  for (JsonObject result : results) {
    if (!result.containsKey("document"))
      continue;

    JsonObject document = result["document"];
    String docName = document["name"].as<String>();

    int slash = docName.lastIndexOf('/');
    if (slash < 0)
      continue;

    String commandId = docName.substring(slash + 1);
    JsonObject fields = document["fields"];

    String action = fields["action"]["stringValue"] | "";
    String userId = fields["userId"]["stringValue"] | "anonymous";

    int slotId = 0;

    if (fields["slotId"].containsKey("integerValue")) {
      slotId = fields["slotId"]["integerValue"].as<String>().toInt();
    } else if (fields["slotId"].containsKey("stringValue")) {
      slotId = fields["slotId"]["stringValue"].as<String>().toInt();
    }

    Serial.printf("[COMMAND] action=%s slot=%d id=%s\n", action.c_str(), slotId,
                  commandId.c_str());

    if (slotId != 1 && slotId != 2) {
      updateCommandStatus(commandId, "failed");
      continue;
    }

    bool handled = false;

    if (action == "RETURN" || action == "RETURN_POWERBANK") {
      handled = executeReturnSequence(slotId, commandId, userId);
      break;

    } else if (action == "OPEN") {

      Serial.printf("[RENTAL] OPEN command received for Slot %d. Updating station to opening.\n", slotId);
      updateSlotReturnStateInStation(slotId, "opening", "OPENING", 30);

      // ORIGINAL OPEN FUNCTION + 30 SECOND RENTAL TIMER
      handled = openTray(slotId);

      if (handled) {
        rentalTrayOpen = true;
        rentalOpenSlotId = slotId;
        rentalOpenedAtMs = millis();

        updateSlotReturnStateInStation(slotId, "open", "READY_FOR_COLLECTION", 30);
        Serial.printf("[RENTAL] Slot %d opened. Auto-close in 30 seconds.\n",
                      slotId);
      } else {
        updateSlotReturnStateInStation(slotId, "closed", "ERROR", 0);
        Serial.printf("[RENTAL] Failed to open Slot %d\n", slotId);
      }

    } else if (action == "CLOSE") {

      // Cancel rental timer if this is the same slot
      if (rentalTrayOpen && rentalOpenSlotId == slotId) {
        rentalTrayOpen = false;
        rentalOpenSlotId = 0;
      }

      Serial.printf("[COMMAND] CLOSE command received for Slot %d. Updating station to closing.\n", slotId);
      updateSlotReturnStateInStation(slotId, "closing", "CLOSING", 0);

      handled = closeTray(slotId);

      updateSlotReturnStateInStation(slotId, "closed", "COMPLETED", 0);
      Serial.printf("[COMMAND] Slot %d closed.\n", slotId);

    } else if (action == "VERIFY_RETURN") {

      bool present =
          readPowerBankPresent((slotId == 1) ? SLOT1_BEAM_PIN : SLOT2_BEAM_PIN);

      Serial.println(present ? "[RETURN] POWERBANK PRESENT"
                             : "[RETURN] POWERBANK NOT PRESENT");

      handled = true;

    } else {
      Serial.printf("[COMMAND] Unknown action: %s\n", action.c_str());
      updateCommandStatus(commandId, "failed");
      continue;
    }

    if (handled) {
      updateCommandStatus(commandId, "completed");
    } else {
      updateCommandStatus(commandId, "failed");
    }

    break;
  }
}

// ============================================================
// FIRESTORE STATION SYNC — SINGLE SOURCE OF TRUTH
// Uses liveSnapshot directly; no redundant re-reads.
// ============================================================

void syncHardwareState() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf(">>> [Firebase Upload] Skipped: WiFi disconnected (%s)\n", WIFI_SSID);
    return;
  }

  const unsigned long cycle = liveSnapshot.cycleNumber;
  String sampleTime = liveSnapshot.timestamp;
  if (sampleTime.length() == 0) {
    sampleTime = getIsoTimestampString();
  }

  bool pb1Present = readPowerBankPresent(SLOTS[0].beamPin);
  bool pb2Present = readPowerBankPresent(SLOTS[1].beamPin);

  bool s1OK = liveSnapshot.slot1Connected && liveSnapshot.slot1Valid;
  bool s2OK = liveSnapshot.slot2Connected && liveSnapshot.slot2Valid;

  float i1 = s1OK ? liveSnapshot.slot1Current_mA : 0.0f;
  float v1 = s1OK ? liveSnapshot.slot1BusVoltage_V : 0.0f;
  float p1 = s1OK ? liveSnapshot.slot1Power_mW : 0.0f;

  float i2 = s2OK ? liveSnapshot.slot2Current_mA : 0.0f;
  float v2 = s2OK ? liveSnapshot.slot2BusVoltage_V : 0.0f;
  float p2 = s2OK ? liveSnapshot.slot2Power_mW : 0.0f;

  if (isnan(i1) || isinf(i1)) i1 = 0.0f;
  if (isnan(v1) || isinf(v1)) v1 = 0.0f;
  if (isnan(p1) || isinf(p1)) p1 = 0.0f;

  if (isnan(i2) || isinf(i2)) i2 = 0.0f;
  if (isnan(v2) || isinf(v2)) v2 = 0.0f;
  if (isnan(p2) || isinf(p2)) p2 = 0.0f;

  // Stable charging status from updateSlotCharging (hysteresis + debounce)
  bool charging1 = slot1ChargeState.isCharging;
  bool charging2 = slot2ChargeState.isCharging;

  // Differentiate: SENSOR ERROR vs NOT PRESENT vs CHARGING vs NOT CHARGING
  String state1;
  if (!s1OK) {
    state1 = "SENSOR ERROR";
  } else if (!pb1Present) {
    state1 = "NOT PRESENT";
  } else {
    state1 = charging1 ? "CHARGING" : "NOT CHARGING";
  }

  String state2;
  if (!s2OK) {
    state2 = "SENSOR ERROR";
  } else if (!pb2Present) {
    state2 = "NOT PRESENT";
  } else {
    state2 = charging2 ? "CHARGING" : "NOT CHARGING";
  }

  DynamicJsonDocument patchDoc(4096);
  patchDoc["fields"]["lastUpdated"]["stringValue"] = getIsoTimestampString();

  JsonArray slotsArray = patchDoc["fields"]["slots"]["arrayValue"]["values"].to<JsonArray>();

  // Slot 1 (INA219 0x41)
  JsonObject slot1Item = slotsArray.createNestedObject();
  JsonObject slot1Map = slot1Item.createNestedObject("mapValue");
  JsonObject slot1Obj = slot1Map.createNestedObject("fields");

  slot1Obj["id"]["integerValue"] = "1";
  slot1Obj["slotId"]["integerValue"] = "1";
  slot1Obj["slotNumber"]["stringValue"] = "01";
  slot1Obj["beamSensor"]["stringValue"] = pb1Present ? "BROKEN" : "CLEAR";
  slot1Obj["powerbankPresent"]["booleanValue"] = pb1Present;
  slot1Obj["present"]["booleanValue"] = pb1Present;
  slot1Obj["current_mA"]["doubleValue"] = i1;
  slot1Obj["busVoltage_V"]["doubleValue"] = v1;
  slot1Obj["power_mW"]["doubleValue"] = p1;
  slot1Obj["chargingState"]["stringValue"] = state1;
  slot1Obj["charging"]["booleanValue"] = charging1;
  slot1Obj["currentSensorConnected"]["booleanValue"] = s1OK;
  slot1Obj["currentSensorAddress"]["stringValue"] = "0x41";
  slot1Obj["ina219Connected"]["booleanValue"] = s1OK;
  slot1Obj["sensorStatus"]["stringValue"] = s1OK ? "CONNECTED" : "SENSOR ERROR";
  slot1Obj["trayState"]["stringValue"] = slotTrayState[0];
  if (slotTrayState[0] == "open" && slotOpenedAt[0].length() > 0) {
    slot1Obj["openedAt"]["stringValue"] = slotOpenedAt[0];
  }
  slot1Obj["measurementTimestamp"]["stringValue"] = sampleTime;

  // Slot 2 (INA219 0x40)
  JsonObject slot2Item = slotsArray.createNestedObject();
  JsonObject slot2Map = slot2Item.createNestedObject("mapValue");
  JsonObject slot2Obj = slot2Map.createNestedObject("fields");

  slot2Obj["id"]["integerValue"] = "2";
  slot2Obj["slotId"]["integerValue"] = "2";
  slot2Obj["slotNumber"]["stringValue"] = "02";
  slot2Obj["beamSensor"]["stringValue"] = pb2Present ? "BROKEN" : "CLEAR";
  slot2Obj["powerbankPresent"]["booleanValue"] = pb2Present;
  slot2Obj["present"]["booleanValue"] = pb2Present;
  slot2Obj["current_mA"]["doubleValue"] = i2;
  slot2Obj["busVoltage_V"]["doubleValue"] = v2;
  slot2Obj["power_mW"]["doubleValue"] = p2;
  slot2Obj["chargingState"]["stringValue"] = state2;
  slot2Obj["charging"]["booleanValue"] = charging2;
  slot2Obj["currentSensorConnected"]["booleanValue"] = s2OK;
  slot2Obj["currentSensorAddress"]["stringValue"] = "0x40";
  slot2Obj["ina219Connected"]["booleanValue"] = s2OK;
  slot2Obj["sensorStatus"]["stringValue"] = s2OK ? "CONNECTED" : "SENSOR ERROR";
  slot2Obj["trayState"]["stringValue"] = slotTrayState[1];
  if (slotTrayState[1] == "open" && slotOpenedAt[1].length() > 0) {
    slot2Obj["openedAt"]["stringValue"] = slotOpenedAt[1];
  }
  slot2Obj["measurementTimestamp"]["stringValue"] = sampleTime;

  String patchPayload;
  serializeJson(patchDoc, patchPayload);

  String patchUrl = String("https://firestore.googleapis.com/v1/projects/") +
                    FIREBASE_PROJECT_ID + "/databases/(default)/documents/stations/" +
                    STATION_ID +
                    "?updateMask.fieldPaths=slots&updateMask.fieldPaths=lastUpdated&key=" +
                    FIREBASE_API_KEY;

  HTTPClient patchHttp;
  patchHttp.setTimeout(3500);
  if (!patchHttp.begin(patchUrl)) {
    Serial.println(">>> [Firebase Upload] Error: http.begin failed");
    return;
  }

  patchHttp.addHeader("Content-Type", "application/json");

  Serial.println();
  Serial.printf(">>> [Firebase Upload] Cycle #%lu (Time: %s)\n", cycle, sampleTime.c_str());
  Serial.printf("    Slot 1 (0x41): Current: %.2f mA | Voltage: %.2f V | Power: %.2f mW | State: %s (Present: %s, Charging: %s)\n",
                i1, v1, p1, state1.c_str(), pb1Present ? "YES" : "NO", charging1 ? "YES" : "NO");
  Serial.printf("    Slot 2 (0x40): Current: %.2f mA | Voltage: %.2f V | Power: %.2f mW | State: %s (Present: %s, Charging: %s)\n",
                i2, v2, p2, state2.c_str(), pb2Present ? "YES" : "NO", charging2 ? "YES" : "NO");

  int patchCode = patchHttp.PATCH(patchPayload);

  if (patchCode == 200) {
    Serial.printf("    HTTP Status : %d (SUCCESS)\n", patchCode);
  } else {
    Serial.printf("    HTTP Status : %d (FAILED)\n", patchCode);
    String errBody = patchHttp.getString();
    if (errBody.length() > 0) {
      Serial.printf("    Response    : %s\n", errBody.c_str());
    }
    Serial.println("    Payload     : " + patchPayload);
  }
  Serial.println("<<< [Firebase Upload Complete]");
  Serial.println();

  patchHttp.end();
}

// ============================================================
// SLOT LED INDICATION
// Empty: Red ON, Green OFF
// Present + not charging/full: Green ON, Red OFF
// Present + charging: alternate red and green
// ============================================================

const unsigned long LED_BLINK_INTERVAL_MS = 1000; // Slower blink
unsigned long lastLedToggleMs = 0;
bool ledBlinkPhase = false;

void updateSlotLEDs() {
  unsigned long now = millis();

  // Toggle the charging indication without blocking motors, sensors or WiFi.
  if (now - lastLedToggleMs >= LED_BLINK_INTERVAL_MS) {
    lastLedToggleMs = now;
    ledBlinkPhase = !ledBlinkPhase;
  }

  // IR LOW means beam broken = power bank present.
  bool slot1Present = (digitalRead(SLOT1_BEAM_PIN) == LOW);
  bool slot2Present = (digitalRead(SLOT2_BEAM_PIN) == LOW);

  // When slot is empty: Red ON, Green OFF
  // When slot has power bank: Green ON, Red OFF
  bool slot1RedOn = !slot1Present;
  bool slot1GreenOn = slot1Present;
  bool slot2RedOn = !slot2Present;
  bool slot2GreenOn = slot2Present;

  // Only alternate LEDs when a power bank is present and confirmed charging.
  if (slot1Present && slot1ChargeState.isCharging) {
    slot1RedOn = ledBlinkPhase;
    slot1GreenOn = !ledBlinkPhase;
  }

  if (slot2Present && slot2ChargeState.isCharging) {
    slot2RedOn = ledBlinkPhase;
    slot2GreenOn = !ledBlinkPhase;
  }

  digitalWrite(SLOT1_RED_PIN, slot1RedOn ? HIGH : LOW);
  digitalWrite(SLOT1_GREEN_PIN, slot1GreenOn ? HIGH : LOW);

  digitalWrite(SLOT2_RED_PIN, slot2RedOn ? HIGH : LOW);
  digitalWrite(SLOT2_GREEN_PIN, slot2GreenOn ? HIGH : LOW);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1500);

  // Diagnostic output: Check why the ESP32 rebooted (e.g. brownout, watchdog, normal power on)
  printResetReason();

  for (int i = 0; i < SLOT_COUNT; i++) {
    pinMode(SLOTS[i].beamPin, INPUT_PULLUP);
  }

  // Configure slot status LEDs
  pinMode(SLOT1_RED_PIN, OUTPUT);
  pinMode(SLOT1_GREEN_PIN, OUTPUT);
  pinMode(SLOT2_RED_PIN, OUTPUT);
  pinMode(SLOT2_GREEN_PIN, OUTPUT);

  // Initialize LEDs based on current physical slot occupancy (Empty: Red ON, Present: Green ON)
  updateSlotLEDs();

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_IN3, OUTPUT);
  pinMode(M1_IN4, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT);
  pinMode(M2_IN4, OUTPUT);

  stopMotor1();
  stopMotor2();

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("\n[INA219] Initializing Slot 1 INA219 (I2C: 0x41)...");
  sensor1OK = sensor1.begin();
  if (sensor1OK) {
    sensor1.setCalibration_32V_2A();
    liveSnapshot.slot1Connected = true;
  }
  Serial.printf("[INA219] Slot 1 Sensor (0x41): %s\n",
                sensor1OK ? "INITIALIZED SUCCESSFULLY"
                          : "FAILED / NOT FOUND");

  Serial.println("[INA219] Initializing Slot 2 INA219 (I2C: 0x40)...");
  sensor2OK = sensor2.begin();
  if (sensor2OK) {
    sensor2.setCalibration_32V_2A();
    liveSnapshot.slot2Connected = true;
  }
  Serial.printf("[INA219] Slot 2 Sensor (0x40): %s\n",
                sensor2OK ? "INITIALIZED SUCCESSFULLY"
                          : "FAILED / NOT FOUND");

  // Initial sensor measurement and diagnostic output
  readAndMonitorSensors();

  connectWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print("[NTP] Waiting for time sync");
    struct tm timeinfo;
    int ntpAttempts = 0;
    while (!getLocalTime(&timeinfo, 1000) && ntpAttempts < 10) {
      Serial.print(".");
      ntpAttempts++;
    }
    if (ntpAttempts < 10) {
      char buf[32];
      strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &timeinfo);
      Serial.printf("\n[NTP] Time synced: %s UTC\n", buf);
    } else {
      Serial.println("\n[NTP] Sync failed. Using UPTIME fallback timestamps.");
    }
  }

  Serial.println("\n[SYSTEM] READY");
  Serial.println("Beam logic: HIGH = CLEAR/EMPTY; LOW = BROKEN/PRESENT");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  unsigned long now = millis();

  // Update both slot LEDs continuously (non-blocking)
  updateSlotLEDs();

  // 1. Dedicated live sensor reading & Serial Monitor output EVERY 1 SECOND
  if (now - lastSensorMonitorMs >= SENSOR_MONITOR_INTERVAL_MS) {
    lastSensorMonitorMs = now;
    readAndMonitorSensors();
  }

  // 2. WiFi maintenance (non-blocking)
  maintainWiFi();

  // 3. AUTOMATIC RENTAL TRAY CLOSING AFTER 30 SECONDS
  if (rentalTrayOpen && now - rentalOpenedAtMs >= 30000) {

    int slotId = rentalOpenSlotId;

    rentalTrayOpen = false;
    rentalOpenSlotId = 0;

    Serial.printf(
        "[RENTAL] 30 seconds passed. Automatically closing Slot %d.\n", slotId);

    updateSlotReturnStateInStation(slotId, "closing", "", -1);

    bool closed = closeTray(slotId);

    if (closed) {
      updateSlotReturnStateInStation(slotId, "closed", "", -1);
      Serial.printf("[RENTAL] Slot %d automatically closed.\n", slotId);
    }
  }

  // 4. Command polling (every 2s if WiFi is connected)
  if (WiFi.status() == WL_CONNECTED &&
      now - lastCommandPollMs >= COMMAND_POLL_INTERVAL_MS) {

    lastCommandPollMs = now;
    pollCommands();
  }

  // 5. Firebase state upload (every 4s if WiFi is connected)
  if (now - lastFirebaseUpdateMs >= FIREBASE_UPDATE_INTERVAL_MS) {
    lastFirebaseUpdateMs = now;
    if (WiFi.status() == WL_CONNECTED) {
      syncHardwareState();
    }
  }

  delay(20);
}
