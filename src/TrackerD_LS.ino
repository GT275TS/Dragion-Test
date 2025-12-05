/*
  TrackerD_LS.ino -- with battery over-voltage detection, smoothing, and Preferences storage
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <CayenneLPP.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <lmic.h>
#include <hal/hal.h>
#include "lmic_pin_map.h"

#ifdef LMIC_PROJECT_CONFIG_H
#include "lmic_project_config.h"
#endif

// Serial & GPS
#define SERIAL_BAUD 115200
TinyGPSPlus gps;
HardwareSerial &gpsSerial = GPS_SERIAL_HARDWARE;

// Cayenne LPP
CayenneLPP lpp(51);

// LIS3DH
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// Preferences for safety config
Preferences prefs;

// EEPROM addresses for keys
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_MAGIC_VALUE 0xA5
#define EEPROM_KEYS_ADDR 1
#define DEV_EUI_SIZE 8
#define APP_EUI_SIZE 8
#define APP_KEY_SIZE 16

// Default OTAA keys (provided by user)
uint8_t devEui[DEV_EUI_SIZE] = { 0xA8,0x40,0x41,0x8D,0x86,0x59,0x63,0x12 };
uint8_t appEui[APP_EUI_SIZE] = { 0xA8,0x40,0x41,0x00,0x00,0x00,0x01,0x02 };
uint8_t appKey[APP_KEY_SIZE] = {
  0x52,0x6C,0x94,0x34,0x44,0x08,0xD7,0xF0,0x45,0xF2,0xD5,0x56,0x67,0x56,0x33,0x25
};

// Movement
float movementThresholdMeters = 50.0;
double lastLat=0, lastLng=0;
bool hasLastLocation=false;

// Battery multiplier from lmic_pin_map.h
const float adcMultiplier = VOLTAGE_DIVIDER_MULTIPLIER;

// LMIC pinmap
const lmic_pinmap lmic_pins = {
  .nss = LORA_NSS_PIN,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LORA_RST_PIN,
  .dio0 = LORA_DIO0_PIN,
  .dio1 = LORA_DIO1_PIN,
  .dio2 = LORA_DIO2_PIN
};

// Safety / OV parameters (defaults; persisted via Preferences)
float OV_THRESHOLD = 4.25f;   // volts -> warning level
float OV_CRITICAL  = 4.30f;   // volts -> critical
float OV_CLEAR     = 4.15f;   // volts -> hysteresis clear point
const unsigned long OV_CONFIRM_MS = 3000; // require 3s confirmation

// State
bool overVoltage = false;
bool ovAlertPending = false;
unsigned long ovStartMs = 0;
unsigned long lastBatteryCheckMs = 0;
const unsigned long BATTERY_CHECK_INTERVAL_MS = 5000;

// Battery smoothing (IIR)
float batteryFiltered = 0.0f;
const float BATTERY_FILTER_ALPHA = 0.12f;

// Forward declarations
void serialProvision();
void writeKeysToEEPROM();
void readKeysFromEEPROM();
void hexStringToBytes(const String &hex, uint8_t *out, size_t expectedLen);
void doJoin();
void sendUplink(bool includeGPS, bool isAlert=false);
void handleDownlink(uint8_t *payload, uint8_t len);
float readBatteryPercent();
float readBatteryVoltage();
float readBatteryVoltageFiltered();
void checkBatterySafety();
void persistSafetyConfigPrefs();
void loadSafetyConfigPrefs();
void scheduleOVAlert(float v, bool critical);
double distanceBetween(double lat1, double lon1, double lat2, double lon2);
void onEvent(ev_t ev);

// LMIC callbacks for keys
void os_getDevEui(u1_t* buf) { memcpy(buf, devEui, DEV_EUI_SIZE); }
void os_getArtEui(u1_t* buf) { memcpy(buf, appEui, APP_EUI_SIZE); }
void os_getDevKey(u1_t* buf) { memcpy(buf, appKey, APP_KEY_SIZE); }

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);
  Serial.println(F("Tracker D-LS boot"));

  // GPS UART
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // I2C for LIS3DH
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // EEPROM (ESP32) for keys
  EEPROM.begin(1024);

  // Read/save keys (existing logic)
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
    Serial.println(F("No keys in EEPROM — writing defaults"));
    writeKeysToEEPROM();
  } else {
    readKeysFromEEPROM();
    Serial.println(F("Loaded keys from EEPROM"));
  }

  // Preferences for safety config
  loadSafetyConfigPrefs();

  // Init LIS3DH
  if (! lis.begin(0x18)) {
    Serial.println(F("LIS3DH not found"));
  } else {
    lis.setRange(LIS3DH_RANGE_2_G);
  }

  // Optional: setup charger disable pin (if your hardware uses it)
  if (CHG_DISABLE_PIN != -1) {
    pinMode(CHG_DISABLE_PIN, OUTPUT);
    digitalWrite(CHG_DISABLE_PIN, LOW); // default: allow charging (assume HIGH disables)
  }

  // LMIC init
  os_init();
  LMIC_reset();

  // Select sub-band for AU915 (sub-band 1 by default)
  LMIC_selectSubBand(1);

  // Start OTAA join
  doJoin();
}

void loop() {
  os_runloop_once();

  // feed GPS
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // periodic battery safety check
  if (millis() - lastBatteryCheckMs > BATTERY_CHECK_INTERVAL_MS) {
    lastBatteryCheckMs = millis();
    checkBatterySafety();
  }

  // simple serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd.equalsIgnoreCase("status")) {
      Serial.print(F("GPS fix: ")); Serial.println(gps.location.isValid());
      Serial.print(F("Battery V: ")); Serial.println(readBatteryVoltage(), 3);
      Serial.print(F("Battery smoothed V: ")); Serial.println(readBatteryVoltageFiltered(), 3);
      Serial.print(F("Battery %: ")); Serial.println(readBatteryPercent());
      Serial.print(F("OV threshold: ")); Serial.println(OV_THRESHOLD);
      Serial.print(F("OV critical: ")); Serial.println(OV_CRITICAL);
      Serial.print(F("OV clear: ")); Serial.println(OV_CLEAR);
      Serial.print(F("OV state: ")); Serial.println(overVoltage ? "OV active" : "normal");
    } else if (cmd.equalsIgnoreCase("sendnow")) {
      sendUplink(gps.location.isValid());
    } else if (cmd.equalsIgnoreCase("help")) {
      Serial.println(F("Commands: status | sendnow | help | provision | ov set <th> <crit> <clear> | ov save"));
    } else if (cmd.equalsIgnoreCase("provision")) {
      Serial.println(F("Entering serial provisioning (overrides stored keys)"));
      serialProvision();
    } else if (cmd.startsWith("ov set")) {
      float a=OV_THRESHOLD,b=OV_CRITICAL,c=OV_CLEAR;
      int n = sscanf(cmd.c_str(),"ov set %f %f %f",&a,&b,&c);
      if (n>=1) OV_THRESHOLD=a;
      if (n>=2) OV_CRITICAL=b;
      if (n>=3) OV_CLEAR=c;
      Serial.print(F("New OV thresholds: ")); Serial.print(OV_THRESHOLD); Serial.print(" "); Serial.print(OV_CRITICAL); Serial.print(" "); Serial.println(OV_CLEAR);
    } else if (cmd.equalsIgnoreCase("ov save")) {
      persistSafetyConfigPrefs();
      Serial.println(F("OV thresholds saved to Preferences"));
    }
  }

  // movement detection
  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    if (!hasLastLocation) {
      lastLat = lat; lastLng = lon; hasLastLocation=true;
      Serial.println(F("Stored initial GPS location"));
    } else {
      double d = distanceBetween(lat, lon, lastLat, lastLng);
      if (d >= movementThresholdMeters) {
        Serial.print(F("Movement detected (m): ")); Serial.println(d);
        sendUplink(true);
        lastLat = lat; lastLng = lon;
      }
    }
  }

  delay(200);
}

// start OTAA join
void doJoin() {
  LMIC_startJoining();
  Serial.println(F("Starting OTAA join"));
}

// convert flexible hex to bytes
void hexStringToBytes(const String &hex, uint8_t *out, size_t expectedLen) {
  String s;
  for (size_t i=0;i<hex.length();i++) {
    char c = hex[i];
    if (isHexadecimalDigit(c)) s += c;
  }
  while (s.length() < expectedLen*2) s += '0';
  for (size_t i=0;i<expectedLen;i++) {
    String b = s.substring(i*2, i*2+2);
    out[i] = (uint8_t) strtoul(b.c_str(), NULL, 16);
  }
}

// serial provisioning
void serialProvision() {
  Serial.println(F("---- Serial provisioning ----"));
  Serial.println(F("Enter DevEUI (hex) or 'skip'"));
  while (!Serial.available()) delay(50);
  String s = Serial.readStringUntil('\n'); s.trim();
  if (!s.equalsIgnoreCase("skip")) hexStringToBytes(s, devEui, DEV_EUI_SIZE);
  Serial.println(F("Enter AppEUI (hex) or 'skip'"));
  while (!Serial.available()) delay(50);
  s = Serial.readStringUntil('\n'); s.trim();
  if (!s.equalsIgnoreCase("skip")) hexStringToBytes(s, appEui, APP_EUI_SIZE);
  Serial.println(F("Enter AppKey (hex) or 'skip'"));
  while (!Serial.available()) delay(50);
  s = Serial.readStringUntil('\n'); s.trim();
  if (!s.equalsIgnoreCase("skip")) hexStringToBytes(s, appKey, APP_KEY_SIZE);
  writeKeysToEEPROM();
  Serial.println(F("Keys saved; restarting..."));
  delay(500);
  ESP.restart();
}

void writeKeysToEEPROM() {
  int addr = EEPROM_KEYS_ADDR;
  for (int i=0;i<DEV_EUI_SIZE;i++) EEPROM.write(addr+i, devEui[i]);
  addr += DEV_EUI_SIZE;
  for (int i=0;i<APP_EUI_SIZE;i++) EEPROM.write(addr+i, appEui[i]);
  addr += APP_EUI_SIZE;
  for (int i=0;i<APP_KEY_SIZE;i++) EEPROM.write(addr+i, appKey[i]);
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.commit();
}

void readKeysFromEEPROM() {
  int addr = EEPROM_KEYS_ADDR;
  for (int i=0;i<DEV_EUI_SIZE;i++) devEui[i] = EEPROM.read(addr+i);
  addr += DEV_EUI_SIZE;
  for (int i=0;i<APP_EUI_SIZE;i++) appEui[i] = EEPROM.read(addr+i);
  addr += APP_EUI_SIZE;
  for (int i=0;i<APP_KEY_SIZE;i++) appKey[i] = EEPROM.read(addr+i);
}

// Build and schedule an uplink (Cayenne LPP)
// If isAlert==true send a small alert payload (voltage + code)
void sendUplink(bool includeGPS, bool isAlert) {
  if (LMIC.opmode & OP_JOINING) { Serial.println(F("Not joined")); return; }
  if (LMIC.opmode & OP_TXRXPEND) { Serial.println(F("TX pending")); return; }

  lpp.reset();
  if (isAlert) {
    float v = readBatteryVoltageFiltered();
    lpp.addAnalogInput(1, v);          // ch1 = voltage (V)
    lpp.addDigitalInput(2, 1);         // ch2 = alert flag (1=OV warning, 2=critical if used)
  } else {
    if (includeGPS && gps.location.isValid()) {
      lpp.addGPS(1, gps.location.lat(), gps.location.lng(), gps.altitude.meters());
    }
    float battPct = readBatteryPercent();
    lpp.addAnalogInput(2, battPct);
  }

  uint8_t port = isAlert ? 3 : 1;
  LMIC_setTxData2(port, lpp.getBuffer(), lpp.getSize(), 0);
  Serial.print(F("Scheduled uplink (alert=")); Serial.print(isAlert); Serial.print(F(") size=")); Serial.println(lpp.getSize());
}

// Simple downlink handler
void handleDownlink(uint8_t *payload, uint8_t len) {
  if (len < 1) return;
  uint8_t cmd = payload[0];
  if (cmd == 0x01) {
    Serial.println(F("Downlink ACK"));
  } else if (cmd == 0x02 && len >= 3) {
    uint16_t newThr = (payload[1] << 8) | payload[2];
    movementThresholdMeters = newThr;
    Serial.print(F("Downlink: set movement threshold = ")); Serial.println(newThr);
  } else if (cmd == 0x03) {
    Serial.println(F("Downlink: request immediate uplink"));
    sendUplink(gps.location.isValid());
  } else {
    Serial.print(F("Unknown downlink cmd: ")); Serial.println(cmd, HEX);
  }
}

// Read battery percent (maps voltage to percent)
float readBatteryPercent() {
  float vbat = readBatteryVoltageFiltered();
  float pct = (vbat - 3.3) / (4.2 - 3.3) * 100.0;
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  return pct;
}

// Read raw battery voltage (in volts)
float readBatteryVoltage() {
  if (BATTERY_ADC_PIN < 0) return 0.0;
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
  int raw = analogRead(BATTERY_ADC_PIN);
  const float ADC_MAX = 4095.0;
  const float ADC_REF = 3.3;
  float v_adc = (raw / ADC_MAX) * ADC_REF;
  float vbat = v_adc * adcMultiplier;
  return vbat;
}

// Read smoothed battery voltage
float readBatteryVoltageFiltered() {
  float v = readBatteryVoltage();
  if (batteryFiltered <= 0.0f) batteryFiltered = v;
  batteryFiltered = BATTERY_FILTER_ALPHA * v + (1.0f - BATTERY_FILTER_ALPHA) * batteryFiltered;
  return batteryFiltered;
}

// Check battery safety and react if thresholds crossed
void checkBatterySafety() {
  float v = readBatteryVoltageFiltered();
  if (!overVoltage) {
    if (v >= OV_THRESHOLD) {
      if (ovStartMs == 0) ovStartMs = millis();
      else if (millis() - ovStartMs >= OV_CONFIRM_MS) {
        overVoltage = true;
        Serial.printf("OV detected v=%.3f V\n", v);
        if (CHG_DISABLE_PIN != -1) digitalWrite(CHG_DISABLE_PIN, HIGH); // assume HIGH disables charger
        scheduleOVAlert(v, false);
      }
    } else {
      ovStartMs = 0;
    }
  } else {
    if (v >= OV_CRITICAL) {
      Serial.printf("OV CRITICAL v=%.3f V\n", v);
      scheduleOVAlert(v, true);
    } else if (v <= OV_CLEAR) {
      Serial.printf("OV cleared v=%.3f V\n", v);
      overVoltage = false;
      ovStartMs = 0;
      if (CHG_DISABLE_PIN != -1) digitalWrite(CHG_DISABLE_PIN, LOW);
    }
  }
}

// Persist safety thresholds using Preferences (NVS)
void persistSafetyConfigPrefs() {
  prefs.begin("safety", false);
  prefs.putFloat("ov_thr", OV_THRESHOLD);
  prefs.putFloat("ov_crit", OV_CRITICAL);
  prefs.putFloat("ov_clear", OV_CLEAR);
  prefs.end();
}

// Load safety thresholds from Preferences
void loadSafetyConfigPrefs() {
  prefs.begin("safety", true);
  OV_THRESHOLD = prefs.getFloat("ov_thr", OV_THRESHOLD);
  OV_CRITICAL  = prefs.getFloat("ov_crit", OV_CRITICAL);
  OV_CLEAR     = prefs.getFloat("ov_clear", OV_CLEAR);
  prefs.end();
}

// Schedule an alert uplink (non-blocking)
void scheduleOVAlert(float v, bool critical) {
  if ((LMIC.opmode & OP_JOINING) || (LMIC.opmode & OP_TXRXPEND)) {
    ovAlertPending = true;
    Serial.println(F("LMIC busy, OV alert pending"));
    return;
  }
  lpp.reset();
  lpp.addAnalogInput(1, v);                     // ch1 = voltage
  lpp.addDigitalInput(2, critical ? 2 : 1);     // ch2 = alert code (1=warning,2=critical)
  LMIC_setTxData2(3, lpp.getBuffer(), lpp.getSize(), 0);
  Serial.println(F("OV alert uplink scheduled"));
}

// Haversine distance (meters)
double distanceBetween(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) + cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// LMIC event callback
void onEvent(ev_t ev) {
  Serial.print(F("LMIC event: ")); Serial.println((int)ev);
  switch(ev) {
    case EV_JOINED:
      Serial.println(F("Joined network"));
      LMIC_setLinkCheckMode(0);
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("TX complete"));
      if (LMIC.dataLen) {
        Serial.print(F("Downlink length=")); Serial.println(LMIC.dataLen);
        uint8_t buf[64];
        for (int i=0;i<LMIC.dataLen && i<64;i++) buf[i] = LMIC.frame[LMIC.dataBeg + i];
        handleDownlink(buf, LMIC.dataLen);
      }
      if (ovAlertPending) {
        ovAlertPending = false;
        scheduleOVAlert(readBatteryVoltageFiltered(), overVoltage && readBatteryVoltageFiltered() >= OV_CRITICAL);
      }
      break;
    default:
      break;
  }
}