# Dragino Tracker D-LS — Arduino (ESP32) OTAA AU915

This project implements:
- LoRaWAN OTAA (AU915) via MCCI LMIC
- Cayenne LPP payload (GPS + battery %)
- LIS3DH accelerometer for motion detection and wake (I2C)
- USB Serial provisioning of OTAA DevEUI/AppEUI/AppKey (stored to EEPROM)
- Simple downlink handling (ACK, set movement threshold, request immediate uplink)

Important: This is configured for the Dragino Tracker D-LS v1.3 board pin mapping confirmed with you.
Before building, confirm src/lmic_pin_map.h matches your board (it should already).

Required libraries (install via PlatformIO or Arduino Library Manager):
- MCCI LoRaWAN LMIC (MCCI)
- TinyGPSPlus
- CayenneLPP
- Adafruit BusIO
- Adafruit Unified Sensor
- Adafruit LIS3DH
- EEPROM (board core provides this / Preferences can be used)

PlatformIO (recommended)
- Install PlatformIO in VSCode.
- Open the folder (the repo root contains platformio.ini).
- Build: PlatformIO: Build
- Upload/Flash: PlatformIO: Upload (select correct serial port)

Arduino IDE
- Use Arduino ESP32 core (select the board matching ESP32-PICO-D4).
- Install listed libraries.
- Open src/TrackerD_LS.ino and upload.

Pin map (already set)
- RESET = IO23
- NSS (CS) = IO18
- SCK = IO5
- MOSI = IO27
- MISO = IO19
- DIO0 = IO26
- DIO1 = IO33
- DIO2 = IO32
- GPS RX = IO10 (SD3/IO10) — GPS TX -> ESP RX
- GPS TX = IO9  (SD2/IO9)  — ESP TX  -> GPS RX
- I2C SDA = IO21, SCL = IO22
- LIS3DH INT = IO14
- Battery ADC = IO34 (PA2 net) with multiplier = 1.212765957 (R24=100k / R23=470k)

Quick provisioning & test checklist
1. Connect device to USB-C, open serial monitor at 115200.
2. On first boot the sketch asks for DevEUI, AppEUI, AppKey. Paste your OTAA keys (hex) or type `skip` for placeholders.
3. Device restarts and tries to join OTAA. Watch serial for join events.
4. Movement test:
   - Shake the tracker to trigger LIS3DH interrupt — device should wake and send an uplink (subject to join).
5. Manual send:
   - Over serial type `sendnow` to request an immediate uplink.
6. Downlink test:
   - Send a small downlink (0x03) to request immediate uplink or 0x02 + 2 bytes to set movement threshold.
7. Battery reading check:
   - On serial `status` should print battery %. Confirm with a meter if needed.

If something fails:
- Check wiring (NSS/RESET/DIO0 pins). If join doesn't start, confirm LoRa pins and that the RFM92 module is powered.
- If GPS doesn't produce NMEA, double-check GPS RX/TX pins or swap RX/TX.
- If LIS3DH is not detected, check I2C pins and pull-ups.

If you want me to push to GitHub:
- Create a private repo GT275TS/dragino-tracker-dls-arduino and give me access OR push these files yourself and invite me; then I can open PRs/fixups and iterate.