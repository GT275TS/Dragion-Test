// lmic_pin_map.h (finalized per your confirmation)

#ifndef LMIC_PIN_MAP_H
#define LMIC_PIN_MAP_H

// --- SPI (ESP32 hardware SPI; SCK = IO5) ---
#define LORA_SCK_PIN    5    // IO5
#define LORA_MOSI_PIN   27   // IO27
#define LORA_MISO_PIN   19   // IO19

// --- LoRa control ---
#define LORA_NSS_PIN    18   // IO18
#define LORA_RST_PIN    23   // IO23
#define LORA_DIO0_PIN   26   // IO26
#define LORA_DIO1_PIN   33   // IO33
#define LORA_DIO2_PIN   32   // IO32

// --- GPS UART ---
#define GPS_SERIAL_HARDWARE Serial1
#define GPS_RX_PIN      10   // IO10 (RXD_MODULE)
#define GPS_TX_PIN      9    // IO9  (TXD_MODULE)

// --- I2C / LIS3DH ---
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22
#define LIS3DH_INT_PIN  14   // IO14

// --- Battery ADC ---
#define BATTERY_ADC_PIN 34   // IO34 (PA2 net)
#define SOLAR_ADC_PIN   -1   // not used

// Optional: charger disable pin (set to actual GPIO if hardware exposes a charger enable/disable)
#define CHG_DISABLE_PIN -1   // set to e.g. 25 if board provides a charger EN pin controllable by MCU

// Voltage divider multiplier (R24=100k, R23=470k)
#define VOLTAGE_DIVIDER_MULTIPLIER 1.212765957f

#endif // LMIC_PIN_MAP_H
