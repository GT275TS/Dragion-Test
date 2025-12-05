#ifndef LMIC_PROJECT_CONFIG_H
#define LMIC_PROJECT_CONFIG_H

// LMIC project config for Arduino builds
// Place this file in the same folder as your .ino so it is picked up by LMIC.

// Region: AU915
#define CFG_au915 1

// LMIC options
#define LMIC_DEBUG_LEVEL 1
#define DISABLE_PING
#define DISABLE_BEACONS
#define USE_ORIGINAL_AES
#define LMIC_LORAWAN_SPEC_VERSION LMIC_LORAWAN_SPEC_VERSION_1_0_3

// If you need LMIC to print via Serial, define LMIC_PRINTF_TO Serial in your build flags or here:
//#define LMIC_PRINTF_TO Serial

#endif // LMIC_PROJECT_CONFIG_H
