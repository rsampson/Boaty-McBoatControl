// config.h - settings for the hand controller.
#pragma once

#include <stdint.h>

// MAC address of the BOAT's ESP32. The boat prints its own MAC on the serial
// port at power-up ("Boat MAC: ..."). Copy it here.
static const uint8_t BOAT_MAC[6] = {0xF8, 0xB3, 0xB7, 0x4F, 0x56, 0xD8};

// ---- Pins ------------------------------------------------------------------
// Joystick pots: wire the ends to 3V3 and GND, wipers to these pins.
// They must be ADC1 pins (GPIO32-39); ADC2 does not work while the radio is on.
#define PIN_JOY_THROTTLE   34     // forward / back axis
#define PIN_JOY_STEER      35     // left / right axis

// SSD1306 128x64 I2C display
#define PIN_OLED_SDA       21
#define PIN_OLED_SCL       22
#define OLED_I2C_ADDR      0x3C

// Push buttons: wire each between its pin and GND (internal pull-ups are used).
#define PIN_BUTTON_NEXT    32     // next display page
#define PIN_BUTTON_SELECT  33     // previous page; on the Nav page: move / hold 1 s to choose
#define PIN_OTA_BUTTON     0      // BOOT button: hold 2 s to enter firmware-update mode

#define PIN_BEEPER         25     // piezo between this pin and GND
#define PIN_STATUS_LED     2      // on-board LED: heartbeat, two flashes per beat

// ---- Joystick --------------------------------------------------------------
// The stick centre is measured at power-up, so leave the stick alone then.
#define JOY_INVERT_THROTTLE  false   // set true if pushing forward drives astern
#define JOY_INVERT_STEER     false   // set true if pushing right turns to port
#define JOY_RAW_MIN          150     // ADC reading (0-4095) at one end of travel
#define JOY_RAW_MAX          3950    // ADC reading at the other end
#define JOY_DEADBAND         0.06f   // fraction of travel around centre ignored
#define JOY_EXPO             0.30f   // 0 = linear, 1 = very soft around centre
#define STEER_GAIN           0.70f   // how hard full stick turns (1.0 = pivot at full power)

// ---- Display ---------------------------------------------------------------
// Boat battery voltage below which the display shows a LOW warning. 0 disables
// it. 10200 mV suits the boat's 3-cell lithium pack (3.4 V per cell).
#define BATT_WARN_MV         10200

// ---- Controller battery ----------------------------------------------------
// battery+ --[R_TOP]--+--[R_BOTTOM]-- GND, junction to PIN_CONTROLLER_BATTERY.
// Take battery+ from the cell itself, BEFORE any step-up converter. The pin must
// be an ADC1 pin (GPIO32-39). Set the pin to -1 if nothing is wired to it.
#define PIN_CONTROLLER_BATTERY  36
#define CTRL_BATT_R_TOP_OHMS    100000.0f
#define CTRL_BATT_R_BOTTOM_OHMS 100000.0f
#define CTRL_BATT_CAL           1.000f
#define CTRL_BATT_WARN_MV       3400      // LOW warning below this; 0 disables it

// ---- Beeper ----------------------------------------------------------------
// Fast beeps: link lost. Double beep: fewer than BEEP_ACK_CRITICAL_PCT percent
// of packets are being acknowledged. Slow low beep: boat battery low.
#define BEEP_ACK_CRITICAL_PCT  50
#define BEEP_LOW_BATTERY       true

// ---- Firmware update over WiFi (OTA) ----------------------------------------
// In update mode the boat link is off and the board runs its own WiFi network.
// Join it and upload to 192.168.4.1.
#define OTA_AP_NAME        "Controller-OTA"
#define OTA_AP_PASSWORD    "boatyboat"   // at least 8 characters; change it

// ---- Transmit power --------------------------------------------------------
// With TX_POWER_AUTO the radio uses only as much power as keeps the other end
// hearing it between these two signal strengths, and returns to full power the
// moment the link looks doubtful. Set it to false to stay at full power.
#define TX_POWER_AUTO        true
#define TX_RSSI_STRONG_DBM   -60     // heard stronger than this: reduce power
#define TX_RSSI_WEAK_DBM     -75     // heard weaker than this: increase power

// Above this reading from the chip's temperature sensor, transmit power is
// capped to let it cool. The sensor is uncalibrated and reads high (about 70 on
// the bench): watch the Info page and set this to suit your board. 0 disables.
#define TEMP_LIMIT_C         85
