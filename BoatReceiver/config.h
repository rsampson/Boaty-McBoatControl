// config.h - settings for the boat.
#pragma once

#include <stdint.h>

// MAC address of the CONTROLLER's ESP32. The controller shows its own MAC on
// its display and serial port at power-up. Copy it here. The boat obeys only
// this address.
static const uint8_t CONTROLLER_MAC[6] = {0xF0, 0x24, 0xF9, 0x43, 0x49, 0x74};

// ---- Pins ------------------------------------------------------------------
// Two DRV8871 drivers, one per motor. Each has two inputs, IN1 and IN2.
#define PIN_LEFT_IN1       25
#define PIN_LEFT_IN2       26
#define PIN_RIGHT_IN1      18
#define PIN_RIGHT_IN2      19

// Battery sense: battery+ --[R_TOP]--+--[R_BOTTOM]-- GND, junction to this pin.
// Take battery+ straight from the pack, not from the 5 V regulator's output:
// a regulated voltage stays constant and says nothing about the charge left.
// Must be an ADC1 pin (GPIO32-39); ADC2 does not work while the radio is on.
#define PIN_BATTERY        34

// NEO-6M GPS on the ESP32's second serial port (9600 baud).
#define PIN_GPS_RX         16     // ESP32 receives here: connect to the module's TX
#define PIN_GPS_TX         17     // ESP32 transmits here: connect to the module's RX

// HMC5883L (or QMC5883L) compass on I2C. Mount it level, as far from the
// motors and battery leads as you can.
#define PIN_COMPASS_SDA    21
#define PIN_COMPASS_SCL    22

#define PIN_STATUS_LED     2      // on-board LED: solid = linked, blinking = no signal
#define PIN_OTA_BUTTON     0      // BOOT button: hold 2 s to enter firmware-update mode

// ---- Firmware update over WiFi (OTA) ----------------------------------------
// In update mode the boat link is off, the motors are off, and the board runs
// its own WiFi network. Join it and upload to 192.168.4.1.
#define OTA_AP_NAME        "Boat-OTA"
#define OTA_AP_PASSWORD    "boatyboat"   // at least 8 characters; change it

// ---- Motors ----------------------------------------------------------------
#define MOTOR_INVERT_LEFT   false  // set true if that motor runs backwards
#define MOTOR_INVERT_RIGHT  false
#define MOTOR_PWM_HZ        20000  // above hearing, well within the DRV8871's range
#define MOTOR_RAMP_MS       300    // time allowed for a 0 -> full power change
#define FAILSAFE_MS         500    // stop the motors after this long without a command

// ---- Battery sense ---------------------------------------------------------
// Sized for a 3-cell lithium pack (12.6 V full): with 100k / 22k the pin sees
// 2.27 V at 12.6 V. Keep the pin below about 3.0 V at your highest battery
// voltage. Trim BATT_CAL against a multimeter if the reading is slightly off.
#define BATT_R_TOP_OHMS     100000.0f
#define BATT_R_BOTTOM_OHMS  22000.0f
#define BATT_CAL            1.000f

// ---- Compass ---------------------------------------------------------------
// After calibrating (controller Nav page), check the heading shown on the
// controller: it should read 0 with the bow pointing north and 90 pointing
// east. If it counts the wrong way round, set COMPASS_REVERSED. If it is off by
// a fixed amount, put the correction in COMPASS_OFFSET_DEG.
#define COMPASS_REVERSED          false
#define COMPASS_OFFSET_DEG        0.0f
#define MAGNETIC_DECLINATION_DEG  0.0f    // local declination, east positive
#define COMPASS_CAL_MS            20000   // how long a calibration lasts

// ---- Autopilot -------------------------------------------------------------
#define NAV_CRUISE_POWER     600     // forward power under autopilot (of 1000)
#define NAV_PIVOT_POWER      400     // power used to spin toward the target
#define NAV_PIVOT_DEG        60.0f   // more than this far off course: spin, don't drive
#define NAV_ARRIVE_RADIUS_M  5.0f    // a waypoint counts as reached inside this
#define NAV_MAX_RANGE_M      1000.0f // refuse a route with any waypoint further than this from home
#define NAV_STICK_OVERRIDE   150     // joystick command (of 1000) that takes back manual control
#define HOME_MIN_SATS        6       // satellites needed before the home point is recorded
#define RTH_DELAY_MS         3000    // signal lost this long: return to the home point
#define NAV_SLOW_RADIUS_M    15.0f   // autopilot eases off inside this distance of its target
#define NAV_SLOW_MIN_PERCENT 40      // ...down to this share of cruise power at the target

// ---- Settings changeable from the controller's Setup page ---------------------
// These are only the values used until a setting is first changed; after that
// the boat keeps its own copy in flash, which survives firmware updates.
#define DEFAULT_TRIM            0       // -30..+30 %, positive slows the right motor
#define DEFAULT_LOW_BATTERY_MV  9900    // return home below this (0 = off); 3.3 V per cell
#define DEFAULT_GEOFENCE_M      0       // return home beyond this distance (0 = off)
#define LOW_BATTERY_HOLD_MS     10000   // battery must stay low this long (it sags under load)

// ---- Self-test -------------------------------------------------------------
#define TEST_POWER           300     // motor power during the self-test (of 1000)
#define TEST_STEP_MS         1500    // how long each motor step runs
#define TEST_BATTERY_MIN_MV  9900    // battery "ok" above this (3-cell lithium, 3.3 V per cell)

// ---- Route -----------------------------------------------------------------
// Waypoints in decimal degrees {latitude, longitude}, visited in order; the boat
// then returns home. North and east are positive. An empty list disables Route
// mode. Example:
//   static const Waypoint WAYPOINTS[] = {
//     {37.4219000, -122.0840000},
//     {37.4222500, -122.0836000},
//   };
struct Waypoint {
  double lat;
  double lon;
};
static const Waypoint WAYPOINTS[] = {
};
