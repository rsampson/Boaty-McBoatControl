// protocol.h - radio link shared by BoatController and BoatReceiver.
//
// THIS FILE EXISTS IN BOTH SKETCH FOLDERS AND MUST BE IDENTICAL IN EACH.
// If you change a packet layout, bump PROTO_VERSION and copy the file across;
// each end drops packets whose version does not match its own.
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#define PROTO_VERSION      5
#define RADIO_CHANNEL      1      // 1..13, must be the same on both ends
#define CONTROL_RATE_HZ    50     // controller -> boat
#define TELEMETRY_RATE_HZ  5      // boat -> controller
#define CMD_MAX            1000   // motor command range is -CMD_MAX..+CMD_MAX

enum : uint8_t {
  MAGIC_CONTROL   = 0xB0,
  MAGIC_TELEMETRY = 0xB1,
};

enum : uint8_t {
  TELEM_FLAG_ROUTE_OK    = 0x01,  // waypoints exist and are all within range of home
  TELEM_FLAG_GPS_PRESENT = 0x02,  // the GPS module is sending data
  TELEM_FLAG_GPS_FIX     = 0x04,  // the position fields below are current
  TELEM_FLAG_HOME_SET    = 0x08,  // the boat has recorded its starting point
  TELEM_FLAG_COMPASS_OK  = 0x10,  // the compass is responding
  TELEM_FLAG_COMPASS_CAL = 0x20,  // the compass has been calibrated
  TELEM_FLAG_NAV_HOLD    = 0x40,  // autopilot is waiting (no GPS fix or no compass)
  TELEM_FLAG_AT_HOME     = 0x80,  // returned home and holding there
};

// What the boat is doing. Reported in telemetry.
enum BoatMode : uint8_t {
  MODE_MANUAL = 0,                // driven by the joystick
  MODE_ROUTE,                     // following the waypoint list
  MODE_HOME,                      // returning to (or holding at) the starting point
  MODE_CALIBRATING,               // compass calibration: motors off, turn the boat by hand
  MODE_SELFTEST,                  // self-test: sensors checked, each motor run briefly
};

// Self-test progress. Reported in telemetry.
enum TestStep : uint8_t {
  TEST_IDLE = 0,                  // no test running (results may remain from the last one)
  TEST_LEFT_AHEAD,
  TEST_LEFT_ASTERN,
  TEST_RIGHT_AHEAD,
  TEST_RIGHT_ASTERN,
  TEST_DONE,
  TEST_ABORTED,
};

// Self-test sensor results. Reported in telemetry.
enum : uint8_t {
  TEST_BATTERY_OK  = 0x01,        // battery reading is above the minimum
  TEST_GPS_DATA    = 0x02,        // GPS module is sending data
  TEST_GPS_FIX     = 0x04,        // GPS has a position fix
  TEST_COMPASS_OK  = 0x08,        // compass is responding
  TEST_COMPASS_CAL = 0x10,        // compass has been calibrated
  TEST_RADIO_OK    = 0x20,        // boat is receiving most control packets
  TEST_RAN         = 0x80,        // a test has been started since power-up
};

// One-shot requests from the controller. The controller repeats the latest
// command in every packet; the boat acts once each time commandSeq changes.
enum : uint8_t {
  CMD_NONE = 0,
  CMD_MODE_MANUAL,
  CMD_MODE_ROUTE,
  CMD_MODE_HOME,
  CMD_SET_HOME,                   // make the boat's present position the home point
  CMD_CALIBRATE_COMPASS,
  CMD_SELF_TEST,
  CMD_SET_PARAM,                  // store paramValue in the boat setting paramId
};

// Boat settings that the controller can change. They are kept in the boat's flash.
enum : uint8_t {
  PARAM_NONE = 0,
  PARAM_TRIM,                     // -30..+30 %: positive slows the right motor (boat bears to starboard)
  PARAM_CRUISE,                   // autopilot forward power, 200..1000
  PARAM_LOW_BATTERY_MV,           // return home below this battery voltage; 0 = off
  PARAM_GEOFENCE_M,               // return home beyond this distance from home; 0 = off
};

// Why the boat is in MODE_HOME. Reported in telemetry.
enum : uint8_t {
  HOME_NONE = 0,
  HOME_COMMANDED,
  HOME_SIGNAL_LOST,
  HOME_ROUTE_DONE,
  HOME_LOW_BATTERY,
  HOME_GEOFENCE,
};

struct __attribute__((packed)) ControlPacket {
  uint8_t  magic;                 // MAGIC_CONTROL
  uint8_t  version;               // PROTO_VERSION
  uint16_t seq;                   // increments every packet
  int16_t  left;                  // -CMD_MAX (full astern) .. +CMD_MAX (full ahead)
  int16_t  right;
  uint8_t  command;               // CMD_*
  uint8_t  commandSeq;            // changes when `command` is a new request
  uint8_t  paramId;               // PARAM_*, used with CMD_SET_PARAM
  int16_t  paramValue;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t  magic;                 // MAGIC_TELEMETRY
  uint8_t  version;               // PROTO_VERSION
  uint16_t batteryMv;             // boat battery, millivolts
  int8_t   rssi;                  // how strongly the boat hears the controller, dBm
  uint8_t  rxPerSec;              // control packets the boat received in the last second
  uint8_t  flags;                 // TELEM_FLAG_*
  int32_t  latE7;                 // latitude,  degrees x 10^7 (north positive)
  int32_t  lonE7;                 // longitude, degrees x 10^7 (east positive)
  uint16_t speedCmS;              // speed over ground, cm/s
  uint16_t courseDeci;            // course over ground, 0.1 degree steps
  uint8_t  sats;                  // satellites in use
  uint8_t  mode;                  // BoatMode
  uint8_t  waypoint;              // waypoint being steered for, 1-based (0 = none)
  uint8_t  waypointCount;         // waypoints in the boat's route
  uint16_t distanceM;             // metres to that waypoint, or to home otherwise
  uint16_t headingDeci;           // compass heading, 0.1 degree steps (0xFFFF = no compass)
  uint8_t  testStep;              // TestStep
  uint8_t  testResults;           // TEST_* bits
  uint16_t testSagMv[4];          // battery voltage drop while each motor step ran
  uint8_t  homeReason;            // HOME_*: why the boat is returning home
  uint16_t homeDistanceM;         // metres from the home point
  uint16_t bearingFromHomeDeg;    // compass bearing from the home point to the boat
  int8_t   trim;                  // current boat settings (see PARAM_*)
  uint16_t cruise;
  uint16_t lowBatteryMv;
  uint16_t geofenceM;
  uint16_t tripDistanceM;         // this trip (since the boat was switched on)
  uint16_t tripTopSpeedCmS;
  uint16_t tripMotorSec;          // time with the motors running
  uint16_t tripMinBatteryMv;
};

// Bring up WiFi in Espressif Long Range mode and register the one peer we talk to.
// Returns false (after printing the reason) if any step fails.
static inline bool linkBegin(const uint8_t peerMac[6]) {
  WiFi.mode(WIFI_STA);

  esp_err_t err;
  #define LINK_TRY(what, call)                                              \
    if ((err = (call)) != ESP_OK) {                                         \
      Serial.printf("Radio: %s failed: %s\n", what, esp_err_to_name(err));  \
      return false;                                                         \
    }

  LINK_TRY("LR protocol", esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));
  LINK_TRY("power save off", esp_wifi_set_ps(WIFI_PS_NONE));
  LINK_TRY("channel", esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE));
  LINK_TRY("tx power", esp_wifi_set_max_tx_power(78));   // units of 0.25 dBm -> 19.5 dBm
  LINK_TRY("esp_now_init", esp_now_init());

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = RADIO_CHANNEL;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  LINK_TRY("add peer", esp_now_add_peer(&peer));

  // Send at the 250 kbit/s long-range rate rather than ESP-NOW's 1 Mbit/s default.
  esp_now_rate_config_t rate = {};
  rate.phymode = WIFI_PHY_MODE_LR;
  rate.rate    = WIFI_PHY_RATE_LORA_250K;
  LINK_TRY("LR rate", esp_now_set_peer_rate_config(peerMac, &rate));

  #undef LINK_TRY
  return true;
}
