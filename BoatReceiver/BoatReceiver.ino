// BoatReceiver - boat end of a twin-motor, differentially steered RC boat.
//
// Receives left/right motor commands over ESP-NOW Long Range and drives two
// DRV8871 H-bridges. Reports battery voltage, link quality, GPS position and
// compass heading back to the controller.
//
// Modes (chosen from the controller's Nav page):
//   manual - joystick control. Motors stop if the signal is lost.
//   route  - follows the waypoints in config.h by GPS and compass, then returns home.
//   home   - returns to the starting point and holds there.
// A self-test (sensors, then each motor in turn) can also be started from there.
// The boat returns home by itself if the signal stays lost for RTH_DELAY_MS, if
// its battery runs low, or if it strays beyond the geofence (when those are set).
// Moving the joystick takes back manual control from either autopilot mode.
//
// Holding the BOOT button for 2 seconds restarts into firmware-update (OTA)
// mode; see config.h.
//
// Board: classic ESP32 (ESP32 Dev Module), Arduino-ESP32 core 3.3 or later.

#include <ArduinoOTA.h>
#include <Wire.h>
#include <Preferences.h>

#include "config.h"
#include "protocol.h"
#include "navigation.h"

static const uint8_t  PWM_BITS = 10;
static const uint32_t PWM_MAX = (1u << PWM_BITS) - 1;
static const uint32_t MOTOR_TICK_MS = 5;

static bool radioOk = false;

// ---- Diagnostics shared with console.ino (the PC serial console) ----------------
static bool     linkLostNow = true;          // no controller heard recently
static bool     benchActive = false;         // a motor is being run from the PC
static int      benchLeft = 0, benchRight = 0;
static uint32_t benchStartMs = 0, benchDurationMs = 0;
static bool     testFromSerial = false;      // self-test was started from the PC
static uint32_t gpsBytes = 0, gpsSentences = 0, gpsChecksumErrors = 0;
static int16_t  compassRawX = 0, compassRawY = 0;
static uint32_t telemetrySent = 0;

// ---- Trip statistics (since power-up) --------------------------------------------
static float    tripDistanceM = 0;
static uint16_t tripTopSpeedCmS = 0;
static uint16_t tripMotorSec = 0;
static uint16_t tripMinBatteryMv = 0;        // 0 = no reading yet

// ---- State written by the radio callback (WiFi task), read by loop() ----------
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
static int16_t  targetLeft = 0;
static int16_t  targetRight = 0;
static uint32_t lastCommandMs = 0;
static bool     commandSeen = false;
static int8_t   lastRssi = 0;
static uint32_t rxCount = 0;
static uint8_t  rxPerSec = 0;        // control packets received in the last second (loop() only)
static uint8_t  rxCommand = CMD_NONE;
static uint8_t  rxCommandSeq = 0;
static uint8_t  rxParamId = PARAM_NONE;
static int16_t  rxParamValue = 0;
static uint8_t  cmdParamId = PARAM_NONE;     // loop()'s copy of the two above
static int16_t  cmdParamValue = 0;
static uint8_t  strangerMac[6];
static bool     strangerSeen = false;

static void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (memcmp(info->src_addr, CONTROLLER_MAC, 6) != 0) {
    portENTER_CRITICAL(&stateMux);
    memcpy(strangerMac, info->src_addr, 6);
    strangerSeen = true;
    portEXIT_CRITICAL(&stateMux);
    return;
  }
  if (len != sizeof(ControlPacket)) return;
  ControlPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != MAGIC_CONTROL || pkt.version != PROTO_VERSION) return;

  portENTER_CRITICAL(&stateMux);
  targetLeft  = constrain(pkt.left, -CMD_MAX, CMD_MAX);
  targetRight = constrain(pkt.right, -CMD_MAX, CMD_MAX);
  lastCommandMs = millis();
  commandSeen = true;
  lastRssi = info->rx_ctrl->rssi;
  rxCount++;
  rxCommand = pkt.command;
  rxCommandSeq = pkt.commandSeq;
  rxParamId = pkt.paramId;
  rxParamValue = pkt.paramValue;
  portEXIT_CRITICAL(&stateMux);
}

// ---- Motors --------------------------------------------------------------------

// cmd: -CMD_MAX (full astern) .. +CMD_MAX (full ahead), 0 = coast.
// Uses drive/brake ("slow decay") PWM: one input held high, the other pulsed
// low. This gives a much more linear speed response than drive/coast.
static void driveMotor(uint8_t pinIn1, uint8_t pinIn2, int cmd, bool invert) {
  if (invert) cmd = -cmd;
  if (cmd == 0) {
    ledcWrite(pinIn1, 0);     // both low: outputs off, driver sleeps
    ledcWrite(pinIn2, 0);
    return;
  }
  uint32_t duty = (uint32_t)abs(cmd) * PWM_MAX / CMD_MAX;
  if (cmd > 0) {
    ledcWrite(pinIn1, PWM_MAX);
    ledcWrite(pinIn2, PWM_MAX - duty);
  } else {
    ledcWrite(pinIn1, PWM_MAX - duty);
    ledcWrite(pinIn2, PWM_MAX);
  }
}

static void stopMotors() {
  driveMotor(PIN_LEFT_IN1, PIN_LEFT_IN2, 0, false);
  driveMotor(PIN_RIGHT_IN1, PIN_RIGHT_IN2, 0, false);
}

// Move `current` toward `target` by at most `step`.
static int approach(int current, int target, int step) {
  if (target > current) return min(current + step, target);
  if (target < current) return max(current - step, target);
  return current;
}

// ---- Battery ---------------------------------------------------------------------

static float batteryMv = 0;

// One unsmoothed reading, in millivolts at the battery.
static float batteryReadMv() {
  const float scale = (BATT_R_TOP_OHMS + BATT_R_BOTTOM_OHMS) / BATT_R_BOTTOM_OHMS * BATT_CAL;
  return analogReadMilliVolts(PIN_BATTERY) * scale;
}

static void sampleBattery() {
  float mv = batteryReadMv();
  batteryMv = (batteryMv == 0) ? mv : batteryMv + 0.05f * (mv - batteryMv);   // smooth out motor noise
}

// ---- GPS -------------------------------------------------------------------------
// Minimal NMEA reader: position, speed and course from RMC, satellite count
// from GGA. `gps` is the place a future autopilot should read from.

struct GpsState {
  int32_t  latE7;           // degrees x 10^7, north positive
  int32_t  lonE7;           // degrees x 10^7, east positive
  uint16_t speedCmS;        // speed over ground
  uint16_t courseDeci;      // course over ground, 0.1 degree (holds last value when stopped)
  uint8_t  sats;
  bool     everHeard;       // any valid sentence received since power-up
  bool     everFixed;
  uint32_t lastSentenceMs;
  uint32_t lastFixMs;
};
static GpsState gps = {};

static bool gpsPresent() { return gps.everHeard && millis() - gps.lastSentenceMs < 3000; }
static bool gpsHasFix()  { return gps.everFixed && millis() - gps.lastFixMs < 3000; }

// NMEA gives ddmm.mmmm (latitude) or dddmm.mmmm (longitude).
static int32_t nmeaToE7(const char *value, const char *hemisphere) {
  double raw = atof(value);
  int degrees = (int)(raw / 100);
  double result = degrees + (raw - degrees * 100.0) / 60.0;
  if (*hemisphere == 'S' || *hemisphere == 'W') result = -result;
  return (int32_t)lround(result * 1e7);
}

static void gpsHandleSentence(char *s) {
  if (s[0] != '$') return;
  char *star = strchr(s, '*');
  if (!star) return;
  uint8_t sum = 0;
  for (char *p = s + 1; p < star; p++) sum ^= (uint8_t)*p;
  if ((uint8_t)strtol(star + 1, nullptr, 16) != sum) {
    gpsChecksumErrors++;
    return;
  }
  gpsSentences++;
  *star = 0;

  // Split on commas in place. Empty fields are kept (strtok would skip them).
  char *field[20];
  int n = 0;
  field[n++] = s;
  for (char *p = s; *p && n < 20; p++) {
    if (*p == ',') {
      *p = 0;
      field[n++] = p + 1;
    }
  }
  if (strlen(field[0]) != 6) return;
  const char *type = field[0] + 3;   // skip "$GP", "$GN", ...

  gps.everHeard = true;
  gps.lastSentenceMs = millis();

  if (strcmp(type, "GGA") == 0 && n > 7) {
    gps.sats = (uint8_t)atoi(field[7]);
  } else if (strcmp(type, "RMC") == 0 && n > 8) {
    if (field[2][0] == 'A' && field[3][0] && field[5][0]) {   // 'A' = valid fix
      gps.latE7 = nmeaToE7(field[3], field[4]);
      gps.lonE7 = nmeaToE7(field[5], field[6]);
      gps.speedCmS = (uint16_t)constrain(atof(field[7]) * 51.4444, 0.0, 65535.0);  // knots -> cm/s
      if (field[8][0]) gps.courseDeci = (uint16_t)(atof(field[8]) * 10.0);
      gps.everFixed = true;
      gps.lastFixMs = millis();
    }
  }
}

static void gpsPoll() {
  static char line[100];
  static uint8_t len = 0;
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    gpsBytes++;
    if (c == '$') len = 0;
    if (c == '\r' || c == '\n') {
      if (len > 6) {
        line[len] = 0;
        gpsHandleSentence(line);
      }
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    } else {
      len = 0;   // over-long line: discard
    }
  }
}

// ---- Firmware update (OTA) mode ------------------------------------------------------
// The long-range link cannot talk to ordinary WiFi devices, so updating is a
// separate mode: a 2 s press of the BOOT button restarts into it.

static const uint32_t OTA_REQUEST_MAGIC = 0x07A0B0A7;
RTC_NOINIT_ATTR static uint32_t otaRequest;   // survives a software restart

static void checkOtaButton() {
  static uint32_t pressedSinceMs = 0;
  if (digitalRead(PIN_OTA_BUTTON) == HIGH) {
    pressedSinceMs = 0;
  } else if (pressedSinceMs == 0) {
    pressedSinceMs = millis() | 1;
  } else if (millis() - pressedSinceMs > 2000) {
    stopMotors();
    otaRequest = OTA_REQUEST_MAGIC;
    ESP.restart();
  }
}

// Never returns. Leave by uploading firmware, pressing BOOT, or resetting.
static void runOtaMode() {
  Serial.printf("OTA mode: join WiFi \"%s\" and upload to 192.168.4.1\n", OTA_AP_NAME);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(OTA_AP_NAME, OTA_AP_PASSWORD);
  ArduinoOTA.setHostname(OTA_AP_NAME);
  ArduinoOTA.begin();
  bool released = false;   // ignore the press that got us here
  for (;;) {
    ArduinoOTA.handle();
    digitalWrite(PIN_STATUS_LED, (millis() / 1000) % 2);   // slow blink
    bool down = digitalRead(PIN_OTA_BUTTON) == LOW;
    if (!down) released = true;
    if (down && released) ESP.restart();
    delay(5);
  }
}

// ---- Compass ---------------------------------------------------------------------
// Supports the HMC5883L and the QMC5883L (often sold as an "HMC5883L").
// There is no tilt compensation, so the heading is only good with the boat level.

static const uint8_t HMC5883L_ADDR = 0x1E;
static const uint8_t QMC5883L_ADDR = 0x0D;
enum : uint8_t { COMPASS_NONE, COMPASS_HMC5883L, COMPASS_QMC5883L };

static uint8_t  compassChip = COMPASS_NONE;
static bool     compassCalibrated = false;
static float    compassOffX = 0, compassOffY = 0;       // hard-iron offsets, raw counts
static float    compassScaleX = 1, compassScaleY = 1;   // evens out the two axes
static float    compassX = 0, compassY = 0;             // corrected and smoothed field
static float    headingDeg = 0;                         // true heading, 0..360
static bool     compassEverRead = false;
static uint32_t compassReadMs = 0;

static bool     calibrating = false;
static uint32_t calStartMs = 0;
static int32_t  calMinX, calMaxX, calMinY, calMaxY;

static bool compassOk() {
  return compassChip != COMPASS_NONE && compassEverRead && millis() - compassReadMs < 1000;
}

static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static void compassBegin() {
  Wire.begin(PIN_COMPASS_SDA, PIN_COMPASS_SCL);
  Wire.setTimeOut(20);   // a stuck bus must not stall motor control for long

  if (i2cProbe(HMC5883L_ADDR) &&
      i2cWrite(HMC5883L_ADDR, 0x00, 0x70) &&    // 8-sample average, 15 Hz
      i2cWrite(HMC5883L_ADDR, 0x01, 0x20) &&    // +/-1.3 gauss
      i2cWrite(HMC5883L_ADDR, 0x02, 0x00)) {    // continuous measurement
    compassChip = COMPASS_HMC5883L;
  } else if (i2cProbe(QMC5883L_ADDR) &&
             i2cWrite(QMC5883L_ADDR, 0x0B, 0x01) &&   // set/reset period
             i2cWrite(QMC5883L_ADDR, 0x09, 0x1D)) {   // 512x oversample, 8 gauss, 200 Hz, continuous
    compassChip = COMPASS_QMC5883L;
  }
  Serial.printf("Compass: %s\n", compassChip == COMPASS_HMC5883L   ? "HMC5883L"
                                 : compassChip == COMPASS_QMC5883L ? "QMC5883L"
                                                                   : "not found");

  Preferences prefs;
  prefs.begin("compass", false);
  if (prefs.isKey("ox")) {
    compassOffX = prefs.getFloat("ox", 0);
    compassOffY = prefs.getFloat("oy", 0);
    compassScaleX = prefs.getFloat("sx", 1);
    compassScaleY = prefs.getFloat("sy", 1);
    compassCalibrated = true;
  }
  prefs.end();
  if (!compassCalibrated) Serial.println("Compass: NOT CALIBRATED - headings will be poor");
}

static bool compassReadRaw(int16_t &x, int16_t &y) {
  bool hmc = (compassChip == COMPASS_HMC5883L);
  uint8_t addr = hmc ? HMC5883L_ADDR : QMC5883L_ADDR;
  Wire.beginTransmission(addr);
  Wire.write(hmc ? 0x03 : 0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)6) != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  if (hmc) {                                  // order X, Z, Y; high byte first
    x = (int16_t)((b[0] << 8) | b[1]);
    y = (int16_t)((b[4] << 8) | b[5]);
    if (x == -4096 || y == -4096) return false;   // sensor overflow marker
  } else {                                    // order X, Y, Z; low byte first
    x = (int16_t)((b[1] << 8) | b[0]);
    y = (int16_t)((b[3] << 8) | b[2]);
  }
  return true;
}

static void compassStartCalibration() {
  calibrating = true;
  calStartMs = millis();
  calMinX = calMinY = INT32_MAX;
  calMaxX = calMaxY = INT32_MIN;
  Serial.println("Compass calibration: turn the boat slowly through a full circle, level");
}

// Returns true when calibration has just ended (successfully or not).
static bool compassFinishCalibrationIfDue() {
  if (!calibrating || millis() - calStartMs < COMPASS_CAL_MS) return false;
  calibrating = false;

  float halfX = (calMaxX - calMinX) / 2.0f;
  float halfY = (calMaxY - calMinY) / 2.0f;
  // A full turn sweeps each axis through hundreds of counts. Much less means
  // the boat was not turned (or the sensor is not reading): keep the old values.
  if (calMinX > calMaxX || halfX < 50 || halfY < 50) {
    Serial.println("Compass calibration FAILED: boat was not turned through a full circle");
    return true;
  }
  compassOffX = (calMaxX + calMinX) / 2.0f;
  compassOffY = (calMaxY + calMinY) / 2.0f;
  float mean = (halfX + halfY) / 2.0f;
  compassScaleX = mean / halfX;
  compassScaleY = mean / halfY;
  compassCalibrated = true;

  Preferences prefs;
  prefs.begin("compass", false);
  prefs.putFloat("ox", compassOffX);
  prefs.putFloat("oy", compassOffY);
  prefs.putFloat("sx", compassScaleX);
  prefs.putFloat("sy", compassScaleY);
  prefs.end();
  Serial.printf("Compass calibrated: offset %.0f, %.0f  scale %.3f, %.3f (saved)\n", compassOffX,
                compassOffY, compassScaleX, compassScaleY);
  return true;
}

static void compassPoll() {
  if (compassChip == COMPASS_NONE) return;
  int16_t rawX, rawY;
  if (!compassReadRaw(rawX, rawY)) return;
  compassRawX = rawX;
  compassRawY = rawY;

  if (calibrating) {
    calMinX = min<int32_t>(calMinX, rawX);
    calMaxX = max<int32_t>(calMaxX, rawX);
    calMinY = min<int32_t>(calMinY, rawY);
    calMaxY = max<int32_t>(calMaxY, rawY);
  }

  float x = (rawX - compassOffX) * compassScaleX;
  float y = (rawY - compassOffY) * compassScaleY;
  if (!compassEverRead) {
    compassX = x;
    compassY = y;
  } else {
    compassX += 0.3f * (x - compassX);   // light smoothing
    compassY += 0.3f * (y - compassY);
  }
  compassEverRead = true;
  compassReadMs = millis();

  float h = atan2f(compassY, compassX) * 57.29578f;
  if (COMPASS_REVERSED) h = -h;
  headingDeg = navWrap360(h + COMPASS_OFFSET_DEG + MAGNETIC_DECLINATION_DEG);
}

// ---- Settings ---------------------------------------------------------------------------
// Changed from the controller's Setup page and kept in flash.

static int8_t   setTrim = DEFAULT_TRIM;
static uint16_t setCruise = NAV_CRUISE_POWER;
static uint16_t setLowBatteryMv = DEFAULT_LOW_BATTERY_MV;
static uint16_t setGeofenceM = DEFAULT_GEOFENCE_M;

static void settingsLoad() {
  Preferences prefs;
  prefs.begin("settings", false);
  setTrim = prefs.getChar("trim", DEFAULT_TRIM);
  setCruise = prefs.getUShort("cruise", NAV_CRUISE_POWER);
  setLowBatteryMv = prefs.getUShort("lowbatt", DEFAULT_LOW_BATTERY_MV);
  setGeofenceM = prefs.getUShort("fence", DEFAULT_GEOFENCE_M);
  prefs.end();
}

// Out-of-range values are ignored, so a corrupted packet cannot do harm.
static void settingsApply(uint8_t id, int16_t value) {
  Preferences prefs;
  prefs.begin("settings", false);
  switch (id) {
    case PARAM_TRIM:
      if (value >= -30 && value <= 30) prefs.putChar("trim", setTrim = (int8_t)value);
      break;
    case PARAM_CRUISE:
      if (value >= 200 && value <= CMD_MAX) prefs.putUShort("cruise", setCruise = value);
      break;
    case PARAM_LOW_BATTERY_MV:
      if (value >= 0 && value <= 20000) prefs.putUShort("lowbatt", setLowBatteryMv = value);
      break;
    case PARAM_GEOFENCE_M:
      if (value >= 0 && value <= 5000) prefs.putUShort("fence", setGeofenceM = value);
      break;
  }
  prefs.end();
  Serial.printf("Settings: trim %d  cruise %u  low battery %u mV  geofence %u m\n", setTrim,
                setCruise, setLowBatteryMv, setGeofenceM);
}

// ---- Autopilot ------------------------------------------------------------------------

static const uint8_t WAYPOINT_COUNT = sizeof(WAYPOINTS) / sizeof(Waypoint);

static uint8_t mode = MODE_MANUAL;
static bool    homeSet = false;
static int32_t homeLatE7 = 0, homeLonE7 = 0;
static bool    routeOk = false;        // waypoints exist and are all near home
static uint8_t waypointIndex = 0;      // next waypoint to reach, in MODE_ROUTE
static bool    atHome = false;
static bool    navHold = false;        // autopilot wants to move but cannot navigate
static bool    overrideArmed = false;  // joystick seen at rest since autopilot took over
static int     navLeft = 0, navRight = 0;
static float   navDistanceM = 0;
static uint8_t homeReason = HOME_NONE;      // why we are in MODE_HOME
static float   homeDistanceM = 0;           // how far the boat is from home
static float   bearingFromHomeDeg = 0;      // which way the boat lies, seen from home

static int32_t waypointLatE7(uint8_t i) { return (int32_t)lround(WAYPOINTS[i].lat * 1e7); }
static int32_t waypointLonE7(uint8_t i) { return (int32_t)lround(WAYPOINTS[i].lon * 1e7); }

static void setHome() {
  homeLatE7 = gps.latE7;
  homeLonE7 = gps.lonE7;
  homeSet = true;
  atHome = false;

  // A mistyped waypoint could send the boat anywhere, so the route is only
  // accepted if every waypoint is within NAV_MAX_RANGE_M of home.
  routeOk = WAYPOINT_COUNT > 0;
  for (uint8_t i = 0; i < WAYPOINT_COUNT; i++) {
    float dist, bearing;
    navDistanceBearing(homeLatE7, homeLonE7, waypointLatE7(i), waypointLonE7(i), dist, bearing);
    if (dist > NAV_MAX_RANGE_M) {
      routeOk = false;
      Serial.printf("Route REFUSED: waypoint %u is %.0f m from home (limit %.0f m)\n", i + 1, dist,
                    (float)NAV_MAX_RANGE_M);
    }
  }
  Serial.printf("Home set: %.7f, %.7f  route %s (%u waypoints)\n", homeLatE7 / 1e7, homeLonE7 / 1e7,
                routeOk ? "ok" : "unavailable", WAYPOINT_COUNT);
}

static void enterMode(uint8_t newMode) {
  mode = newMode;
  waypointIndex = 0;
  atHome = false;
  navHold = false;
  overrideArmed = false;
  navLeft = navRight = 0;
  homeReason = HOME_NONE;
}

static void goHome(uint8_t reason) {
  enterMode(MODE_HOME);
  homeReason = reason;
}

static void selfTestStart();
static void selfTestAbort(const char *why);

static void handleCommand(uint8_t command, bool stickAtRest) {
  if (mode == MODE_SELFTEST) selfTestAbort("new command from controller");
  switch (command) {
    case CMD_SELF_TEST:
      // Only from rest in manual mode: the motors are about to run by themselves.
      if (mode == MODE_MANUAL && stickAtRest) {
        testFromSerial = false;
        selfTestStart();
      }
      break;
    case CMD_MODE_MANUAL:
      if (mode != MODE_CALIBRATING) enterMode(MODE_MANUAL);
      break;
    case CMD_MODE_ROUTE:
      if (mode != MODE_CALIBRATING && homeSet && routeOk && gpsHasFix() && compassOk()) {
        enterMode(MODE_ROUTE);
      }
      break;
    case CMD_MODE_HOME:
      if (mode != MODE_CALIBRATING && homeSet) goHome(HOME_COMMANDED);
      break;
    case CMD_SET_PARAM:
      settingsApply(cmdParamId, cmdParamValue);
      break;
    case CMD_SET_HOME:
      if (gpsHasFix()) setHome();
      break;
    case CMD_CALIBRATE_COMPASS:
      if (mode == MODE_MANUAL && compassChip != COMPASS_NONE) {
        enterMode(MODE_CALIBRATING);
        compassStartCalibration();
      }
      break;
  }
}

// Works out navLeft / navRight for the current mode. Called 20 times a second.
static void navUpdate() {
  navLeft = navRight = 0;
  navHold = false;
  bool canNavigate = homeSet && gpsHasFix();
  float distance = 0, bearing = 0;
  if (canNavigate) {
    navDistanceBearing(homeLatE7, homeLonE7, gps.latE7, gps.lonE7, homeDistanceM, bearingFromHomeDeg);
  }

  if (mode != MODE_ROUTE && mode != MODE_HOME) {
    // Not under autopilot: just keep the distance-to-home readout current.
    if (canNavigate) navDistanceBearing(gps.latE7, gps.lonE7, homeLatE7, homeLonE7, distance, bearing);
    navDistanceM = distance;
    return;
  }

  if (!canNavigate || !compassOk()) {
    navHold = true;       // motors stay off until GPS and compass are back
    return;
  }

  if (mode == MODE_ROUTE) {
    navDistanceBearing(gps.latE7, gps.lonE7, waypointLatE7(waypointIndex),
                       waypointLonE7(waypointIndex), distance, bearing);
    if (distance < NAV_ARRIVE_RADIUS_M) {
      waypointIndex++;
      if (waypointIndex >= WAYPOINT_COUNT) {                    // route finished: go home
        mode = MODE_HOME;
        homeReason = HOME_ROUTE_DONE;
      }
      return;             // steer for the new target on the next pass
    }
  } else {
    navDistanceBearing(gps.latE7, gps.lonE7, homeLatE7, homeLonE7, distance, bearing);
    // Stop on arrival; set off again only if the boat drifts well away.
    if (atHome && distance > 2 * NAV_ARRIVE_RADIUS_M) atHome = false;
    if (!atHome && distance < NAV_ARRIVE_RADIUS_M) atHome = true;
  }
  navDistanceM = distance;
  if (mode == MODE_HOME && atHome) return;

  // Ease off close to the target so the boat does not overshoot and circle.
  int cruise = setCruise;
  if (distance < NAV_SLOW_RADIUS_M) {
    cruise = max(cruise * NAV_SLOW_MIN_PERCENT / 100, (int)(cruise * distance / NAV_SLOW_RADIUS_M));
  }
  navSteer(navWrap180(bearing - headingDeg), cruise, NAV_PIVOT_POWER, NAV_PIVOT_DEG, CMD_MAX,
           navLeft, navRight);
}

// ---- Self-test ------------------------------------------------------------------------
// Started from the controller. Checks the sensors, then runs each motor ahead
// and astern in turn. The boat cannot sense whether a motor really turns, so
// the motor steps are for the operator to watch; the battery voltage drop
// during each step is reported as a hint (no drop at all suggests no current).

static uint8_t  testStep = TEST_IDLE;
static uint8_t  testResults = 0;
static uint16_t testSagMv[4] = {0, 0, 0, 0};
static uint32_t testStepStartMs = 0, testSampleMs = 0;
static float    testBaselineMv = 0, testSumMv = 0;
static uint16_t testSamples = 0;
static int      testLeft = 0, testRight = 0;
static const char *const TEST_STEP_NAMES[] = {"idle", "left ahead", "left astern", "right ahead",
                                              "right astern", "done", "aborted"};

static void selfTestStart() {
  enterMode(MODE_SELFTEST);
  testResults = TEST_RAN |
                (batteryMv >= TEST_BATTERY_MIN_MV ? TEST_BATTERY_OK : 0) |
                (gpsPresent() ? TEST_GPS_DATA : 0) |
                (gpsHasFix() ? TEST_GPS_FIX : 0) |
                (compassOk() ? TEST_COMPASS_OK : 0) |
                (compassCalibrated ? TEST_COMPASS_CAL : 0) |
                (rxPerSec >= CONTROL_RATE_HZ * 8 / 10 ? TEST_RADIO_OK : 0);
  memset(testSagMv, 0, sizeof(testSagMv));
  testBaselineMv = batteryMv;
  testStep = TEST_LEFT_AHEAD;
  testStepStartMs = millis();
  testSumMv = 0;
  testSamples = 0;

  Serial.println("\n===== SELF-TEST =====");
  Serial.printf("Battery : %s  %.2f V (minimum %.2f V)\n",
                (testResults & TEST_BATTERY_OK) ? "ok  " : "FAIL", batteryMv / 1000.0f,
                TEST_BATTERY_MIN_MV / 1000.0f);
  Serial.printf("GPS     : %s  %s, %u satellites\n", (testResults & TEST_GPS_DATA) ? "ok  " : "FAIL",
                !(testResults & TEST_GPS_DATA) ? "no data from module"
                : (testResults & TEST_GPS_FIX) ? "position fix" : "data but no fix yet",
                gps.sats);
  Serial.printf("Compass : %s  %s", (testResults & TEST_COMPASS_OK) ? "ok  " : "FAIL",
                !(testResults & TEST_COMPASS_OK) ? "not responding"
                : (testResults & TEST_COMPASS_CAL) ? "calibrated" : "NOT calibrated");
  if (testResults & TEST_COMPASS_OK) Serial.printf(", heading %.0f", headingDeg);
  Serial.println();
  Serial.printf("Radio   : %s  %u of %u packets/s, %d dBm\n",
                (testResults & TEST_RADIO_OK) ? "ok  " : "FAIL", rxPerSec, CONTROL_RATE_HZ, lastRssi);
  Serial.println("Motors  : watch each one turn the right way");
}

static void selfTestAbort(const char *why) {
  Serial.printf("Self-test ABORTED: %s\n=====================\n", why);
  testStep = TEST_ABORTED;
  testLeft = testRight = 0;
  enterMode(MODE_MANUAL);
}

// Call every pass of loop() while in MODE_SELFTEST. Sets testLeft / testRight.
static void selfTestUpdate() {
  uint32_t now = millis();
  uint32_t elapsed = now - testStepStartMs;
  testLeft = testRight = 0;

  if (elapsed < TEST_STEP_MS) {
    bool astern = (testStep == TEST_LEFT_ASTERN || testStep == TEST_RIGHT_ASTERN);
    int power = astern ? -TEST_POWER : TEST_POWER;
    if (testStep == TEST_LEFT_AHEAD || testStep == TEST_LEFT_ASTERN) testLeft = power;
    else testRight = power;
    // Measure the battery once the motor has had time to get going.
    if (elapsed > TEST_STEP_MS / 3 && now - testSampleMs >= 20) {
      testSampleMs = now;
      testSumMv += batteryReadMv();
      testSamples++;
    }
    return;
  }
  if (elapsed < TEST_STEP_MS + 500) return;   // pause with motors off between steps

  float sag = testSamples ? testBaselineMv - testSumMv / testSamples : 0;
  testSagMv[testStep - TEST_LEFT_AHEAD] = (uint16_t)constrain(sag, 0.0f, 65535.0f);
  Serial.printf("  %-12s ran, battery dropped %u mV\n", TEST_STEP_NAMES[testStep],
                testSagMv[testStep - TEST_LEFT_AHEAD]);

  if (testStep == TEST_RIGHT_ASTERN) {
    testStep = TEST_DONE;
    Serial.println("Self-test complete\n=====================");
    enterMode(MODE_MANUAL);
  } else {
    testStep++;
    testStepStartMs = now;
    testSumMv = 0;
    testSamples = 0;
  }
}

// ---- Setup / loop ------------------------------------------------------------------

void setup() {
  // Motors first, so the driver inputs are held low as early as possible.
  ledcAttach(PIN_LEFT_IN1, MOTOR_PWM_HZ, PWM_BITS);
  ledcAttach(PIN_LEFT_IN2, MOTOR_PWM_HZ, PWM_BITS);
  ledcAttach(PIN_RIGHT_IN1, MOTOR_PWM_HZ, PWM_BITS);
  ledcAttach(PIN_RIGHT_IN2, MOTOR_PWM_HZ, PWM_BITS);
  stopMotors();

  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_OTA_BUTTON, INPUT_PULLUP);
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);

  Serial.begin(115200);
  Serial.println("\n\nBoat receiver");

  bool otaRequested = (otaRequest == OTA_REQUEST_MAGIC) && (esp_reset_reason() == ESP_RST_SW);
  otaRequest = 0;
  if (otaRequested) runOtaMode();

  Serial2.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  compassBegin();
  settingsLoad();

  radioOk = linkBegin(CONTROLLER_MAC);
  Serial.printf("Boat MAC: %s  (put this in the controller's config.h)\n",
                WiFi.macAddress().c_str());
  if (!radioOk) return;   // motors stay stopped; loop() just flashes the LED

  esp_now_register_recv_cb(onReceive);
}

void loop() {
  static uint32_t motorTickMs = 0, batteryTickMs = 0, telemetryTickMs = 0, secondTickMs = 0;
  static uint32_t navTickMs = 0;
  static uint32_t rxCountAtLastSecond = 0;
  static int      outLeft = 0, outRight = 0;
  static bool     commandSeqKnown = false;
  static uint8_t  lastCommandSeq = 0;

  uint32_t now = millis();

  checkOtaButton();
  gpsPoll();
  consolePoll();

  if (!radioOk) {
    digitalWrite(PIN_STATUS_LED, (now / 100) % 2);   // fast flash: radio fault
    return;
  }

  // Snapshot what the radio callback last gave us.
  int16_t wantLeft, wantRight;
  uint32_t commandAge, rxTotal;
  int8_t rssi;
  bool seen, stranger;
  uint8_t strangerCopy[6], command, commandSeq;
  portENTER_CRITICAL(&stateMux);
  wantLeft = targetLeft;
  wantRight = targetRight;
  commandAge = millis() - lastCommandMs;
  seen = commandSeen;
  rssi = lastRssi;
  rxTotal = rxCount;
  command = rxCommand;
  commandSeq = rxCommandSeq;
  cmdParamId = rxParamId;
  cmdParamValue = rxParamValue;
  stranger = strangerSeen;
  memcpy(strangerCopy, strangerMac, 6);
  portEXIT_CRITICAL(&stateMux);

  bool linkLost = !seen || commandAge > FAILSAFE_MS;
  linkLostNow = linkLost;

  // The starting point is the first good fix after power-up.
  if (!homeSet && gpsHasFix() && gps.sats >= HOME_MIN_SATS) setHome();

  // One-shot commands from the controller. The first packet only tells us the
  // controller's current sequence number: a stale command is never replayed.
  if (seen) {
    if (!commandSeqKnown) {
      commandSeqKnown = true;
      lastCommandSeq = commandSeq;
    } else if (commandSeq != lastCommandSeq) {
      lastCommandSeq = commandSeq;
      handleCommand(command, wantLeft == 0 && wantRight == 0);
    }
  }

  // Joystick override: moving the stick takes back manual control. The stick
  // must first be seen at rest, so a stick held over while the link dropped
  // cannot cancel the return home the moment the link comes back.
  if (!linkLost && (mode == MODE_ROUTE || mode == MODE_HOME)) {
    int stick = max(abs(wantLeft), abs(wantRight));
    if (stick == 0) {
      overrideArmed = true;
    } else if (overrideArmed && stick > NAV_STICK_OVERRIDE) {
      enterMode(MODE_MANUAL);
    }
  }

  // Self-test: stop it the moment the operator intervenes or the link drops.
  if (mode == MODE_SELFTEST) {
    // (A test started from the PC runs with no controller, so has no link to lose.)
    if (linkLost && !testFromSerial) {
      selfTestAbort("signal lost");
    } else if (!linkLost && max(abs(wantLeft), abs(wantRight)) > NAV_STICK_OVERRIDE) {
      selfTestAbort("joystick moved");
    } else {
      selfTestUpdate();
    }
  }

  // Signal lost for a while: return to the starting point. This latches; the
  // boat keeps coming home after the link returns, until the stick is moved.
  if (seen && commandAge > RTH_DELAY_MS && homeSet && (mode == MODE_MANUAL || mode == MODE_ROUTE)) {
    Serial.println("Signal lost: returning home");
    goHome(HOME_SIGNAL_LOST);
  }

  // Battery low for a while: come home while there is still charge to do it.
  // Trips once; if the operator takes back control it stays off until the
  // battery has clearly recovered (or the boat is restarted).
  static uint32_t lowBatterySinceMs = 0;
  static bool lowBatteryTripped = false;
  bool batteryLow = setLowBatteryMv > 0 && batteryMv > 1000 && batteryMv < setLowBatteryMv;
  if (!batteryLow) {
    lowBatterySinceMs = 0;
    if (batteryMv > setLowBatteryMv + 200) lowBatteryTripped = false;
  } else if (!lowBatterySinceMs) {
    lowBatterySinceMs = now | 1;
  } else if (!lowBatteryTripped && now - lowBatterySinceMs > LOW_BATTERY_HOLD_MS && homeSet &&
             (mode == MODE_MANUAL || mode == MODE_ROUTE)) {
    Serial.println("Battery low: returning home");
    lowBatteryTripped = true;
    goHome(HOME_LOW_BATTERY);
  }

  // Geofence: driven too far from home, so come back. Trips once per excursion;
  // if the operator takes back control it re-arms when the boat is back inside.
  static bool fenceArmed = true;
  if (setGeofenceM > 0 && homeSet && gpsHasFix()) {
    if (homeDistanceM < setGeofenceM * 0.9f) {
      fenceArmed = true;
    } else if (fenceArmed && homeDistanceM > setGeofenceM && mode == MODE_MANUAL && !linkLost) {
      Serial.println("Geofence crossed: returning home");
      fenceArmed = false;
      goHome(HOME_GEOFENCE);
    }
  }

  // Compass and autopilot, 20 times a second.
  if (now - navTickMs >= 50) {
    navTickMs = now;
    compassPoll();
    if (compassFinishCalibrationIfDue() && mode == MODE_CALIBRATING) enterMode(MODE_MANUAL);
    navUpdate();
  }

  // Motors: ramp toward whatever the current mode asks for.
  if (now - motorTickMs >= MOTOR_TICK_MS) {
    motorTickMs = now;
    // A bench motor run from the PC ends on time, or as soon as a controller appears.
    if (benchActive && (now - benchStartMs >= benchDurationMs || mode != MODE_MANUAL || !linkLost)) {
      benchActive = false;
    }
    int goalLeft = 0, goalRight = 0;
    if (mode == MODE_MANUAL) {
      if (!linkLost) {
        goalLeft = wantLeft;
        goalRight = wantRight;
      } else if (benchActive) {
        goalLeft = benchLeft;
        goalRight = benchRight;
      }
    } else if (mode == MODE_ROUTE || mode == MODE_HOME) {
      goalLeft = navLeft;
      goalRight = navRight;
    } else if (mode == MODE_SELFTEST) {
      goalLeft = testLeft;
      goalRight = testRight;
    }
    if (mode == MODE_MANUAL && linkLost && !benchActive) {
      outLeft = outRight = 0;                      // manual and no signal: stop at once
    } else {
      int step = max(1, (int)(CMD_MAX * MOTOR_TICK_MS / MOTOR_RAMP_MS));
      outLeft = approach(outLeft, goalLeft, step);
      outRight = approach(outRight, goalRight, step);
    }
    // Trim slows one motor so the boat tracks straight.
    int trimLeft = 100 - max(0, -(int)setTrim), trimRight = 100 - max(0, (int)setTrim);
    driveMotor(PIN_LEFT_IN1, PIN_LEFT_IN2, outLeft * trimLeft / 100, MOTOR_INVERT_LEFT);
    driveMotor(PIN_RIGHT_IN1, PIN_RIGHT_IN2, outRight * trimRight / 100, MOTOR_INVERT_RIGHT);
  }

  if (now - batteryTickMs >= 50) {
    batteryTickMs = now;
    sampleBattery();
  }

  if (now - secondTickMs >= 1000) {
    secondTickMs = now;
    rxPerSec = (uint8_t)min<uint32_t>(255, rxTotal - rxCountAtLastSecond);
    rxCountAtLastSecond = rxTotal;

    // Trip statistics. GPS speed jitters by a few tenths of a m/s at rest, so
    // only count it when the boat is clearly moving.
    if (gpsHasFix() && gps.speedCmS > 30) {
      tripDistanceM += gps.speedCmS / 100.0f;
      if (gps.sats >= 5) tripTopSpeedCmS = max(tripTopSpeedCmS, gps.speedCmS);
    }
    if ((outLeft != 0 || outRight != 0) && tripMotorSec < 65535) tripMotorSec++;
    if (batteryMv > 1000 && (tripMinBatteryMv == 0 || batteryMv < tripMinBatteryMv)) {
      tripMinBatteryMv = (uint16_t)batteryMv;
    }
  }

  // Once-a-second status line (kept quiet while the PC console is in use).
  static uint32_t statusTickMs = 0;
  bool statusDue = now - statusTickMs >= 1000;
  if (statusDue) statusTickMs = now;
  if (statusDue && !consoleQuiet()) {
    static const char *const MODE_NAMES[] = {"manual", "route", "home", "calibrating", "self-test"};
    Serial.printf("%s %-11s L%+5d R%+5d  rx %u/s  rssi %d dBm  batt %.2f V  ",
                  linkLost ? "NO SIGNAL" : "linked   ", MODE_NAMES[mode], outLeft, outRight,
                  rxPerSec, rssi, batteryMv / 1000.0f);
    if (compassOk()) Serial.printf("hdg %.0f  ", headingDeg); else Serial.print("hdg --  ");
    if (mode == MODE_ROUTE) Serial.printf("wp %u/%u  ", waypointIndex + 1, WAYPOINT_COUNT);
    if (homeSet) Serial.printf("%.0f m to go  ", navDistanceM);
    if (gpsHasFix()) {
      Serial.printf("gps %.7f, %.7f  %.1f m/s  %.1f deg  %u sats\n", gps.latE7 / 1e7,
                    gps.lonE7 / 1e7, gps.speedCmS / 100.0, gps.courseDeci / 10.0, gps.sats);
    } else {
      Serial.println(gpsPresent() ? "gps: no fix yet" : "gps: no data from module");
    }
    if (stranger) {
      Serial.printf("Ignoring packets from %02X:%02X:%02X:%02X:%02X:%02X "
                    "(not CONTROLLER_MAC in config.h)\n",
                    strangerCopy[0], strangerCopy[1], strangerCopy[2], strangerCopy[3],
                    strangerCopy[4], strangerCopy[5]);
      portENTER_CRITICAL(&stateMux);
      strangerSeen = false;
      portEXIT_CRITICAL(&stateMux);
    }
  }

  // Telemetry only while we can hear the controller; no point shouting otherwise.
  if (!linkLost && now - telemetryTickMs >= 1000 / TELEMETRY_RATE_HZ) {
    telemetryTickMs = now;
    TelemetryPacket pkt;
    pkt.magic = MAGIC_TELEMETRY;
    pkt.version = PROTO_VERSION;
    pkt.batteryMv = (uint16_t)constrain(batteryMv, 0.0f, 65535.0f);
    pkt.rssi = rssi;
    pkt.rxPerSec = rxPerSec;
    pkt.flags = (routeOk ? TELEM_FLAG_ROUTE_OK : 0) |
                (gpsPresent() ? TELEM_FLAG_GPS_PRESENT : 0) |
                (gpsHasFix() ? TELEM_FLAG_GPS_FIX : 0) |
                (homeSet ? TELEM_FLAG_HOME_SET : 0) |
                (compassOk() ? TELEM_FLAG_COMPASS_OK : 0) |
                (compassCalibrated ? TELEM_FLAG_COMPASS_CAL : 0) |
                (navHold ? TELEM_FLAG_NAV_HOLD : 0) |
                ((mode == MODE_HOME && atHome) ? TELEM_FLAG_AT_HOME : 0);
    pkt.latE7 = gps.latE7;
    pkt.lonE7 = gps.lonE7;
    pkt.speedCmS = gps.speedCmS;
    pkt.courseDeci = gps.courseDeci;
    pkt.sats = gps.sats;
    pkt.mode = mode;
    pkt.waypoint = (mode == MODE_ROUTE) ? waypointIndex + 1 : 0;
    pkt.waypointCount = WAYPOINT_COUNT;
    pkt.distanceM = (uint16_t)constrain(navDistanceM, 0.0f, 65535.0f);
    pkt.headingDeci = compassOk() ? (uint16_t)(headingDeg * 10.0f) : 0xFFFF;
    pkt.testStep = testStep;
    pkt.testResults = testResults;
    memcpy(pkt.testSagMv, testSagMv, sizeof(pkt.testSagMv));
    pkt.homeReason = (mode == MODE_HOME) ? homeReason : HOME_NONE;
    pkt.homeDistanceM = (uint16_t)constrain(homeDistanceM, 0.0f, 65535.0f);
    pkt.bearingFromHomeDeg = (uint16_t)bearingFromHomeDeg;
    pkt.trim = setTrim;
    pkt.cruise = setCruise;
    pkt.lowBatteryMv = setLowBatteryMv;
    pkt.geofenceM = setGeofenceM;
    pkt.tripDistanceM = (uint16_t)constrain(tripDistanceM, 0.0f, 65535.0f);
    pkt.tripTopSpeedCmS = tripTopSpeedCmS;
    pkt.tripMotorSec = tripMotorSec;
    pkt.tripMinBatteryMv = tripMinBatteryMv;
    esp_now_send(CONTROLLER_MAC, (const uint8_t *)&pkt, sizeof(pkt));
    telemetrySent++;
  }

  // LED: solid = linked, slow blink = no signal, quick double-rate blink = autopilot.
  bool autopilot = (mode == MODE_ROUTE || mode == MODE_HOME);
  digitalWrite(PIN_STATUS_LED, autopilot ? (now / 150) % 2 : linkLost ? (now / 500) % 2 : HIGH);
}
