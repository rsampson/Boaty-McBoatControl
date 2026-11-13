// BoatController - hand controller for a twin-motor, differentially steered boat.
//
// Reads a two-pot joystick, mixes throttle and steering into left/right motor
// commands and sends them to the boat over ESP-NOW Long Range.
//
// An SSD1306 display has seven pages: link health and batteries, boat GPS,
// navigation, setup, trip statistics, self-test results, and system information. A piezo beeper warns when the link is
// failing or lost, or the boat battery is low.
//
// Buttons: NEXT steps through the pages. SELECT steps back, except on the Nav
// page, where a short press moves along the menu and a 1 second hold sends the
// highlighted choice to the boat (manual / run route / return home / set home /
// calibrate compass / run self-test).
//
// RECALL: hold BOTH buttons for one second, on any page, to order the boat home
// whatever it is doing. Hold both again to withdraw the order.
//
// On the Setup page a short press of SELECT moves down the list and a 1 second
// hold starts editing. While editing, NEXT is plus and SELECT is minus; hold
// SELECT to save, or hold NEXT to cancel.
//
// Holding the BOOT button for 2 seconds restarts into firmware-update (OTA)
// mode; see config.h.
//
// Board: classic ESP32 (ESP32 Dev Module), Arduino-ESP32 core 3.3 or later.
// Libraries: Adafruit SSD1306, Adafruit GFX.

#include <Wire.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config.h"
#include "protocol.h"

static const uint32_t LINK_TIMEOUT_MS = 1000;   // no telemetry for this long = link lost

// While the boat is not acknowledging, send at this slow rate instead of
// CONTROL_RATE_HZ. Every unacknowledged packet is retried several times by the
// radio, and at long-range speeds that keeps the transmitter on almost
// continuously at full power: enough to make the board very hot.
static const uint32_t SEARCH_RATE_HZ = 5;
static const uint32_t SEARCH_AFTER_MS = 1000;   // no acknowledgement for this long = searching

static Adafruit_SSD1306 display(128, 64, &Wire, -1);
static bool displayOk = false;
static bool radioOk = false;

// Joystick centre readings, measured at power-up.
static int centreThrottle = 2048;
static int centreSteer = 2048;
static bool centreSuspect = false;

// ---- State shared between the radio callbacks, the control task and loop() ----
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
static TelemetryPacket telemetry = {};          // last telemetry from the boat
static int8_t   telemetryRssi = 0;              // how strongly we hear the boat
static uint32_t telemetryAtMs = 0;
static bool     telemetrySeen = false;

// Running totals and raw readings for console.ino (the PC serial console).
static volatile uint32_t txTotal = 0, ackTotal = 0, telemetryTotal = 0;
static volatile int rawThrottle = 0, rawSteer = 0;

static volatile uint32_t sendAttempts = 0;      // counted by the send callback
static volatile uint32_t sendAcked = 0;
static volatile uint8_t  ackPercent = 0;        // over the last second
static volatile uint32_t lastAckMs = 0;         // when the boat last acknowledged a packet
static volatile bool     everAcked = false;
static volatile int16_t  cmdLeft = 0;           // what we are currently sending
static volatile int16_t  cmdRight = 0;

// ---- Types (declared before any function so Arduino's prototypes can use them) ----

enum Page : uint8_t { PAGE_LINK, PAGE_GPS, PAGE_NAV, PAGE_SETUP, PAGE_TRIP, PAGE_TEST, PAGE_INFO, PAGE_COUNT };
static uint8_t page = PAGE_LINK;

// One consistent copy of everything the display and beeper need.
struct Status {
  TelemetryPacket t;      // last telemetry (may be stale: check `linked`)
  int8_t  rssiHere;
  bool    seen;           // telemetry received at least once since power-up
  bool    linked;         // telemetry is current
  uint8_t ackPct;
};

enum Press : uint8_t { PRESS_NONE, PRESS_SHORT, PRESS_LONG };

struct Button {
  uint8_t  pin;
  bool     down;          // debounced state
  bool     longFired;     // this press has already been reported as a long one
  uint32_t changedMs;
};
static Button buttonNext = {PIN_BUTTON_NEXT, false, false, 0};
static Button buttonSelect = {PIN_BUTTON_SELECT, false, false, 0};

// One-shot command for the boat, repeated in every control packet. The boat
// acts each time the sequence number changes.
static volatile uint8_t txCommand = CMD_NONE;
static volatile uint8_t txCommandSeq = 0;
// Recall is a standing order, not a command: it is sent in every packet while
// it is on, so a single packet getting through is enough, and it is exempt from
// the "never queue a command" rule (it can only ever bring the boat home).
static volatile bool recallActive = false;

static volatile uint8_t txParamId = PARAM_NONE;     // used with CMD_SET_PARAM
static volatile int16_t txParamValue = 0;
// A command is dropped if the boat has not had the chance to hear it within this
// time, so nothing sent near the edge of range can take effect minutes later.
static const uint32_t COMMAND_LIFETIME_MS = 2000;
static uint32_t txCommandSetMs = 0;

// Controller-side setting (kept in this board's flash) and its own battery.
static volatile uint8_t speedLimitPct = 100;        // caps joystick power
static volatile uint16_t controllerBatteryMv = 0;   // 0 = not measured

// One line of the Setup page.
struct SetupItem {
  const char *name;
  uint8_t param;          // PARAM_* held by the boat, or PARAM_NONE for a controller setting
  int16_t minValue, maxValue, step;
};

// The one definition of "the boat is being heard". Call with stateMux held.
static bool telemetryFresh(uint32_t now) {
  return telemetrySeen && now - telemetryAtMs < LINK_TIMEOUT_MS;
}

// ---- Radio callbacks (run in the WiFi task: keep them short) ----------------

static void onSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  uint32_t now = millis();
  portENTER_CRITICAL(&stateMux);    // controlTask reads and clears these counters
  sendAttempts = sendAttempts + 1;
  txTotal = txTotal + 1;
  if (status == ESP_NOW_SEND_SUCCESS) {
    sendAcked = sendAcked + 1;
    ackTotal = ackTotal + 1;
    lastAckMs = now;
    everAcked = true;
  }
  portEXIT_CRITICAL(&stateMux);
}

static void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(TelemetryPacket)) return;
  if (memcmp(info->src_addr, BOAT_MAC, 6) != 0) return;
  TelemetryPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != MAGIC_TELEMETRY || pkt.version != PROTO_VERSION) return;

  portENTER_CRITICAL(&stateMux);
  telemetry = pkt;
  telemetryRssi = info->rx_ctrl->rssi;
  telemetryAtMs = millis();
  telemetrySeen = true;
  portEXIT_CRITICAL(&stateMux);
  telemetryTotal = telemetryTotal + 1;
}

// ---- Joystick ----------------------------------------------------------------

static int readRaw(int pin) {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(pin);
  return sum / 8;
}

static int measureCentre(int pin) {
  long sum = 0;
  for (int i = 0; i < 64; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  int centre = sum / 64;
  // A centre far from mid-travel means the stick was being held (or a wire is
  // off). Fall back to mid-scale rather than drive off with a bad zero.
  int lo = JOY_RAW_MIN + (JOY_RAW_MAX - JOY_RAW_MIN) * 30 / 100;
  int hi = JOY_RAW_MIN + (JOY_RAW_MAX - JOY_RAW_MIN) * 70 / 100;
  if (centre < lo || centre > hi) {
    centreSuspect = true;
    return (JOY_RAW_MIN + JOY_RAW_MAX) / 2;
  }
  return centre;
}

// Returns stick position as -1..+1 with deadband and expo applied.
static float readAxis(int pin, int centre, bool invert) {
  int raw = readRaw(pin);
  if (pin == PIN_JOY_THROTTLE) rawThrottle = raw; else rawSteer = raw;
  float v = (raw >= centre)
              ? (float)(raw - centre) / (float)(JOY_RAW_MAX - centre)
              : (float)(raw - centre) / (float)(centre - JOY_RAW_MIN);
  v = constrain(v, -1.0f, 1.0f);

  float mag = fabsf(v);
  if (mag < JOY_DEADBAND) return 0.0f;
  mag = (mag - JOY_DEADBAND) / (1.0f - JOY_DEADBAND);          // no step at the deadband edge
  mag = JOY_EXPO * mag * mag * mag + (1.0f - JOY_EXPO) * mag;  // finer control near centre
  if (v < 0) mag = -mag;
  return invert ? -mag : mag;
}

// ---- Control task: read stick, mix, send. Runs at a steady CONTROL_RATE_HZ ----
// It has its own task so that slow display updates cannot delay control packets.

static void controlTask(void *) {
  const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_RATE_HZ);
  TickType_t wake = xTaskGetTickCount();
  uint16_t seq = 0;
  uint32_t windowStartMs = millis();

  for (;;) {
    float throttle = readAxis(PIN_JOY_THROTTLE, centreThrottle, JOY_INVERT_THROTTLE);
    float steer    = readAxis(PIN_JOY_STEER, centreSteer, JOY_INVERT_STEER) * STEER_GAIN;

    // Differential mix. With no throttle the motors counter-rotate and the boat
    // pivots. If either side would exceed full power, scale both so the turn
    // ratio is kept instead of being clipped away.
    float left  = throttle + steer;
    float right = throttle - steer;
    float peak = max(fabsf(left), fabsf(right));
    if (peak > 1.0f) {
      left /= peak;
      right /= peak;
    }

    ControlPacket pkt;
    pkt.magic   = MAGIC_CONTROL;
    pkt.version = PROTO_VERSION;
    pkt.seq     = seq++;
    // The stick was not at rest when its centre was measured (held over, or a
    // wire is off), so its readings cannot be trusted: command nothing at all
    // until the controller is restarted with the stick centred.
    if (centreSuspect) left = right = 0.0f;

    float limit = speedLimitPct / 100.0f;
    pkt.left    = (int16_t)lroundf(left * limit * CMD_MAX);
    pkt.right   = (int16_t)lroundf(right * limit * CMD_MAX);
    pkt.command    = txCommand;
    pkt.commandSeq = txCommandSeq;
    pkt.paramId    = txParamId;
    pkt.paramValue = txParamValue;
    // Tell the boat how well we hear it, so it can set its own transmit power.
    uint32_t now = millis();
    portENTER_CRITICAL(&stateMux);
    pkt.rssi = telemetryFresh(now) ? telemetryRssi : 0;
    pkt.flags = recallActive ? CTRL_FLAG_RECALL : 0;
    portEXIT_CRITICAL(&stateMux);
    cmdLeft  = pkt.left;
    cmdRight = pkt.right;
    // Full rate while the boat is answering; otherwise only every Nth cycle.
    // The first acknowledgement brings the full rate straight back.
    bool searching = !everAcked || now - lastAckMs > SEARCH_AFTER_MS;
    if (!searching || pkt.seq % (CONTROL_RATE_HZ / SEARCH_RATE_HZ) == 0) {
      esp_now_send(BOAT_MAC, (const uint8_t *)&pkt, sizeof(pkt));
    }

    // Once a second, work out what fraction of packets the boat acknowledged.
    if (millis() - windowStartMs >= 1000) {
      windowStartMs = millis();
#if PIN_CONTROLLER_BATTERY >= 0
      // This task owns the ADC, so the controller's own battery is read here too.
      float mv = analogReadMilliVolts(PIN_CONTROLLER_BATTERY) *
                 ((CTRL_BATT_R_TOP_OHMS + CTRL_BATT_R_BOTTOM_OHMS) / CTRL_BATT_R_BOTTOM_OHMS) * CTRL_BATT_CAL;
      controllerBatteryMv = controllerBatteryMv ? (uint16_t)(controllerBatteryMv + 0.3f * (mv - controllerBatteryMv))
                                                : (uint16_t)mv;
#endif
      portENTER_CRITICAL(&stateMux);
      uint32_t attempts = sendAttempts, acked = sendAcked;
      sendAttempts = 0;
      sendAcked = 0;
      portEXIT_CRITICAL(&stateMux);
      ackPercent = attempts ? (uint8_t)min<uint32_t>(100, acked * 100 / attempts) : 0;
    }

    vTaskDelayUntil(&wake, period);
  }
}

// ---- Display -----------------------------------------------------------------

static bool recallConfirmed = false;   // the boat's telemetry shows it has the recall order

static Status readStatus() {
  Status st;
  uint32_t now = millis();
  portENTER_CRITICAL(&stateMux);
  st.t = telemetry;
  st.rssiHere = telemetryRssi;
  st.seen = telemetrySeen;
  st.linked = telemetryFresh(now);
  portEXIT_CRITICAL(&stateMux);
  st.ackPct = ackPercent;
  return st;
}

static void drawHeader(const char *title, bool blink) {
  // A recall takes over the headline of every page. It blinks until the boat
  // confirms it has the order (set by loop() in recallConfirmed).
  if (recallActive) {
    title = "RECALL";
    blink = !recallConfirmed;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  if (!blink || (millis() / 400) % 2) display.print(title);
  display.setTextSize(1);
  display.setCursor(110, 0);
  display.printf("%d/%d", page + 1, PAGE_COUNT);
}

static void drawLinkPage(const Status &st) {
  const TelemetryPacket &t = st.t;
  bool blinkOn = (millis() / 400) % 2;

  drawHeader(st.linked ? "LINK OK" : "NO LINK", !st.linked);

  // Packets getting through: acknowledged by the boat / counted by the boat.
  int rxPct = st.linked ? min(100, t.rxPerSec * 100 / CONTROL_RATE_HZ) : 0;
  display.setCursor(0, 18);
  display.printf("Ack %3d%%   Rx %3d%%", st.ackPct, rxPct);

  // Quality bar from the acknowledge rate.
  display.drawRect(0, 28, 128, 6, SSD1306_WHITE);
  display.fillRect(0, 28, 128 * st.ackPct / 100, 6, SSD1306_WHITE);

  // Signal strength at each end (closer to 0 is stronger).
  display.setCursor(0, 37);
  if (st.linked) {
    display.printf("dBm boat%4d here%4d", t.rssi, st.rssiHere);
  } else {
    display.print("dBm boat --- here ---");
  }

  // Batteries: the boat's (last known value, marked "?", if the link drops)
  // and this controller's. A low one blinks "LOW!".
  display.setCursor(0, 47);
  if (!st.seen) {
    display.print("Boat -.--V");
  } else if (st.linked && BATT_WARN_MV > 0 && t.batteryMv < BATT_WARN_MV && blinkOn) {
    display.print("Boat LOW! ");
  } else {
    display.printf("Boat %u.%02uV%s", t.batteryMv / 1000, (t.batteryMv % 1000) / 10, st.linked ? "" : "?");
  }
  uint16_t myMv = controllerBatteryMv;
  if (myMv) {
    display.setCursor(72, 47);
    if (CTRL_BATT_WARN_MV > 0 && myMv < CTRL_BATT_WARN_MV && blinkOn) {
      display.print("Me LOW!");
    } else {
      display.printf("Me %u.%02uV", myMv / 1000, (myMv % 1000) / 10);
    }
  }

  // What we are commanding.
  display.setCursor(0, 57);
  if (centreSuspect && blinkOn) {
    display.print("STICK FAULT: restart");
  } else {
    display.printf("L%+5d  R%+5d", cmdLeft, cmdRight);
  }
}

static void printDegrees(int32_t e7) {
  uint32_t mag = (e7 < 0) ? (uint32_t)(-(int64_t)e7) : (uint32_t)e7;
  display.printf("%c%lu.%07lu", e7 < 0 ? '-' : '+', (unsigned long)(mag / 10000000UL),
                 (unsigned long)(mag % 10000000UL));
}

static void drawGpsPage(const Status &st) {
  const TelemetryPacket &t = st.t;
  bool fix = st.linked && (t.flags & TELEM_FLAG_GPS_FIX);
  bool present = st.linked && (t.flags & TELEM_FLAG_GPS_PRESENT);

  drawHeader(!st.linked ? "NO LINK" : fix ? "GPS FIX" : present ? "NO FIX" : "NO GPS", !fix);

  // A position is worth showing even when stale: it is where the boat was last
  // heard from. It is marked "last known" until it is current again.
  bool havePosition = st.seen && (t.latE7 != 0 || t.lonE7 != 0);

  display.setCursor(0, 18);
  if (st.linked) {
    display.printf("Sats %u", t.sats);
  } else {
    display.print("Sats --");
  }

  // Compass at the right: a ring with a north mark, and a needle pointing the
  // way the bow points (straight up = north). The figure is beside it.
  bool haveHeading = st.seen && t.headingDeci != 0xFFFF;
  display.setCursor(54, 18);
  if (haveHeading) display.printf("Hdg %u", t.headingDeci / 10); else display.print("Hdg ---");
  const int cx = 114, cy = 34, radius = 11;
  display.drawCircle(cx, cy, radius, SSD1306_WHITE);
  display.drawLine(cx, cy - radius, cx, cy - radius - 2, SSD1306_WHITE);   // north mark
  if (haveHeading) {
    float angle = t.headingDeci / 10.0f * DEG_TO_RAD;
    display.drawLine(cx, cy, cx + (int)lroundf((radius - 2) * sinf(angle)),
                     cy - (int)lroundf((radius - 2) * cosf(angle)), SSD1306_WHITE);
  }

  display.setCursor(0, 28);
  display.print("Lat ");
  if (havePosition) printDegrees(t.latE7); else display.print("---");
  display.setCursor(0, 38);
  display.print("Lon ");
  if (havePosition) printDegrees(t.lonE7); else display.print("---");

  display.setCursor(0, 48);
  if (fix) {
    display.printf("%u.%u m/s  %u deg", t.speedCmS / 100, (t.speedCmS % 100) / 10,
                   t.courseDeci / 10);
  } else if (havePosition) {
    display.print("(last known position)");
  }

  // Find my boat: where it is (or was last heard) relative to the home point.
  display.setCursor(0, 57);
  if (st.seen && (t.flags & TELEM_FLAG_HOME_SET)) {
    display.printf("From home %um @ %u", t.homeDistanceM, t.bearingFromHomeDeg);
  }
}

// ---- Nav page: boat mode, progress, readiness, and the command menu ----

static const char *const NAV_MENU[] = {"Manual control", "Run route", "Return home",
                                       "Set home here", "Calibrate compass", "Run self-test"};
static const uint8_t NAV_MENU_COMMAND[] = {CMD_MODE_MANUAL, CMD_MODE_ROUTE, CMD_MODE_HOME,
                                           CMD_SET_HOME, CMD_CALIBRATE_COMPASS, CMD_SELF_TEST};
static const uint8_t NAV_MENU_COUNT = sizeof(NAV_MENU_COMMAND);
static uint8_t  navMenuIndex = 0;
static uint32_t navMarkMs = 0;      // when a menu choice was last sent or refused (0 = never)
static char     navMark = '*';      // '*' sent, 'x' refused for want of a link

// The only way a command leaves the controller. Refuses (returns false) when
// the boat is not linked: a command is never queued for a boat that cannot
// hear it. loop() expires it after COMMAND_LIFETIME_MS.
static bool queueCommand(bool boatLinked, uint8_t command, uint8_t paramId = PARAM_NONE,
                         int16_t paramValue = 0) {
  if (!boatLinked) return false;
  txParamId = paramId;
  txParamValue = paramValue;
  txCommand = command;              // command first, then the sequence number that announces it
  txCommandSeq = txCommandSeq + 1;
  txCommandSetMs = millis();
  return true;
}

// Returns whether the command was sent; either way the menu marker shows it.
static bool sendNavCommand(bool boatLinked, uint8_t command) {
  bool sent = queueCommand(boatLinked, command);
  navMark = sent ? '*' : 'x';
  navMarkMs = millis() | 1;
  return sent;
}

static void drawNavPage(const Status &st) {
  const TelemetryPacket &t = st.t;
  bool blinkOn = (millis() / 400) % 2;

  const char *title = "NO LINK";
  if (st.linked) {
    switch (t.mode) {
      case MODE_ROUTE:       title = "ROUTE"; break;
      case MODE_HOME:        title = (t.flags & TELEM_FLAG_AT_HOME) ? "AT HOME" : "HOMING"; break;
      case MODE_CALIBRATING: title = "TURN 360"; break;
      case MODE_SELFTEST:    title = "TESTING"; break;
      default:               title = "MANUAL"; break;
    }
  }
  drawHeader(title, !st.linked);

  // Progress: where the boat is heading and how far it has to go.
  display.setCursor(0, 18);
  if (!st.linked) {
    display.print("Boat not heard");
  } else if ((t.flags & TELEM_FLAG_NAV_HOLD) && blinkOn) {
    // The autopilot wants to move but cannot: say what it is waiting for.
    display.print(!(t.flags & TELEM_FLAG_HOME_SET) ? "NO HOME POINT YET"
                  : !(t.flags & TELEM_FLAG_GPS_FIX) ? "WAITING FOR GPS FIX"
                                                    : "WAITING FOR COMPASS");
  } else {
    if (t.mode == MODE_ROUTE) {
      display.printf("WP%u/%u %um ", t.waypoint, t.waypointCount, t.distanceM);
    } else if (t.flags & TELEM_FLAG_HOME_SET) {
      display.printf("Home %um ", t.distanceM);
    } else {
      display.print("No home yet ");
    }
    if (t.headingDeci == 0xFFFF) {
      display.print("Hdg ---");
    } else {
      display.printf("Hdg %u", t.headingDeci / 10);
    }
  }

  // Readiness: + ready, - not ready, ? compass present but not calibrated.
  // While the boat is coming home by its own decision, say why instead.
  static const char *const HOME_REASONS[] = {"", "", "signal was lost", "route finished",
                                             "boat battery low", "geofence crossed", "recall"};
  display.setCursor(0, 27);
  if (st.linked && t.mode == MODE_HOME && t.homeReason >= HOME_SIGNAL_LOST &&
      t.homeReason <= HOME_RECALL) {
    display.printf("Why: %s%s", HOME_REASONS[t.homeReason],
                   (t.flags2 & TELEM2_GPS_STEERING) ? ", by GPS" : "");
  } else if (st.linked) {
    char compass = !(t.flags & TELEM_FLAG_COMPASS_OK) ? '-'
                   : (t.flags & TELEM_FLAG_COMPASS_CAL) ? '+' : '?';
    display.printf("GPS%c Cmp%c Home%c Rte%c", (t.flags & TELEM_FLAG_GPS_FIX) ? '+' : '-', compass,
                   (t.flags & TELEM_FLAG_HOME_SET) ? '+' : '-',
                   (t.flags & TELEM_FLAG_ROUTE_OK) ? '+' : '-');
  }

  // Menu: three lines, scrolled to keep the highlighted entry in view. The
  // marker changes to '*' briefly after a choice has been sent, or to 'x' if
  // it was refused because the boat is not linked.
  bool justMarked = navMarkMs && millis() - navMarkMs < 1500;
  int first = constrain((int)navMenuIndex - 1, 0, NAV_MENU_COUNT - 3);
  for (int row = 0; row < 3; row++) {
    int item = first + row;
    display.setCursor(0, 37 + row * 9);
    display.print(item != navMenuIndex ? ' ' : justMarked ? navMark : '>');
    display.print(' ');
    display.print(NAV_MENU[item]);
  }
}

// ---- Setup page: settings changed with the buttons ----

static const SetupItem SETUP_ITEMS[] = {
  {"Speed limit %", PARAM_NONE,           20,  100, 10},   // controller: caps joystick power
  {"Trim L/R %",    PARAM_TRIM,          -30,   30,  1},   // + slows the right motor
  {"Cruise %",      PARAM_CRUISE,         20,  100,  5},   // autopilot power
  {"LowBatt home",  PARAM_LOW_BATTERY_MV,  0, 13000, 100}, // volts; off below 9.00
  {"Geofence m",    PARAM_GEOFENCE_M,      0, 1000, 20},   // 0 = off
};
static const uint8_t SETUP_COUNT = sizeof(SETUP_ITEMS) / sizeof(SETUP_ITEMS[0]);
static const int16_t LOW_BATTERY_LOWEST_MV = 9000;
static uint8_t setupIndex = 0;
static bool    setupEditing = false;
static int16_t setupValue = 0;      // the value being edited

// The setting's present value, in the units shown on the page.
static int16_t setupCurrent(uint8_t index, const Status &st) {
  switch (SETUP_ITEMS[index].param) {
    case PARAM_TRIM:           return st.t.trim;
    case PARAM_CRUISE:         return st.t.cruise / 10;
    case PARAM_LOW_BATTERY_MV: return st.t.lowBatteryMv;
    case PARAM_GEOFENCE_M:     return st.t.geofenceM;
    default:                   return speedLimitPct;
  }
}

static void setupPrintValue(uint8_t index, int16_t value) {
  char text[8];
  switch (SETUP_ITEMS[index].param) {
    case PARAM_TRIM:
      snprintf(text, sizeof(text), "%+d", value);
      break;
    case PARAM_LOW_BATTERY_MV:
      if (value == 0) snprintf(text, sizeof(text), "off");
      else snprintf(text, sizeof(text), "%d.%02dV", value / 1000, (value % 1000) / 10);
      break;
    case PARAM_GEOFENCE_M:
      if (value == 0) snprintf(text, sizeof(text), "off");
      else snprintf(text, sizeof(text), "%d", value);
      break;
    default:
      snprintf(text, sizeof(text), "%d", value);
      break;
  }
  display.printf("%6s", text);
}

// Boat settings can only be edited while the boat is there to receive them.
static void setupBeginEdit(const Status &st) {
  if (SETUP_ITEMS[setupIndex].param != PARAM_NONE && !st.linked) return;
  setupValue = setupCurrent(setupIndex, st);
  setupEditing = true;
}

static void setupStep(int direction) {
  const SetupItem &item = SETUP_ITEMS[setupIndex];
  int value = setupValue + direction * item.step;
  if (item.param == PARAM_LOW_BATTERY_MV && value < LOW_BATTERY_LOWEST_MV) {
    value = (direction > 0) ? LOW_BATTERY_LOWEST_MV : 0;   // step between "off" and the lowest voltage
  }
  setupValue = (int16_t)constrain(value, (int)item.minValue, (int)item.maxValue);
}

static void setupSave(bool boatLinked) {
  const SetupItem &item = SETUP_ITEMS[setupIndex];
  if (item.param == PARAM_NONE) {
    speedLimitPct = (uint8_t)setupValue;
    Preferences prefs;
    prefs.begin("controller", false);
    prefs.putUChar("speed", speedLimitPct);
    prefs.end();
  } else {
    // The page shows the boat's own value from telemetry, so a successful save
    // is confirmed when the new value appears there. If the link dropped
    // mid-edit the change is discarded.
    queueCommand(boatLinked, CMD_SET_PARAM, item.param,
                 (item.param == PARAM_CRUISE) ? setupValue * 10 : setupValue);
  }
  setupEditing = false;
}

static void drawSetupPage(const Status &st) {
  drawHeader(setupEditing ? "EDIT" : "SETUP", false);
  bool blinkOn = (millis() / 300) % 2;
  for (uint8_t i = 0; i < SETUP_COUNT; i++) {
    bool selected = (i == setupIndex);
    display.setCursor(0, 18 + i * 9);
    display.print(!selected ? ' ' : !setupEditing ? '>' : blinkOn ? '#' : ' ');
    display.printf("%-13s", SETUP_ITEMS[i].name);
    if (selected && setupEditing) {
      setupPrintValue(i, setupValue);
    } else if (SETUP_ITEMS[i].param != PARAM_NONE && !st.seen) {
      display.print("    --");            // boat setting, boat not heard yet
    } else {
      setupPrintValue(i, setupCurrent(i, st));
    }
  }
}

// ---- Trip page: statistics since the boat was switched on ----

static void drawTripPage(const Status &st) {
  const TelemetryPacket &t = st.t;
  drawHeader("TRIP", false);
  if (!st.seen) {
    display.setCursor(0, 22);
    display.print("Boat not heard yet.");
    return;
  }
  display.setCursor(0, 18);
  display.printf("Distance   %5u m", t.tripDistanceM);
  display.setCursor(0, 28);
  display.printf("Top speed  %2u.%u m/s", t.tripTopSpeedCmS / 100, (t.tripTopSpeedCmS % 100) / 10);
  display.setCursor(0, 38);
  display.printf("Motor time %3u:%02u", t.tripMotorSec / 60, t.tripMotorSec % 60);
  display.setCursor(0, 48);
  if (t.tripMinBatteryMv) {
    display.printf("Lowest batt %u.%02uV", t.tripMinBatteryMv / 1000, (t.tripMinBatteryMv % 1000) / 10);
  }
  if (!st.linked) {
    display.setCursor(0, 57);
    display.print("(last known values)");
  }
}

// ---- Test page: self-test progress and results ----

static void drawTestPage(const Status &st) {
  const TelemetryPacket &t = st.t;
  static const char *const STEP_TITLES[] = {"TEST", "L AHEAD", "L ASTERN", "R AHEAD",
                                            "R ASTERN", "DONE", "ABORTED"};
  bool running = st.linked && t.testStep >= TEST_LEFT_AHEAD && t.testStep <= TEST_RIGHT_ASTERN;
  drawHeader(!st.linked ? "NO LINK" : STEP_TITLES[min<uint8_t>(t.testStep, TEST_ABORTED)],
             !st.linked || running);

  if (!st.seen || !(t.testResults & TEST_RAN)) {
    display.setCursor(0, 22);
    display.print("No test run yet.");
    display.setCursor(0, 36);
    display.print("Nav page menu:");
    display.setCursor(0, 46);
    display.print("  Run self-test");
    return;
  }

  // Results stay on screen after the test, and after the link drops.
  uint8_t r = t.testResults;
  display.setCursor(0, 18);
  display.printf("Battery %s %u.%02uV", (r & TEST_BATTERY_OK) ? "ok  " : "FAIL", t.batteryMv / 1000,
                 (t.batteryMv % 1000) / 10);
  display.setCursor(0, 27);
  display.printf("GPS     %s %s", (r & TEST_GPS_DATA) ? "ok  " : "FAIL",
                 !(r & TEST_GPS_DATA) ? "no data" : (r & TEST_GPS_FIX) ? "fix" : "no fix");
  display.setCursor(0, 36);
  display.printf("Compass %s %s", (r & TEST_COMPASS_OK) ? "ok  " : "FAIL",
                 !(r & TEST_COMPASS_OK) ? "absent" : (r & TEST_COMPASS_CAL) ? "cal" : "no cal");
  display.setCursor(0, 45);
  display.printf("Radio   %s %ddBm", (r & TEST_RADIO_OK) ? "ok  " : "FAIL", t.rssi);
  // Battery drop in mV for: left ahead, left astern, right ahead, right astern.
  display.setCursor(0, 55);
  display.printf("mV %u %u %u %u", t.testSagMv[0], t.testSagMv[1], t.testSagMv[2], t.testSagMv[3]);
}

static void drawInfoPage(const Status &st) {
  drawHeader("INFO", false);
  display.setCursor(0, 18);
  display.print("Me   ");
  display.print(WiFi.macAddress().substring(6));    // last 3 bytes fit the line
  display.setCursor(0, 28);
  display.printf("Boat %02X:%02X:%02X:%02X", BOAT_MAC[2], BOAT_MAC[3], BOAT_MAC[4], BOAT_MAC[5]);
  // Chip temperature (uncalibrated sensor) and transmit power at each end.
  display.setCursor(0, 38);
  display.printf("Me   %3dC tx %2ddBm", (int)linkTempC, linkTxPowerQdbm / 4);
  display.setCursor(0, 48);
  if (st.linked) {
    display.printf("Boat %3dC tx %2ddBm", st.t.tempC, st.t.txPowerDbm);
  } else {
    display.print("Boat  --C tx --dBm");
  }
  display.setCursor(0, 57);
  display.printf("Ch%d v%d BOOT 2s=update", RADIO_CHANNEL, PROTO_VERSION);
}

static void drawMessage(const char *line1, const char *line2, const char *line3 = "") {
  if (!displayOk) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(line1);
  display.println();
  display.println(line2);
  display.println();
  display.println(line3);
  display.display();
}

// ---- Buttons -------------------------------------------------------------------

// Call often. A short press is reported on release, a long press as soon as
// the button has been held for a second.
static Press poll(Button &b) {
  bool down = digitalRead(b.pin) == LOW;
  uint32_t now = millis();
  if (down != b.down) {
    if (now - b.changedMs < 40) return PRESS_NONE;   // debounce
    b.down = down;
    b.changedMs = now;
    if (down) {
      b.longFired = false;
      return PRESS_NONE;
    }
    return b.longFired ? PRESS_NONE : PRESS_SHORT;
  }
  if (down && !b.longFired && now - b.changedMs >= 1000) {
    b.longFired = true;
    return PRESS_LONG;
  }
  return PRESS_NONE;
}

// ---- Beeper --------------------------------------------------------------------

static void beep(uint16_t hz) {   // 0 = silent
  static uint16_t current = 0;
  if (hz == current) return;
  current = hz;
  if (hz) tone(PIN_BEEPER, hz); else noTone(PIN_BEEPER);
}

// Stays quiet until the boat has been heard once, so it does not nag while the
// boat is still switched off. Most urgent condition wins.
static void updateBeeper(const Status &st) {
  static uint32_t ackBadSinceMs = 0, battLowSinceMs = 0;
  uint32_t now = millis();
  if (!st.seen) {         // the rule above, for every condition below
    beep(0);
    return;
  }

  bool ackBad = st.linked && st.ackPct < BEEP_ACK_CRITICAL_PCT;
  if (!ackBad) ackBadSinceMs = 0; else if (!ackBadSinceMs) ackBadSinceMs = now | 1;

  // Battery voltage sags under load, so only warn once it has stayed low.
  uint16_t myMv = controllerBatteryMv;
  bool battLow = BEEP_LOW_BATTERY &&
                 ((BATT_WARN_MV > 0 && st.linked && st.t.batteryMv < BATT_WARN_MV) ||
                  (CTRL_BATT_WARN_MV > 0 && myMv > 0 && myMv < CTRL_BATT_WARN_MV));
  if (!battLow) battLowSinceMs = 0; else if (!battLowSinceMs) battLowSinceMs = now | 1;

  uint16_t hz = 0;
  if (!st.linked) {
    if (now % 300 < 100) hz = 2700;                           // link lost: fast beeps
  } else if (ackBadSinceMs && now - ackBadSinceMs > 1000) {
    uint32_t phase = now % 2000;                              // link critical: double beep
    if (phase < 80 || (phase >= 160 && phase < 240)) hz = 2700;
  } else if (battLowSinceMs && now - battLowSinceMs > 5000) {
    if (now % 5000 < 300) hz = 1500;                          // battery low: slow low beep
  }
  beep(hz);
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
    beep(0);
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
  drawMessage("FIRMWARE UPDATE MODE", "WiFi: " OTA_AP_NAME, "192.168.4.1  BOOT=exit");
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

// ---- Setup / loop --------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nBoat controller");

  pinMode(PIN_BUTTON_NEXT, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SELECT, INPUT_PULLUP);
  txCommandSeq = (uint8_t)esp_random();   // so a restarted controller's commands still look new
  pinMode(PIN_OTA_BUTTON, INPUT_PULLUP);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BEEPER, OUTPUT);
  digitalWrite(PIN_BEEPER, LOW);

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(400000);
  // The display library does not notice a missing display, so ask the bus first.
  Wire.beginTransmission(OLED_I2C_ADDR);
  displayOk = (Wire.endTransmission() == 0) && display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
  if (!displayOk) Serial.println("SSD1306 display not found - continuing without it");

  bool otaRequested = (otaRequest == OTA_REQUEST_MAGIC) && (esp_reset_reason() == ESP_RST_SW);
  otaRequest = 0;
  if (otaRequested) runOtaMode();

  Preferences prefs;
  prefs.begin("controller", false);
  speedLimitPct = constrain(prefs.getUChar("speed", 100), 20, 100);
  prefs.end();
#if PIN_CONTROLLER_BATTERY >= 0
  analogSetPinAttenuation(PIN_CONTROLLER_BATTERY, ADC_11db);
#endif

  analogSetPinAttenuation(PIN_JOY_THROTTLE, ADC_11db);
  analogSetPinAttenuation(PIN_JOY_STEER, ADC_11db);

  radioOk = linkBegin(BOAT_MAC);
  String mac = WiFi.macAddress();
  Serial.printf("Controller MAC: %s  (put this in the boat's config.h)\n", mac.c_str());

  if (!radioOk) {
    drawMessage("RADIO FAILED", "See serial output", "Hold BOOT 2s = update");
    return;   // nothing is transmitted; loop() still watches the BOOT button
  }

  drawMessage("Controller MAC:", mac.c_str());
  centreThrottle = measureCentre(PIN_JOY_THROTTLE);
  centreSteer = measureCentre(PIN_JOY_STEER);
  Serial.printf("Stick centre: throttle %d, steer %d%s\n", centreThrottle, centreSteer,
                centreSuspect ? "  (off-centre at power-up: using mid-scale)" : "");
  delay(1500);  // leave the MAC on screen long enough to read

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 5, nullptr, 1);
}

void loop() {
  static uint32_t drawnMs = 0;

  checkOtaButton();
  consolePoll();
  if (!radioOk) {
    digitalWrite(PIN_STATUS_LED, (millis() / 100) % 2);   // fast flicker: radio fault
    delay(10);
    return;
  }

  bool redraw = false;
  Status st = readStatus();
  // Heartbeat: two flashes per beat marks this as the controller.
  heartbeatLed(PIN_STATUS_LED, HEARTBEAT_FLASHES_CONTROLLER, st.linked);
  recallConfirmed = st.linked && (st.t.flags2 & TELEM2_RECALL);

  Press next = poll(buttonNext);
  Press select = poll(buttonSelect);

  // Both buttons held for a second: toggle the recall. While both are down
  // neither button's own press is acted on, now or when they are released.
  static uint32_t chordSinceMs = 0;
  static bool chordFired = false;
  if (buttonNext.down && buttonSelect.down) {
    next = select = PRESS_NONE;
    buttonNext.longFired = buttonSelect.longFired = true;
    if (!chordSinceMs) {
      chordSinceMs = millis() | 1;
    } else if (!chordFired && millis() - chordSinceMs >= 1000) {
      chordFired = true;
      recallActive = !recallActive;
      setupEditing = false;
      redraw = true;
    }
  } else {
    chordSinceMs = 0;
    chordFired = false;
  }
  if (next != PRESS_NONE || select != PRESS_NONE) redraw = true;

  if (page == PAGE_SETUP && setupEditing) {
    // Editing a setting: NEXT is plus, SELECT is minus; hold SELECT to save, hold NEXT to cancel.
    if (next == PRESS_SHORT) setupStep(+1);
    else if (select == PRESS_SHORT) setupStep(-1);
    else if (select == PRESS_LONG) setupSave(st.linked);
    else if (next == PRESS_LONG) setupEditing = false;
  } else {
    if (next == PRESS_SHORT) page = (page + 1) % PAGE_COUNT;
    if (page == PAGE_NAV) {
      if (select == PRESS_SHORT) {
        navMenuIndex = (navMenuIndex + 1) % NAV_MENU_COUNT;
      } else if (select == PRESS_LONG) {
        bool sent = sendNavCommand(st.linked, NAV_MENU_COMMAND[navMenuIndex]);
        if (sent && NAV_MENU_COMMAND[navMenuIndex] == CMD_SELF_TEST) page = PAGE_TEST;   // watch it run
      }
    } else if (page == PAGE_SETUP) {
      if (select == PRESS_SHORT) setupIndex = (setupIndex + 1) % SETUP_COUNT;
      else if (select == PRESS_LONG) setupBeginEdit(st);
    } else if (select == PRESS_SHORT) {
      page = (page + PAGE_COUNT - 1) % PAGE_COUNT;
    }
  }

  // Transmit power, once a second: only as much as the boat needs to hear us well.
  static uint32_t powerTickMs = 0;
  if (millis() - powerTickMs >= 1000) {
    powerTickMs = millis();
    linkPowerUpdate(st.linked && st.ackPct >= LINK_HEALTHY_PCT, st.t.rssi);
  }

  // Expire a command the boat has had long enough to hear. The sequence number
  // is left alone, so a boat that missed it sees only "no command".
  if (txCommand != CMD_NONE && millis() - txCommandSetMs > COMMAND_LIFETIME_MS) txCommand = CMD_NONE;

  updateBeeper(st);

  if (displayOk && !consoleDisplayBusy() && (redraw || millis() - drawnMs >= 100)) {
    drawnMs = millis();
    switch (page) {
      case PAGE_GPS:  drawGpsPage(st); break;
      case PAGE_NAV:  drawNavPage(st); break;
      case PAGE_SETUP: drawSetupPage(st); break;
      case PAGE_TRIP: drawTripPage(st); break;
      case PAGE_TEST: drawTestPage(st); break;
      case PAGE_INFO: drawInfoPage(st);  break;
      default:        drawLinkPage(st); break;
    }
    display.display();
  }
  delay(5);
}
