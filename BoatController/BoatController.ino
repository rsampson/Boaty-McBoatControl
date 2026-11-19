// BoatController - hand controller for a twin-motor, differentially steered boat.
//
// Reads a two-pot joystick, mixes throttle and steering into left/right motor
// commands and sends them to the boat over ESP-NOW Long Range.
//
// An SSD1306 display has seven pages: link health and batteries, boat GPS,
// navigation, setup, trip statistics, self-test results, and system information. A piezo beeper warns when the link is
// failing or lost, or the boat battery is low.
//
// Buttons: two levels. The top level is a list of page titles. NEXT steps down
// the list (INFO wraps round to LINK) and SELECT opens the highlighted page. On
// an opened page a short press of SELECT returns to the list.
//
// Nav page: NEXT moves along the menu and a 1 second hold of SELECT sends the
// highlighted choice to the boat (manual / run route / return home / set home /
// calibrate compass / run self-test).
//
// Setup page: NEXT moves down the list and a 1 second hold of SELECT starts
// editing. While editing, NEXT is plus and SELECT is minus; hold SELECT to save,
// or hold NEXT to cancel.
//
// RECALL: hold BOTH buttons for one second, on any screen, to order the boat home
// whatever it is doing. Hold both again to withdraw the order.
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
static uint8_t page = PAGE_LINK;      // the highlighted title, or the open page
static bool    inDetails = false;     // false: the list of titles; true: `page` is open
static uint8_t detailScroll = 0;      // where the user has scrolled to (0 whenever a page is opened)
static uint8_t detailScrollMax = 0;   // last valid first row of what was drawn; NEXT wraps past it

// The rows of an open page, in the large font: 10 characters across. A page
// builds its rows, then drawRows() shows as many as fit and scrolls the rest.
struct RowList {
  static const uint8_t COLS = 10, MAX = 12;
  char    text[MAX][COLS + 1];
  uint8_t n = 0;
  void add(const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
    if (n >= MAX) return;
    va_list args;
    va_start(args, fmt);
    int length = vsnprintf(text[n], COLS + 1, fmt, args);   // too long is cut off, never overflows
    va_end(args);
    if (length > COLS) text[n][COLS - 1] = '~';              // and marked, so a clipped value is not mistaken for a real one
    n++;
  }
  // A label at the left and a value at the right edge.
  void addPair(const char *label, const char *value) {
    if (n >= MAX) return;
    int pad = max(1, (int)COLS - (int)strlen(label) - (int)strlen(value));
    int length = snprintf(text[n], COLS + 1, "%s%*s%s", label, pad, "", value);
    if (length > COLS) text[n][COLS - 1] = '~';
    n++;
  }
  // "Top 2.5m/s", or "Top 12m/s" once the decimal will not fit.
  void addSpeed(const char *label, uint16_t cmPerSec) {
    if (cmPerSec >= 1000) add("%s %um/s", label, cmPerSec / 100);
    else add("%s %u.%um/s", label, cmPerSec / 100, (cmPerSec % 100) / 10);
  }
  // "Dist 850m", or "Dist 12km" from 10 km.
  void addDistance(const char *label, uint32_t metres) {
    if (metres >= 10000) add("%s %lukm", label, (unsigned long)(metres / 1000));
    else add("%s %lum", label, (unsigned long)metres);
  }
  // A battery sag in millivolts; three digits is all there is room for.
  void addSag(const char *label, uint16_t mv) {
    if (mv > 999) add("%s >999", label); else add("%s %u", label, mv);
  }
};

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

// Clears the screen and draws the one-line banner in the large font, if there
// is anything to say. A recall takes the banner of every page: blinking until
// the boat confirms it has the order (set by loop() in recallConfirmed). Returns
// the height used, so the page knows where its own content starts.
static uint8_t drawBanner(const char *text, bool blink) {
  if (recallActive) {
    text = "RECALL";
    blink = !recallConfirmed;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  if (!text) return 0;
  display.setTextSize(2);
  display.setCursor(2, 0);
  if (!blink || (millis() / 400) % 2) display.print(text);
  display.setTextSize(1);
  return 16;
}

// Draws the rows from y = top to the bottom of the screen, 16 pixels each, with
// a thin scroll bar at the right edge when they do not all fit. `selected` is
// the row to highlight (-1 for none); the view follows it. With none, NEXT
// scrolls: loop() moves detailScroll on, wrapping past detailScrollMax.
// `outline` draws the highlight as a frame instead of a filled bar.
static void drawRows(const RowList &list, uint8_t top, int selected = -1, bool outline = false) {
  const uint8_t visible = (64 - top) / 16;
  const uint8_t maxFirst = list.n > visible ? list.n - visible : 0;
  detailScrollMax = maxFirst;
  // Where the user scrolled to is only changed by NEXT and, on a page with a
  // selected row, by the selection moving. A banner appearing or going shrinks
  // or grows the window but leaves that position alone.
  uint8_t first = min<uint8_t>(detailScroll, maxFirst);
  if (selected >= 0) {
    if (selected < first) first = selected;
    else if (selected >= first + visible) first = selected - visible + 1;
    detailScroll = first;
  }
  const bool bar = list.n > visible;
  const int width = bar ? 124 : 128;
  display.setTextSize(2);
  for (uint8_t row = 0; row < visible && first + row < list.n; row++) {
    uint8_t i = first + row;
    int y = top + row * 16;
    bool hit = (i == selected);
    bool filled = hit && !outline;
    if (filled) display.fillRect(0, y, width, 16, SSD1306_WHITE);
    if (hit && outline) display.drawRect(0, y, width, 16, SSD1306_WHITE);
    display.setTextColor(filled ? SSD1306_BLACK : SSD1306_WHITE);
    display.setCursor(2, y + 1);
    display.print(list.text[i]);
  }
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  if (bar) {
    int h = 64 - top;
    int thumb = max(4, h * visible / list.n);
    display.drawFastVLine(127, top, h, SSD1306_WHITE);
    display.fillRect(126, top + (h - thumb) * first / maxFirst, 2, thumb, SSD1306_WHITE);
  }
}

// ---- Top level: the list of page titles ----

static const char *const PAGE_TITLES[PAGE_COUNT] = {"LINK", "GPS", "NAV", "SETUP", "TRIP", "TEST", "INFO"};

// Titles in the large font, as many as fit, scrolled so the highlighted one is
// always shown. A recall takes the top row, blinking until the boat confirms it.
static void drawTitleList() {
  display.clearDisplay();
  display.setTextSize(2);
  uint8_t rows = recallActive ? 3 : 4;
  uint8_t top = (4 - rows) * 16;
  if (recallActive) {
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    if (recallConfirmed || (millis() / 400) % 2) display.print("RECALL");
  }
  uint8_t first = constrain((int)page - (int)rows + 1, 0, PAGE_COUNT - rows);
  for (uint8_t row = 0; row < rows; row++) {
    uint8_t i = first + row;
    bool selected = (i == page);
    if (selected) display.fillRect(0, top + row * 16, 128, 16, SSD1306_WHITE);
    display.setTextColor(selected ? SSD1306_BLACK : SSD1306_WHITE);
    display.setCursor(4, top + row * 16 + 1);
    display.print(PAGE_TITLES[i]);
  }
  display.setTextColor(SSD1306_WHITE);
}

static void drawLinkPage(const Status &st) {
  const TelemetryPacket &t = st.t;
  bool blinkOn = (millis() / 400) % 2;

  uint8_t top = drawBanner(centreSuspect ? "BAD STICK" : !st.linked ? "NO LINK" : nullptr, true);
  RowList rows;
  if (centreSuspect) rows.add("Restart me");

  // Packets getting through: acknowledged by the boat / counted by the boat.
  int rxPct = st.linked ? min(100, t.rxPerSec * 100 / CONTROL_RATE_HZ) : 0;
  rows.add("Ack %3d%%", st.ackPct);
  rows.add("Rx  %3d%%", rxPct);

  // Signal strength at each end (closer to 0 is stronger).
  if (st.linked) {
    rows.add("Boat%4ddB", t.rssi);
    rows.add("Here%4ddB", st.rssiHere);
  } else {
    rows.add("Boat ---dB");
    rows.add("Here ---dB");
  }

  // Batteries: the boat's (last known value if the link drops: the banner says
  // so) and this controller's. A low one blinks "LOW!".
  if (!st.seen) {
    rows.add("Bat -.--V");
  } else if (st.linked && BATT_WARN_MV > 0 && t.batteryMv < BATT_WARN_MV && blinkOn) {
    rows.add("Bat LOW!");
  } else {
    rows.add("Bat %u.%02uV", t.batteryMv / 1000, (t.batteryMv % 1000) / 10);
  }
  uint16_t myMv = controllerBatteryMv;
  if (myMv) {
    if (CTRL_BATT_WARN_MV > 0 && myMv < CTRL_BATT_WARN_MV && blinkOn) {
      rows.add("Me LOW!");
    } else {
      rows.add("Me %u.%02uV", myMv / 1000, (myMv % 1000) / 10);
    }
  }

  // What we are commanding.
  rows.add("L %+5d", cmdLeft);
  rows.add("R %+5d", cmdRight);
  drawRows(rows, top);
}

// A coordinate to five decimal places (about a metre), with a hemisphere letter
// in place of a sign: "N51.12345", "W122.12345".
static void addCoordinate(RowList &rows, int32_t e7, char positive, char negative) {
  uint32_t mag = (e7 < 0) ? (uint32_t)(-(int64_t)e7) : (uint32_t)e7;
  rows.add("%c%lu.%05lu", e7 < 0 ? negative : positive, (unsigned long)(mag / 10000000UL),
           (unsigned long)((mag % 10000000UL) / 100));
}

static void drawGpsPage(const Status &st) {
  const TelemetryPacket &t = st.t;
  bool fix = st.linked && (t.flags & TELEM_FLAG_GPS_FIX);
  bool present = st.linked && (t.flags & TELEM_FLAG_GPS_PRESENT);

  uint8_t top = drawBanner(!st.linked ? "NO LINK" : nullptr, true);
  RowList rows;
  if (st.linked) rows.add(fix ? "GPS FIX" : present ? "NO FIX" : "NO GPS");

  if (st.linked) rows.add("Sats %u", t.sats); else rows.add("Sats --");
  bool haveHeading = st.seen && t.headingDeci != 0xFFFF;
  if (haveHeading) rows.add("Hdg %u", t.headingDeci / 10); else rows.add("Hdg ---");

  // A position is worth showing even when stale: it is where the boat was last
  // heard from. It is marked until it is current again.
  bool havePosition = st.seen && (t.latE7 != 0 || t.lonE7 != 0);
  if (havePosition) {
    if (!fix) rows.add("last known");
    addCoordinate(rows, t.latE7, 'N', 'S');
    addCoordinate(rows, t.lonE7, 'E', 'W');
  } else {
    rows.add("N ---");
    rows.add("E ---");
  }
  if (fix) {
    rows.addSpeed("Spd", t.speedCmS);
    rows.add("Crs %u", t.courseDeci / 10);
  }

  // Find my boat: where it is (or was last heard) relative to the home point.
  if (st.seen && (t.flags & TELEM_FLAG_HOME_SET)) {
    rows.addDistance("Away", t.homeDistanceM);
    rows.add("Brng %u", t.bearingFromHomeDeg);
  }
  drawRows(rows, top);
}

// ---- Nav page: boat mode, progress, readiness, and the command menu ----

static const char *const NAV_MENU[] = {"Manual", "Route", "Home", "Set home", "Calibrate", "Self-test"};
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

  // The mode and progress are the first small line; a recall takes the big
  // banner above them.
  uint8_t top = drawBanner(nullptr, false);
  display.setTextWrap(false);       // a long status line is clipped, never spilled onto the menu

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
  display.setCursor(0, top);
  if (!st.linked) {
    if (blinkOn) display.print("NO LINK");
  } else if ((t.flags & TELEM_FLAG_NAV_HOLD) && blinkOn) {
    // The autopilot wants to move but cannot: say what it is waiting for.
    display.print(!(t.flags & TELEM_FLAG_HOME_SET) ? "NO HOME POINT YET"
                  : !(t.flags & TELEM_FLAG_GPS_FIX) ? "WAITING FOR GPS FIX"
                                                    : "WAITING FOR COMPASS");
  } else {
    // Mode, how far there is to go, and the heading ("H---" if there is none).
    display.print(title);
    if (t.mode == MODE_ROUTE) display.printf(" %u/%u %um", t.waypoint, t.waypointCount, t.distanceM);
    else if (t.flags & TELEM_FLAG_HOME_SET) display.printf(" %um", t.distanceM);
    if (t.headingDeci == 0xFFFF) display.print(" H---"); else display.printf(" H%u", t.headingDeci / 10);
  }

  // Readiness: + ready, - not ready, ? compass present but not calibrated.
  // While the boat is coming home by its own decision, say why instead.
  static const char *const HOME_REASONS[] = {"", "", "signal was lost", "route finished",
                                             "boat battery low", "geofence crossed", "recall"};
  display.setCursor(0, top + 8);
  if (st.linked && t.mode == MODE_HOME && t.homeReason >= HOME_SIGNAL_LOST &&
      t.homeReason <= HOME_RECALL) {
    display.printf("Why: %s%s", HOME_REASONS[t.homeReason],
                   (t.flags2 & TELEM2_GPS_STEERING) ? ", GPS" : "");
  } else if (st.linked) {
    char compass = !(t.flags & TELEM_FLAG_COMPASS_OK) ? '-'
                   : (t.flags & TELEM_FLAG_COMPASS_CAL) ? '+' : '?';
    display.printf("GPS%c Cmp%c Home%c Rte%c", (t.flags & TELEM_FLAG_GPS_FIX) ? '+' : '-', compass,
                   (t.flags & TELEM_FLAG_HOME_SET) ? '+' : '-',
                   (t.flags & TELEM_FLAG_ROUTE_OK) ? '+' : '-');
  }

  display.setTextWrap(true);

  // Menu in the large font, scrolled to keep the highlighted entry in view. The
  // first character is a space, or briefly '*' after a choice has been sent, or
  // 'x' if it was refused because the boat is not linked.
  bool justMarked = navMarkMs && millis() - navMarkMs < 1500;
  RowList rows;
  for (uint8_t item = 0; item < NAV_MENU_COUNT; item++) {
    rows.add("%c%s", (item == navMenuIndex && justMarked) ? navMark : ' ', NAV_MENU[item]);
  }
  drawRows(rows, top + 16, navMenuIndex);
}

// ---- Setup page: settings changed with the buttons ----

static const SetupItem SETUP_ITEMS[] = {
  // Names are short: with the value they must fit 10 characters.
  {"Speed",  PARAM_NONE,           20,  100, 10},   // % controller: caps joystick power
  {"Trim",   PARAM_TRIM,          -30,   30,  1},   // % L/R, + slows the right motor
  {"Cruise", PARAM_CRUISE,         20,  100,  5},   // % autopilot power
  {"Batt",   PARAM_LOW_BATTERY_MV,  0, 13000, 100}, // return-home voltage; off below 9.00
  {"Fence",  PARAM_GEOFENCE_M,      0, 1000, 20},   // metres, 0 = off
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

static void setupFormatValue(uint8_t index, int16_t value, char *text, size_t size) {
  switch (SETUP_ITEMS[index].param) {
    case PARAM_TRIM:
      snprintf(text, size, "%+d", value);
      break;
    case PARAM_LOW_BATTERY_MV:
      if (value == 0) snprintf(text, size, "off");
      else snprintf(text, size, "%d.%02dV", value / 1000, (value % 1000) / 10);
      break;
    case PARAM_GEOFENCE_M:
      if (value == 0) snprintf(text, size, "off");
      else snprintf(text, size, "%d", value);
      break;
    default:
      snprintf(text, size, "%d", value);
      break;
  }
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
  uint8_t top = drawBanner(nullptr, false);
  RowList rows;
  for (uint8_t i = 0; i < SETUP_COUNT; i++) {
    char value[8];
    if (i == setupIndex && setupEditing) {
      setupFormatValue(i, setupValue, value, sizeof(value));
    } else if (SETUP_ITEMS[i].param != PARAM_NONE && !st.seen) {
      snprintf(value, sizeof(value), "--");            // boat setting, boat not heard yet
    } else {
      setupFormatValue(i, setupCurrent(i, st), value, sizeof(value));
    }
    rows.addPair(SETUP_ITEMS[i].name, value);
  }
  // A filled bar marks the setting; while it is being edited, a frame instead.
  drawRows(rows, top, setupIndex, setupEditing);
}

// ---- Trip page: statistics since the boat was switched on ----

static void drawTripPage(const Status &st) {
  const TelemetryPacket &t = st.t;
  uint8_t top = drawBanner(st.seen && !st.linked ? "NO LINK" : nullptr, true);
  RowList rows;
  if (!st.seen) {
    rows.add("No boat");
    rows.add("heard yet");
  } else {
    rows.addDistance("Dist", t.tripDistanceM);
    rows.addSpeed("Top", t.tripTopSpeedCmS);
    rows.add("Run %u:%02u", t.tripMotorSec / 60, t.tripMotorSec % 60);
    if (t.tripMinBatteryMv) rows.add("Low %u.%02uV", t.tripMinBatteryMv / 1000, (t.tripMinBatteryMv % 1000) / 10);
  }
  drawRows(rows, top);
}

// ---- Test page: self-test progress and results ----

static void drawTestPage(const Status &st) {
  const TelemetryPacket &t = st.t;
  static const char *const STEP_TITLES[] = {"TEST", "L AHEAD", "L ASTERN", "R AHEAD",
                                            "R ASTERN", "DONE", "ABORTED"};
  bool running = st.linked && t.testStep >= TEST_LEFT_AHEAD && t.testStep <= TEST_RIGHT_ASTERN;
  // The banner is the step in progress (blinking while it runs), or NO LINK.
  uint8_t top = drawBanner(!st.linked ? "NO LINK"
                           : t.testStep != TEST_IDLE ? STEP_TITLES[min<uint8_t>(t.testStep, TEST_ABORTED)]
                                                     : nullptr,
                           !st.linked || running);
  RowList rows;
  if (!st.seen || !(t.testResults & TEST_RAN)) {
    rows.add("No test");
    rows.add("run yet.");
    rows.add("Nav menu:");
    rows.add("Self-test");
  } else {
    // Results stay on screen after the test, and after the link drops.
    uint8_t r = t.testResults;
    rows.add((r & TEST_BATTERY_OK) ? "Batt ok" : "Batt FAIL");
    rows.add("Bat %u.%02uV", t.batteryMv / 1000, (t.batteryMv % 1000) / 10);
    rows.add(!(r & TEST_GPS_DATA) ? "GPS FAIL" : (r & TEST_GPS_FIX) ? "GPS fix ok" : "GPS no fix");
    rows.add(!(r & TEST_COMPASS_OK) ? "Cmp absent" : (r & TEST_COMPASS_CAL) ? "Cmp ok cal" : "Cmp no cal");
    rows.add((r & TEST_RADIO_OK) ? "Radio ok" : "Radio FAIL");
    rows.add("Boat%4ddB", t.rssi);
    // Battery drop in mV for: left ahead, left astern, right ahead, right astern.
    rows.add("Sag mV:");
    rows.addSag("L fwd", t.testSagMv[0]);
    rows.addSag("L rev", t.testSagMv[1]);
    rows.addSag("R fwd", t.testSagMv[2]);
    rows.addSag("R rev", t.testSagMv[3]);
  }
  drawRows(rows, top);
}

// Addresses and radio details are too long for the large font, so this page
// stays small. It needs no banner, but a recall still takes the top.
static void drawInfoPage(const Status &st) {
  uint8_t top = drawBanner(nullptr, false);
  uint8_t pitch = top ? 9 : 10;
  display.setCursor(0, top);
  display.print("Me   ");
  display.print(WiFi.macAddress().substring(6));    // last 3 bytes fit the line
  display.setCursor(0, top + pitch);
  display.printf("Boat %02X:%02X:%02X:%02X", BOAT_MAC[2], BOAT_MAC[3], BOAT_MAC[4], BOAT_MAC[5]);
  // Chip temperature (uncalibrated sensor) and transmit power at each end.
  display.setCursor(0, top + 2 * pitch);
  display.printf("Me   %3dC tx %2ddBm", (int)linkTempC, linkTxPowerQdbm / 4);
  display.setCursor(0, top + 3 * pitch);
  if (st.linked) {
    display.printf("Boat %3dC tx %2ddBm", st.t.tempC, st.t.txPowerDbm);
  } else {
    display.print("Boat  --C tx --dBm");
  }
  display.setCursor(0, top + 4 * pitch);
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
static const uint32_t SHORT_PRESS_MAX_MS = 600;   // a tap is shorter than this; a hold counts at 1000

static Press poll(Button &b) {
  bool down = digitalRead(b.pin) == LOW;
  uint32_t now = millis();
  if (down != b.down) {
    if (now - b.changedMs < 40) return PRESS_NONE;   // debounce
    uint32_t heldMs = now - b.changedMs;
    b.down = down;
    b.changedMs = now;
    if (down) {
      b.longFired = false;
      return PRESS_NONE;
    }
    // Released after a long press that did fire, or after holding too long to
    // be a tap but not long enough to count (a hold that was let go early):
    // neither is a short press, so a hesitant hold never acts as "go back".
    if (b.longFired || heldMs > SHORT_PRESS_MAX_MS) return PRESS_NONE;
    return PRESS_SHORT;
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

  if (!inDetails) {
    // Top level: NEXT moves down the titles, SELECT opens the highlighted page.
    if (next == PRESS_SHORT) page = (page + 1) % PAGE_COUNT;
    else if (select == PRESS_SHORT) { inDetails = true; detailScroll = 0; }
  } else if (page == PAGE_SETUP && setupEditing) {
    // Editing a setting: NEXT is plus, SELECT is minus; hold SELECT to save, hold NEXT to cancel.
    if (next == PRESS_SHORT) setupStep(+1);
    else if (select == PRESS_SHORT) setupStep(-1);
    else if (select == PRESS_LONG) setupSave(st.linked);
    else if (next == PRESS_LONG) setupEditing = false;
  } else {
    // An open page: a short SELECT goes back to the titles. NEXT scrolls the
    // rows, except on Nav and Setup where it moves the marker (the view follows
    // it) and a held SELECT acts.
    if (select == PRESS_SHORT) {
      inDetails = false;
    } else if (page == PAGE_NAV) {
      if (next == PRESS_SHORT) {
        navMenuIndex = (navMenuIndex + 1) % NAV_MENU_COUNT;
      } else if (select == PRESS_LONG) {
        bool sent = sendNavCommand(st.linked, NAV_MENU_COMMAND[navMenuIndex]);
        if (sent && NAV_MENU_COMMAND[navMenuIndex] == CMD_SELF_TEST) {   // watch it run
          page = PAGE_TEST;
          detailScroll = 0;
        }
      }
    } else if (page == PAGE_SETUP) {
      if (next == PRESS_SHORT) setupIndex = (setupIndex + 1) % SETUP_COUNT;
      else if (select == PRESS_LONG) setupBeginEdit(st);
    } else if (page != PAGE_INFO && next == PRESS_SHORT) {
      // One row down, back to the top after the last. (drawRows() leaves
      // detailScroll alone, so first bring it inside what is shown.)
      uint8_t shown = min<uint8_t>(detailScroll, detailScrollMax);
      detailScroll = (shown >= detailScrollMax) ? 0 : shown + 1;
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
    if (!inDetails) drawTitleList();
    else switch (page) {
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
