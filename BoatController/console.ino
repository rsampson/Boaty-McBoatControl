// console.ino - serial diagnostic console for the controller, used by tools/boatdiag.py.
//
// Send a command line at 115200 baud; the reply is "key=value" lines followed
// by "OK", or a single "ERR reason" line. Commands:
//   id               board identity, firmware build, reset reason, memory
//   status           link counters and the latest telemetry from the boat
//   stick            raw and processed joystick readings
//   buttons          the state of each button (1 = pressed)
//   beep <hz> <ms>   sound the beeper (100..10000 Hz, at most 2000 ms)
//   display          show a test pattern for 3 seconds
//   recall <0|1>     withdraw or give the recall order (as holding both buttons does)

static uint32_t consoleDisplayUntilMs = 0;
static bool     consoleDisplayHeld = false;

// True while the test pattern is on screen and normal drawing must wait.
bool consoleDisplayBusy() {
  if (consoleDisplayHeld && (int32_t)(millis() - consoleDisplayUntilMs) >= 0) {
    consoleDisplayHeld = false;
  }
  return consoleDisplayHeld;
}

static const char *resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power_on";
    case ESP_RST_EXT:      return "reset_pin";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "crash";
    case ESP_RST_INT_WDT:  return "interrupt_watchdog";
    case ESP_RST_TASK_WDT: return "task_watchdog";
    case ESP_RST_WDT:      return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    default:               return "unknown";
  }
}

static void consoleId() {
  Serial.println("board=controller");
  Serial.printf("build=%s %s\n", __DATE__, __TIME__);
  Serial.printf("proto=%d\n", PROTO_VERSION);
  Serial.printf("channel=%d\n", RADIO_CHANNEL);
  Serial.printf("mac=%s\n", WiFi.macAddress().c_str());
  Serial.printf("peer=%02X:%02X:%02X:%02X:%02X:%02X\n", BOAT_MAC[0], BOAT_MAC[1], BOAT_MAC[2],
                BOAT_MAC[3], BOAT_MAC[4], BOAT_MAC[5]);
  Serial.printf("radio_ok=%d\n", radioOk);
  Serial.printf("display_ok=%d\n", displayOk);
  Serial.printf("reset_reason=%s\n", resetReasonName());
  Serial.printf("heap_free=%u\n", (unsigned)ESP.getFreeHeap());
  Serial.printf("heap_min=%u\n", (unsigned)ESP.getMinFreeHeap());
  Serial.printf("uptime_ms=%lu\n", (unsigned long)millis());
  Serial.printf("temp_c=%.1f\n", temperatureRead());
}

static void consoleStatus() {
  Status st = readStatus();
  Serial.printf("linked=%d\n", st.linked);
  Serial.printf("ack_pct=%u\n", st.ackPct);
  Serial.printf("tx_total=%lu\n", (unsigned long)txTotal);
  Serial.printf("ack_total=%lu\n", (unsigned long)ackTotal);
  Serial.printf("telemetry_total=%lu\n", (unsigned long)telemetryTotal);
  Serial.printf("rssi_here=%d\n", st.rssiHere);
  Serial.printf("tx_power_dbm=%.1f\n", linkTxPowerQdbm / 4.0f);
  Serial.printf("tx_power_cap_dbm=%.1f\n", linkThermalCapQdbm / 4.0f);
  Serial.printf("cmd_left=%d\n", cmdLeft);
  Serial.printf("cmd_right=%d\n", cmdRight);
  Serial.printf("page=%u\n", page);
  Serial.printf("recall=%d\n", recallActive);
  Serial.printf("speed_limit_pct=%u\n", speedLimitPct);
  Serial.printf("battery_mv=%u\n", controllerBatteryMv);
  Serial.printf("battery_warn_mv=%d\n", CTRL_BATT_WARN_MV);
  if (st.seen) {
    Serial.printf("boat_rssi=%d\n", st.t.rssi);
    Serial.printf("boat_rx_per_sec=%u\n", st.t.rxPerSec);
    Serial.printf("boat_battery_mv=%u\n", st.t.batteryMv);
    Serial.printf("boat_mode=%u\n", st.t.mode);
    Serial.printf("boat_flags=%u\n", st.t.flags);
  }
}

static void consoleStick() {
  // The control task owns the ADC while it runs; use its latest readings.
  int throttle = radioOk ? (int)rawThrottle : readRaw(PIN_JOY_THROTTLE);
  int steer = radioOk ? (int)rawSteer : readRaw(PIN_JOY_STEER);
  Serial.printf("raw_throttle=%d\n", throttle);
  Serial.printf("raw_steer=%d\n", steer);
  Serial.printf("centre_throttle=%d\n", centreThrottle);
  Serial.printf("centre_steer=%d\n", centreSteer);
  Serial.printf("centre_suspect=%d\n", centreSuspect);
  Serial.printf("raw_min=%d\n", JOY_RAW_MIN);
  Serial.printf("raw_max=%d\n", JOY_RAW_MAX);
  Serial.printf("cmd_left=%d\n", cmdLeft);
  Serial.printf("cmd_right=%d\n", cmdRight);
}

static void consoleRun(char *line) {
  char *command = strtok(line, " ");
  if (!command) return;
  const char *error = nullptr;

  if (!strcmp(command, "id")) {
    consoleId();
  } else if (!strcmp(command, "status")) {
    consoleStatus();
  } else if (!strcmp(command, "stick")) {
    consoleStick();
  } else if (!strcmp(command, "buttons")) {
    Serial.printf("next=%d\n", digitalRead(PIN_BUTTON_NEXT) == LOW);
    Serial.printf("select=%d\n", digitalRead(PIN_BUTTON_SELECT) == LOW);
    Serial.printf("boot=%d\n", digitalRead(PIN_OTA_BUTTON) == LOW);
  } else if (!strcmp(command, "beep")) {
    char *hzText = strtok(nullptr, " ");
    char *msText = strtok(nullptr, " ");
    int hz = hzText ? atoi(hzText) : 0;
    int ms = msText ? atoi(msText) : 0;
    if (hz < 100 || hz > 10000 || ms < 1 || ms > 2000) {
      error = "usage: beep <100..10000 Hz> <1..2000 ms>";
    } else {
      tone(PIN_BEEPER, hz, ms);
    }
  } else if (!strcmp(command, "recall")) {
    char *value = strtok(nullptr, " ");
    if (!value || (value[0] != '0' && value[0] != '1')) error = "usage: recall <0|1>";
    else recallActive = (value[0] == '1');
  } else if (!strcmp(command, "display")) {
    if (!displayOk) {
      error = "display not found";
    } else {
      // Border, diagonals and a filled block: shows dead rows, columns and offsets.
      display.clearDisplay();
      display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
      display.drawLine(0, 0, 127, 63, SSD1306_WHITE);
      display.drawLine(0, 63, 127, 0, SSD1306_WHITE);
      display.fillRect(48, 24, 32, 16, SSD1306_WHITE);
      display.display();
      consoleDisplayUntilMs = millis() + 3000;
      consoleDisplayHeld = true;
    }
  } else {
    error = "unknown command";
  }

  if (error) Serial.printf("ERR %s\n", error);
  else Serial.println("OK");
}

void consolePoll() {
  static char line[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = 0;
      if (len) consoleRun(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}
