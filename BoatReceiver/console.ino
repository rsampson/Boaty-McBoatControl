// console.ino - serial diagnostic console for the boat, used by tools/boatdiag.py.
//
// Send a command line at 115200 baud; the reply is "key=value" lines followed
// by "OK", or a single "ERR reason" line. Commands:
//   id                        board identity, firmware build, reset reason, memory
//   status                    mode, link, battery, GPS, compass, self-test results
//   i2c                       scan the I2C bus
//   compass                   raw and corrected compass readings
//   gps                       GPS traffic counters and last fix
//   battery                   battery reading, smoothed and instantaneous
//   motor <L|R> <power> <ms>  run one motor (-1000..1000, at most 3000 ms).
//                             Bench use only: refused while a controller is linked.
//   selftest                  run the self-test without a controller
//   stop                      stop a bench motor run or self-test
//
// While commands are arriving, the once-a-second status line is suppressed.

static uint32_t consoleLastCommandMs = 0;
static bool     consoleUsed = false;

bool consoleQuiet() {
  return consoleUsed && millis() - consoleLastCommandMs < 5000;
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
  Serial.println("board=boat");
  Serial.printf("build=%s %s\n", __DATE__, __TIME__);
  Serial.printf("proto=%d\n", PROTO_VERSION);
  Serial.printf("channel=%d\n", RADIO_CHANNEL);
  Serial.printf("mac=%s\n", WiFi.macAddress().c_str());
  Serial.printf("peer=%02X:%02X:%02X:%02X:%02X:%02X\n", CONTROLLER_MAC[0], CONTROLLER_MAC[1],
                CONTROLLER_MAC[2], CONTROLLER_MAC[3], CONTROLLER_MAC[4], CONTROLLER_MAC[5]);
  Serial.printf("radio_ok=%d\n", radioOk);
  Serial.printf("reset_reason=%s\n", resetReasonName());
  Serial.printf("heap_free=%u\n", (unsigned)ESP.getFreeHeap());
  Serial.printf("heap_min=%u\n", (unsigned)ESP.getMinFreeHeap());
  Serial.printf("uptime_ms=%lu\n", (unsigned long)millis());
}

static void consoleGps() {
  Serial.printf("present=%d\n", gpsPresent());
  Serial.printf("fix=%d\n", gpsHasFix());
  Serial.printf("sats=%u\n", gps.sats);
  Serial.printf("bytes=%lu\n", (unsigned long)gpsBytes);
  Serial.printf("sentences=%lu\n", (unsigned long)gpsSentences);
  Serial.printf("checksum_errors=%lu\n", (unsigned long)gpsChecksumErrors);
  Serial.printf("lat_e7=%ld\n", (long)gps.latE7);
  Serial.printf("lon_e7=%ld\n", (long)gps.lonE7);
  Serial.printf("speed_cm_s=%u\n", gps.speedCmS);
  Serial.printf("course_deci=%u\n", gps.courseDeci);
}

static void consoleCompass() {
  Serial.printf("chip=%s\n", compassChip == COMPASS_HMC5883L   ? "HMC5883L"
                             : compassChip == COMPASS_QMC5883L ? "QMC5883L"
                                                               : "none");
  Serial.printf("ok=%d\n", compassOk());
  Serial.printf("calibrated=%d\n", compassCalibrated);
  Serial.printf("raw_x=%d\n", compassRawX);
  Serial.printf("raw_y=%d\n", compassRawY);
  Serial.printf("offset_x=%.1f\n", compassOffX);
  Serial.printf("offset_y=%.1f\n", compassOffY);
  Serial.printf("scale_x=%.3f\n", compassScaleX);
  Serial.printf("scale_y=%.3f\n", compassScaleY);
  Serial.printf("heading=%.1f\n", headingDeg);
}

static void consoleStatus() {
  Serial.printf("mode=%u\n", mode);
  Serial.printf("linked=%d\n", !linkLostNow);
  Serial.printf("rx_per_sec=%u\n", rxPerSec);
  Serial.printf("rx_total=%lu\n", (unsigned long)rxCount);
  Serial.printf("telemetry_sent=%lu\n", (unsigned long)telemetrySent);
  Serial.printf("rssi=%d\n", lastRssi);
  Serial.printf("battery_mv=%.0f\n", batteryMv);
  Serial.printf("gps_present=%d\n", gpsPresent());
  Serial.printf("gps_fix=%d\n", gpsHasFix());
  Serial.printf("compass_ok=%d\n", compassOk());
  Serial.printf("compass_calibrated=%d\n", compassCalibrated);
  Serial.printf("home_set=%d\n", homeSet);
  Serial.printf("route_ok=%d\n", routeOk);
  Serial.printf("waypoints=%u\n", WAYPOINT_COUNT);
  Serial.printf("home_reason=%u\n", homeReason);
  Serial.printf("set_trim=%d\n", setTrim);
  Serial.printf("set_cruise=%u\n", setCruise);
  Serial.printf("set_low_battery_mv=%u\n", setLowBatteryMv);
  Serial.printf("set_geofence_m=%u\n", setGeofenceM);
  Serial.printf("trip_distance_m=%.0f\n", tripDistanceM);
  Serial.printf("trip_top_speed_cm_s=%u\n", tripTopSpeedCmS);
  Serial.printf("trip_motor_sec=%u\n", tripMotorSec);
  Serial.printf("trip_min_battery_mv=%u\n", tripMinBatteryMv);
  Serial.printf("test_step=%u\n", testStep);
  Serial.printf("test_results=%u\n", testResults);
  Serial.printf("test_sag_mv=%u,%u,%u,%u\n", testSagMv[0], testSagMv[1], testSagMv[2], testSagMv[3]);
}

static void consoleI2c() {
  Serial.print("found=");
  bool first = true;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2cProbe(addr)) {
      Serial.printf("%s0x%02x", first ? "" : ",", addr);
      first = false;
    }
  }
  Serial.println();
}

// Returns nullptr on success, or the reason for refusal.
static const char *consoleMotor(const char *side, const char *powerText, const char *msText) {
  if (!side || !powerText || !msText) return "usage: motor <L|R> <power> <ms>";
  int power = atoi(powerText);
  long ms = atol(msText);
  bool left = (side[0] == 'L' || side[0] == 'l');
  if (!left && side[0] != 'R' && side[0] != 'r') return "side must be L or R";
  if (power < -CMD_MAX || power > CMD_MAX) return "power must be -1000..1000";
  if (ms < 1 || ms > 3000) return "ms must be 1..3000";
  if (!linkLostNow) return "controller is linked";
  if (mode != MODE_MANUAL) return "boat is not in manual mode";
  benchLeft = left ? power : 0;
  benchRight = left ? 0 : power;
  benchStartMs = millis();
  benchDurationMs = (uint32_t)ms;
  benchActive = true;
  return nullptr;
}

static void consoleRun(char *line) {
  consoleUsed = true;
  consoleLastCommandMs = millis();

  char *command = strtok(line, " ");
  if (!command) return;
  const char *error = nullptr;

  if (!strcmp(command, "id")) {
    consoleId();
  } else if (!strcmp(command, "status")) {
    consoleStatus();
  } else if (!strcmp(command, "i2c")) {
    consoleI2c();
  } else if (!strcmp(command, "compass")) {
    consoleCompass();
  } else if (!strcmp(command, "gps")) {
    consoleGps();
  } else if (!strcmp(command, "battery")) {
    Serial.printf("battery_mv=%.0f\n", batteryMv);
    Serial.printf("now_mv=%.0f\n", batteryReadMv());
    Serial.printf("pin_mv=%lu\n", (unsigned long)analogReadMilliVolts(PIN_BATTERY));
    Serial.printf("cal=%.3f\n", (float)BATT_CAL);
    Serial.printf("min_mv=%d\n", TEST_BATTERY_MIN_MV);
  } else if (!strcmp(command, "motor")) {
    char *side = strtok(nullptr, " ");
    char *power = strtok(nullptr, " ");
    char *ms = strtok(nullptr, " ");
    error = consoleMotor(side, power, ms);
  } else if (!strcmp(command, "selftest")) {
    if (mode != MODE_MANUAL) {
      error = "boat is not in manual mode";
    } else if (!linkLostNow) {
      error = "controller is linked: start it from the controller";
    } else {
      testFromSerial = true;
      selfTestStart();
    }
  } else if (!strcmp(command, "stop")) {
    benchActive = false;
    if (mode == MODE_SELFTEST) selfTestAbort("stopped from the PC");
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
