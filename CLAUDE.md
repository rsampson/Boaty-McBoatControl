# CLAUDE.md

Developer notes for this project. The user-facing build and operating guide is
[README.md](README.md); keep it in step with any change to wiring, buttons,
display pages or behaviour.

## What this is

Firmware for a twin-motor, differentially steered RC boat and its hand
controller, both on classic ESP32 boards, talking over ESP-NOW in Espressif Long
Range mode. Plus a Python diagnostic tool for the PC.

```
BoatController/           hand controller firmware (one Arduino sketch)
  BoatController.ino      joystick, radio, display pages, buttons, beeper, OTA
  console.ino             serial diagnostic console
  config.h                pins, tuning, BOAT_MAC
  protocol.h              radio packets and link setup  (IDENTICAL COPY)
BoatReceiver/             boat firmware (one Arduino sketch)
  BoatReceiver.ino        radio, motors, GPS, compass, autopilot, self-test, OTA
  console.ino             serial diagnostic console
  navigation.h            pure navigation maths (no hardware; PC-testable)
  config.h                pins, tuning, CONTROLLER_MAC, WAYPOINTS
  protocol.h              radio packets and link setup  (IDENTICAL COPY)
tools/boatdiag.py         PC diagnostics over USB serial (needs pyserial)
ESP32Transmit.ino,        the owner's earlier attempt, kept for reference only.
ESP8266Receive.ino        Not built. Do not edit or delete.
```

## Build, flash, test

```
arduino-cli compile --fqbn esp32:esp32:esp32 --warnings default BoatController
arduino-cli compile --fqbn esp32:esp32:esp32 --warnings default BoatReceiver
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyACM0 <sketch>
.venv/bin/python tools/boatdiag.py        # talks to whichever firmware is on the board
```

- Core: arduino-esp32 **3.3.x**. The ESP-NOW callback signatures and the LEDC
  API (`ledcAttach(pin, freq, bits)`, `ledcWrite(pin, duty)`) are the 3.x ones;
  they will not compile on 2.x.
- Libraries: Adafruit SSD1306 and Adafruit GFX (controller only). Everything
  else is in the core. The GPS and compass drivers are written in the sketch.
- Both sketches must compile with **no warnings**. Always compile both after
  touching `protocol.h`.
- Python: the system Python is externally managed. Use the project `.venv`.
- Both sketches are at about 78% of the default 1.3 MB app partition. Large
  additions will need a different partition scheme.

## Rules that are easy to break

1. **`protocol.h` is duplicated.** Arduino sketches cannot share a file, so each
   folder has a copy. Edit `BoatController/protocol.h`, then copy it over
   `BoatReceiver/protocol.h`, and check with `cmp`.
2. **Bump `PROTO_VERSION` whenever a packet layout or enum value changes.** Each
   end drops packets with a different version, which is what stops a
   half-updated pair from misbehaving. Both boards must then be reflashed.
3. **Arduino generates function prototypes at the top of the `.ino`.** Any
   struct or enum used as a function parameter or return type must be defined
   before the first function in the file, or in a header. The controller has a
   "Types" block near the top for this. Symptom if you get it wrong:
   `'Foo' does not name a type` pointing at a line that looks fine.
4. **`console.ino` is concatenated after the main `.ino`.** It can use every
   static in the main file. The main file can call functions defined in
   `console.ino` (prototypes are hoisted) but cannot see its variables.
5. **Radio callbacks run in the WiFi task.** They only copy data under
   `stateMux` and return. No `delay()`, no Serial, no I2C in them. `loop()`
   takes a snapshot under the same mutex.
6. **On the controller, only `controlTask` reads the ADC.** It runs at 50 Hz on
   its own FreeRTOS task so that display updates (about 25 ms of I2C each)
   cannot delay control packets. Anything else that needs an analogue reading
   gets it from a variable that task fills in (`rawThrottle`,
   `controllerBatteryMv`).
7. **ADC inputs must be on ADC1 (GPIO32-39).** ADC2 does not work while WiFi is
   running.
8. **Long Range mode and normal WiFi are mutually exclusive.** That is why OTA
   is a separate boot mode (a flag in RTC memory, then `ESP.restart()`), not
   something that runs alongside the link.
9. **`display.begin()` returns true with no display attached.** The controller
   probes the I2C address first. Found by testing on hardware.
10. **Never weaken a failsafe to make something else work.** See the mode rules
    below; each one exists for a stated reason.

## Radio protocol (`protocol.h`)

- `linkBegin(peerMac)`: station mode, `WIFI_PROTOCOL_LR`, power save off, fixed
  `RADIO_CHANNEL`, maximum transmit power, one unicast peer, 250 kbit/s LR rate.
- **Controller to boat, 50 Hz: `ControlPacket`.** Left and right motor commands
  (−1000..+1000, already mixed on the controller) plus a one-shot command.
- **Boat to controller, 5 Hz: `TelemetryPacket`.** Everything the display
  shows. Only sent while the boat can hear the controller.
- **Commands are events, not state.** The controller repeats the latest
  `command` and `commandSeq` in every packet; the boat acts once when
  `commandSeq` changes. The boat takes the first sequence number it hears as its
  baseline without acting, so a boat that restarts never replays an old command.
  The controller starts its sequence at a random value for the mirror-image
  reason. `CMD_SET_PARAM` carries `paramId` and `paramValue`.
- Each end accepts packets only from its configured peer MAC, with the right
  length, magic byte and version.
- Link quality on the display comes from three independent measurements:
  ESP-NOW delivery acknowledgements counted on the controller, packets per
  second counted by the boat, and RSSI at each end.

## Boat firmware

`loop()` never blocks. It runs fixed-rate ticks off `millis()`: motors every
5 ms, compass and autopilot every 50 ms, battery every 50 ms, statistics every
second, telemetry at 5 Hz.

### Modes (`BoatMode`, variable `mode`)

| Mode | Motors driven by | Left by |
|---|---|---|
| `MODE_MANUAL` | Joystick commands; zero at once if the link is lost | Commands; automatic return home |
| `MODE_ROUTE` | Autopilot toward `WAYPOINTS[waypointIndex]` | Last waypoint (goes home); stick override; command |
| `MODE_HOME` | Autopilot toward the home point; off once within the arrival radius | Stick override; command |
| `MODE_CALIBRATING` | Nothing | Ends by itself after `COMPASS_CAL_MS` |
| `MODE_SELFTEST` | The test sequence | Finishes; any abort condition |

All transitions go through `enterMode()` or `goHome(reason)`.

### Safety rules and why

- **Manual and link lost for `FAILSAFE_MS` (500 ms): motors to zero
  immediately**, bypassing the ramp.
- **Link lost for `RTH_DELAY_MS` (3 s) with a home point: `MODE_HOME`.** This
  latches. The boat keeps returning after the link comes back, so a boat at the
  edge of range does not flip between modes.
- **Stick override.** In route or home mode, a joystick command above
  `NAV_STICK_OVERRIDE` returns to manual, but only after the stick has been seen
  at rest (`overrideArmed`). Without that, a stick held over when the link
  dropped would cancel the return home the moment the link returned.
- **No fix or no compass under autopilot: motors off** (`navHold`), mode
  unchanged, resumes when the sensors return.
- **Route acceptance (`routeOk`).** Checked when the home point is set: at least
  one waypoint, and every waypoint within `NAV_MAX_RANGE_M` of home. This guards
  against a mistyped coordinate.
- **Low battery and geofence trip once.** If the operator overrides, low battery
  re-arms only after the voltage recovers by 200 mV; the geofence re-arms when
  the boat is back inside 90% of the radius. Otherwise the operator could never
  keep manual control.
- **Self-test and bench motor commands** refuse to start unless the boat is in
  manual mode. The self-test aborts on stick movement, link loss or any command.
  Bench runs from the PC are capped at 3 s and stop if a controller appears.
- **If the radio fails to start, the motors are never driven.**

### Subsystems

- **Motors.** DRV8871, two PWM inputs each, 20 kHz, 10-bit. Drive/brake ("slow
  decay") PWM: one input held high and the other pulsed low, which gives a more
  linear speed response than drive/coast. Both low is coast. Commands are
  slew-limited (`MOTOR_RAMP_MS`). Trim scales one side down just before output.
- **GPS.** NEO-6M on `Serial2` at 9600 baud. A small NMEA parser handles RMC
  (position, speed, course, validity) and GGA (satellite count), with checksum
  checking. State is in the `gps` struct; `gpsHasFix()` and `gpsPresent()` time
  out after 3 s.
- **Compass.** HMC5883L (0x1E) or QMC5883L (0x0D), detected at start-up, read
  directly over `Wire`. Hard-iron offsets and axis scale come from a min/max
  calibration and are stored in NVS namespace `compass`. No tilt compensation.
- **Navigation maths** (`navigation.h`). `navDistanceBearing()` uses a
  flat-earth approximation; `navSteer()` pivots on the spot beyond
  `NAV_PIVOT_DEG` of heading error and steers proportionally inside it. It has
  no hardware dependencies so that it can be compiled and tested on a PC.
- **Settings.** Trim, cruise power, low-battery voltage and geofence radius live
  in NVS namespace `settings`. The `DEFAULT_*` values in `config.h` apply only
  until a setting is first saved.
- **Battery.** Divider on GPIO34, `analogReadMilliVolts`, exponentially
  smoothed. The pack is a 3-cell lithium; thresholds are in millivolts.
- **Home point.** The first fix with at least `HOME_MIN_SATS` satellites after
  power-up, or wherever `CMD_SET_HOME` is received. Held in RAM only.

## Controller firmware

- **`controlTask` (50 Hz).** Reads the joystick, applies centre calibration,
  deadband and expo, mixes to left/right (`left = throttle + steer`,
  `right = throttle − steer`, both scaled down together if either exceeds full
  power so the turn ratio is kept), applies the speed limit and sends.
- **`loop()`.** Buttons, beeper, display. Display pages are the `draw…Page()`
  functions selected by the `Page` enum; all take a `Status` snapshot from
  `readStatus()`.
- **Buttons.** `poll()` returns a short press on release, or a long press after
  one second held. Long presses confirm anything that makes the boat act, to
  prevent accidents.
- **Stale data.** Pages keep showing the last telemetry when the link drops,
  marked as stale. This is deliberate: the last known position is how a
  stranded boat gets found.
- **Controller settings.** Only the speed limit, in NVS namespace `controller`.
- **Joystick centre** is measured at power-up. If it is far from mid-travel the
  firmware falls back to mid-scale and warns (`centreSuspect`).

## Serial console and PC tool

Each firmware answers line commands on USB serial at 115200 baud. The reply is
`key=value` lines ending in `OK`, or one `ERR reason` line. The commands are
listed at the top of each `console.ino`. `tools/boatdiag.py` drives these; it
ignores any line that is not `key=value`, so ordinary log output does not
confuse it. When adding a firmware feature, expose its state in `status` and
report it in the tool.

The boat's once-a-second status line is suppressed for 5 s after each console
command.

## Verification status

**Verified on a bare ESP32 (no peripherals):** both firmwares boot; Long Range
radio initialises; the console and PC tool work; console motor commands and the
serial-started self-test run to completion; missing display, compass, GPS and
battery are reported correctly.

**Verified with two bare ESP32s on the bench:** pairing by MAC; the Long Range
link in both directions (100% of control packets acknowledged, 99% counted by
the boat, all telemetry received, about −35 dBm at each end); the controller's
slow search rate with no boat (5 packets/s) and its return to 50 packets/s once
the boat answers.

**Verified on a PC:** the NMEA parser against standard sentences, and the
navigation maths (distances, bearings, steering direction).

**Never tested:** range beyond the bench; motors; GPS and compass hardware; the
display pages; buttons; beeper; OTA upload; compass calibration; the autopilot
and every automatic return-home path; settings transfer from the Setup page.
Treat all of these as unproven. Autopilot gains (`NAV_*`) are first guesses.

## The owner's boards

| Role | MAC | Set in |
|---|---|---|
| Boat | `F8:B3:B7:4F:56:D8` | `BOAT_MAC` in `BoatController/config.h` |
| Controller | `F0:24:F9:43:49:74` | `CONTROLLER_MAC` in `BoatReceiver/config.h` |

Before flashing, read the attached board's MAC (`esptool read-mac`, or the
tool's `id`) and flash the firmware that matches its role. Serial port names
are not a reliable guide; they depend on the order the boards were plugged in.

A controller with no boat answering used to transmit almost continuously
(radio retries of every unacknowledged packet) and ran very hot. That is why
`SEARCH_RATE_HZ` exists; do not remove it.

## Working with the owner

- Before a change that touches more than one file, list the files and what
  changes in each, and wait for approval.
- Do exactly what was asked. Offer larger ideas in a sentence; do not build them
  unasked.
- Flash the attached board only when asked to.
- `sudo` is not available in a session.

## Ideas not yet built

Agreed to wait until the boat has been tested on water, because they depend on
how the compass and GPS behave in practice:

- Heading hold in manual mode when the stick is straight ahead.
- Record a route while driving, store it in flash, and replay it.
- Station keeping (hold the present position).
- Cross-track steering between waypoints; blending GPS course with the compass
  heading when moving.
- A black-box log in flash, downloadable by the PC tool.
- A live map and a route planner on the PC.
