# Boaty-McBoatControl

A radio-controlled model boat with twin motors, steered by running the two
motors at different speeds. It has a hand controller with a joystick and a small
display, a long-range radio link, GPS and compass, an autopilot that can follow
a list of waypoints, and an automatic return to its starting point if the radio
link is lost.

This guide covers building the hardware, loading the firmware, operating the
boat, and running the diagnostics. Notes for people changing the code are in
[CLAUDE.md](CLAUDE.md).

> **Status.** Both firmwares compile, boot on a bare ESP32 and answer the PC
> diagnostic tool. Nothing has yet been tested with motors, sensors, a display
> or two boards talking to each other, and the autopilot has never run on water.
> Do first runs with the propellers off or the boat tethered.

## Contents

1. [What you need](#1-what-you-need)
2. [Building the boat](#2-building-the-boat)
3. [Building the controller](#3-building-the-controller)
4. [Loading the firmware](#4-loading-the-firmware)
5. [First-time setup](#5-first-time-setup)
6. [Driving](#6-driving)
7. [The controller display](#7-the-controller-display)
8. [Autonomous navigation](#8-autonomous-navigation)
9. [What the boat does by itself](#9-what-the-boat-does-by-itself)
10. [Lights and beeps](#10-lights-and-beeps)
11. [Updating firmware over WiFi](#11-updating-firmware-over-wifi)
12. [Diagnostics](#12-diagnostics)
13. [Troubleshooting](#13-troubleshooting)

## 1. What you need

**Boat**

| Part | Notes |
|---|---|
| ESP32 development board | The classic ESP32 (ESP32-WROOM-32, "ESP32 Dev Module"). Not an S2, S3 or C3. |
| 2 x DRV8871 motor driver boards | One per motor. Needs 6.5 V or more on its motor supply. |
| 2 x brushed DC motors with propellers | Check the driver's current limit suits them. |
| NEO-6M GPS module (GY-NEO6MV2) with antenna | |
| HMC5883L compass module (GY-271) | Boards carrying a QMC5883L chip also work; the firmware detects which it has. |
| 3-cell lithium battery | 12.6 V full, about 9.9 V empty. |
| 12 V to 5 V step-down regulator | Powers the ESP32. At least 1 A. |
| Resistors: 100 kΩ and 22 kΩ | Battery voltage sensing. |
| Fuse and power switch | In the battery's positive lead. |

**Controller**

| Part | Notes |
|---|---|
| ESP32 development board | Classic ESP32, as above. |
| Two-axis joystick | The common thumb-stick module with two potentiometers. |
| SSD1306 OLED display, 128x64, I2C | Address 0x3C. |
| 2 x push buttons | Momentary, normally open. |
| Passive piezo beeper | |
| 1 lithium cell and a 5 V step-up converter | Powers the ESP32. |
| Resistors: 2 x 100 kΩ | Battery voltage sensing. |

**Computer:** a PC with a USB port, [arduino-cli](https://arduino.github.io/arduino-cli/)
(or the Arduino IDE) and Python 3.

## 2. Building the boat

### Power

```
battery + --- fuse --- switch ---+--- VM of both DRV8871 boards
                                 +--- 5 V regulator input
                                 +--- 100 kΩ ---+--- 22 kΩ --- GND
                                                |
                                             GPIO34

5 V regulator output --- ESP32 "5V" / "VIN" pin
ESP32 "3V3" pin      --- GPS VCC, compass VCC
All grounds joined together (battery -, regulator, ESP32, drivers, GPS, compass).
```

Take the 100 kΩ resistor from the **battery**, not from the regulator's 5 V
output, or the boat cannot tell how much charge is left. Never connect battery
voltage directly to an ESP32 pin.

### Signal wiring

| ESP32 pin | Goes to |
|---|---|
| GPIO25 | Left DRV8871 IN1 |
| GPIO26 | Left DRV8871 IN2 |
| GPIO18 | Right DRV8871 IN1 |
| GPIO19 | Right DRV8871 IN2 |
| GPIO16 | GPS module **TX** |
| GPIO17 | GPS module **RX** |
| GPIO21 | Compass SDA |
| GPIO22 | Compass SCL |
| GPIO34 | Middle of the battery sense resistors |

Each motor connects to the two output terminals of its driver. If a motor turns
out to run backwards, it is easier to fix in software (step 5) than to rewire.

### Mounting the compass

- Mount it **flat and level**. The heading is wrong when the boat heels, as
  there is no tilt correction.
- Keep it **as far as you can** from the motors, the battery leads and anything
  made of steel. Motor current disturbs it.
- Give the GPS antenna a clear view of the sky.

## 3. Building the controller

### Power

```
cell + ---+--- 5 V step-up input          step-up output --- ESP32 "5V" / "VIN"
          +--- 100 kΩ ---+--- 100 kΩ --- GND
                         |
                      GPIO36
```

As on the boat, sense the **cell**, not the step-up output. If you do not fit
the two resistors, set `PIN_CONTROLLER_BATTERY` to `-1` in
`BoatController/config.h`, or the display will show a meaningless voltage.

### Signal wiring

| ESP32 pin | Goes to |
|---|---|
| GPIO34 | Joystick forward/back wiper |
| GPIO35 | Joystick left/right wiper |
| 3V3 and GND | The two ends of both joystick potentiometers |
| GPIO21 | Display SDA |
| GPIO22 | Display SCL |
| GPIO32 | NEXT button (other side to GND) |
| GPIO33 | SELECT button (other side to GND) |
| GPIO25 | Piezo beeper (other side to GND) |
| GPIO36 | Middle of the battery sense resistors |

The display is powered from 3V3 and GND. The buttons need no resistors. The
ESP32's own BOOT button is also used (see section 11).

## 4. Loading the firmware

Install the ESP32 board support (version 3.3 or later) and the display library:

```
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit SSD1306" "Adafruit GFX Library"
```

The two boards must be told each other's address:

1. Plug in the boat's ESP32 and load its firmware:
   ```
   arduino-cli compile --fqbn esp32:esp32:esp32 BoatReceiver
   arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyACM0 BoatReceiver
   ```
   (Use your own port name: `/dev/ttyUSB0`, `COM3` and so on.)
2. Do the same for the controller with `BoatController`.
3. Find each board's MAC address. Either run the diagnostic tool (section 12),
   which prints it, or open a serial monitor at 115200 baud and reset the board.
   The controller also shows its address on the display for a moment at start-up.
4. Put the **boat's** address in `BOAT_MAC` in `BoatController/config.h`, and
   the **controller's** address in `CONTROLLER_MAC` in `BoatReceiver/config.h`.
   An address `F8:B3:B7:4F:56:D8` is written `{0xF8, 0xB3, 0xB7, 0x4F, 0x56, 0xD8}`.
5. Compile and upload both again.

Both boards must always run firmware built from the same copy of this project.
If they do not match they ignore each other.

## 5. First-time setup

Do these once, on the bench, in this order.

1. **Run the diagnostics** (section 12) on each board. Fix anything marked FAIL.
2. **Check the link.** Switch both on. The controller should show `LINK OK`.
3. **Check motor direction** with the propellers off. Run the self-test from the
   Nav page (section 7). The display names each motor and direction in turn. If
   a motor runs the wrong way, set `MOTOR_INVERT_LEFT` or `MOTOR_INVERT_RIGHT`
   to `true` in `BoatReceiver/config.h`.
4. **Check the joystick.** Forward should drive both motors ahead; right should
   speed up the left motor. If an axis is reversed, set `JOY_INVERT_THROTTLE` or
   `JOY_INVERT_STEER` in `BoatController/config.h`.
5. **Calibrate the compass.** On the Nav page choose `Calibrate compass`. The
   display shows `TURN 360`. Within 20 seconds, turn the boat slowly through one
   full circle, keeping it level. The result is stored permanently.
6. **Check the compass.** The Nav page shows the heading. It should read about 0
   with the bow pointing north and 90 pointing east.
   - Counts the wrong way round: set `COMPASS_REVERSED` to `true`.
   - Off by a constant amount: put the correction in `COMPASS_OFFSET_DEG`.
   - Set `MAGNETIC_DECLINATION_DEG` for your area (east positive).
   - Run the motors and watch the heading. If it swings, move the compass
     further from the motors and wiring.
7. **Set the trim** on the water. If the boat pulls to one side with the stick
   straight ahead, adjust `Trim L/R` on the Setup page.

## 6. Driving

- **Switch on with the joystick centred.** The controller measures the stick's
  centre in the first moment after power-up.
- Push forward and back for speed, left and right to steer. With no forward
  speed, left and right spin the boat on the spot.
- Wait for a GPS fix before you set off, so the boat records where home is. The
  Nav page shows `Home+` when it has.
- Moving the joystick always takes back control from the autopilot.

## 7. The controller display

The **NEXT** button steps through seven pages. The **SELECT** button steps
backwards, except on the Nav and Setup pages where it works the menus.

| Page | Shows |
|---|---|
| 1 Link | `LINK OK` or `NO LINK`; percentage of packets acknowledged and received, with a bar; signal strength at each end in dBm (nearer 0 is stronger); boat and controller battery voltage; the motor commands being sent. |
| 2 GPS | Fix state, satellites, latitude and longitude, speed and course, and the boat's distance and bearing from home. If the link drops, the last known position stays on screen. |
| 3 Nav | The boat's mode in large letters, progress, a readiness line, and the command menu. |
| 4 Setup | Settings you can change. |
| 5 Trip | Distance travelled, top speed, motor running time and lowest battery voltage since the boat was switched on. |
| 6 Test | Self-test progress and results. |
| 7 Info | Addresses, radio channel and reminders of the button actions. |

### Nav page

The large text is what the boat is doing: `MANUAL`, `ROUTE`, `HOMING`,
`AT HOME`, `TURN 360` (compass calibration), `TESTING` or `NO LINK`.

The readiness line reads, for example, `GPS+ Cmp+ Home+ Rte-`:

| Item | `+` means | Otherwise |
|---|---|---|
| GPS | The boat has a position fix | `-` no fix |
| Cmp | Compass working and calibrated | `?` not calibrated, `-` not found |
| Home | Home point recorded | `-` not yet |
| Rte | Waypoints loaded and accepted | `-` none, or one is too far from home |

**Menu.** A short press of SELECT moves the `>` marker. **Hold SELECT for one
second** to send the choice; the marker turns to `*` briefly.

| Choice | Effect |
|---|---|
| Manual control | Joystick control. |
| Run route | Follow the waypoints, then return home. Needs all four readiness items. |
| Return home | Go to the home point and stop there. |
| Set home here | Make the boat's present position the home point. |
| Calibrate compass | Start the 20-second calibration. |
| Run self-test | Check the sensors, then run each motor briefly. |

### Setup page

A short press of SELECT moves down the list. **Hold SELECT** to edit the
highlighted setting: NEXT is plus, SELECT is minus. **Hold SELECT** again to
save, or **hold NEXT** to cancel.

| Setting | Range | Meaning |
|---|---|---|
| Speed limit % | 20–100 | Caps the joystick's power. Useful for beginners. |
| Trim L/R % | −30 to +30 | Positive slows the right motor, so the boat bears right. |
| Cruise % | 20–100 | Power the autopilot uses. |
| LowBatt home | off, 9.0–13.0 V | The boat returns home below this voltage. Default 9.9 V. |
| Geofence m | off, 20–1000 | The boat returns home if driven further than this from home. |

All but the speed limit are stored in the boat, so they can only be edited while
the boat is linked. The page shows the boat's own value; seeing the new number
there confirms it was saved.

## 8. Autonomous navigation

### Entering waypoints

Waypoints are a list in `BoatReceiver/config.h`, in decimal degrees, north and
east positive:

```cpp
static const Waypoint WAYPOINTS[] = {
  {37.4219000, -122.0840000},
  {37.4222500, -122.0836000},
};
```

Take them from a map application, using at least six decimal places. Then
compile and upload the boat firmware (by USB, or over WiFi as in section 11).
The list is empty as supplied, which disables Route mode.

### Running a route

1. Switch on at the place you want the boat to return to, and wait for `Home+`.
2. Check all four readiness items show `+`.
3. Choose `Run route` and hold SELECT.

The boat steers for each waypoint in turn, counting one as reached when it is
within 5 m, then returns home and stops. It slows down as it nears each target.
The Nav page shows the waypoint number, the distance to go and the heading.

### Safeguards

- A route is refused if any waypoint is more than 1000 m from home, so a
  mistyped coordinate cannot send the boat away (`NAV_MAX_RANGE_M`).
- If the boat loses its GPS fix or its compass while under autopilot, it stops
  its motors and waits. The Nav page flashes `WAITING GPS/COMPASS`.
- Moving the joystick more than a little takes back manual control at once.

GPS positions are only good to a few metres. Do not route the boat close to
obstacles, the bank or other water users, and keep it in sight.

## 9. What the boat does by itself

| Situation | What happens |
|---|---|
| Radio link lost for half a second, in manual | Motors stop. |
| Radio link lost for 3 seconds | The boat returns home and stops there. |
| Link comes back while returning | It keeps returning until you move the joystick. |
| Battery below the `LowBatt home` setting for 10 seconds | The boat returns home. |
| Driven past the geofence | The boat returns home. If you take back control while still outside, the fence is ignored until the boat is back inside. |
| End of a route | The boat returns home. |
| At home | Motors off. It sets off again only if it drifts more than 10 m away. |

When the boat is coming home by its own decision, the Nav page says why.

All of these need a recorded home point, which needs a GPS fix with at least six
satellites. **With no home point the boat can only stop its motors.**

## 10. Lights and beeps

**Controller beeper** (silent until the boat has been heard once):

| Sound | Meaning |
|---|---|
| Fast repeated beeps | Link lost. |
| Double beep every 2 seconds | Fewer than half the packets are getting through. |
| One low beep every 5 seconds | Boat or controller battery low. |

**Boat LED** (the ESP32's on-board LED):

| Pattern | Meaning |
|---|---|
| Steady | Linked to the controller. |
| Slow blink | No signal. |
| Quick blink | Under autopilot. |
| Very fast flicker | Radio failed to start. |
| One-second blink | Firmware update mode. |

## 11. Updating firmware over WiFi

The long-range radio mode cannot talk to ordinary WiFi devices, so updating is a
separate mode.

1. **Hold the ESP32's BOOT button for 2 seconds.** The board restarts into
   update mode. The boat's motors are off in this mode.
2. On your computer, join the WiFi network `Boat-OTA` or `Controller-OTA`.
   The password is in the board's `config.h` (`OTA_AP_PASSWORD`); change it from
   the default.
3. Upload to address `192.168.4.1` using the Arduino IDE's network port or
   arduino-cli's network upload.
4. The board restarts into normal operation when the upload finishes. To leave
   update mode without uploading, press BOOT again or reset the board.

This feature has not been tested yet. USB upload always works.

## 12. Diagnostics

### From the controller

`Run self-test` on the Nav page checks the boat's battery, GPS, compass and
radio, then runs each motor ahead and astern for 1.5 seconds at low power. The
Test page shows the step in progress and the results. Moving the joystick stops
the test.

The boat cannot sense whether a motor is really turning, so **watch the
motors**. The last line of the Test page gives the battery voltage drop during
each motor step; no drop at all suggests that motor drew no current.

### From a PC

Plug one or both boards into USB. Set up once:

```
python3 -m venv .venv
.venv/bin/pip install -r tools/requirements.txt
```

Then:

```
.venv/bin/python tools/boatdiag.py                 # automatic checks
.venv/bin/python tools/boatdiag.py --interactive   # adds checks that need your hands
.venv/bin/python tools/boatdiag.py --motors        # also runs each boat motor
```

The tool finds the boards itself (name them with `--port` if it cannot) and
prints one line per check, marked PASS, FAIL, WARN or info.

| Board | Automatic checks | With `--interactive` |
|---|---|---|
| Boat | Firmware and address; last reset cause; memory; radio; compass found and calibrated; GPS data, errors and fix; battery voltage; route and settings. | Compass turn test with a quality score; battery calibration against a multimeter; confirm each motor's direction (with `--motors`). |
| Controller | Firmware and address; reset cause; memory; radio; display found; joystick centre and noise; stuck buttons; beeper and display test pattern; its battery. | Joystick full-range test with suggested settings; press each button; confirm the beep and the display pattern. |
| Both at once | Matching firmware, channel and addresses; packets sent, acknowledged and received over 10 seconds; signal strength. | |

`--motors` turns the propellers. The tool asks before it does so.

Each board also accepts typed commands in a serial monitor at 115200 baud. The
list is at the top of `console.ino` in each firmware folder.

## 13. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `NO LINK` always | Addresses not set or swapped in the two `config.h` files; boards running different firmware versions; boat not powered. Run the PC tool with both boards connected: it checks all three. |
| Motors do nothing | Battery below 6.5 V at the drivers; wiring to IN1/IN2; no link. |
| A motor runs backwards | Set its `MOTOR_INVERT_` option. |
| Boat creeps with the stick centred | Controller was switched on with the stick off-centre. Switch it off and on again. |
| `STICK CAL? re-power` on the display | Same cause; or a joystick wire is off. |
| Battery voltage reads wrong | Resistor values differ from `config.h`; adjust `BATT_CAL`. The PC tool's interactive mode works out the value. |
| `GPS-` for a long time | A first fix can take several minutes outdoors and may never come indoors. |
| `Cmp-` | Compass wiring, or SDA and SCL swapped. |
| Heading wrong or wandering | Not calibrated; compass too near motors or steel; boat not level. |
| `Rte-` with waypoints entered | A waypoint is over 1000 m from home, or no home point yet. The boat's serial output names the waypoint. |
| Boat returns home unexpectedly | Read the reason on the Nav page: signal, battery or geofence. |
| Under autopilot the boat circles or weaves | Compass problem (see section 5), or reduce `Cruise %`. |
