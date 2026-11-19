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

**New to the controller?** Start with the [operating manual](#operating-manual-using-the-controller).

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

## Operating manual: using the controller

This part is written for someone who has never used the controller. It assumes
the boat and controller have already been built, loaded and set up (sections 1
to 5). Everything here is also covered in more detail later in this guide.

> **Be careful.** The autopilot and the automatic return home have not yet been
> proved on the water. Always keep the boat in sight, stay away from people and
> swimmers, and keep your hands clear of the propellers whenever the boat is
> switched on.

### What you are holding

| Part | What it does |
|---|---|
| **Joystick** | Drives the boat. Forward and back is speed; left and right is steering. |
| **Screen** | Shows the boat's state. Big letters; it is the only place the controller talks to you. |
| **NEXT button** | Moves down: to the next page title, the next menu line, or scrolls the page. |
| **SELECT button** | Opens a page, goes back, and (when held) confirms an action. |
| **Beeper** | Warns you with beeps (see "What the beeps mean" below). |
| **Small `BOOT` button on the board** | For firmware updates only. Leave it alone: holding it for 2 seconds restarts the controller into update mode. |

**Short press or hold?** A *short press* is a quick tap. A *hold* means keeping
the button down for **one full second**. A press in between (more than half a
second, but let go before the second is up) does nothing at all, so you can
simply try again. Anything that makes the boat do
something needs a hold, so that a tap by accident can never do it.

### Switching on

1. Put the boat on the water, or on a stand with the propellers clear of
   anything, and switch the boat on.
2. **Let go of the joystick**, so it sits in the middle, then switch the
   controller on. The controller measures where the middle is in the first
   moment, so do not touch the stick until the screen has lit up.
3. Wait for the boat to be heard. Open the **Link** page (see below). `NO LINK`
   flashes at the top until the boat answers; it normally takes a few seconds.
4. Wait for a GPS fix. Open the **Nav** page and wait for `GPS+` and `Home+` in
   its second small line. The boat records where "home" is only once it has a
   GPS fix, and it cannot come home without one.
5. If `BAD STICK` flashes on the Link page, the joystick was not centred or is
   not working. The controller sends **nothing** until it is restarted. Switch
   it off, let go of the stick and switch it on again.

### Driving

- **Push the stick forward** and the boat goes ahead; **pull it back** and it
  goes astern. The further you push, the faster it goes.
- **Push left or right** and the boat turns that way. With the stick centred
  forward and back, pushing it left or right spins the boat on the spot.
- **Let go** and the boat stops. The stick springs back to the middle.
- The stick has a small dead zone in the middle, so tiny movements do nothing.
- If the boat creeps to one side with the stick straight ahead, adjust `Trim` on
  the Setup page (below).
- If you want the boat gentler, lower `Speed` on the Setup page. At 50 the
  stick's full travel gives half power. This is a good setting for beginners.

### Reading the screen

When you switch on, the screen shows a **list of page titles**: `LINK`, `GPS`,
`NAV`, `SETUP`, `TRIP`, `TEST`, `INFO`. One is highlighted as a white bar.

- **Tap NEXT** to move the bar down the list. After `INFO` it goes back to
  `LINK`. (It only goes down, never up.)
- **Tap SELECT** to open the highlighted page.
- **Tap SELECT again** on any open page to go back to the list.
- On an open page, **tap NEXT** to scroll to the rows below. A thin bar on the
  right edge shows that there is more, and where you are. After the last row it
  starts again at the top.
- The **Nav** and **Setup** pages are different: there, NEXT moves the
  highlighted line instead (the page moves with it).

Which page to look at:

| Page | Look here to find out... |
|---|---|
| **Link** | Can the boat hear me? `Ack` is the share of my messages the boat confirms (aim for 90% or more); `Boat` and `Here` are signal strengths (nearer 0 is better, for example -50 is strong and -90 is weak); `Bat` is the boat's battery and `Me` is the controller's. |
| **GPS** | Where is the boat? Satellites, heading, latitude and longitude, speed, and its distance and direction from home. If the link drops, the last known position stays here, marked `last known`: write it down if you need to go and find the boat. |
| **Nav** | What is the boat doing, and what can I tell it to do? The first small line is its mode; the second is the readiness line; the big menu is where you give orders. |
| **Setup** | Settings: speed limit, trim and so on. |
| **Trip** | Distance, top speed, motor time and lowest battery voltage since the boat was switched on. |
| **Test** | The result of the last self-test. |
| **Info** | Technical details. Normally you can ignore this page. |

**Two warnings that take over the top of the screen:**

- **`NO LINK`** (flashing) means the boat has not been heard for a moment. The
  numbers underneath are the last ones received, not live ones.
- **`RECALL`** means a recall is on (see "Bringing the boat home").

### Giving the boat an order (the Nav menu)

1. Open the **Nav** page (move the bar to `NAV`, tap SELECT).
2. **Tap NEXT** until the highlight is on the order you want.
3. **Hold SELECT for one second.** A `*` appears in front of the line briefly:
   the order was sent. An `x` instead means the boat cannot hear you, so
   nothing was sent. Orders are never saved up for later; try again when the
   link is back.

| Menu line | What the boat does |
|---|---|
| `Manual` | Gives control back to the joystick. |
| `Route` | Follows the saved waypoints, then comes home. Needs `GPS+ Cmp+ Home+ Rte+` on the readiness line. |
| `Home` | Drives back to the home point and stops there. |
| `Set home` | Makes where the boat is **right now** the new home point. |
| `Calibrate` | Starts compass calibration: turn the boat slowly through one full circle within 20 seconds. |
| `Self-test` | Checks the boat's parts and runs each motor briefly. Keep the propellers clear. Moving the joystick stops it. |

The readiness line, for example `GPS+ Cmp+ Home+ Rte-`, tells you if the boat is
ready to look after itself: `+` is good, `-` is not ready, and `?` after `Cmp`
means the compass has not been calibrated.

**Taking the boat back from the autopilot.** While the boat is on a route or
coming home, **let go of the stick** (so it is centred), then push it. The boat
returns to manual control. Releasing first is deliberate: it stops a stick that
was already pushed over from cancelling the return home by accident. (This does
not work during a recall; see below.)

### Changing a setting (the Setup page)

1. Open the **Setup** page. Tap NEXT to move the highlight to the setting you
   want.
2. **Hold SELECT.** The highlighted bar becomes an outline: you are now editing.
3. **Tap NEXT** to raise the value or **tap SELECT** to lower it.
4. **Hold SELECT** to save, or **hold NEXT** to cancel and keep the old value.

| Setting | What it does |
|---|---|
| `Speed` | Caps how much power the joystick can give, as a percentage. Lower is gentler. |
| `Trim` | Corrects a boat that pulls to one side. If it veers right, make it more negative. |
| `Cruise` | How fast the boat drives itself when on a route or coming home. |
| `Batt` | The boat comes home by itself if its battery falls below this voltage. `off` turns it off. |
| `Fence` | The boat comes home if it goes further than this many metres from home. `off` turns it off. |

All except `Speed` are stored in the boat, so they can only be changed while the
boat is linked. A row showing `--` means the boat has not been heard yet.

### Bringing the boat home

**If the boat is in trouble, do this first. It is the emergency button.**

> **Hold both buttons together for one second.**

The screen shows `RECALL`: flashing until the boat has confirmed it got the
order, then steady. The boat now drives home and stops there, and **moving the
joystick does not cancel it**, so a nudge cannot stop the rescue. To cancel,
hold both buttons together for one second again.

A recall only works if the boat has a home point and a GPS fix. A boat without
a good compass still comes home, less neatly, by steering on its GPS track. It
cannot help a boat whose battery is flat or whose motor has failed.

You do not have to do anything for the usual problems. The boat returns home
**by itself** if the radio link is lost for 3 seconds, if its battery falls
below the `Batt` setting, if it goes beyond the `Fence` distance, or when a
route finishes. The Nav page then says why (`Why: signal was lost`, and so
on). With the link lost for only half a second the motors simply stop.

### What the beeps mean

The controller is silent until it has heard the boat once.

| Sound | Meaning | What to do |
|---|---|---|
| Fast repeated beeps | The link is lost. | Wait a few seconds. The boat starts to come home on its own after 3 seconds. Hold both buttons for a recall if you want to be sure. |
| Two beeps, then a pause | Fewer than half of your messages are getting through. | The boat is near the edge of range. Bring it back towards you. |
| One low beep every 5 seconds | The boat's or controller's battery is low. | Bring the boat in. Check the `Link` page to see which one. |

### Finishing

1. Bring the boat in, or send it `Home`, and wait until the Nav page says
   `AT HOME` (or the boat is in your hand).
2. Switch the boat off, then the controller.

### If something looks wrong

| You see | Likely cause |
|---|---|
| `NO LINK` flashing and fast beeps | The boat is out of range or switched off. Wait; it comes home by itself. |
| `BAD STICK` | Joystick was off-centre at power-up. Switch the controller off and on again with the stick released. |
| `x` appears when you send an order | The boat is not linked. Wait for the link to return and send it again. |
| `NO HOME POINT YET` flashing | The boat had no GPS fix when it started. Wait for `GPS+`, or use `Set home` once it has a fix. |
| `WAITING FOR GPS FIX` or `WAITING FOR COMPASS` flashing | The boat wants to drive itself but cannot. Its motors are off until it can. |
| The boat will not go with the stick pushed | Check the Link page: if `Ack` is low or `NO LINK` shows, the boat is not hearing you. Check `Speed` on the Setup page is not too low. |

More help, and the meaning of the LED patterns, is in section 10 and the
troubleshooting table in section 13.

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
5. **Calibrate the compass.** On the Nav page choose `Calibrate`. The
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
   straight ahead, adjust `Trim` on the Setup page.

## 6. Driving

- **Switch on with the joystick centred.** The controller measures the stick's
  centre in the first moment after power-up.
- Push forward and back for speed, left and right to steer. With no forward
  speed, left and right spin the boat on the spot.
- Wait for a GPS fix before you set off, so the boat records where home is. The
  Nav page shows `Home+` when it has.
- Moving the joystick takes back control from the autopilot (except during a
  recall, below).

### Recall: getting the boat back

**Hold both buttons together for one second**, on any page. The display shows
`RECALL` in large letters on every page: blinking until the boat confirms it has
the order, then steady. Hold both buttons again to withdraw it and return to
manual control.

A recall overrides everything else. The boat comes home from manual driving, a
route, the self-test or a compass calibration, and **the joystick does not
cancel it**, so a knocked stick cannot interrupt the retrieval. The order is
sent in every packet while it is on, so it gets through on a poor link and stays
in force if the link drops.

| The boat's situation | What a recall does |
|---|---|
| Home point set, GPS fix, compass good | Returns home and stops there. |
| Compass missing, faulty or not calibrated | Returns home steering by its GPS course. This is slower and less direct: the boat moves off straight ahead first, then curves toward home. The Nav page shows `Why: recall, by GPS`. |
| No GPS fix | Motors stop. It sets off as soon as a fix arrives. |
| No home point recorded | Motors stop. Nothing can be done until the boat has had a GPS fix. |

The Nav page says which of these applies. A recall cannot help if the battery is
exhausted or a motor has failed.

## 7. The controller display

The display has two levels. The top level is a list of the seven page titles in
large letters, with the highlighted one shown as a white bar. The list scrolls
when it does not all fit.

- **NEXT** moves the highlight down one title. After `INFO` it wraps round to
  `LINK`. There is no way to move up.
- **SELECT** opens the highlighted page. A short press of **SELECT** on an open
  page returns to the list.
- On an open page the rows are in large letters, four at a time. **NEXT**
  scrolls down one row and wraps to the top; a thin bar down the right edge
  shows where you are when there is more than fits. Nothing else is shown at the
  top: `RECALL` (blinking until the boat confirms it, then steady) or a blinking
  `NO LINK` takes the top row only when it applies. A page that shows
  old values (Link, GPS, Trip, Test) shows `NO LINK` when the boat is not heard. This banner replaces the
  per-value `?` marks the older layout used.
- On the **Nav** and **Setup** pages NEXT moves the highlight down the menu
  instead; the view follows it.
- The **Info** page is in small letters, because the addresses will not fit in
  large ones.

The controller starts on the list with `LINK` highlighted. The page numbers
below are only the order of the list.

| Page | Shows |
|---|---|
| 1 Link | Percentage of packets acknowledged (`Ack`) and received (`Rx`); signal strength at each end in dB (`Boat`, `Here`; nearer 0 is stronger); boat (`Bat`) and controller (`Me`) battery voltage; the motor commands being sent (`L`, `R`). `BAD STICK` blinks if the joystick failed its power-up check. |
| 2 GPS | Fix state, satellites, the boat's compass heading, latitude (`N`/`S`) and longitude (`E`/`W`) to five decimals, speed and course, and the boat's distance (`Away`) and bearing (`Brng`) from home. If the link drops, the last known position stays on screen, marked `last known`. |
| 3 Nav | The boat's mode and progress and a readiness line in small letters, then the command menu in large letters. |
| 4 Setup | Settings you can change. |
| 5 Trip | Distance travelled, top speed, motor running time and lowest battery voltage since the boat was switched on. |
| 6 Test | Self-test progress and results: battery, GPS, compass and radio (with signal strength), and the battery voltage drop for each motor step. |
| 7 Info | Addresses; the chip temperature reading and transmit power at each end; radio channel and firmware protocol version. |

### Nav page

The first line says what the boat is doing: `MANUAL`, `ROUTE`, `HOMING`,
`AT HOME`, `TURN 360` (compass calibration), `TESTING` or `NO LINK`, followed by
the waypoint and distance to go (`ROUTE 2/5 85m`), or the distance home, and the
heading (`H214`, or `H---`).

The readiness line reads, for example, `GPS+ Cmp+ Home+ Rte-`:

| Item | `+` means | Otherwise |
|---|---|---|
| GPS | The boat has a position fix | `-` no fix |
| Cmp | Compass working and calibrated | `?` not calibrated, `-` not found |
| Home | Home point recorded | `-` not yet |
| Rte | Waypoints loaded and accepted | `-` none, or one is too far from home |

**Menu.** A short press of NEXT moves the highlight. **Hold SELECT for one
second** to send the choice; a `*` appears in front of it briefly. With no link
the choice is refused and an `x` appears instead: commands are never saved up
for later.

| Choice | Effect |
|---|---|
| Manual | Joystick control. |
| Route | Follow the waypoints, then return home. Needs all four readiness items. |
| Home | Go to the home point and stop there. |
| Set home | Make the boat's present position the home point. |
| Calibrate | Start the 20-second calibration. |
| Self-test | Check the sensors, then run each motor briefly. |

### Setup page

A short press of NEXT moves the highlight down the list. **Hold SELECT** to edit
the highlighted setting (its bar becomes a frame while you edit): NEXT is plus, SELECT is minus. **Hold SELECT** again to
save, or **hold NEXT** to cancel. While a setting is being edited a short press
of SELECT means minus, not "go back"; once the edit is saved or cancelled it
goes back to the list again.

| Setting | Range | Meaning |
|---|---|---|
| Speed (%) | 20–100 | Caps the joystick's power. Useful for beginners. |
| Trim (%) | −30 to +30 | Positive slows the right motor, so the boat bears right. |
| Cruise (%) | 20–100 | Power the autopilot uses. |
| Batt | off, 9.0–13.0 V | The boat returns home below this voltage. Default 9.9 V. |
| Fence (m) | off, 20–1000 | The boat returns home if driven further than this from home. |

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
3. Choose `Route` and hold SELECT.

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
| Recall given from the controller | The boat returns home from any state; only withdrawing the recall ends it (section 6). |
| Battery below the `Batt` setting for 10 seconds | The boat returns home. |
| Driven past the geofence | The boat returns home. If you take back control while still outside, the fence is ignored until the boat is back inside. |
| End of a route | The boat returns home. |
| At home | Motors off. It sets off again only if it drifts more than 10 m away. |

When the boat is coming home by its own decision, the Nav page says why.

The boat only steers itself with a **calibrated compass**. Without one it stops
its motors and drifts instead of returning, so calibrate before the first outing.

All of these need a recorded home point, which needs a GPS fix with at least six
satellites. **With no home point the boat can only stop its motors.**

### Transmit power

Each radio uses only as much power as the other end needs to hear it well, and
goes back to full power the moment the link looks doubtful or is lost. At close
range both ends drop to their minimum (2 dBm); full power is 19.5 dBm. This
keeps the boards cooler and saves a little battery. The Info page shows the
power in use at each end.

If a chip's temperature reading passes `TEMP_LIMIT_C`, its power is capped until
it cools. The sensor is uncalibrated and reads high (about 70 at room
temperature on the boards used here), so the default limit of 85 is a starting
point: note what the Info page reads when a board is as warm as you are happy
with, and set the limit in that board's `config.h`.

To switch all of this off and transmit at full power always, set
`TX_POWER_AUTO` to `false` and `TEMP_LIMIT_C` to `0` in both `config.h` files.

## 10. Lights and beeps

**Controller beeper** (silent until the boat has been heard once):

| Sound | Meaning |
|---|---|
| Fast repeated beeps | Link lost. |
| Double beep every 2 seconds | Fewer than half the packets are getting through. |
| One low beep every 5 seconds | Boat or controller battery low. |

**On-board LED** (both boards). It is a heartbeat: it blinks only while the
firmware is running, and the number of flashes in each beat tells you which
firmware is on the board.

| Pattern | Meaning |
|---|---|
| **One** short flash per beat | Boat firmware. |
| **Two** short flashes per beat | Controller firmware. |
| A beat every second | Linked to the other unit. |
| A beat every three seconds | Running, but not linked. |
| Continuous quick blinking (boat) | Under autopilot. |
| Very fast flicker | Radio failed to start. |
| Even one-second on, one-second off | Firmware update mode. |
| Dark or steadily lit | The firmware is not running. |

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

`Self-test` on the Nav page checks the boat's battery, GPS, compass and
radio, then runs each motor ahead and astern for 1.5 seconds at low power. The
Test page shows the step in progress and the results. Moving the joystick stops
the test.

The boat cannot sense whether a motor is really turning, so **watch the
motors**. The `Sag mV` rows at the bottom of the Test page give the battery voltage drop during
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
| `STICK FAULT: restart` on the display, and the boat will not move | The joystick was off-centre at switch-on, or a joystick wire is off. The controller sends no motor commands until it is restarted with the stick centred. |
| Battery voltage reads wrong | Resistor values differ from `config.h`; adjust `BATT_CAL`. The PC tool's interactive mode works out the value. |
| `GPS-` for a long time | A first fix can take several minutes outdoors and may never come indoors. |
| `Cmp-` | Compass wiring, or SDA and SCL swapped. |
| Heading wrong or wandering | Not calibrated; compass too near motors or steel; boat not level. |
| `Rte-` with waypoints entered | A waypoint is over 1000 m from home, or no home point yet. The boat's serial output names the waypoint. |
| Boat returns home unexpectedly | Read the reason on the Nav page: signal, battery or geofence. |
| Under autopilot the boat circles or weaves | Compass problem (see section 5), or reduce `Cruise %`. |
