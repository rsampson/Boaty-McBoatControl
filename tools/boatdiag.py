#!/usr/bin/env python3
"""boatdiag - PC diagnostics for the RC boat and its controller.

Plug one or both boards into USB and run:

    .venv/bin/python tools/boatdiag.py                 automatic checks
    .venv/bin/python tools/boatdiag.py --interactive   adds checks that need your hands
    .venv/bin/python tools/boatdiag.py --motors        also runs each boat motor briefly

Boards are found automatically; use --port to name them. With both boards
connected the radio link between them is tested as well.
"""

import argparse
import math
import re
import sys
import time

import serial
from serial.tools import list_ports

KEY_VALUE = re.compile(r"^([a-z0-9_]+)=(.*)$")
# USB vendor IDs of the serial chips found on ESP32 boards: Silicon Labs CP210x,
# WCH CH340/CH9102, FTDI, and Espressif's own. Auto-detection opens only these,
# so it never disturbs an unrelated device such as a printer or modem.
ESP_USB_VENDORS = {0x10C4, 0x1A86, 0x0403, 0x303A}
ZERO_MAC = "00:00:00:00:00:00"
TX_POWER_FULL_DBM = 19.5    # the radio's maximum; anything lower means power control or a heat cap is acting
CLOSE_RANGE_DBM = -55       # both ends hear each other this strongly: power should come down


class DiagError(Exception):
    pass


class Board:
    """One board's serial console."""

    def __init__(self, port):
        self.port = port
        self.info = {}
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = 115200
        ser.timeout = 0.2
        # Most ESP32 boards reset when DTR/RTS toggle. Hold them inactive so
        # opening the port disturbs the board as little as possible.
        ser.dtr = False
        ser.rts = False
        ser.open()
        self.ser = ser

    def command(self, text, timeout=2.0):
        """Send a command; return its key=value reply as a dict."""
        self.ser.reset_input_buffer()
        self.ser.write((text + "\n").encode())
        values = {}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.ser.readline().decode(errors="replace").strip()
            if line == "OK":
                return values
            if line.startswith("ERR"):
                raise DiagError(line[3:].strip() or "command refused")
            match = KEY_VALUE.match(line)
            if match:
                values[match.group(1)] = match.group(2)
        raise DiagError(f"no reply to '{text}'")

    def identify(self, wait=10.0):
        """Wait for the board to answer 'id' (it may have just reset)."""
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            try:
                self.info = self.command("id", timeout=1.0)
                if "board" in self.info:
                    return True
            except DiagError:
                pass
        return False

    @property
    def kind(self):
        return self.info.get("board", "?")


class Report:
    """Collects and prints results."""

    MARKS = {"pass": "[ PASS ]", "fail": "[ FAIL ]", "warn": "[ WARN ]", "info": "[ info ]"}

    def __init__(self):
        self.counts = {"pass": 0, "fail": 0, "warn": 0, "info": 0}

    def section(self, title):
        print(f"\n== {title} ==")

    def add(self, level, name, detail=""):
        self.counts[level] += 1
        print(f"{self.MARKS[level]} {name}" + (f": {detail}" if detail else ""))

    def check(self, ok, name, detail="", fail_level="fail"):
        self.add("pass" if ok else fail_level, name, detail)
        return ok

    def summary(self):
        c = self.counts
        print(f"\n{c['pass']} passed, {c['fail']} failed, {c['warn']} warnings")
        return 1 if c["fail"] else 0


def num(values, key, default=0.0):
    try:
        return float(values[key])
    except (KeyError, ValueError):
        return default


def ask(question):
    """Yes/no question; True for yes."""
    return input(f"    {question} [y/n] ").strip().lower().startswith("y")


def poll(board, command, seconds, interval=0.1):
    """Repeat a command for a while; return every reply."""
    samples = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        samples.append(board.command(command))
        time.sleep(interval)
    return samples


# ---- Checks common to both boards -------------------------------------------


def check_common(board, report):
    info = board.info
    report.add("info", "Firmware", f"built {info.get('build', '?')}, protocol v{info.get('proto', '?')}, "
               f"channel {info.get('channel', '?')}")
    report.add("info", "MAC address", f"{info.get('mac', '?')} (this board)")
    if "temp_c" in info:
        report.add("info", "Chip temperature sensor", f"{num(info, 'temp_c'):.0f} C (uncalibrated)")

    reason = info.get("reset_reason", "unknown")
    bad_resets = {"crash", "interrupt_watchdog", "task_watchdog", "watchdog", "brownout"}
    if reason in bad_resets:
        hint = " - check the power supply" if reason == "brownout" else " - firmware fault"
        report.add("fail", "Last reset", reason + hint)
    else:
        report.add("pass", "Last reset", reason)

    heap = num(info, "heap_min")
    report.check(heap > 50_000, "Free memory", f"{heap / 1024:.0f} kB minimum free")
    report.check(info.get("radio_ok") == "1", "Radio start-up",
                 "long-range mode running" if info.get("radio_ok") == "1" else "failed to start")

    other = "controller" if board.kind == "boat" else "boat"
    if info.get("peer") == ZERO_MAC:
        report.add("warn", "Peer address", f"not set - put the {other}'s MAC in this board's config.h")
    else:
        report.add("pass", "Peer address", f"{info.get('peer')} ({other})")


# ---- Boat --------------------------------------------------------------------


def check_boat(board, report, args):
    report.section(f"Boat on {board.port}")
    check_common(board, report)

    # Compass
    found = board.command("i2c", timeout=6.0).get("found", "")
    report.add("info", "I2C devices", found or "none")
    compass = board.command("compass")
    if compass.get("chip") == "none":
        report.add("fail", "Compass", "no HMC5883L (0x1e) or QMC5883L (0x0d) answered - check wiring")
    else:
        report.check(compass.get("ok") == "1", "Compass", f"{compass['chip']} "
                     + ("reading" if compass.get("ok") == "1" else "found but not reading"))
        report.check(compass.get("calibrated") == "1", "Compass calibration",
                     "stored" if compass.get("calibrated") == "1"
                     else "not calibrated - use the controller's Nav page", fail_level="warn")
        if args.interactive:
            compass_turn_test(board, report)

    # GPS: watch the counters for a few seconds.
    before = board.command("gps")
    time.sleep(3.0)
    after = board.command("gps")
    new_bytes = num(after, "bytes") - num(before, "bytes")
    new_sentences = num(after, "sentences") - num(before, "sentences")
    new_errors = num(after, "checksum_errors") - num(before, "checksum_errors")
    if new_bytes == 0:
        report.add("fail", "GPS data", "nothing received - check wiring (module TX to GPIO16) and power")
    elif new_sentences == 0:
        report.add("fail", "GPS data", f"{new_bytes:.0f} bytes but no valid sentences - wrong baud rate or noise")
    else:
        report.add("pass", "GPS data", f"{new_sentences / 3:.1f} sentences per second")
        report.check(new_errors == 0, "GPS checksums", f"{new_errors:.0f} errors in 3 s", fail_level="warn")
        if after.get("fix") == "1":
            report.add("pass", "GPS fix", f"{after.get('sats')} satellites, "
                       f"{num(after, 'lat_e7') / 1e7:.6f}, {num(after, 'lon_e7') / 1e7:.6f}")
        else:
            report.add("warn", "GPS fix", "no fix yet (normal indoors or just after power-up)")

    # Battery
    battery = board.command("battery")
    mv = num(battery, "battery_mv")
    report.check(mv >= num(battery, "min_mv"), "Battery voltage",
                 f"{mv / 1000:.2f} V (minimum {num(battery, 'min_mv') / 1000:.2f} V)")
    if args.interactive and mv > 1000:
        text = input("    Battery voltage on your multimeter (V), or Enter to skip: ").strip()
        if text:
            try:
                suggested = num(battery, "cal", 1.0) * float(text) * 1000 / mv
                report.add("info", "Battery calibration", f"set BATT_CAL to {suggested:.3f} in the boat's config.h")
            except ValueError:
                report.add("warn", "Battery calibration", "could not read that number")

    # Navigation readiness and link
    status = board.command("status")
    waypoints = int(num(status, "waypoints"))
    if waypoints == 0:
        report.add("warn", "Route", "no waypoints in config.h - Route mode is disabled")
    else:
        report.add("info", "Route", f"{waypoints} waypoints, "
                   + ("accepted" if status.get("route_ok") == "1" else "not yet checked against home"))
    low = num(status, "set_low_battery_mv")
    fence = num(status, "set_geofence_m")
    report.add("info", "Settings", f"trim {num(status, 'set_trim'):+.0f}%, cruise {num(status, 'set_cruise') / 10:.0f}%, "
               f"low-battery return {'off' if low == 0 else f'{low / 1000:.2f} V'}, "
               f"geofence {'off' if fence == 0 else f'{fence:.0f} m'}")
    report.add("info", "Controller link", "linked" if status.get("linked") == "1" else "no controller heard")

    if args.motors:
        motor_test(board, report, args)


def compass_turn_test(board, report):
    print("    Turn the boat slowly through a full circle, level. Sampling for 20 seconds...")
    samples = poll(board, "compass", 20.0)
    xs = [num(s, "raw_x") for s in samples]
    ys = [num(s, "raw_y") for s in samples]
    span_x, span_y = max(xs) - min(xs), max(ys) - min(ys)
    if min(span_x, span_y) < 100:
        report.add("fail", "Compass turn test", f"readings barely changed (x {span_x:.0f}, y {span_y:.0f} counts)")
        return
    centre_x, centre_y = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    radii = [math.hypot((x - centre_x) / span_x, (y - centre_y) / span_y) for x, y in zip(xs, ys)]
    mean = sum(radii) / len(radii)
    spread = math.sqrt(sum((r - mean) ** 2 for r in radii) / len(radii)) / mean
    # A clean turn traces an ellipse: every point the same scaled distance from the centre.
    report.check(spread < 0.10, "Compass turn test",
                 f"offset {centre_x:.0f}, {centre_y:.0f} counts; roundness error {spread * 100:.0f}%"
                 + ("" if spread < 0.10 else " - interference, or the boat was tilted"), fail_level="warn")


def motor_test(board, report, args):
    print("\n    MOTOR TEST: the propellers will turn. Keep clear.")
    if not ask("Run each motor now?"):
        report.add("info", "Motor test", "skipped")
        return
    for side, name in (("L", "Left"), ("R", "Right")):
        for power, direction in ((300, "ahead"), (-300, "astern")):
            label = f"{name} motor {direction}"
            rest = num(board.command("battery"), "now_mv")
            try:
                board.command(f"motor {side} {power} 1500")
            except DiagError as error:
                report.add("fail", label, str(error))
                return
            time.sleep(0.7)
            loaded = [num(board.command("battery"), "now_mv") for _ in range(5)]
            time.sleep(1.5)
            drop = rest - sum(loaded) / len(loaded)
            detail = f"battery dropped {drop:.0f} mV"
            if args.interactive:
                report.check(ask(f"Did the {name.lower()} motor run {direction}?"), label, detail)
            else:
                report.add("info", label, detail + " (watch that it turned the right way)")


# ---- Controller --------------------------------------------------------------


def check_controller(board, report, args):
    report.section(f"Controller on {board.port}")
    check_common(board, report)
    report.check(board.info.get("display_ok") == "1", "Display",
                 "SSD1306 found" if board.info.get("display_ok") == "1"
                 else "not found at its I2C address - check wiring")

    # Joystick at rest
    samples = poll(board, "stick", 2.0, interval=0.05)
    first = samples[0]
    report.check(first.get("centre_suspect") == "0", "Joystick centre at power-up",
                 f"throttle {first.get('centre_throttle')}, steer {first.get('centre_steer')}"
                 + ("" if first.get("centre_suspect") == "0" else " - off-centre: stick held, or a pot is not wired"))
    for axis in ("throttle", "steer"):
        values = [num(s, f"raw_{axis}") for s in samples]
        noise = max(values) - min(values)
        mean = sum(values) / len(values)
        if mean < 20 or mean > 4075:
            report.add("fail", f"Joystick {axis} at rest", f"reads {mean:.0f} of 4095 - stuck at a rail, check wiring")
        else:
            report.check(noise < 80, f"Joystick {axis} at rest", f"reads {mean:.0f}, noise {noise:.0f} counts",
                         fail_level="warn")

    # Buttons at rest
    buttons = board.command("buttons")
    stuck = [name for name in ("next", "select", "boot") if buttons.get(name) == "1"]
    report.check(not stuck, "Buttons at rest", "all released" if not stuck
                 else f"reading as pressed: {', '.join(stuck)} - shorted or held")

    # Beeper and display: the board can only be told to drive them.
    try:
        board.command("beep 2700 300")
        time.sleep(0.4)
        if args.interactive:
            report.check(ask("Did you hear a beep?"), "Beeper")
        else:
            report.add("info", "Beeper", "tone sent (listen for it)")
    except DiagError as error:
        report.add("fail", "Beeper", str(error))
    if board.info.get("display_ok") == "1":
        board.command("display")
        if args.interactive:
            report.check(ask("Does the display show a border, a cross and a solid block?"), "Display pattern")
        else:
            report.add("info", "Display pattern", "shown for 3 seconds (border, cross, solid block)")

    if args.interactive:
        stick_range_test(board, report)
        button_press_test(board, report)

    status = board.command("status")
    my_mv = num(status, "battery_mv")
    if my_mv == 0:
        report.add("info", "Controller battery", "not measured (monitor disabled, or radio not running)")
    else:
        report.check(my_mv >= num(status, "battery_warn_mv"), "Controller battery", f"{my_mv / 1000:.2f} V",
                     fail_level="warn")
    report.add("info", "Speed limit", f"{status.get('speed_limit_pct')}%")
    report.add("info", "Boat link", f"linked, {status.get('ack_pct')}% acknowledged"
               if status.get("linked") == "1" else "boat not heard")


def stick_range_test(board, report):
    print("    Move the joystick slowly round its full range, into every corner. Sampling for 10 seconds...")
    samples = poll(board, "stick", 10.0, interval=0.03)
    lows, highs = [], []
    for axis in ("throttle", "steer"):
        values = [num(s, f"raw_{axis}") for s in samples]
        lows.append(min(values))
        highs.append(max(values))
        report.check(max(values) - min(values) > 2000, f"Joystick {axis} travel",
                     f"{min(values):.0f} to {max(values):.0f}")
    # The firmware uses one range for both axes: take the travel both can reach, less a margin.
    report.add("info", "Joystick range", f"suggested JOY_RAW_MIN {max(lows) + 50:.0f}, JOY_RAW_MAX {min(highs) - 50:.0f} "
               f"(now {samples[0].get('raw_min')}, {samples[0].get('raw_max')})")


def button_press_test(board, report):
    for key, name in (("next", "NEXT"), ("select", "SELECT")):
        print(f"    Press the {name} button (10 seconds)...")
        deadline = time.monotonic() + 10
        seen = False
        while time.monotonic() < deadline and not seen:
            seen = board.command("buttons").get(key) == "1"
            time.sleep(0.03)
        report.check(seen, f"{name} button", "press detected" if seen else "no press seen")


# ---- Both boards: the radio link ------------------------------------------------


def check_link(boat, controller, report, seconds):
    report.section("Radio link (both boards)")
    same_proto = boat.info.get("proto") == controller.info.get("proto")
    report.check(same_proto, "Protocol versions", f"boat v{boat.info.get('proto')}, controller v{controller.info.get('proto')}"
                 + ("" if same_proto else " - reflash both boards from the same source"))
    report.check(boat.info.get("channel") == controller.info.get("channel"), "Radio channels",
                 f"boat {boat.info.get('channel')}, controller {controller.info.get('channel')}")
    pair_a = controller.info.get("peer", "").upper() == boat.info.get("mac", "").upper()
    pair_b = boat.info.get("peer", "").upper() == controller.info.get("mac", "").upper()
    report.check(pair_a, "Controller is addressed to this boat",
                 "" if pair_a else f"BOAT_MAC should be {boat.info.get('mac')}")
    report.check(pair_b, "Boat is addressed to this controller",
                 "" if pair_b else f"CONTROLLER_MAC should be {controller.info.get('mac')}")
    if not (same_proto and pair_a and pair_b):
        report.add("info", "Link traffic test", "skipped until the settings above are fixed")
        return

    print(f"    Measuring the link for {seconds} seconds...")
    c0, b0 = controller.command("status"), boat.command("status")
    time.sleep(seconds)
    c1, b1 = controller.command("status"), boat.command("status")
    sent = num(c1, "tx_total") - num(c0, "tx_total")
    acked = num(c1, "ack_total") - num(c0, "ack_total")
    received = num(b1, "rx_total") - num(b0, "rx_total")
    telemetry_out = num(b1, "telemetry_sent") - num(b0, "telemetry_sent")
    telemetry_in = num(c1, "telemetry_total") - num(c0, "telemetry_total")
    if sent == 0:
        report.add("fail", "Control packets", "the controller sent nothing")
        return
    report.check(acked / sent >= 0.95, "Control packets acknowledged", f"{acked:.0f} of {sent:.0f} ({acked / sent:.0%})")
    report.check(received / sent >= 0.95, "Control packets received by boat",
                 f"{received:.0f} of {sent:.0f} ({received / sent:.0%})")
    if telemetry_out:
        telemetry_in = min(telemetry_in, telemetry_out)   # the two counters are read a moment apart
        report.check(telemetry_in / telemetry_out >= 0.90, "Telemetry received by controller",
                     f"{telemetry_in:.0f} of {telemetry_out:.0f}")
    else:
        report.add("fail", "Telemetry", "the boat sent none")
    report.add("info", "Signal strength", f"{b1.get('rssi')} dBm at the boat, {c1.get('rssi_here')} dBm at the controller")

    # Transmit power control: at close range both ends should have backed off from
    # full power without losing packets. The link test above has usually given it
    # time already; if not, wait a little for the first step down.
    def reduced(c, b):
        return num(b, "tx_power_dbm") < TX_POWER_FULL_DBM and num(c, "tx_power_dbm") < TX_POWER_FULL_DBM

    close = num(b1, "rssi") > CLOSE_RANGE_DBM and num(c1, "rssi_here") > CLOSE_RANGE_DBM
    deadline = time.monotonic() + 8
    while close and not reduced(c1, b1) and time.monotonic() < deadline:
        time.sleep(1)
        c1, b1 = controller.command("status"), boat.command("status")
    detail = (f"boat {num(b1, 'tx_power_dbm'):.1f} dBm, controller {num(c1, 'tx_power_dbm'):.1f} dBm "
              f"(full power is {TX_POWER_FULL_DBM})")
    # Once power has come down the received signal is weaker too, so "reduced"
    # is judged first: it is proof the control is working whatever the range.
    if reduced(c1, b1):
        report.add("pass", "Transmit power control", detail)
    elif close:
        report.add("warn", "Transmit power control", detail + " - not reduced although the boards are close")
    else:
        report.add("info", "Transmit power control", detail + " - boards too far apart to expect a reduction")
    for name, status in (("Boat", b1), ("Controller", c1)):
        cap = num(status, "tx_power_cap_dbm", TX_POWER_FULL_DBM)
        if cap < TX_POWER_FULL_DBM:
            report.add("warn", f"{name} is hot", f"transmit power capped at {cap:.1f} dBm")


# ---- Main ----------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="Diagnostics for the RC boat and controller.")
    parser.add_argument("--port", action="append", help="serial port (repeat for two boards); default: find them")
    parser.add_argument("--interactive", action="store_true", help="add checks that ask you to do or confirm things")
    parser.add_argument("--motors", action="store_true", help="run each boat motor briefly (propellers turn!)")
    parser.add_argument("--link-seconds", type=float, default=10.0, help="length of the radio link test")
    args = parser.parse_args()

    ports = args.port or [p.device for p in list_ports.comports() if p.vid in ESP_USB_VENDORS]
    if not ports:
        print("No ESP32-style USB serial ports found. Plug a board in, or name one with --port.")
        return 2
    wait = 10.0 if args.port else 6.0   # a port you named gets longer to answer

    report = Report()
    boards = {}
    for port in ports:
        try:
            board = Board(port)
        except serial.SerialException as error:
            print(f"{port}: cannot open ({error})")
            continue
        if not board.identify(wait):
            print(f"{port}: no boat or controller firmware answered")
            continue
        if board.kind in boards:
            print(f"{port}: a second {board.kind} - ignored")
            continue
        boards[board.kind] = board

    if not boards:
        print("No boards answered.")
        return 2

    def run(what, check, *check_args):
        """One group of checks. A failure here must not stop the other groups."""
        try:
            check(*check_args)
        except (DiagError, serial.SerialException) as error:
            report.add("fail", f"{what} checks stopped", str(error))

    if "boat" in boards:
        run("Boat", check_boat, boards["boat"], report, args)
    if "controller" in boards:
        run("Controller", check_controller, boards["controller"], report, args)
    if len(boards) == 2:
        run("Link", check_link, boards["boat"], boards["controller"], report, args.link_seconds)
    else:
        missing = "controller" if "boat" in boards else "boat"
        print(f"\n(Connect the {missing} as well to test the radio link.)")
    return report.summary()


if __name__ == "__main__":
    sys.exit(main())
