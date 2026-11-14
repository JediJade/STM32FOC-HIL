#!/usr/bin/env python3
"""
HIL BENCH - the instrument for the motor that is not there.

    python hil.py                      pick the ports in the window
    python hil.py --hil COM13 --drive COM4
    python hil.py --model              no hardware: a demo of the window

Two boards sit on this bench. The DRIVE runs the motor control firmware and
believes it is turning a motor. The HIL board measures the drive's six
bridge pins, runs the motor model, and drives the sensor signals back. This
window shows both of them at once, in units a person can read, and says
plainly whether they agree.

Nothing here can be damaged: there is no power stage, no battery and no
motor. Six wires between two microcontrollers.
"""

import json
import math
import os
import sys
import time
import tkinter as tk
from tkinter import filedialog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import braun as B          # noqa: E402
import serialport          # noqa: E402

W, H = 1280, 820
RAIL = 232
X0 = RAIL + 36
X1 = W - 36
POLL_HZ = 20
TRACE_SECONDS = 30

DRIVE_PERIOD = 1800          # the drive's TIM1 ARR
UNITS_PER_TURN = 65536.0


# --------------------------------------------------------------- settings --

DEFAULT_SETUP = {
    "name": "default bench motor",
    "bus_voltage": 24.0,        # V across the bridge
    "current_full_scale": 16.0,  # A at a Q15 count of 32767
    "pole_pairs": 7,
    "torque_constant": 0.035,   # N.m per amp of q-axis current
    "hall_width_deg": 180.0,    # how much of a turn one Hall sensor is on
    "preset": 1,                # 0 gimbal, 1 hub
}

SETUP_HELP = {
    "bus_voltage": "DC voltage the bridge would switch. Scales every volt shown.",
    "current_full_scale": "Current at full Q15 scale. Scales every amp shown.",
    "pole_pairs": "Electrical turns per mechanical turn. Sets shaft rpm.",
    "torque_constant": "Newton metres per amp of q-axis current.",
    "hall_width_deg": "Degrees of a turn one Hall sensor stays on. 180 is a latch.",
    "preset": "Which motor the model is: 0 gimbal, 1 hub.",
}


class Units:
    """Raw firmware counts in, engineering units out."""

    def __init__(self, setup=None):
        self.setup = dict(DEFAULT_SETUP)
        if setup:
            self.setup.update(setup)

    # ---- speed
    def rpm_elec(self, units_per_ms):
        return (units_per_ms or 0) * 60000.0 / UNITS_PER_TURN

    def rpm_shaft(self, units_per_ms):
        pp = max(1, int(self.setup["pole_pairs"]))
        return self.rpm_elec(units_per_ms) / pp

    def hz_elec(self, units_per_ms):
        return self.rpm_elec(units_per_ms) / 60.0

    # ---- current
    def amps(self, q15):
        return (q15 or 0) * self.setup["current_full_scale"] / 32768.0

    @staticmethod
    def vector(ia, ib):
        """How much current there is, from two of the three phases.

        Not hypot(ia, ib). Those two are 120 degrees apart, not 90, so
        their hypotenuse swings by half its own size as the rotor turns
        and is not a measure of anything. The Clarke pair is the pair at
        right angles: alpha is phase a, beta is the other two combined.
        Its length is the amplitude, and it stands still.
        """
        ia, ib = ia or 0, ib or 0
        beta = (ia + 2.0 * ib) / 1.7320508075688772
        return math.hypot(ia, beta)

    # ---- voltage
    def volts_phase(self, duty):
        """A compare value of 0..1800 is a phase voltage about the mid rail."""
        return ((duty or 0) / DRIVE_PERIOD - 0.5) * self.setup["bus_voltage"]

    def volts_from_q15(self, q15):
        return (q15 or 0) / 32768.0 * self.setup["bus_voltage"] / 2.0

    # ---- torque
    def torque(self, iq_q15):
        return self.amps(iq_q15) * self.setup["torque_constant"]


# ------------------------------------------------------------------ links --

class Board:
    """One console on one port, polled from the window's tick."""

    def __init__(self, port, hello):
        self.port = port
        self.hello = hello
        self.ser = serialport.Serial(port)
        self.state = {}
        self.log = []
        self.error = None
        self.version = None
        self._buf = ""
        self.send(hello)

    def send(self, text):
        try:
            self.ser.write_line(text)
            self.log.append("> " + text)
            del self.log[:-60]
        except OSError as exc:
            self.error = str(exc)

    def poll(self):
        try:
            chunk = self.ser.read()
        except OSError as exc:
            self.error = str(exc)
            return
        if not chunk:
            return
        self._buf += chunk.decode("ascii", "replace")
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            self._take(line.strip())

    def _take(self, line):
        if not line:
            return
        self.log.append(line)
        del self.log[:-60]
        parts = line.split()
        if not parts:
            return
        if parts[0] in ("hil", "fw"):
            self.version = line
            return
        # the drive's state line opens with `ang`, with no word in front
        if parts[0] == "ang":
            parts = ["s"] + parts
        if parts[0] in ("s", "v", "x", "d", "can", "hall", "sto"):
            pairs = dict(zip(parts[1::2], parts[2::2]))
            for key, value in pairs.items():
                try:
                    self.state[key] = int(value)
                except ValueError:
                    self.state[key] = value

    def close(self):
        try:
            self.ser.close()
        except OSError:
            pass


class FakeBoard:
    """No hardware: enough motion to see the window work."""

    error = None

    FREE_SPEED = 2200.0
    # On the class, not the instance: the drive and the bench are two
    # windows onto one imaginary rig, and a load put on the shaft is on
    # the shaft both of them are looking at.
    load = 0
    lock = 0

    def __init__(self, kind):
        self.kind = kind
        self.port = "model"
        self.version = "hil 1 (model)" if kind == "hil" else "fw 4 (model)"
        self.log = ["-- no board: these numbers are made up in this window --"]
        self.state = {}
        self._t = time.perf_counter()
        self._ang = 0.0

    def send(self, text):
        self.log.append("> " + text)
        # The load controls are the ones worth answering without a board:
        # the point of them is watching the speed sag and the current
        # rise, and that can be shown in arithmetic.
        parts = text.split()
        if len(parts) == 3 and parts[0] == "set":
            if parts[1] == "load":
                FakeBoard.load = max(0, int(parts[2]))
            elif parts[1] == "lock":
                FakeBoard.lock = 1 if parts[2] != "0" else 0

    def poll(self):
        now = time.perf_counter()
        dt, self._t = now - self._t, now
        # A drag the controller is still winning against costs a little
        # speed and a lot of current; past about 25 it wins instead.
        spd = 0.0 if self.lock else max(0.0, self.FREE_SPEED - 62.0 * self.load)
        self._ang = (self._ang + spd * dt * 1000.0) % UNITS_PER_TURN
        th = self._ang / UNITS_PER_TURN * 2 * math.pi
        self.state.update({
            "ang": int(self._ang), "spd": int(spd),
            "ia": int(min(9000, 1500 + 290 * self.load) * math.sin(th)),
            "ib": int(min(9000, 1500 + 290 * self.load) * math.sin(th - 2.094)),
            "iq": int(min(9000, 1500 + 290 * self.load)), "id": -20,
            "moe": 1, "live": 1, "ovl": 0, "late": 0,
            "load": self.load, "lock": self.lock,
            "hall": (0b101, 0b100, 0b110, 0b010, 0b011, 0b001)[
                int(self._ang / UNITS_PER_TURN * 6) % 6],
            "per": 3598,
            "da": int(900 + 700 * math.sin(th)),
            "db": int(900 + 700 * math.sin(th - 2.094)),
            "dc": int(900 + 700 * math.sin(th + 2.094)),
            "vd": -200, "vq": 12000,
        })

    def close(self):
        pass


# -------------------------------------------------------------- recording --

class Recorder:
    """Samples kept for the scope, and written out as a spreadsheet."""

    COLUMNS = ("t_s", "duty_a", "duty_b", "duty_c", "period_ticks",
               "hil_speed_units_ms", "hil_angle", "hall",
               "ia_counts", "ib_counts", "drive_speed_units_ms",
               "drive_iq_counts", "load", "lock", "overlaps", "late")

    def __init__(self, seconds=TRACE_SECONDS, rate=POLL_HZ):
        self.limit = int(seconds * rate)
        self.rows = []
        self.t0 = None
        self.on = False

    def start(self):
        self.rows = []
        self.t0 = time.perf_counter()
        self.on = True

    def stop(self):
        self.on = False

    def add(self, hil, drive):
        if self.t0 is None:
            self.t0 = time.perf_counter()
        row = (round(time.perf_counter() - self.t0, 3),
               hil.get("da"), hil.get("db"), hil.get("dc"), hil.get("per"),
               hil.get("spd"), hil.get("ang"), hil.get("hall"),
               hil.get("ia"), hil.get("ib"),
               drive.get("spd"), drive.get("iq"),
               hil.get("load"), hil.get("lock"),
               hil.get("ovl"), hil.get("late"))
        self.rows.append(row)
        if not self.on and len(self.rows) > self.limit:
            del self.rows[:-self.limit]

    def column(self, name):
        i = self.COLUMNS.index(name)
        return [r[i] for r in self.rows if r[i] is not None]

    def write_csv(self, path):
        with open(path, "w", encoding="utf-8", newline="") as fh:
            fh.write(",".join(self.COLUMNS) + "\n")
            for row in self.rows:
                fh.write(",".join("" if v is None else str(v) for v in row) + "\n")
        return len(self.rows)


# ------------------------------------------------------------- the checks --

class Check:
    """One thing that can be right or wrong, in a sentence."""

    def __init__(self, key, title, explain, wrong=None):
        self.key = key
        self.title = title
        self.explain = explain
        # The title is the good news, which is what a row of green lamps
        # should read as. A summary of what is failing needs the bad news
        # in its own words, or it says "something is wrong: both boards
        # agree".
        self.wrong = wrong or title.lower()
        self.ok = None
        self.detail = ""


def build_checks():
    return [
        Check("link", "HIL board answering",
              "The bench board is on its port and replying.",
              "the bench board is not answering"),
        Check("pwm", "Drive is switching",
              "Pulses are arriving on all three phase pins.",
              "the drive is not switching"),
        Check("period", "Carrier is 20 kHz",
              "Every pulse period measures 50 us, as the drive intends.",
              "the carrier is not 20 kHz"),
        Check("bridge", "Bridge pattern safe",
              "No sample has caught a phase with both transistors on. "
              "On real hardware that is a short across the supply.",
              "a phase had both transistors on"),
        Check("realtime", "Model keeping up",
              "Every 50 us tick finished before the next one arrived.",
              "the model is not keeping up"),
        Check("hall", "Hall pattern valid",
              "Never all-on or all-off: those two mean a broken sensor.",
              "the hall pattern is impossible"),
        Check("sync", "Both boards agree",
              "The drive's speed and the model's true speed match, which "
              "means the loop is closed through the wires.",
              "the two boards do not agree"),
    ]


# --------------------------------------------------------------- the pages --

class Page:
    key = ""
    title = ""
    subtitle = ""

    def __init__(self, app):
        self.app = app
        self.cv = app.cv
        self.items = []

    def text(self, x, y, text="", size=13, color=B.INK, anchor="nw",
             num=False, width=None, face=None):
        item = self.cv.create_text(
            x, y, text=text, fill=color, anchor=anchor, width=width,
            font=(face, -size) if face else B.font(size, num=num))
        self.items.append(item)
        return item

    def rect(self, *args, **kw):
        item = self.cv.create_rectangle(*args, **kw)
        self.items.append(item)
        return item

    def image(self, *args, **kw):
        item = self.cv.create_image(*args, **kw)
        self.items.append(item)
        return item

    def chart(self, *args, **kw):
        """A chart draws a dozen canvas items; they all carry its tag, so
        one entry in the page's list takes the whole thing off again."""
        ch = B.Chart(self.cv, *args, **kw)
        self.items.append(ch.tag)
        return ch

    def button(self, x, y, d, colour, fn, label):
        btn = B.RoundButton(self.cv, x, y, d, colour, fn, label=label,
                            label_side="e", label_size=12, label_color=B.INK2)
        self.items.append(btn.tag)
        return btn

    def set(self, item, text, colour=None):
        self.cv.itemconfigure(item, text=text)
        if colour is not None:
            self.cv.itemconfigure(item, fill=colour)

    def build(self):
        pass

    def update(self):
        pass

    def destroy(self):
        for item in self.items:
            self.cv.delete(item)
        self.items = []


class OverviewPage(Page):
    key, title = "overview", "overview"
    subtitle = "what the bench is doing, and whether anything is wrong"

    def build(self):
        cv = self.cv
        self.lamps = []
        self.checks = self.app.checks
        self.text(X0, 120, "BENCH STATUS", 12, B.INK3)
        for i, chk in enumerate(self.checks):
            y = 150 + i * 46
            self.rect(X0, y - 6, X1 - 330, y + 36, fill=B.PANEL, width=0)
            lamp = B.Lamp(cv, X0 + 20, y + 14, "", 13, B.INK)
            self.items.append(lamp.item)
            self.items.append(lamp.text)
            title = self.text(X0 + 40, y + 2, chk.title, 15, B.INK)
            detail = self.text(X0 + 40, y + 20, "", 12, B.INK3,
                               width=X1 - 330 - X0 - 60)
            self.lamps.append((chk, lamp, title, detail))

        xa = X1 - 300
        self.text(xa, 120, "MOTOR, AS THE MODEL HAS IT", 12, B.INK3)
        self.big = {}
        rows = (("shaft", "shaft speed", "rpm"), ("elec", "electrical", "Hz"),
                ("torque", "torque", "N·m"), ("current", "current", "A rms"),
                ("volt", "phase A voltage", "V peak"))
        for i, (key, label, unit) in enumerate(rows):
            y = 152 + i * 78
            self.text(xa, y, label, 12, B.INK3)
            self.big[key] = self.text(xa, y + 16, "–", 30, B.INK, num=True)
            self.text(xa + 2, y + 52, unit, 11, B.INK3)
        self.text(X0, 544, "SHAFT SPEED", 12, B.INK3)
        self.speed_chart = self.chart(X0, 564, X1 - 330 - X0, 212, unit="rpm")

        self.text(xa, 544, "LOAD ON THE SHAFT", 12, B.INK3)
        self.t_load = self.text(xa, 564, "\u2013", 30, B.INK, num=True)
        self.t_load_unit = self.text(xa + 2, 600, "drag", 11, B.INK3)
        self.button(xa + 24, 646, 34, B.DARK2, lambda: self.step(-1), "")
        self.text(xa + 24, 646, "\u2212", 20, B.WHITE, anchor="center")
        self.button(xa + 86, 646, 34, B.ORANGE, lambda: self.step(1), "")
        self.text(xa + 86, 646, "+", 20, B.WHITE, anchor="center")
        self.button(xa + 16, 702, 22, B.DARK2,
                    lambda: self.app.hil_send("set load 0"), "no load")
        self.button(xa + 16, 734, 22, B.DARK2, self.toggle_lock,
                    "clamp the shaft")
        self.text(xa, 762,
                  "Drag opposes the rotor whichever way it turns. Raise it "
                  "and watch the speed dip and the current answer; raise it "
                  "far enough and the drive runs out of torque.",
                  11, B.INK3, width=300)

        self.text(X0, 150 + len(self.checks) * 46 + 16, "", 12, B.INK3)
        self.summary = self.text(X0, 150 + len(self.checks) * 46 + 20, "",
                                 13, B.ORANGE, width=X1 - 330 - X0)

    LADDER = (0, 1, 2, 3, 4, 6, 8, 11, 15, 20, 27, 36, 48, 64, 85, 110)

    def step(self, direction):
        """One rung at a time, and the rungs get further apart: the first
        few are what a bearing costs, the last are a brake."""
        st = self.app.hil.state if self.app.hil else {}
        now = st.get("load", 0) or 0
        rungs = self.LADDER
        if direction > 0:
            nxt = next((v for v in rungs if v > now), rungs[-1])
        else:
            nxt = next((v for v in reversed(rungs) if v < now), 0)
        self.app.hil_send(f"set load {nxt}")

    def toggle_lock(self):
        st = self.app.hil.state if self.app.hil else {}
        self.app.hil_send("set lock %d" % (0 if st.get("lock") else 1))

    def update(self):
        app = self.app
        for chk, lamp, title, detail in self.lamps:
            colour = (B.MUTE if chk.ok is None else
                      (B.GREEN if chk.ok else B.RED))
            lamp.set(colour)
            self.set(detail, chk.detail or chk.explain)
            self.set(title, chk.title, B.INK if chk.ok is not False else B.RED)

        u = app.units
        st = app.hil.state if app.hil else {}
        ds = app.drive.state if app.drive else {}
        spd = st.get("spd", 0)
        iq = ds.get("iq", 0)
        mag = u.vector(st.get("ia"), st.get("ib")) / 1.4142135623730951
        self.set(self.big["shaft"], f"{u.rpm_shaft(spd):,.0f}")
        self.set(self.big["elec"], f"{u.hz_elec(spd):,.1f}")
        self.set(self.big["torque"], f"{u.torque(iq):.3f}")
        self.set(self.big["current"], f"{u.amps(mag):.2f}")
        self.set(self.big["volt"], f"{u.volts_phase(st.get('da', 900)):.1f}")

        bad = [c for c in app.checks if c.ok is False]
        if bad:
            self.set(self.summary,
                     "Something is wrong: " + "; ".join(c.wrong for c in bad)
                     + ". The line beside each red lamp says more.", B.RED)
        elif all(c.ok for c in app.checks):
            self.set(self.summary, "Everything checks out. The drive is "
                                   "controlling a motor that lives in the "
                                   "other microcontroller.", B.INK2)
        else:
            self.set(self.summary, "Waiting for both boards and for the drive "
                                   "to start switching.", B.ORANGE)

        load = st.get("load")
        if load is None:
            self.set(self.t_load, "\u2013")
            self.set(self.t_load_unit, "this firmware has no load setting")
        elif st.get("lock"):
            self.set(self.t_load, "clamped", B.RED)
            self.set(self.t_load_unit, "the shaft is held still")
        else:
            self.set(self.t_load, str(load), B.ORANGE if load else B.INK)
            self.set(self.t_load_unit, "drag, 0 is a free shaft")

        rpm = [u.rpm_shaft(v) for v in
               app.rec.column("hil_speed_units_ms")[-600:]]
        if len(rpm) > 2:
            lo, hi, step = B.axis(min(rpm), max(rpm), 200.0)
            self.speed_chart.note("")
            self.speed_chart.draw([(rpm, B.ORANGE, 1.8, True)], lo, hi, step,
                                  seconds=len(rpm) / POLL_HZ)
        else:
            self.speed_chart.note("waiting for the first samples")


class SignalsPage(Page):
    key, title = "signals", "signals"
    subtitle = "the six bridge pins, as this board measures them"

    def build(self):
        cv = self.cv
        self.text(X0, 120, "PHASE DUTY, MEASURED ON THE DRIVE'S PINS", 12, B.INK3)
        self.bars = []
        for i, name in enumerate("ABC"):
            y = 152 + i * 56
            self.text(X0, y + 6, f"phase {name}", 13, B.INK2)
            self.rect(X0 + 90, y, X0 + 90 + 420, y + 26, fill=B.SLOT, width=0)
            bar = self.rect(X0 + 90, y, X0 + 90, y + 26, fill=B.ORANGE, width=0)
            val = self.text(X0 + 530, y - 2, "–", 17, B.INK, num=True)
            volt = self.text(X0 + 530, y + 20, "", 12, B.INK3)
            self.bars.append((bar, val, volt))

        self.text(X0, 330, "CARRIER", 12, B.INK3)
        self.carrier = self.text(X0, 350, "–", 22, B.INK, num=True)
        self.carrier_note = self.text(X0, 384, "", 12, B.INK3, width=380)

        self.text(X0 + 420, 330, "DEAD TIME ADDED BACK", 12, B.INK3)
        self.dead = self.text(X0 + 420, 350, "–", 22, B.INK, num=True)
        self.text(X0 + 420, 384,
                  "The pin is high for one dead-time less than the drive "
                  "asked for. The model is given back what the controller "
                  "intended.", 12, B.INK3, width=330)

        self.text(X0, 450, "HALL OUTPUTS, DRIVEN BACK TO THE DRIVE", 12, B.INK3)
        self.hall_lamps = []
        for i, (pin, drive_pin) in enumerate((("PB12", "PB5"), ("PB13", "PB6"),
                                              ("PB14", "PB7"))):
            lamp = B.Lamp(self.cv, X0 + 16, 486 + i * 32,
                          f"{pin}  →  drive {drive_pin}", 13, B.INK2)
            self.items.append(lamp.item)
            self.items.append(lamp.text)
            self.hall_lamps.append(lamp)
        self.hall_note = self.text(X0 + 300, 486, "", 12, B.INK3, width=300)

        self.text(X1 - 300, 450, "BRIDGE SAFETY", 12, B.INK3)
        self.ovl = self.text(X1 - 300, 470, "–", 26, B.INK, num=True)
        self.text(X1 - 300, 506,
                  "samples that caught a phase with both transistors on. "
                  "Must stay at zero.", 12, B.INK3, width=280)
        self.button(X1 - 284, 580, 26, B.DARK2,
                    lambda: self.app.hil_send("set ovl 0"), "reset the counter")

        self.text(X0, 610, "FORCE A HALL PATTERN (for checking the wires)",
                  12, B.INK3)
        for i, code in enumerate((0, 1, 2, 4, 7)):
            self.button(X0 + 16 + i * 92, 646, 26, B.DARK2,
                        lambda c=code: self.app.hil_send(f"set hall {c}"),
                        f"{code:03b}")
        self.button(X0 + 16 + 5 * 92, 646, 26, B.GREEN,
                    lambda: self.app.hil_send("set hall -1"), "back to model")

    def update(self):
        st = self.app.hil.state if self.app.hil else {}
        u = self.app.units
        for i, (bar, val, volt) in enumerate(self.bars):
            duty = st.get(("da", "db", "dc")[i], 0) or 0
            frac = max(0.0, min(1.0, duty / DRIVE_PERIOD))
            y = 152 + i * 56
            self.cv.coords(bar, X0 + 90, y, X0 + 90 + int(420 * frac), y + 26)
            self.set(val, f"{duty}   {frac * 100:.1f}%")
            self.set(volt, f"{u.volts_phase(duty):+.2f} V about the mid rail")

        per = st.get("per", 0) or 0
        if per:
            khz = 72000.0 / per
            self.set(self.carrier, f"{khz:.2f} kHz")
            self.set(self.carrier_note,
                     f"{per} timer ticks per period = {per / 72.0:.2f} us. "
                     f"The drive builds this from a 72 MHz clock.")
        else:
            self.set(self.carrier, "–")
            self.set(self.carrier_note, "No pulses: the drive is not switching.")
        self.set(self.dead, "1.000 us")

        hall = st.get("hall", 0) or 0
        for i, lamp in enumerate(self.hall_lamps):
            lamp.set(B.GREEN if (hall >> i) & 1 else None)
        if hall in (0, 7):
            self.set(self.hall_note,
                     f"code {hall:03b} - all on or all off. Three working "
                     f"sensors never do that.", B.RED)
        else:
            self.set(self.hall_note,
                     f"code {hall:03b}. One of the six states a working set "
                     f"of sensors produces.", B.INK3)
        ovl = st.get("ovl", 0) or 0
        self.set(self.ovl, str(ovl), B.RED if ovl else B.INK)


class ScopePage(Page):
    key, title = "scope", "scope"
    subtitle = "what the bench has been doing, sampled 20 times a second"

    CHANNELS = (("hil_speed_units_ms", "model speed", "units/ms", B.ORANGE),
                ("duty_a", "phase A duty", "of 1800", B.ORANGE),
                ("ia_counts", "phase A current", "Q15 counts", B.YELLOW),
                ("drive_speed_units_ms", "drive's measured speed",
                 "units/ms", B.GREEN),
                ("load", "drag on the shaft", "units", B.RED))

    def build(self):
        self.channel = 0
        self.scope = self.chart(X0, 130, X1 - X0 - 300, 300,
                                unit=self.CHANNELS[0][2])
        self.readout = self.text(X0, 448, "", 13, B.INK2, width=X1 - X0 - 320)
        self.text(X0, 500, "ALIASING", 12, B.INK3)
        self.text(X0, 520,
                  "This window asks the board 20 times a second, so anything "
                  "faster than 10 Hz is not drawn as it happens. Phase "
                  "currents swing at the electrical frequency - tens or "
                  "hundreds of hertz - so the trace of a current is a sample "
                  "of it, not a picture of it. Speed and duty move slowly "
                  "enough to be read directly.", 12, B.INK3, width=X1 - X0 - 320)

        xa = X1 - 290
        self.text(xa, 120, "CHANNEL", 12, B.INK3)
        self.b_chan = []
        for i, (_key, label, unit, _c) in enumerate(self.CHANNELS):
            self.b_chan.append(self.button(
                xa + 16, 152 + i * 40, 24, B.DARK2,
                lambda k=i: self.pick(k), f"{label}  ({unit})"))
        self.text(xa, 330, "RECORDING", 12, B.INK3)
        self.b_rec = self.button(xa + 16, 362, 28, B.RED, self.toggle, "record")
        self.button(xa + 16, 404, 26, B.DARK2, self.app.export_csv,
                    "export to a spreadsheet")
        self.rec_note = self.text(xa, 440, "", 12, B.INK3, width=270)

    def pick(self, index):
        self.channel = index

    def toggle(self):
        rec = self.app.rec
        if rec.on:
            rec.stop()
        else:
            rec.start()

    def update(self):
        key, label, unit, colour = self.CHANNELS[self.channel]
        for i, b in enumerate(self.b_chan):
            b.set_color(B.ORANGE if i == self.channel else B.DARK2)
            b.set_label(color=B.INK if i == self.channel else B.INK2)
        self.scope.unit(unit)
        data = self.app.rec.column(key)[-600:]
        if len(data) > 2:
            lo, hi, step = B.axis(min(data), max(data), 40)
            self.scope.note("")
            self.scope.draw([(data, colour, 1.8, True)], lo, hi, step,
                            seconds=len(data) / POLL_HZ)
            mean = sum(data) / len(data)
            self.set(self.readout,
                     f"{label}:  now {data[-1]:,.0f} {unit}   "
                     f"mean {mean:,.0f}   min {min(data):,.0f}   "
                     f"max {max(data):,.0f}   "
                     f"swing {max(data) - min(data):,.0f}")
        else:
            self.scope.clear()
            self.scope.note("Waiting for samples.")
            self.set(self.readout, "")
        rec = self.app.rec
        self.b_rec.set_color(B.GREEN if rec.on else B.RED)
        self.b_rec.set_label(text="recording" if rec.on else "record")
        self.set(self.rec_note,
                 f"{len(rec.rows)} samples held"
                 + (" and still recording." if rec.on
                    else f", the last {TRACE_SECONDS} seconds."))


class MotorPage(Page):
    key, title = "motor", "motor setup"
    subtitle = "what the numbers mean, and what the model is pretending to be"

    def build(self):
        self.rows = []
        self.text(X0, 120, "SCALING - these turn counts into units", 12, B.INK3)
        for i, key in enumerate(("bus_voltage", "current_full_scale",
                                 "pole_pairs", "torque_constant",
                                 "hall_width_deg", "preset")):
            y = 152 + i * 74
            self.rect(X0, y - 8, X1 - 300, y + 54, fill=B.PANEL, width=0)
            self.text(X0 + 18, y, key.replace("_", " "), 15, B.INK)
            self.text(X0 + 18, y + 24, SETUP_HELP[key], 12, B.INK3, width=440)
            entry = tk.Entry(self.cv, font=B.font(16, num=True), bd=0,
                             relief="flat", bg=B.PAPER, fg=B.INK, justify="right",
                             insertbackground=B.ORANGE, highlightthickness=1,
                             highlightbackground=B.LINE, highlightcolor=B.ORANGE)
            entry.insert(0, str(self.app.units.setup[key]))
            entry.bind("<Return>", lambda e, k=key: self.commit(k))
            win = self.cv.create_window(X1 - 480, y + 6, anchor="nw",
                                        window=entry, width=140, height=32)
            self.items.append(win)
            self.rows.append((key, entry))

        xa = X1 - 280
        self.text(xa, 120, "SETUP FILE", 12, B.INK3)
        self.button(xa + 16, 152, 28, B.DARK2, self.load, "import a setup")
        self.button(xa + 16, 196, 28, B.DARK2, self.save, "export this setup")
        self.note = self.text(xa, 240, "", 12, B.INK3, width=260)
        self.text(xa, 330, "APPLY TO THE BOARD", 12, B.INK3)
        self.button(xa + 16, 362, 26, B.ORANGE, self.apply_board,
                    "send motor and hall")
        self.text(xa, 400,
                  "The motor preset and the Hall width live in the HIL "
                  "board; everything else above is how this window reads "
                  "the numbers it is sent.", 12, B.INK3, width=260)

    def commit(self, key):
        for name, entry in self.rows:
            if name != key:
                continue
            try:
                value = float(entry.get())
            except ValueError:
                self.set(self.note, f"{key}: that is not a number.", B.RED)
                return
            if key in ("pole_pairs", "preset"):
                value = int(value)
            self.app.units.setup[key] = value
            self.set(self.note, f"{key} is now {value}.", B.INK3)

    def refresh_entries(self):
        for key, entry in self.rows:
            entry.delete(0, "end")
            entry.insert(0, str(self.app.units.setup[key]))

    def load(self):
        path = filedialog.askopenfilename(
            parent=self.app, title="Import a motor setup",
            filetypes=(("Motor setup", "*.json"),))
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError) as exc:
            self.set(self.note, f"could not read it: {exc}", B.RED)
            return
        known = {k: v for k, v in data.items() if k in DEFAULT_SETUP}
        self.app.units.setup.update(known)
        self.refresh_entries()
        self.set(self.note, f"loaded {len(known)} settings from "
                            f"{os.path.basename(path)}.", B.INK3)

    def save(self):
        path = filedialog.asksaveasfilename(
            parent=self.app, title="Export this motor setup",
            defaultextension=".json", initialfile="motor-setup.json",
            filetypes=(("Motor setup", "*.json"),))
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(self.app.units.setup, fh, indent=2)
        except OSError as exc:
            self.set(self.note, f"could not write it: {exc}", B.RED)
            return
        self.set(self.note, f"written to {os.path.basename(path)}.", B.INK3)

    def apply_board(self):
        setup = self.app.units.setup
        width = int(setup["hall_width_deg"] / 360.0 * UNITS_PER_TURN)
        self.app.hil_send(f"set mot {int(setup['preset'])}")
        self.app.hil_send(f"set hwid {width}")
        self.set(self.note, f"sent: preset {int(setup['preset'])}, "
                            f"hall width {width} units.", B.INK3)


class SyncPage(Page):
    key, title = "sync", "sync"
    subtitle = "the same quantity, measured on both boards, side by side"

    ROWS = (("speed", "speed", "units/ms"),
            ("rpm", "shaft speed", "rpm"),
            ("angle", "electrical angle", "units"),
            ("current", "q-axis current", "A"),
            ("voltage", "phase voltage amplitude", "V"))

    def build(self):
        self.text(X0, 120, "DRIVE", 12, B.INK3)
        self.text(X0 + 300, 120, "HIL BENCH", 12, B.INK3)
        self.text(X0 + 560, 120, "DIFFERENCE", 12, B.INK3)
        self.cells = {}
        for i, (key, label, unit) in enumerate(self.ROWS):
            y = 152 + i * 66
            self.rect(X0, y - 8, X1 - 300, y + 48, fill=B.PANEL, width=0)
            self.text(X0 + 18, y + 26, f"{label}   ({unit})", 12, B.INK3)
            self.cells[key] = (
                self.text(X0 + 18, y, "–", 24, B.INK, num=True),
                self.text(X0 + 300, y, "–", 24, B.INK, num=True),
                self.text(X0 + 560, y, "–", 20, B.INK2, num=True))

        self.verdict = self.text(X0, 500, "", 14, B.INK, width=X1 - 300 - X0)
        self.text(X0, 560, "WHY THIS MATTERS", 12, B.INK3)
        self.text(X0, 580,
                  "The drive works out its speed from the Hall edges arriving "
                  "on its pins. The HIL board knows the true speed of the "
                  "model that produced those edges. Neither board can see the "
                  "other's number. When the two agree, the loop really is "
                  "closed through the wires - and the same reasoning runs "
                  "backwards: the duty measured here, turned into volts, "
                  "should match the voltage the drive says it applied.",
                  12, B.INK3, width=X1 - 300 - X0)

        xa = X1 - 280
        self.text(xa, 120, "THE DRIVE'S ANGLE", 12, B.INK3)
        self.text(xa, 142,
                  "Until the drive is told to read its pins, it is steering "
                  "by a model of its own and the rotor here is only being "
                  "dragged along behind it.", 12, B.INK3, width=260)
        self.button(xa + 16, 214, 26, B.ORANGE, self.drive_to_pins,
                    "take the angle from the wires")
        self.button(xa + 16, 256, 24, B.DARK2, self.drive_to_model,
                    "back to its own model")
        self.text(xa, 300, "REVERSE CHECK", 12, B.INK3)
        self.reverse = self.text(xa, 322, "", 12, B.INK2, width=260)

    def drive_to_pins(self):
        self.app.drive_send("off")
        self.app.drive_send("set hsrc 1")
        self.app.drive_send("set sens 1")

    def drive_to_model(self):
        self.app.drive_send("off")
        self.app.drive_send("set sens 0")
        self.app.drive_send("set hsrc 0")

    def update(self):
        app = self.app
        u = app.units
        d = app.drive.state if app.drive else {}
        h = app.hil.state if app.hil else {}

        def put(key, left, right, fmt="{:,.0f}", tol=0.05):
            l_item, r_item, diff_item = self.cells[key]
            if left is None or right is None:
                self.set(l_item, "–")
                self.set(r_item, "–")
                self.set(diff_item, "")
                return
            self.set(l_item, fmt.format(left))
            self.set(r_item, fmt.format(right))
            span = max(abs(left), abs(right), 1e-6)
            err = abs(left - right) / span
            self.set(diff_item, f"{err * 100:.1f}%",
                     B.GREEN if err <= tol else B.RED)

        d_spd = d.get("spd")
        h_spd = h.get("spd")
        put("speed", d_spd, h_spd)
        put("rpm", None if d_spd is None else u.rpm_shaft(d_spd),
            None if h_spd is None else u.rpm_shaft(h_spd))
        d_ang, h_ang = d.get("ang"), h.get("ang")
        if d_ang is not None and h_ang is not None:
            wrap = (d_ang - h_ang) % 65536
            wrap = wrap if wrap < 32768 else wrap - 65536
            l_item, r_item, diff_item = self.cells["angle"]
            self.set(l_item, f"{d_ang:,}")
            self.set(r_item, f"{h_ang:,}")
            self.set(diff_item, f"{wrap / 65536 * 360:+.0f}°",
                     B.GREEN if abs(wrap) < 6000 else B.MUTE)
        ia, ib = h.get("ia"), h.get("ib")
        if ia is not None and ib is not None:
            put("current", u.amps(d.get("iq")),
                u.amps(u.vector(ia, ib)), "{:.2f}", 0.10)
        vd, vq = d.get("vd"), d.get("vq")
        if vd is not None and vq is not None and h.get("da") is not None:
            drive_v = u.volts_from_q15(math.sqrt(vd * vd + vq * vq))
            duties = [h.get(k, 900) for k in ("da", "db", "dc")]
            measured = (max(duties) - min(duties)) / DRIVE_PERIOD \
                * u.setup["bus_voltage"] / math.sqrt(3.0)
            put("voltage", drive_v, measured, "{:.2f}", 0.25)
            self.set(self.reverse,
                     f"The drive reports a voltage vector of "
                     f"{math.sqrt(vd * vd + vq * vq):,.0f} counts. The duties "
                     f"measured here spread over "
                     f"{max(duties) - min(duties)} of 1800, which at "
                     f"{u.setup['bus_voltage']:.0f} V is {measured:.2f} V "
                     f"line to line over root three.")

        chk = next(c for c in app.checks if c.key == "sync")
        if chk.ok is None:
            self.set(self.verdict, "Connect both boards to compare them.", B.INK3)
        elif chk.ok:
            self.set(self.verdict,
                     "The two boards agree. The drive is controlling a motor "
                     "it can only reach through six wires.", B.GREEN)
        else:
            self.set(self.verdict,
                     "The two do not agree yet. If the drive is not set to "
                     "`hsrc 1` and `sens 1`, it is steering by its own "
                     "internal model and the rotor here is only being "
                     "dragged along.", B.ORANGE)


# ---------------------------------------------------------------- the app --

class HilApp(tk.Tk):
    TICK_MS = 1000 // POLL_HZ
    PAGES = (OverviewPage, SignalsPage, ScopePage, MotorPage, SyncPage)

    def __init__(self, hil_port=None, drive_port=None, model=False):
        super().__init__()
        self.title("HIL bench")
        self.geometry(f"{W}x{H}+50+30")
        self.resizable(False, False)
        self.configure(bg=B.PAPER)
        self.face = self._pick(("Leelawadee UI", "Segoe UI", "Arial"))
        self.light = self._pick(("Leelawadee UI Semilight", "Segoe UI Light",
                                 "Segoe UI", "Arial"))
        B.UI = self.face
        try:
            here = os.path.dirname(os.path.abspath(__file__))
            self.iconbitmap(default=os.path.join(os.path.dirname(here),
                                                 "unitymbed.ico"))
        except tk.TclError:
            pass

        self.units = Units()
        self.rec = Recorder()
        self.checks = build_checks()
        self.hil = None
        self.drive = None
        self.notice = ""

        self.cv = tk.Canvas(self, width=W, height=H, bg=B.PAPER,
                            highlightthickness=0)
        self.cv.pack(fill="both", expand=True)
        self._build_rail()

        self.pages = {}
        self.page = None
        self.show("overview")

        self.mode = "model" if model else "boards"
        if model:
            self.hil = FakeBoard("hil")
            self.drive = FakeBoard("drive")
        else:
            if hil_port:
                self.connect("hil", hil_port)
            if drive_port:
                self.connect("drive", drive_port)
        self.after(self.TICK_MS, self._tick)

    @staticmethod
    def _pick(names):
        import tkinter.font as tkfont
        have = set(tkfont.families())
        for name in names:
            if name in have:
                return name
        return names[-1]

    # ---- the rail

    def _build_rail(self):
        cv = self.cv
        cv.create_rectangle(0, 0, RAIL, H, fill=B.DARK, width=0)
        cv.create_image(28, 26, anchor="nw", image=B.grille(10, 3, 11, 4))
        cv.create_text(28, 72, text="HIL", anchor="nw", fill=B.WHITE,
                       font=(self.light, -34))
        cv.create_text(28, 110, text="bench", anchor="nw", fill=B.MUTE,
                       font=(self.light, -34))
        cv.create_text(28, 156, text="hardware in the loop", anchor="nw",
                       fill=B.MUTE, font=(self.face, -12))

        self.nav = {}
        for i, cls in enumerate(self.PAGES):
            y = 212 + i * 52
            tag = f"nav{cls.key}"
            cv.create_rectangle(22, y - 10, RAIL - 20, y + 28, fill="",
                                outline="", tags=(tag,))
            bar = cv.create_rectangle(22, y - 6, 25, y + 24, fill=B.ORANGE,
                                      width=0, tags=(tag,))
            label = cv.create_text(42, y, text=cls.title, anchor="nw",
                                   fill=B.MUTE, font=(self.face, -16),
                                   tags=(tag,))
            cv.tag_bind(tag, "<ButtonPress-1>",
                        lambda e, k=cls.key: self.show(k))
            self.nav[cls.key] = (bar, label)

        cv.create_text(28, 470, text="MODE", anchor="nw", fill=B.MUTE,
                       font=(self.face, -11))
        self.b_mode = []
        for i, (key, label) in enumerate((("boards", "real boards"),
                                          ("model", "no hardware"))):
            self.b_mode.append((key, B.RoundButton(
                cv, 44, 500 + i * 32, 22, B.DARK2,
                lambda k=key: self.set_mode(k), label=label, label_side="e",
                label_size=12, label_color=B.MUTE)))

        cv.create_text(28, 564, text="BOARDS", anchor="nw", fill=B.MUTE,
                       font=(self.face, -11))
        self.lamp_hil = B.Lamp(cv, 34, 590, "HIL board", 11, B.MUTE,
                               off="#3A3A35")
        self.lamp_drive = B.Lamp(cv, 34, 614, "drive", 11, B.MUTE, off="#3A3A35")
        self.t_ports = cv.create_text(28, 634, text="", anchor="nw",
                                      fill=B.MUTE, font=(self.face, -11),
                                      width=RAIL - 56)

        self.port_items = []
        cv.create_text(28, 680, text="PORTS", anchor="nw", fill=B.MUTE,
                       font=(self.face, -11))
        B.RoundButton(cv, RAIL - 40, 684, 20, B.DARK2, self.rescan,
                      glyph="\u21bb", glyph_size=12)
        cv.create_text(28, H - 28, text="no power stage, no motor, six wires",
                       anchor="nw", fill="#55554E", font=(self.face, -11),
                       width=RAIL - 56)

        self.t_title = cv.create_text(X0, 34, text="", anchor="nw", fill=B.INK,
                                      font=(self.light, -30))
        self.t_sub = cv.create_text(X0, 76, text="", anchor="nw", fill=B.INK2,
                                    font=(self.face, -13))
        self.t_notice = cv.create_text(X0, H - 46, text="", anchor="nw",
                                       fill=B.ORANGE, font=(self.face, -12),
                                       width=X1 - X0)
        self.rescan()

    def rescan(self):
        for item in self.port_items:
            self.cv.delete(item)
        self.port_items = []
        try:
            ports = [p for p, _k in serialport.list_ports()]
        except Exception:                        # noqa: BLE001
            ports = []
        for i, port in enumerate(ports[:3]):
            y = 706 + i * 26
            tag = f"port{i}"
            self.port_items.append(self.cv.create_text(
                46, y, text=port, anchor="nw", fill=B.WHITE,
                font=(self.face, -14), tags=(tag,)))
            self.port_items.append(self.cv.create_rectangle(
                28, y - 4, RAIL - 20, y + 20, fill="", outline="", tags=(tag,)))
            self.cv.tag_bind(tag, "<ButtonPress-1>",
                             lambda e, p=port: self.offer(p))
        if not ports:
            self.port_items.append(self.cv.create_text(
                46, 706, text="no serial ports", anchor="nw", fill=B.MUTE,
                font=(self.face, -12)))

    def set_mode(self, key):
        """Real boards, or arithmetic in this window.

        The window is the same either way: the same checks, the same
        charts, the same numbers in the same places. That is the point -
        somebody can learn to read it with nothing plugged in.
        """
        if key == self.mode:
            return
        for board in (self.hil, self.drive):
            if board is not None:
                board.close()
        self.hil = self.drive = None
        self.mode = key
        if key == "model":
            self.hil = FakeBoard("hil")
            self.drive = FakeBoard("drive")
            self.notice = ("no hardware: every number on these pages is "
                           "being made up in this window.")
        else:
            self.notice = "pick the ports below: the HIL board first."
            self.rescan()

    def offer(self, port):
        """A click on a port connects whichever board is still missing."""
        if self.mode == "model":
            self.notice = ("this window is in no-hardware mode. switch to "
                           "real boards first.")
        elif self.hil is None:
            self.connect("hil", port)
        elif self.drive is None:
            self.connect("drive", port)
        else:
            self.notice = (f"Both boards are connected. Close the window to "
                           f"change them.")

    def connect(self, which, port):
        hello = "ver"
        try:
            board = Board(port, hello)
        except OSError as exc:
            self.notice = str(exc)
            return
        if which == "hil":
            self.hil = board
        else:
            self.drive = board
        self.notice = ""

    def hil_send(self, text):
        if self.hil is not None:
            self.hil.send(text)
            self.notice = f"sent to the HIL board: {text}"
        else:
            self.notice = "No HIL board connected."

    def drive_send(self, text):
        if self.drive is not None:
            self.drive.send(text)
            self.notice = f"sent to the drive: {text}"
        else:
            self.notice = "No drive connected."

    def export_csv(self):
        path = filedialog.asksaveasfilename(
            parent=self, title="Export the recording",
            defaultextension=".csv", initialfile="hil-run.csv",
            filetypes=(("Spreadsheet", "*.csv"),))
        if not path:
            return
        try:
            n = self.rec.write_csv(path)
        except OSError as exc:
            self.notice = f"could not write it: {exc}"
            return
        self.notice = f"{n} samples written to {os.path.basename(path)}"

    # ---- pages

    def show(self, key):
        if self.page is not None:
            self.page.destroy()
        cls = next(c for c in self.PAGES if c.key == key)
        page = cls(self)
        page.build()
        self.page = page
        self.cv.itemconfigure(self.t_title, text=cls.title)
        self.cv.itemconfigure(self.t_sub, text=cls.subtitle)
        for k, (bar, label) in self.nav.items():
            on = k == key
            self.cv.itemconfigure(bar, state="normal" if on else "hidden")
            self.cv.itemconfigure(label, fill=B.WHITE if on else B.MUTE)

    # ---- the tick

    def _run_checks(self):
        h = self.hil.state if self.hil else {}
        d = self.drive.state if self.drive else {}
        by_key = {c.key: c for c in self.checks}

        c = by_key["link"]
        if self.hil is None:
            c.ok, c.detail = None, "No HIL board connected yet."
        elif self.hil.error:
            c.ok, c.detail = False, self.hil.error
        else:
            c.ok = True
            c.detail = f"{self.hil.port}, firmware {self.hil.version or '?'}."

        c = by_key["pwm"]
        live, moe = h.get("live"), h.get("moe")
        if live is None:
            c.ok, c.detail = None, c.explain
        elif live and moe:
            c.ok, c.detail = True, "All three phases are switching."
        else:
            c.ok = False
            c.detail = ("The drive's outputs are off. Arm them, or the model "
                        "has nothing to turn it.")

        c = by_key["period"]
        per = h.get("per") or 0
        if not per:
            c.ok, c.detail = None, c.explain
        else:
            khz = 72000.0 / per
            c.ok = abs(khz - 20.0) < 0.3
            c.detail = (f"{per} ticks = {per / 72.0:.2f} us per period "
                        f"({khz:.2f} kHz).")

        c = by_key["bridge"]
        ovl = h.get("ovl")
        if ovl is None:
            c.ok, c.detail = None, c.explain
        elif ovl == 0:
            c.ok, c.detail = True, "No overlap seen on any phase."
        else:
            c.ok = False
            c.detail = (f"{ovl} samples caught both transistors on. On real "
                        f"hardware that is a short across the supply.")

        c = by_key["realtime"]
        late = h.get("late")
        if late is None:
            c.ok, c.detail = None, c.explain
        elif late == 0:
            c.ok, c.detail = True, "No tick has overrun its 50 us."
        else:
            c.ok, c.detail = False, f"{late} ticks arrived late."

        c = by_key["hall"]
        hall = h.get("hall")
        if hall is None:
            c.ok, c.detail = None, c.explain
        elif hall in (0, 7):
            c.ok = False
            c.detail = (f"code {hall:03b}: all on or all off. A wire is "
                        f"missing, or a pattern is being forced.")
        else:
            c.ok, c.detail = True, f"code {hall:03b}, one of the six valid states."

        c = by_key["sync"]
        if self.drive is None or d.get("spd") is None or h.get("spd") is None:
            c.ok, c.detail = None, "Connect the drive as well to compare."
        else:
            a, b = d["spd"], h["spd"]
            span = max(abs(a), abs(b), 1.0)
            err = abs(a - b) / span
            c.ok = err < 0.08 and abs(b) > 20
            c.detail = (f"drive {a} units/ms, model {b} units/ms, "
                        f"{err * 100:.1f}% apart."
                        + ("" if c.ok else
                           "  Set `hsrc 1` and `sens 1` on the drive to close "
                           "the loop through the wires."))

    def _tick(self):
        for board, ask in ((self.hil, "s"), (self.drive, "s")):
            if board is None:
                continue
            board.poll()
            if isinstance(board, Board):
                board.send(ask)
        if self.drive is not None and isinstance(self.drive, Board):
            self.drive.send("x")           # vd and vq, for the reverse check

        self.rec.add(self.hil.state if self.hil else {},
                     self.drive.state if self.drive else {})
        self._run_checks()

        for key, button in self.b_mode:
            on = key == self.mode
            button.set_color(B.ORANGE if on else B.DARK2)
            button.set_label(color=B.WHITE if on else B.MUTE)
        self.lamp_hil.set(B.GREEN if (self.hil and not self.hil.error) else None)
        self.lamp_drive.set(B.GREEN if (self.drive and not self.drive.error)
                            else None)
        self.cv.itemconfigure(self.t_ports, text="\n".join(filter(None, (
            f"HIL: {self.hil.port}" if self.hil else "HIL: not connected",
            f"drive: {self.drive.port}" if self.drive else "drive: not connected",
        ))))
        self.cv.itemconfigure(self.t_notice, text=self.notice)

        try:
            self.page.update()
        except tk.TclError:
            pass
        self.after(self.TICK_MS, self._tick)


def main(argv):
    def arg(name):
        return argv[argv.index(name) + 1] if name in argv else None

    app = HilApp(hil_port=arg("--hil"), drive_port=arg("--drive"),
                 model="--model" in argv)
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
