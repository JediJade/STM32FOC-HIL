"""
The serial port, exactly as the FOC studio opens it.

Lifted from `studio/boardlink.py` of the STM32FOC project so that the two
behave identically: no pyserial, win32 straight through ctypes, and a failed
read or write raises rather than returning nothing - because that is how an
unplugged board shows itself.
"""

import ctypes
import ctypes.wintypes as wt
import queue
import re
import threading
import time
import winreg


UNITS_PER_RPM = 65536.0 / 60000.0
SPEED_MAX_RPM = 6000            # OUTER_SPEED_MAX is 6560 units/ms = 6006 rpm
POLL_HZ = 100.0
HISTORY_SECONDS = 20.0

# Listed as a port, last and labelled for what it is: fwsim.py standing in
# for a board, so that everything a board does can be tried without one.
EMULATOR_PORT = "emulator"
emulator_enabled = True

# name -> (default the firmware is built with, lowest, highest), for console
# 3. A board with console 2 has the first nine and refuses the rest.

VEHICLE_STATES = ("off", "waiting for the throttle to be released", "running",
                  "braking", "throttle fault")
SENSOR_NAMES = ("encoder", "hall sensors", "observer")
SENSOR_PHASES = ("", "hall sensors", "forced angle, starting",
                 "following the back-emf", "calibrating")
CAL_RESULTS = {0: "not calibrated", 1: "calibrated",
               2: "failed: the two directions saw different states",
               3: "failed: not six states - are three sensors connected?",
               4: "failed: the states do not follow one another in a ring",
               9: "stopped by a command"}

# bits of selftest.ok_flags, in the order selftest.h defines them
FLAGS = (
    (0x01, "dead-time"),
    (0x02, "adc trigger"),
    (0x04, "fixed-point math"),
    (0x08, "current loop"),
    (0x10, "alignment"),
    (0x20, "speed and position"),
    (0x40, "break input"),
    (0x80, "current sensing"),
)

CHANNELS = ("rpm", "ref", "iq", "iqref", "id", "thr", "idfw")


# ----------------------------------------------------------------- serial ---

GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3
MAXDWORD = 0xFFFFFFFF
INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wt.DWORD), ("BaudRate", wt.DWORD), ("fFlags", wt.DWORD),
        ("wReserved", wt.WORD), ("XonLim", wt.WORD), ("XoffLim", wt.WORD),
        ("ByteSize", wt.BYTE), ("Parity", wt.BYTE), ("StopBits", wt.BYTE),
        ("XonChar", ctypes.c_char), ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char), ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char), ("wReserved1", wt.WORD),
    ]


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", wt.DWORD),
        ("ReadTotalTimeoutMultiplier", wt.DWORD),
        ("ReadTotalTimeoutConstant", wt.DWORD),
        ("WriteTotalTimeoutMultiplier", wt.DWORD),
        ("WriteTotalTimeoutConstant", wt.DWORD),
    ]


k32 = ctypes.WinDLL("kernel32", use_last_error=True)
k32.CreateFileW.restype = wt.HANDLE
k32.CreateFileW.argtypes = [wt.LPCWSTR, wt.DWORD, wt.DWORD, ctypes.c_void_p,
                            wt.DWORD, wt.DWORD, wt.HANDLE]
k32.ReadFile.argtypes = [wt.HANDLE, ctypes.c_void_p, wt.DWORD,
                         ctypes.POINTER(wt.DWORD), ctypes.c_void_p]
k32.WriteFile.argtypes = [wt.HANDLE, ctypes.c_void_p, wt.DWORD,
                          ctypes.POINTER(wt.DWORD), ctypes.c_void_p]
k32.GetCommState.argtypes = [wt.HANDLE, ctypes.POINTER(DCB)]
k32.SetCommState.argtypes = [wt.HANDLE, ctypes.POINTER(DCB)]
k32.SetCommTimeouts.argtypes = [wt.HANDLE, ctypes.POINTER(COMMTIMEOUTS)]
k32.CloseHandle.argtypes = [wt.HANDLE]


class Serial:
    """Open, read what has arrived, write a line. Same settings as the
    port class in foc-monitor.py; unlike it, a failed read or write raises,
    because that is how an unplugged board shows itself."""

    def __init__(self, port, baud=115200):
        self.h = k32.CreateFileW(f"\\\\.\\{port}", GENERIC_READ | GENERIC_WRITE,
                                 0, None, OPEN_EXISTING, 0, None)
        if self.h is None or self.h == INVALID_HANDLE_VALUE:
            err = ctypes.get_last_error()
            self.h = None
            why = {2: "the port does not exist",
                   5: "another program has it open"}.get(err, f"win32 error {err}")
            raise OSError(f"cannot open {port}: {why}")

        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        if not k32.GetCommState(self.h, ctypes.byref(dcb)):
            self.close()
            raise OSError(f"{port} is not a serial port")
        dcb.BaudRate = baud
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        # fBinary | DTR_CONTROL_ENABLE: some CDC hosts pass data only once
        # DTR is asserted, and it costs nothing to assert it.
        dcb.fFlags = 0x0001 | (1 << 4)
        if not k32.SetCommState(self.h, ctypes.byref(dcb)):
            self.close()
            raise OSError(f"{port} refused the port settings")
        # reads return at once with whatever has arrived
        k32.SetCommTimeouts(self.h, ctypes.byref(
            COMMTIMEOUTS(MAXDWORD, 0, 0, 0, 500)))

    def read(self, n=4096):
        buf = ctypes.create_string_buffer(n)
        got = wt.DWORD(0)
        if not k32.ReadFile(self.h, buf, n, ctypes.byref(got), None):
            raise OSError(f"read failed, win32 error {ctypes.get_last_error()}")
        return buf.raw[:got.value]

    def write_line(self, text):
        data = (text + "\r").encode("ascii", "ignore")
        got = wt.DWORD(0)
        if (not k32.WriteFile(self.h, data, len(data), ctypes.byref(got), None)
                or got.value != len(data)):
            raise OSError(f"write failed, win32 error {ctypes.get_last_error()}")

    def close(self):
        if self.h is not None:
            k32.CloseHandle(self.h)
            self.h = None


def list_ports():
    """(port, kind) for every serial port Windows knows about."""
    out = {}
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                             r"HARDWARE\DEVICEMAP\SERIALCOMM")
    except OSError:
        return []
    with key:
        i = 0
        while True:
            try:
                device, port, _ = winreg.EnumValue(key, i)
            except OSError:
                break
            low = device.lower()
            if "usbser" in low:
                kind = "usb serial device"
            elif "bth" in low:
                kind = "bluetooth"
            elif "\\serial" in low:
                kind = "built-in port"
            else:
                kind = device.rsplit("\\", 1)[-1].lower()
            out[port] = kind
            i += 1
    ports = sorted(out.items(), key=lambda p: (len(p[0]), p[0]))
    # The FOC studio appends its emulator here; this project has none.
    return ports


# ------------------------------------------------------------------ parse ---

FIELD = re.compile(r"\b(ang|spd|id|iq|iqref|moe)\s+(-?\d+)")
FAULT = re.compile(r"\bfault\s+(.+?)\s*$")
STATE_KEYS = ("ang", "spd", "id", "iq", "iqref", "moe")




def port_identities():
    """port -> the USB instance id Windows has for it, where there is one.

    SERIALCOMM says which ports exist and nothing about what is on them.
    The instance id is the device's own serial number, which is the one
    thing that tells two identical Blue Pills apart - and the reason
    their firmware stopped sharing one. FOC000001 is the drive,
    HIL000001 is this board.

    Returns {} rather than raising if the registry is not shaped the way
    this expects: a port list with no names is still a port list.
    """
    out = {}
    root = r"SYSTEM\CurrentControlSet\Enum\USB"
    try:
        usb = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, root)
    except OSError:
        return out
    with usb:
        for i in range(4096):
            try:
                vid = winreg.EnumKey(usb, i)
            except OSError:
                break
            try:
                vid_key = winreg.OpenKey(usb, vid)
            except OSError:
                continue
            with vid_key:
                for j in range(4096):
                    try:
                        inst = winreg.EnumKey(vid_key, j)
                    except OSError:
                        break
                    try:
                        with winreg.OpenKey(vid_key,
                                            inst + r"\Device Parameters") as p:
                            port, _ = winreg.QueryValueEx(p, "PortName")
                    except OSError:
                        continue
                    # An instance id with no & in it is the device's own
                    # serial number; one with & is Windows making one up
                    # from where it was plugged in, which says nothing.
                    out[port] = inst if "&" not in inst else ""
    return out


def describe_ports():
    """(port, kind, identity) for every serial port, identity possibly ''."""
    names = port_identities()
    return [(port, kind, names.get(port, "")) for port, kind in list_ports()]
