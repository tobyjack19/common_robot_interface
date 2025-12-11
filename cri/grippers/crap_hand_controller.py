import time
import re
from dataclasses import dataclass
from typing import Optional, Dict, Any

import serial  # pyserial
from serial.tools import list_ports

from .base import GripperControllerBase

DEFAULT_BAUD = 115200
DEFAULT_PORT = "COM4"

START_MARK = '<'
END_MARK = '>'


def _autodetect_port() -> Optional[str]:
    ports = list(list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc for k in ["arduino", "wchusb", "ch340"]) or "usb" in hwid:
            return p.device
    return ports[0].device if ports else None


@dataclass
class _Status:
    current: Optional[int] = None
    target: Optional[int] = None
    max_speed: Optional[float] = None
    accel: Optional[float] = None
    enabled: Optional[bool] = None
    min_pos: Optional[int] = None
    max_pos: Optional[int] = None


class CRAPHandController(GripperControllerBase):
    """CRAP hand gripper controller (Arduino stepper based).

    Serial protocol uses framed messages <...> for low latency. This class
    wraps the transport and provides a CRI-style gripper API.
    """

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD, timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self._data_started = False
        self._data_buf = ""
        self._last_limits = (0, 3000)
        self._connect()

    @property
    def info(self) -> str:
        return f"CRAPHandController[{self.port}@{self.baud}]"

    # Lifecycle
    def _connect(self) -> None:
        if self.port.lower() == "auto":
            detected = _autodetect_port()
            if detected:
                self.port = detected
        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=0,
            write_timeout=1,
            rtscts=False,
            dsrdtr=False,
        )
        time.sleep(2.5)
        self._flush()
        self._wait_for_ready(5.0)
        if self.ser and self.ser.is_open and not self._try_sync():
            time.sleep(0.5)
            self._flush()
            self._wait_for_ready(2.0)
        # Cache limits if available
        st = self._status()
        if st.min_pos is not None and st.max_pos is not None:
            self._last_limits = (st.min_pos, st.max_pos)

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    # Low-level framed IO
    def _flush(self) -> None:
        assert self.ser is not None
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def _wait_for_ready(self, seconds: float = 2.0) -> None:
        assert self.ser is not None
        end = time.time() + seconds
        while time.time() < end:
            msg = self._recv_frame()
            if msg and msg.strip() == "ready":
                return
            time.sleep(0.01)

    def _write(self, cmd: str) -> None:
        assert self.ser is not None
        payload = f"{START_MARK}{cmd}{END_MARK}".encode("ascii")
        self.ser.write(payload)
        self.ser.flush()

    def _recv_frame(self) -> str:
        assert self.ser is not None
        if self.ser.in_waiting > 0:
            x = self.ser.read().decode("ascii", errors="ignore")
            if self._data_started:
                if x != END_MARK:
                    self._data_buf += x
                else:
                    self._data_started = False
                    msg = self._data_buf
                    self._data_buf = ""
                    return msg
            elif x == START_MARK:
                self._data_buf = ""
                self._data_started = True
        return ""

    def _transact(self, cmd: str, read_for_s: float = 0.3) -> str:
        self._write(cmd)
        end = time.time() + read_for_s
        resp = ""
        while time.time() < end:
            msg = self._recv_frame()
            if msg:
                resp = msg
                break
        return resp

    def _try_sync(self) -> bool:
        assert self.ser is not None
        end = time.time() + 0.5
        while time.time() < end:
            if self._recv_frame():
                return True
            time.sleep(0.01)
        return False

    # Parsing and helpers
    def _status_line(self) -> str:
        self._write("?")
        end = time.time() + 0.4
        while time.time() < end:
            msg = self._recv_frame()
            if msg:
                return msg
        return ""

    @staticmethod
    def _parse_status(line: str) -> _Status:
        s = _Status()
        try:
            if line.startswith("S "):
                parts = line.split()
                if len(parts) >= 8:
                    s.current = int(parts[1])
                    s.target = int(parts[2])
                    s.max_speed = float(parts[3])
                    s.accel = float(parts[4])
                    s.enabled = parts[5] == '1'
                    s.min_pos = int(parts[6])
                    s.max_pos = int(parts[7])
            else:
                cur = re.search(r"Current:\s*(-?\d+)", line)
                tgt = re.search(r"Target:\s*(-?\d+)", line)
                spd = re.search(r"MaxSpeed\(sps\):\s*([0-9.]+)", line)
                acc = re.search(r"Accel\(sps\^2\):\s*([0-9.]+)", line)
                ena = re.search(r"Enabled:\s*(\d)", line)
                lim = re.search(r"Limits\[min,max\]:\s*(-?\d+)\s*,\s*(-?\d+)", line)
                if cur:
                    s.current = int(cur.group(1))
                if tgt:
                    s.target = int(tgt.group(1))
                if spd:
                    s.max_speed = float(spd.group(1))
                if acc:
                    s.accel = float(acc.group(1))
                if ena:
                    s.enabled = ena.group(1) == "1"
                if lim:
                    s.min_pos = int(lim.group(1))
                    s.max_pos = int(lim.group(2))
        except Exception:
            pass
        return s

    def _status(self) -> _Status:
        return self._parse_status(self._status_line())

    # Base class API implementations
    def enable(self, on: bool) -> None:
        self._transact(f"E {1 if on else 0}")

    def home(self) -> None:
        self._transact("H")

    def home_index(self) -> None:
        self._transact("ZH")

    def set_speed(self, value: float) -> None:
        self._transact(f"V {float(value)}")

    def set_accel(self, value: float) -> None:
        self._transact(f"A {float(value)}")

    def set_limits(self, min_pos: int, max_pos: int) -> None:
        self._transact(f"L {int(min_pos)} {int(max_pos)}")
        self._last_limits = (int(min_pos), int(max_pos))

    def _steps_from_fraction(self, value_01: float) -> int:
        v = max(0.0, min(1.0, float(value_01)))
        # Prefer reported limits if available
        st = self._status()
        if st.min_pos is not None and st.max_pos is not None:
            lo, hi = st.min_pos, st.max_pos
        else:
            lo, hi = self._last_limits
        rng = max(1, hi - lo)
        return int(round(lo + v * rng))

    def set_position(self, value_01: float, wait: bool = False, timeout_s: float = 30.0) -> None:
        steps = self._steps_from_fraction(value_01)
        self._transact(f"P {steps}")
        if wait:
            self._wait_until_reached(timeout_s)

    def jog(self, delta_steps: int, wait: bool = False, timeout_s: float = 30.0) -> None:
        st = self._status()
        base = st.current if st.current is not None else 0
        target = base + int(delta_steps)
        if st.min_pos is not None and st.max_pos is not None:
            target = max(st.min_pos, min(st.max_pos, target))
        self._transact(f"P {target}")
        if wait:
            self._wait_until_reached(timeout_s)

    def _wait_until_reached(self, timeout_s: float = 30.0, tol_steps: int = 0) -> bool:
        end = time.time() + timeout_s
        stable = 0
        while time.time() < end:
            st = self._status()
            if st.current is not None and st.target is not None:
                if abs(st.current - st.target) <= tol_steps:
                    stable += 1
                    if stable >= 3:
                        return True
                else:
                    stable = 0
            time.sleep(0.05)
        return False

    def current_position(self) -> Optional[float]:
        st = self._status()
        if st.current is None:
            return None
        lo, hi = (st.min_pos, st.max_pos) if (st.min_pos is not None and st.max_pos is not None) else self._last_limits
        rng = max(1, hi - lo)
        return max(0.0, min(1.0, (st.current - lo) / rng))

    def target_position(self) -> Optional[float]:
        st = self._status()
        if st.target is None:
            return None
        lo, hi = (st.min_pos, st.max_pos) if (st.min_pos is not None and st.max_pos is not None) else self._last_limits
        rng = max(1, hi - lo)
        return max(0.0, min(1.0, (st.target - lo) / rng))

    def status(self) -> Dict[str, Any]:
        st = self._status()
        return {
            'current_steps': st.current,
            'target_steps': st.target,
            'max_speed': st.max_speed,
            'accel': st.accel,
            'enabled': st.enabled,
            'min_pos': st.min_pos,
            'max_pos': st.max_pos,
        }
