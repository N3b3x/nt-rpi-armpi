"""50–100 Hz joint interpolator. One batched PWM packet per tick (SWRS-ARM-004)."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence


JointCmd = List[tuple]  # [(servo_id, pulse), ...]


@dataclass
class TrajectorySample:
    t_s: float
    joints_deg: List[float]


@dataclass
class ApplianceState:
    joints_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    busy: bool = False
    following: bool = False
    estop: bool = False
    loop_hz: float = 0.0
    packets: int = 0


def deg_to_pulse(deg: float) -> int:
    """Map [-90, 90] deg to 500–2500 µs (HiWonder PWM)."""
    clamped = max(-90.0, min(90.0, deg))
    return int(round((clamped + 90.0) * (2000.0 / 180.0) + 500.0))


def lerp(a: Sequence[float], b: Sequence[float], u: float) -> List[float]:
    u = max(0.0, min(1.0, u))
    return [x + (y - x) * u for x, y in zip(a, b)]


class TrajectoryInterpolator:
    """Main-thread-safe interpolator. Hardware write is injected."""

    SERVO_IDS = (6, 5, 4, 3)  # base, lift, shoulder, elbow
    LOOKAHEAD = 12  # consumed ring; analog of Dobot CP sample window

    def __init__(
        self,
        write_packet: Optional[Callable[[float, JointCmd], None]] = None,
        rate_hz: float = 80.0,
    ) -> None:
        self._write = write_packet
        self._dt = 1.0 / max(rate_hz, 1.0)
        self._lock = threading.Lock()
        self._samples: List[TrajectorySample] = []
        self._t0: Optional[float] = None
        self.state = ApplianceState()
        self.last_packet: Optional[JointCmd] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._hz_n = 0
        self._hz_t = time.monotonic()

    def estop(self) -> None:
        with self._lock:
            self._samples = []
            self._t0 = None
            self.state.estop = True
            self.state.busy = False
            self.state.following = False

    def clear_estop(self) -> None:
        with self._lock:
            self.state.estop = False

    def replace_goal(self, joints_deg: Sequence[float], duration_s: float = 0.2) -> None:
        now = 0.0
        current = list(self.state.joints_deg)
        self.load(
            [
                TrajectorySample(0.0, current),
                TrajectorySample(max(duration_s, 0.05), list(joints_deg)),
            ],
            replace=True,
        )
        _ = now

    def load(self, samples: Sequence[TrajectorySample], replace: bool = False) -> None:
        ordered = sorted(samples, key=lambda s: s.t_s)
        if not ordered:
            raise ValueError("trajectory has no samples")
        with self._lock:
            if self.state.estop:
                raise RuntimeError("estop latched")
            if replace or not self._samples:
                self._samples = list(ordered)
                self._t0 = time.monotonic()
            else:
                offset = self._samples[-1].t_s if self._samples else 0.0
                for s in ordered:
                    self._samples.append(
                        TrajectorySample(s.t_s + offset, list(s.joints_deg))
                    )
            self.state.busy = True
            self.state.following = True

    def _target_at(self, t: float) -> List[float]:
        samples = self._samples
        if not samples:
            return list(self.state.joints_deg)
        if t <= samples[0].t_s:
            return list(samples[0].joints_deg)
        if t >= samples[-1].t_s:
            return list(samples[-1].joints_deg)
        for i in range(1, len(samples)):
            if t <= samples[i].t_s:
                span = samples[i].t_s - samples[i - 1].t_s
                u = 0.0 if span <= 0 else (t - samples[i - 1].t_s) / span
                return lerp(samples[i - 1].joints_deg, samples[i].joints_deg, u)
        return list(samples[-1].joints_deg)

    def tick(self, now: Optional[float] = None) -> Optional[JointCmd]:
        """Advance one control period. Returns the batched packet or None."""
        with self._lock:
            if self.state.estop:
                return None
            if not self._samples or self._t0 is None:
                self.state.busy = False
                self.state.following = False
                return None
            t = (now if now is not None else time.monotonic()) - self._t0
            # Drop consumed samples; keep one previous + up to LOOKAHEAD future.
            while len(self._samples) > 2 and self._samples[1].t_s <= t:
                self._samples.pop(0)
            joints = self._target_at(t)
            self.state.joints_deg = joints
            packet: JointCmd = [
                (sid, deg_to_pulse(deg))
                for sid, deg in zip(self.SERVO_IDS, joints)
            ]
            self.last_packet = packet
            self.state.packets += 1
            finished = t >= self._samples[-1].t_s
            if finished:
                self._samples = []
                self._t0 = None
                self.state.busy = False
                self.state.following = False
        if self._write is not None:
            self._write(self._dt, packet)
        self._hz_n += 1
        now_hz = time.monotonic()
        if now_hz - self._hz_t >= 1.0:
            self.state.loop_hz = self._hz_n / (now_hz - self._hz_t)
            self._hz_n = 0
            self._hz_t = now_hz
        return packet

    def start_background(self) -> None:
        if self._thread and self._thread.is_alive():
            return

        def _loop() -> None:
            last = time.monotonic()
            n = 0
            while not self._stop.is_set():
                t0 = time.monotonic()
                self.tick(t0)
                n += 1
                if t0 - last >= 1.0:
                    self.state.loop_hz = n / (t0 - last)
                    n = 0
                    last = t0
                sleep = self._dt - (time.monotonic() - t0)
                if sleep > 0:
                    time.sleep(sleep)

        self._stop.clear()
        self._thread = threading.Thread(target=_loop, daemon=True)
        self._thread.start()

    def stop_background(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
