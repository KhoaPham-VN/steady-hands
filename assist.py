#!/usr/bin/env python3
"""
assist.py - Steady Hands assist functions. No sockets, no MuJoCo; standard library only.

  GripLimiter   caps the finger force (5 N with assist on, 60 N off); opening is never limited
  MotionScaler  out = anchor_out + scale * (in - anchor_in); re-anchors on every reset / scale change
  JitterFilter  1-euro filter (Casiez, Roussel & Vogel, CHI 2012) on each position axis

All tunable values are in ASSIST_PARAMS below; egg_sim.py writes them into every trial log.
Freeze them before logged trials start (PLAN_STEADY_HANDS.md, phase 4).

  python assist.py          # print the parameters
  python test_assist.py     # unit tests
"""
from __future__ import annotations

import json
import math

ASSIST_PARAMS = {
    # grip limiter (section 7.1)
    "cap_on_n": 5.0,             # force cap with assist on (egg weighs 0.59 N; 2 N already lifts it)
    "cap_off_n": 60.0,           # force cap with assist off (2 x break threshold)
    "yellow_n": 15.0,            # colour cue: yellow from here, red when broken (break_n in egg_sim.py)
    # motion scaling (section 7.2)
    "scale_travel": 1.0,
    "scale_precision": 0.3,      # 10 cm of hand travel -> 3 cm of gripper travel
    "precision_enter_m": 0.08,   # auto mode: precision when the gripper is within 8 cm of egg / target
    "precision_exit_m": 0.10,    # ... and travel again beyond 10 cm (hysteresis)
    # jitter filter (section 7.3). Plan starting point is 1 Hz / beta 0; 1 Hz gives ~157 ms lag at
    # 0.2 Hz, which fails the phase-2 lag test (< 80 ms), so we start at 3 Hz. Tune with eval_offline.py.
    "filter_min_cutoff_hz": 3.0,
    "filter_beta": 0.0,          # cutoff increase per m/s of hand speed
    "filter_d_cutoff_hz": 1.0,   # cutoff of the speed estimate
    "default_rate_hz": 83.33,    # used when timestamps are missing or implausible
}


class GripLimiter:
    """Force cap for the finger actuators. The cap only limits closing force."""

    def __init__(self, cap_on_n=None, cap_off_n=None):
        self.cap_on = ASSIST_PARAMS["cap_on_n"] if cap_on_n is None else cap_on_n
        self.cap_off = ASSIST_PARAMS["cap_off_n"] if cap_off_n is None else cap_off_n

    def cap(self, assist_on: bool) -> float:
        return self.cap_on if assist_on else self.cap_off

    def forcerange(self, assist_on: bool) -> tuple[float, float]:
        """(min, max) actuator force. Positive = closing; opening keeps the full force."""
        return (-self.cap_off, self.cap(assist_on))


class MotionScaler:
    """out = anchor_out + scale * (in - anchor_in), on 3-vectors (lists or tuples)."""

    def __init__(self, scale: float = 1.0):
        self.scale = scale
        self.anchor_in = None
        self.anchor_out = None
        self.last_in = None
        self.last_out = None

    def reset(self, x_in, x_out):
        """Anchor the current input to the current output (ENGAGE, assist switch)."""
        self.anchor_in, self.anchor_out = list(x_in), list(x_out)
        self.last_in, self.last_out = list(x_in), list(x_out)

    def set_scale(self, scale: float):
        """Change the scale without a jump: re-anchor at the last input/output pair."""
        if scale == self.scale:
            return
        self.scale = scale
        if self.last_in is not None:
            self.reset(self.last_in, self.last_out)

    def __call__(self, x_in):
        if self.anchor_in is None:
            raise RuntimeError("MotionScaler.reset() must be called first")
        out = [o + self.scale * (i - a) for i, a, o in zip(x_in, self.anchor_in, self.anchor_out)]
        self.last_in, self.last_out = list(x_in), out
        return out


def _alpha(cutoff_hz: float, dt: float) -> float:
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    return 1.0 / (1.0 + tau / dt)


class OneEuro:
    """1-euro filter on one scalar signal."""

    def __init__(self, min_cutoff, beta, d_cutoff):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.x = None
        self.dx = 0.0

    def reset(self):
        self.x, self.dx = None, 0.0

    def __call__(self, x: float, dt: float) -> float:
        if self.x is None:
            self.x, self.dx = x, 0.0
            return x
        dx = (x - self.x) / dt
        self.dx += _alpha(self.d_cutoff, dt) * (dx - self.dx)
        cutoff = self.min_cutoff + self.beta * abs(self.dx)
        self.x += _alpha(cutoff, dt) * (x - self.x)
        return self.x


class JitterFilter:
    """1-euro filter on each axis of a 3-D position. Feed it timestamps in seconds if available."""

    def __init__(self, min_cutoff=None, beta=None, d_cutoff=None, rate_hz=None):
        p = ASSIST_PARAMS
        self.min_cutoff = p["filter_min_cutoff_hz"] if min_cutoff is None else min_cutoff
        self.beta = p["filter_beta"] if beta is None else beta
        self.d_cutoff = p["filter_d_cutoff_hz"] if d_cutoff is None else d_cutoff
        self.default_dt = 1.0 / (p["default_rate_hz"] if rate_hz is None else rate_hz)
        self.axes = [OneEuro(self.min_cutoff, self.beta, self.d_cutoff) for _ in range(3)]
        self.t = None
        self.out = None

    def reset(self):
        for a in self.axes:
            a.reset()
        self.t, self.out = None, None

    def __call__(self, x, t: float | None = None):
        if t is None or self.t is None:
            dt = self.default_dt
        else:
            dt = t - self.t
            if dt == 0 and self.out is not None:
                return list(self.out)          # same glove frame sent twice: no new information
            if not (0 < dt < 0.2):
                dt = self.default_dt
        if t is not None:
            self.t = t
        self.out = [a(float(v), dt) for a, v in zip(self.axes, x)]
        return list(self.out)


if __name__ == "__main__":
    import argparse
    argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter).parse_args()
    print(json.dumps(ASSIST_PARAMS, indent=2))
