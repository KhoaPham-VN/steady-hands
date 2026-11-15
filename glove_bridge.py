#!/usr/bin/env python3
"""
glove_bridge.py - Rokoko Smartgloves II + Coil Pro  ->  robot-agnostic teleop stream.

Reads the Rokoko Device SDK driver directly (no Studio, no hand solver needed):
  * RGMP  (binary motion stream, TCP 127.0.0.1:12276)
  * RCSP  (commands/events, TCP 127.0.0.1:45451) - glove button + ring LED

and publishes one small JSON message per tick over UDP, e.g. to 127.0.0.1:9870:

  {
    "t": 1731550000.123,          # host time (s)
    "seq": 1234,
    "engaged": true,              # clutch / dead-man state (glove button or 'e')
    "hands": {
      "right": {
        "ts_us": 123456789,       # glove capture timestamp (us, device clock)
        "calibrated": true,
        "curl":  {"thumb": 0.1, "index": 0.8, "middle": 0.9, "ring": 0.9, "little": 0.7},  # 0 open .. 1 fist
        "grip":  0.85,            # mean(index, middle, ring) - handy for parallel grippers
        "pinch_m": 0.034,         # distance thumb-sensor <-> index-sensor (m, hub frame)
        "coil_ok": true,          # Coil Pro absolute pose valid this frame
        "wrist_pos": [x, y, z],   # hand (hub) position in Coil Pro frame (m)       | null if !coil_ok
        "wrist_quat": [x, y, z, w],  # hand (hub) orientation in Coil Pro frame      | null if !coil_ok
        "wrist_dpos": [dx, dy, dz],  # position change since the clutch engaged (m) | null if not engaged
        "wrist_dquat": [x, y, z, w], # rotation change since clutch engaged (world) | null if not engaged
        "ee_pos":  [X, Y, Z],        # wrist_pos in the ROBOT frame (default X fwd, Y left, Z up; --coil-to-robot)
        "ee_quat": [x, y, z, w],     # wrist_quat in the robot frame
        "ee_dpos": [dX, dY, dZ],     # wrist_dpos in the robot frame, times --scale  <- feed this to the arm
        "ee_dquat": [x, y, z, w],    # wrist_dquat in the robot frame
        "hub_quat_ned": [x, y, z, w],# hub orientation vs LTP_NED (works without coil)
        "heading_ok": true,          # magnetometer heading converged
        "finger_quat": {"thumb": [x,y,z,w], ...}  # raw sensor-in-hub orientations
      },
      "left": { ... }
    }
  }

Controls
  Glove button (Smartgloves II):  single press = toggle ENGAGE (clutch)
                                  double press = calibrate OPEN hand (that glove)
                                  long press   = calibrate FIST (that glove)
  Keyboard (type + Enter):  e = toggle engage, o = calibrate open (both), f = calibrate fist (both),
                            s = save calibration, q = quit
  Ring LED: blue = idle, green = engaged, yellow = calibrating

Calibration: hold the pose still; sampling starts 1.0 s after the trigger and lasts 0.5 s.
Do OPEN first (flat hand, fingers together), then FIST. Saved to calib.json automatically.

Run
  terminal 1:  rokoko-sdk --auto-stream-usb        (gloves on USB; Wi-Fi gloves stream by default)
  terminal 2:  python3 glove_bridge.py --udp 127.0.0.1:9870
  terminal 3:  python3 udp_monitor.py               (see the numbers move)

Standard library only. Python 3.10+.
Protocol details follow sdk.rokoko.com (RCSP spec, RGMP v2 spec, Smartgloves RGMP reference).
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import queue
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Optional

FINGERS = ("thumb", "index", "middle", "ring", "little")
SENSOR_OF = {f: f"{f}_finger_sensor" for f in FINGERS}

# ─────────────────────────────────────────────────────────────────────────────
# Quaternion / transform math. Quaternions are (x, y, z, w) like RGMP.
# ─────────────────────────────────────────────────────────────────────────────

Quat = tuple[float, float, float, float]
Vec3 = tuple[float, float, float]
IDENTITY: Quat = (0.0, 0.0, 0.0, 1.0)


def q_mul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_conj(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def q_dot(a: Quat, b: Quat) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]


def q_neg(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], -q[3])


def q_normalize(q) -> Quat:
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12:
        return IDENTITY
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_rotate(q: Quat, v: Vec3) -> Vec3:
    r = q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), q_conj(q))
    return (r[0], r[1], r[2])


def parse_axis_map(spec: str) -> tuple[list[list[float]], Quat]:
    """'y,x,-z' -> rotation matrix M (robot = M @ coil) and its quaternion.

    Each entry says which Coil axis feeds robot X, Y, Z. Must be a proper rotation
    (no mirroring), otherwise orientations would come out with flipped handedness.
    """
    names = {"x": 0, "y": 1, "z": 2}
    rows = []
    for tok in spec.replace(" ", "").lower().split(","):
        sign = -1.0 if tok.startswith("-") else 1.0
        axis = tok.lstrip("+-")
        if axis not in names:
            raise ValueError(f"bad axis {tok!r} in {spec!r}")
        r = [0.0, 0.0, 0.0]
        r[names[axis]] = sign
        rows.append(r)
    if len(rows) != 3 or sorted(r.index(next(v for v in r if v)) for r in rows) != [0, 1, 2]:
        raise ValueError(f"{spec!r} must use each of x, y, z exactly once")
    m = rows
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
           - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    if det < 0:
        raise ValueError(f"{spec!r} is a mirror, not a rotation (flip the sign of one axis)")
    # rotation matrix -> quaternion (x, y, z, w)
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    elif m[0][0] >= m[1][1] and m[0][0] >= m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    elif m[1][1] >= m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)
    return m, q_normalize(q)


def mat_vec(m, v) -> Vec3:
    return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))


@dataclass
class Pose:
    p: Vec3
    q: Quat

    def compose(self, other: "Pose") -> "Pose":
        """self ∘ other: maps other's target frame into self's reference frame."""
        rp = q_rotate(self.q, other.p)
        return Pose((self.p[0] + rp[0], self.p[1] + rp[1], self.p[2] + rp[2]), q_normalize(q_mul(self.q, other.q)))

    def inverse(self) -> "Pose":
        qi = q_conj(self.q)
        rp = q_rotate(qi, self.p)
        return Pose((-rp[0], -rp[1], -rp[2]), qi)


# ─────────────────────────────────────────────────────────────────────────────
# Finger curl: projection of the sensor's rotation (relative to the hub) onto the
# open->fist rotation axis, measured per user by calibration. Works the same for
# both hands and the thumb, and survives >180 deg curls by tracking quaternion
# sign continuity frame to frame.
# ─────────────────────────────────────────────────────────────────────────────

# Before calibration: assume flat hand = identity (docs: "when the glove lies flat,
# all coordinate axes are aligned") and use these rough ranges (rad) for a preview.
DEFAULT_RANGE = {"thumb": 1.0, "index": 2.6, "middle": 2.6, "ring": 2.6, "little": 2.4}
PALM_CHECK = True   # disable with --no-palm-check if calibrated curls come out inverted


@dataclass
class FingerTrack:
    name: str
    q: Optional[Quat] = None          # latest sensor-in-hub orientation, sign-continuous
    p: Optional[Vec3] = None          # latest sensor-in-hub position (m)
    q_open: Optional[Quat] = None
    q_fist: Optional[Quat] = None
    axis: Optional[Vec3] = None
    phi: Optional[float] = None       # open->fist angle (rad), may exceed pi

    def update(self, q: Quat, p: Vec3) -> None:
        q = q_normalize(q)
        if self.q is None:
            ref = self.q_open if self.q_open is not None else IDENTITY
            if q_dot(q, ref) < 0:
                q = q_neg(q)
        elif q_dot(q, self.q) < 0:
            q = q_neg(q)
        self.q, self.p = q, p

    def reset_continuity(self) -> None:
        self.q = None

    def finalize(self) -> bool:
        if self.q_open is None or self.q_fist is None:
            return False
        d = q_mul(q_conj(self.q_open), self.q_fist)
        s = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
        if s < 1e-3:
            self.axis = self.phi = None
            return False
        axis = (d[0] / s, d[1] / s, d[2] / s)
        phi = 2.0 * math.atan2(s, d[3])
        # The same fist orientation can be reached "the short way" through a
        # hyper-extended path if tracking skipped frames. Pick the anatomical path.
        flip = False
        if self.name == "thumb":
            flip = phi > math.radians(250)
        elif PALM_CHECK:
            # Halfway along the path the finger (sensor -Y, pointing to the fingertip)
            # must point toward the palm: +Z in the glove frame (+X right, +Y wrist, +Z down).
            half = (axis[0] * math.sin(phi / 4), axis[1] * math.sin(phi / 4), axis[2] * math.sin(phi / 4), math.cos(phi / 4))
            v = q_rotate(q_mul(self.q_open, half), (0.0, -1.0, 0.0))
            flip = v[2] < -0.3
        if flip:
            axis, phi = (-axis[0], -axis[1], -axis[2]), 2.0 * math.pi - phi
        self.axis, self.phi = axis, phi
        return True

    @property
    def calibrated(self) -> bool:
        return self.axis is not None and self.phi is not None and self.phi > 0.05

    def curl(self) -> Optional[float]:
        if self.q is None:
            return None
        if not self.calibrated:
            # Rough preview only: total rotation away from "flat" (sign-agnostic, so it
            # works on either glove whatever the axis conventions turn out to be).
            theta = 2.0 * math.acos(min(1.0, abs(self.q[3])))
            return max(0.0, min(1.0, theta / DEFAULT_RANGE[self.name]))
        d = q_mul(q_conj(self.q_open), self.q)
        ax = self.axis
        theta = 2.0 * math.atan2(d[0] * ax[0] + d[1] * ax[1] + d[2] * ax[2], d[3])
        # Wrong quaternion branch (tracking started mid-fist, or frames were skipped):
        # a finger can't hyper-extend ~110 deg or curl ~250 deg, so hop to the other
        # branch and fix continuity for the following frames.
        if theta < -0.6 * math.pi or theta > 1.4 * math.pi:
            theta += 2.0 * math.pi if theta < 0 else -2.0 * math.pi
            self.q = q_neg(self.q)
        return max(0.0, min(1.0, theta / self.phi))


@dataclass
class HandState:
    hand: str
    device_id: Optional[int] = None
    ts_us: int = 0
    last_rx: float = 0.0
    fingers: dict[str, FingerTrack] = field(default_factory=lambda: {f: FingerTrack(f) for f in FINGERS})
    arm_in_hub: Optional[Pose] = None
    arm_in_coil: Optional[Pose] = None
    hub_q_ned: Optional[Quat] = None
    coil_present: bool = False
    coil_flip_decided: bool = False
    coil_emf_saturated: bool = False
    heading_ok: bool = False
    engage_ref: Optional[Pose] = None

    @property
    def coil_ok(self) -> bool:
        return (self.coil_present and self.coil_flip_decided and not self.coil_emf_saturated
                and self.arm_in_coil is not None and self.arm_in_hub is not None)

    def hand_in_coil(self) -> Optional[Pose]:
        # TRANSFORM(target, ref) maps target -> ref:  p_ref = T * p_target  (RGMP spec)
        # hub in coil = (arm in coil) ∘ (arm in hub)^-1
        if not self.coil_ok:
            return None
        return self.arm_in_coil.compose(self.arm_in_hub.inverse())


# ─────────────────────────────────────────────────────────────────────────────
# RGMP v2 client (motion stream)
# ─────────────────────────────────────────────────────────────────────────────

_BASE = {"INT32": ("i", 4), "UINT32": ("I", 4), "INT64": ("q", 8),
         "UINT64": ("Q", 8), "FLOAT": ("f", 4), "DOUBLE": ("d", 8)}


def parse_data_type(dt: str) -> tuple[str, int]:
    dt = dt.replace(" ", "")
    base, _, dims = dt.partition("[")
    ch, size = _BASE[base]
    count = 1
    if dims:
        for n in dims.rstrip("]").split(","):
            count *= int(n)
    return f"<{count}{ch}", size * count


@dataclass
class StreamSlot:
    key: str                 # what we use it for
    offset: int
    fmt: str
    bits: dict[str, int]     # STATUS_FLAGS: flag name -> bit index


def _wanted_key(s: dict) -> Optional[str]:
    mt, tf, rf = s.get("measure_type"), s.get("target_frame"), s.get("reference_frame")
    if mt == "TRANSFORM" and rf == "hub":
        for f, sensor in SENSOR_OF.items():
            if tf == sensor:
                return f"finger:{f}"
        if tf == "arm_sensor":
            return "arm_in_hub"
    if mt == "TRANSFORM" and tf == "arm_sensor" and rf == "coil_pro":
        return "arm_in_coil"
    if mt == "STATUS_FLAGS" and tf == "arm_sensor" and rf == "coil_pro":
        return "coil_flags"
    if mt == "ORIENTATION" and tf == "hub" and rf == "LTP_NED":
        return "hub_ned"
    if mt == "STATUS_FLAGS" and tf == "hub" and rf in (None, "hub"):
        return "hub_flags"
    return None


class RgmpReader(threading.Thread):
    def __init__(self, host: str, port: int, hands: dict[str, HandState], lock: threading.Lock, log):
        super().__init__(daemon=True)
        self.host, self.port, self.hands, self.lock, self.log = host, port, hands, lock, log
        self.layouts: dict[tuple[int, int], list[StreamSlot]] = {}   # (device_id, group_id) -> slots
        self.dev_hand: dict[int, str] = {}
        self.frames = 0
        self.connected = False

    def run(self) -> None:
        backoff = 0.5
        while True:
            try:
                with socket.create_connection((self.host, self.port), timeout=5.0) as sock:
                    sock.settimeout(None)
                    self.connected = True
                    self.log(f"RGMP connected to {self.host}:{self.port}")
                    backoff = 0.5
                    self._loop(sock)
            except (OSError, ConnectionError) as e:
                if self.connected:
                    self.log(f"RGMP connection lost: {e}")
                else:
                    self.log(f"RGMP: cannot reach {self.host}:{self.port} ({e}); is rokoko-sdk running? retrying...")
            self.connected = False
            with self.lock:
                for h in self.hands.values():
                    for ft in h.fingers.values():
                        ft.reset_continuity()
            self.layouts.clear()
            self.dev_hand.clear()
            time.sleep(backoff)
            backoff = min(backoff * 2, 5.0)

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> bytes:
        buf = bytearray(n)
        view, got = memoryview(buf), 0
        while got < n:
            k = sock.recv_into(view[got:], n - got)
            if k == 0:
                raise ConnectionError("server closed the connection")
            got += k
        return bytes(buf)

    def _loop(self, sock: socket.socket) -> None:
        while True:
            prefix, length = struct.unpack("<II", self._recv_exact(sock, 8))
            payload = self._recv_exact(sock, length)
            if prefix == 1:
                self._on_definition(json.loads(payload.decode("utf-8")))
            elif prefix == 2:
                self._on_data(payload)
            elif prefix == 3:
                (dev,) = struct.unpack_from("<I", payload, 0)
                hand = self.dev_hand.pop(dev, None)
                for k in [k for k in self.layouts if k[0] == dev]:
                    del self.layouts[k]
                self.log(f"RGMP: device {dev} ({hand}) disconnected")
                if hand:
                    with self.lock:
                        for ft in self.hands[hand].fingers.values():
                            ft.reset_continuity()
            # unknown frame types are ignored (forward compatible)

    def _on_definition(self, d: dict) -> None:
        dtype = d.get("device_type", "")
        dev = int(d.get("device_id", 0))
        if not dtype.startswith("Smartgloves"):
            self.log(f"RGMP: ignoring device {dev} type={dtype!r}")
            return
        hand = (d.get("device_info") or {}).get("hand")
        if hand not in ("left", "right"):
            self.log(f"RGMP: glove {dev} without left/right in device_info, ignoring")
            return
        for k in [k for k in self.layouts if k[0] == dev]:
            del self.layouts[k]
        found = set()
        for gid, g in enumerate(d.get("groups", [])):
            if g.get("name") != "default":
                continue       # skip emf_only diagnostics group
            slots, off = [], 0
            for s in g.get("streams", []):
                fmt, size = parse_data_type(s["data_type"])
                key = _wanted_key(s)
                if key:
                    bits = {name: int(b) for b, name in (s.get("bit_mapping") or {}).items()}
                    slots.append(StreamSlot(key, off, fmt, bits))
                    found.add(key)
                off += size
            self.layouts[(dev, gid)] = slots
        self.dev_hand[dev] = hand
        with self.lock:
            self.hands[hand].device_id = dev
            for ft in self.hands[hand].fingers.values():
                ft.reset_continuity()
        missing = [k for k in ([f"finger:{f}" for f in FINGERS] + ["arm_in_hub", "arm_in_coil", "coil_flags"])
                   if k not in found]
        self.log(f"RGMP: {dtype} {hand} glove, device_id={dev}"
                 + (f"  (missing streams: {missing})" if missing else ""))

    def _on_data(self, payload: bytes) -> None:
        dev, gid, ts = struct.unpack_from("<IIQ", payload, 0)
        slots = self.layouts.get((dev, gid))
        if not slots:
            return
        hand = self.dev_hand.get(dev)
        if hand is None:
            return
        body = memoryview(payload)[16:]
        self.frames += 1
        with self.lock:
            h = self.hands[hand]
            h.ts_us, h.last_rx = ts, time.time()
            for sl in slots:
                v = struct.unpack_from(sl.fmt, body, sl.offset)
                k = sl.key
                if k.startswith("finger:"):
                    h.fingers[k[7:]].update((v[3], v[4], v[5], v[6]), (v[0], v[1], v[2]))
                elif k == "arm_in_hub":
                    h.arm_in_hub = Pose((v[0], v[1], v[2]), q_normalize(v[3:7]))
                elif k == "arm_in_coil":
                    h.arm_in_coil = Pose((v[0], v[1], v[2]), q_normalize(v[3:7]))
                elif k == "hub_ned":
                    h.hub_q_ned = q_normalize(v[0:4])
                elif k == "coil_flags":
                    f = v[0]
                    bit = lambda name: name in sl.bits and bool((f >> sl.bits[name]) & 1)
                    h.coil_present = bit("is_coil_pro_present")
                    h.coil_flip_decided = bit("emf_flip_decided")
                    h.coil_emf_saturated = bit("is_emf_saturated")
                elif k == "hub_flags":
                    f = v[0]
                    h.heading_ok = "is_heading_reliable" in sl.bits and bool((f >> sl.bits["is_heading_reliable"]) & 1)


# ─────────────────────────────────────────────────────────────────────────────
# RCSP client (button events + LED). Optional: the bridge works without it.
# ─────────────────────────────────────────────────────────────────────────────

class RcspWorker(threading.Thread):
    MARKER, HVER, HSIZE = 0xDC, 1, 8
    CMD, OK, ERR, EVENT = 1, 2, 3, 4

    def __init__(self, host: str, port: int, events: "queue.Queue", log):
        super().__init__(daemon=True)
        self.host, self.port, self.events, self.log = host, port, events, log
        self.cmd_q: "queue.Queue[tuple[str, dict]]" = queue.Queue()
        self.dev_hand: dict[int, str] = {}
        self._buf = bytearray()
        self._ids = itertools.count(1)
        self.connected = False

    # framing ------------------------------------------------------------------
    def _send(self, sock, name: str, args: Optional[dict]) -> str:
        tid = str(next(self._ids))
        cmd = {"Command": name, "TrackId": tid, "Version": 1}
        if args is not None:
            cmd["Arguments"] = args
        body = json.dumps(cmd).encode()
        sock.sendall(bytes([self.MARKER, self.HVER, self.HSIZE, self.CMD]) + len(body).to_bytes(4, "little") + body)
        return tid

    def _pump(self, sock, timeout: float) -> list[tuple[int, dict]]:
        sock.settimeout(timeout)
        try:
            chunk = sock.recv(65536)
            if not chunk:
                raise ConnectionError("RCSP server closed the connection")
            self._buf.extend(chunk)
        except socket.timeout:
            pass
        out = []
        while len(self._buf) >= 8:
            if self._buf[0] != self.MARKER:
                raise ConnectionError(f"RCSP bad marker 0x{self._buf[0]:02x}")
            hsize, ptype = self._buf[2], self._buf[3]
            plen = int.from_bytes(self._buf[4:8], "little")
            if len(self._buf) < hsize + plen:
                break
            payload = json.loads(bytes(self._buf[hsize:hsize + plen]).decode() or "{}")
            del self._buf[:hsize + plen]
            out.append((ptype, payload))
        return out

    def command(self, sock, name: str, args: Optional[dict] = None, timeout: float = 5.0) -> dict:
        self._send(sock, name, args)
        deadline = time.time() + timeout
        while time.time() < deadline:
            for ptype, payload in self._pump(sock, 0.2):
                if ptype == self.EVENT:
                    self._on_event(payload)
                elif ptype == self.ERR:
                    e = payload.get("Error") or {}
                    raise RuntimeError(f"{name}: {e.get('Code')}: {e.get('Message')}")
                elif ptype == self.OK:
                    return payload.get("Response") or {}
        raise TimeoutError(f"{name}: no response")

    # events -------------------------------------------------------------------
    def _on_event(self, ev: dict) -> None:
        if ev.get("Topic") == "ButtonPushed":
            data = ev.get("EventData") or {}
            hand = self.dev_hand.get(data.get("DeviceId"))
            if hand:
                self.events.put(("button", hand, data.get("ButtonPushType")))

    # main ---------------------------------------------------------------------
    def run(self) -> None:
        while True:
            try:
                with socket.create_connection((self.host, self.port), timeout=5.0) as sock:
                    self._buf.clear()
                    self._setup(sock)
                    self.connected = True
                    last_scan = time.time()
                    while True:
                        try:
                            name, args = self.cmd_q.get_nowait()
                            try:
                                self.command(sock, name, args)
                            except (RuntimeError, TimeoutError) as e:
                                self.log(f"RCSP {e}")
                        except queue.Empty:
                            for ptype, payload in self._pump(sock, 0.1):
                                if ptype == self.EVENT:
                                    self._on_event(payload)
                        if time.time() - last_scan > 5.0 and len(self.dev_hand) < 2:
                            self._setup(sock, quiet=True)   # pick up gloves plugged in later
                            last_scan = time.time()
            except (OSError, ConnectionError, RuntimeError, TimeoutError, json.JSONDecodeError) as e:
                if self.connected:
                    self.log(f"RCSP lost ({e}); button/LED disabled until reconnect")
            self.connected = False
            time.sleep(2.0)

    def _setup(self, sock, quiet: bool = False) -> None:
        devices = self.command(sock, "ListDevices").get("Devices", [])
        for d in devices:
            dev = d.get("DeviceId")
            if d.get("DeviceType") != "Smartgloves" or d.get("IsBootloader") or dev in self.dev_hand:
                continue
            try:
                info = self.command(sock, "GetDeviceInfo", {"DeviceId": dev})
                is_left = (info.get("Smartgloves") or {}).get("IsLeft")
                hand = "left" if is_left else "right"
                self.command(sock, "DeviceSubscribe", {
                    "DeviceId": dev,
                    "Publishers": [{"Publisher": "DeviceEvents", "Topics": ["ButtonPushed"]}]})
                self.dev_hand[dev] = hand
                self.events.put(("rcsp_glove", hand, dev))
                self.log(f"RCSP: {hand} glove DeviceId={dev} - button + LED active")
            except (RuntimeError, TimeoutError) as e:
                self.log(f"RCSP: glove {dev} setup failed: {e}")
        if not quiet and not self.dev_hand:
            self.log("RCSP: connected, no Smartgloves listed yet")

    def set_led(self, hand: str, rgb: tuple[int, int, int]) -> None:
        for dev, h in self.dev_hand.items():
            if h == hand:
                self.cmd_q.put(("SetColorLed", {"DeviceId": dev, "LedId": "RingLed",
                                                "Red": rgb[0], "Green": rgb[1], "Blue": rgb[2]}))


# ─────────────────────────────────────────────────────────────────────────────
# Bridge: calibration, clutch, publishing
# ─────────────────────────────────────────────────────────────────────────────

LED_IDLE, LED_ENGAGED, LED_CALIB = (0, 0, 255), (0, 255, 0), (255, 180, 0)


class Bridge:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.hands = {"left": HandState("left"), "right": HandState("right")}
        self.events: "queue.Queue" = queue.Queue()
        self.engaged = False
        self.seq = 0
        self.calib_jobs: dict[str, dict] = {}   # hand -> {"kind", "t0", "t1", "acc": {finger: [sum quat]}, "n"}
        self.led_state: dict[str, tuple] = {}
        self.rgmp = RgmpReader(args.rgmp_host, args.rgmp_port, self.hands, self.lock, self.log)
        self.rcsp = None if args.no_rcsp else RcspWorker(args.rcsp_host, args.rcsp_port, self.events, self.log)
        self.udp_addrs = []
        for target in args.udp.split(","):
            host, _, port = target.strip().rpartition(":")
            self.udp_addrs.append((host or "127.0.0.1", int(port)))
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.axis_m, self.axis_q = parse_axis_map(args.coil_to_robot)
        self.log(f"Coil -> robot axes: robot X,Y,Z = coil {args.coil_to_robot}  (scale {args.scale})")
        self.load_calibration()

    @staticmethod
    def log(msg: str) -> None:
        sys.stderr.write(f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        sys.stderr.flush()

    # calibration persistence ------------------------------------------------
    def load_calibration(self) -> None:
        path = self.args.calib
        if not os.path.exists(path):
            return
        try:
            data = json.load(open(path))
            for hand, fingers in data.items():
                for f, c in fingers.items():
                    ft = self.hands[hand].fingers[f]
                    ft.q_open = tuple(c["q_open"]) if c.get("q_open") else None
                    ft.q_fist = tuple(c["q_fist"]) if c.get("q_fist") else None
                    ft.finalize()
            self.log(f"loaded calibration from {path}")
        except Exception as e:  # noqa: BLE001
            self.log(f"could not load {path}: {e}")

    def save_calibration(self) -> None:
        with self.lock:
            data = {h: {f: {"q_open": ft.q_open, "q_fist": ft.q_fist} for f, ft in hs.fingers.items()}
                    for h, hs in self.hands.items()}
        with open(self.args.calib, "w") as fh:
            json.dump(data, fh, indent=1)
        self.log(f"saved calibration to {self.args.calib}")

    def start_calibration(self, hand: str, kind: str) -> None:
        now = time.time()
        self.calib_jobs[hand] = {"kind": kind, "t0": now + 1.0, "t1": now + 1.5,
                                 "acc": {f: [0.0, 0.0, 0.0, 0.0] for f in FINGERS}, "n": 0}
        self.log(f"calibrating {kind.upper()} on {hand} hand - hold still...")
        self.led(hand, LED_CALIB)

    def _step_calibration(self) -> None:
        now = time.time()
        for hand, job in list(self.calib_jobs.items()):
            if now < job["t0"]:
                continue
            with self.lock:
                hs = self.hands[hand]
                if now <= job["t1"]:
                    for f, ft in hs.fingers.items():
                        if ft.q is not None:
                            a = job["acc"][f]
                            for i in range(4):
                                a[i] += ft.q[i]
                    job["n"] += 1
                    continue
                del self.calib_jobs[hand]
                if job["n"] == 0:
                    self.log(f"calibration {job['kind']} on {hand}: no data (glove streaming?)")
                    continue
                ok = []
                for f, ft in hs.fingers.items():
                    q = q_normalize(job["acc"][f])
                    if job["kind"] == "open":
                        ft.q_open = q
                    else:
                        ft.q_fist = q
                    if ft.finalize():
                        ok.append(f"{f}={math.degrees(ft.phi):.0f}°")
            self.log(f"calibration {job['kind'].upper()} {hand}: {job['n']} samples"
                     + (f"  ranges: {' '.join(ok)}" if ok else "  (now do the other pose)"))
            self.led(hand, LED_ENGAGED if self.engaged else LED_IDLE)
            self.save_calibration()

    # clutch ---------------------------------------------------------------------
    def set_engaged(self, on: bool) -> None:
        self.engaged = on
        with self.lock:
            for hs in self.hands.values():
                hs.engage_ref = hs.hand_in_coil() if on else None
        self.log("ENGAGED - robot follows" if on else "DISENGAGED - robot holds")
        for hand in self.hands:
            self.led(hand, LED_ENGAGED if on else LED_IDLE)

    def led(self, hand: str, rgb) -> None:
        if self.rcsp and self.led_state.get(hand) != rgb:
            self.led_state[hand] = rgb
            self.rcsp.set_led(hand, rgb)

    # message --------------------------------------------------------------------
    def build_message(self) -> dict:
        now = time.time()
        out = {"t": now, "seq": self.seq, "engaged": self.engaged, "hands": {}}
        with self.lock:
            for hand, hs in self.hands.items():
                if now - hs.last_rx > 0.5:
                    continue     # stale / not connected
                curls = {f: ft.curl() for f, ft in hs.fingers.items()}
                vals = [curls[f] for f in ("index", "middle", "ring") if curls[f] is not None]
                th, ix = hs.fingers["thumb"].p, hs.fingers["index"].p
                pose = hs.hand_in_coil()
                if self.engaged and pose is not None and hs.engage_ref is None:
                    hs.engage_ref = pose          # coil came back while engaged
                dpos = dquat = None
                if self.engaged and pose is not None and hs.engage_ref is not None:
                    r = hs.engage_ref
                    dpos = [pose.p[i] - r.p[i] for i in range(3)]
                    dquat = list(q_normalize(q_mul(pose.q, q_conj(r.q))))
                out["hands"][hand] = {
                    "ts_us": hs.ts_us,
                    "calibrated": all(ft.calibrated for ft in hs.fingers.values()),
                    "curl": {f: (round(c, 4) if c is not None else None) for f, c in curls.items()},
                    "grip": round(sum(vals) / len(vals), 4) if vals else None,
                    "pinch_m": round(math.dist(th, ix), 4) if th and ix else None,
                    "coil_ok": pose is not None,
                    "wrist_pos": [round(c, 5) for c in pose.p] if pose else None,
                    "wrist_quat": [round(c, 5) for c in pose.q] if pose else None,
                    "wrist_dpos": [round(c, 5) for c in dpos] if dpos else None,
                    "wrist_dquat": [round(c, 5) for c in dquat] if dquat else None,
                    # Same poses in the robot frame (X forward, Y left, Z up by default)
                    "ee_pos": self._r(mat_vec(self.axis_m, pose.p)) if pose else None,
                    "ee_quat": self._r(q_normalize(q_mul(self.axis_q, pose.q))) if pose else None,
                    "ee_dpos": self._r([c * self.args.scale for c in mat_vec(self.axis_m, dpos)]) if dpos else None,
                    "ee_dquat": self._r(q_normalize(q_mul(q_mul(self.axis_q, tuple(dquat)), q_conj(self.axis_q))))
                                if dquat else None,
                    "hub_quat_ned": [round(c, 5) for c in hs.hub_q_ned] if hs.hub_q_ned else None,
                    "heading_ok": hs.heading_ok,
                    "finger_quat": {f: ([round(c, 5) for c in ft.q] if ft.q else None)
                                    for f, ft in hs.fingers.items()},
                }
        return out

    @staticmethod
    def _r(v) -> list:
        return [round(c, 5) for c in v]

    def status_line(self, msg: dict) -> str:
        parts = [f"{'ENGAGED ' if self.engaged else 'idle    '}"]
        for hand in ("left", "right"):
            h = msg["hands"].get(hand)
            if not h:
                parts.append(f"{hand}: --")
                continue
            c = h["curl"]
            bars = " ".join(f"{f[0].upper()}{int(round((c[f] or 0) * 9))}" for f in FINGERS)
            pos = (" pos=" + ",".join(f"{v:+.2f}" for v in h["wrist_pos"])) if h["wrist_pos"] else " coil:--"
            cal = "" if h["calibrated"] else " (uncal)"
            parts.append(f"{hand}: {bars}{pos}{cal}")
        return " | ".join(parts)

    # main loop --------------------------------------------------------------------
    def stdin_reader(self) -> None:
        for line in sys.stdin:
            self.events.put(("key", line.strip().lower(), None))

    def run(self) -> None:
        self.log(f"publishing UDP JSON to {', '.join(f'{h}:{p}' for h, p in self.udp_addrs)} at {self.args.rate} Hz")
        self.rgmp.start()
        if self.rcsp:
            self.rcsp.start()
        threading.Thread(target=self.stdin_reader, daemon=True).start()
        period = 1.0 / self.args.rate
        next_tick, last_status, last_frames = time.time(), 0.0, 0
        try:
            while True:
                while not self.events.empty():
                    kind, a, b = self.events.get()
                    if kind == "button":
                        if b == "SinglePress":
                            self.set_engaged(not self.engaged)
                        elif b == "DoublePress":
                            self.start_calibration(a, "open")
                        elif b == "LongPress":
                            self.start_calibration(a, "fist")
                    elif kind == "rcsp_glove":
                        self.led_state.pop(a, None)
                        self.led(a, LED_ENGAGED if self.engaged else LED_IDLE)
                    elif kind == "key":
                        if a == "e":
                            self.set_engaged(not self.engaged)
                        elif a in ("o", "f"):
                            for hand in self.hands:
                                self.start_calibration(hand, "open" if a == "o" else "fist")
                        elif a == "s":
                            self.save_calibration()
                        elif a == "q":
                            return
                self._step_calibration()
                msg = self.build_message()
                payload = json.dumps(msg, separators=(",", ":")).encode()
                for addr in self.udp_addrs:
                    try:
                        self.udp.sendto(payload, addr)
                    except OSError:
                        pass
                self.seq += 1
                now = time.time()
                if now - last_status >= 1.0 and not self.args.quiet:
                    fps = (self.rgmp.frames - last_frames) / max(now - last_status, 1e-3)
                    last_frames = self.rgmp.frames
                    sys.stderr.write(f"  {self.status_line(msg)}  [{fps:.0f} glove frames/s]\n")
                    last_status = now
                next_tick += period
                time.sleep(max(0.0, next_tick - time.time()))
        except KeyboardInterrupt:
            pass
        finally:
            for hand in self.hands:
                if self.rcsp:
                    self.rcsp.set_led(hand, (0, 0, 0))
            time.sleep(0.2)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--rgmp-host", default="127.0.0.1")
    p.add_argument("--rgmp-port", type=int, default=12276)
    p.add_argument("--rcsp-host", default="127.0.0.1")
    p.add_argument("--rcsp-port", type=int, default=45451)
    p.add_argument("--no-rcsp", action="store_true", help="don't use RCSP (no glove button/LED)")
    p.add_argument("--udp", default="127.0.0.1:9870", help="where to send JSON, host:port[,host:port...] e.g. 127.0.0.1:9870,192.0.2.20:9870")
    p.add_argument("--rate", type=float, default=60.0, help="publish rate Hz (gloves deliver 83.33 Hz)")
    p.add_argument("--calib", default="calib.json")
    p.add_argument("--quiet", action="store_true", help="no 1 Hz status line")
    p.add_argument("--coil-to-robot", default="y,x,-z",
                   help="which Coil axis feeds robot X,Y,Z (default 'y,x,-z' = measured at the venue: "
                        "coil +x left, +y forward, +z down -> robot X forward, Y left, Z up)")
    p.add_argument("--scale", type=float, default=1.0,
                   help="multiply ee_dpos (hand motion -> robot motion), e.g. 0.5 for finer control")
    p.add_argument("--no-palm-check", action="store_true",
                   help="disable the 'fingers curl toward glove +Z' sanity check used at calibration")
    args = p.parse_args()
    global PALM_CHECK
    PALM_CHECK = not args.no_palm_check
    Bridge(args).run()


if __name__ == "__main__":
    main()
