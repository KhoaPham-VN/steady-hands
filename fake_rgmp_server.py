#!/usr/bin/env python3
"""
fake_rgmp_server.py - stand-in for `rokoko-sdk` when nobody is wearing the gloves.

Serves, per the Rokoko SDK specs:
  * RGMP v2 on TCP 12276: two "Smartgloves II" devices (left + right) at 83.33 Hz
  * RCSP    on TCP 45451: ListDevices, GetDeviceInfo, DeviceSubscribe, SetColorLed,
                          and scripted ButtonPushed events

Scripted motion (loops every --loop seconds, default 20):
  0-4 s  open hand          4-8 s  fist            8+ s  slow open/close cycles
  wrist moves in a circle ~0.5 m from the Coil Pro
Scripted buttons (with --buttons):
  1.5 s DoublePress (calibrate OPEN)  5.0 s LongPress (calibrate FIST)  8.5 s SinglePress (engage)

The data is synthetic and deliberately awkward (quaternion sign flips, curls >180 deg,
non-default group order, odd flag bit positions) so client code gets exercised.

Usage:  python3 fake_rgmp_server.py --buttons      then run glove_bridge.py as usual.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import socket
import struct
import threading
import time

LEFT_ID, RIGHT_ID = 2978051819, 2978051820
FINGERS = ("thumb", "index", "middle", "ring", "little")
SENSORS = [f"{f}_finger" for f in FINGERS] + ["arm"]
RATE = 250.0 / 3.0


def q_axis_angle(axis, ang):
    n = math.sqrt(sum(a * a for a in axis))
    s = math.sin(ang / 2) / n
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(ang / 2))


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def q_rot(q, v):
    r = q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return r[:3]


# ── definition ────────────────────────────────────────────────────────────────

def flags(names_by_bit):
    return {str(b): n for b, n in names_by_bit.items()}


def definition(dev_id: int, hand: str) -> dict:
    default = []
    for s in SENSORS:
        default += [
            {"data_type": "FLOAT[7]", "measure_type": "TRANSFORM", "target_frame": f"{s}_sensor", "reference_frame": "hub"},
            {"data_type": "FLOAT[3]", "measure_type": "ANGULAR_VELOCITY", "target_frame": f"{s}_imu"},
            {"data_type": "FLOAT[3]", "measure_type": "PROPER_ACCELERATION", "target_frame": f"{s}_imu"},
            {"data_type": "UINT32", "measure_type": "STATUS_FLAGS", "target_frame": f"{s}_sensor",
             "bit_mapping": flags({0: "is_emf_saturated", 1: "is_accelerometer_saturated", 2: "is_gyroscope_saturated"})},
        ]
    default += [
        {"data_type": "FLOAT[4]", "measure_type": "ORIENTATION", "target_frame": "hub", "reference_frame": "LTP_NED"},
        {"data_type": "FLOAT[3]", "measure_type": "ANGULAR_VELOCITY", "target_frame": "hub_imu"},
        {"data_type": "FLOAT[3]", "measure_type": "PROPER_ACCELERATION", "target_frame": "hub_imu"},
        {"data_type": "UINT32", "measure_type": "STATUS_FLAGS", "target_frame": "hub",
         "bit_mapping": flags({0: "is_magnetometer_saturated", 1: "is_accelerometer_saturated",
                               2: "is_gyroscope_saturated", 5: "is_heading_reliable"})},
        {"data_type": "FLOAT[7]", "measure_type": "TRANSFORM", "target_frame": "arm_sensor", "reference_frame": "coil_pro"},
        {"data_type": "FLOAT", "measure_type": "CUSTOM", "target_frame": "arm_sensor", "reference_frame": "coil_pro",
         "custom_label": "relative_emf_residual"},
        {"data_type": "UINT32", "measure_type": "STATUS_FLAGS", "target_frame": "arm_sensor", "reference_frame": "coil_pro",
         "bit_mapping": flags({0: "is_emf_saturated", 1: "not_is_accelerometer_used", 2: "is_accelerometer_saturated",
                               3: "is_gyroscope_saturated", 6: "is_coil_pro_present", 9: "emf_flip_decided"})},
    ]
    emf_only = []
    for s in SENSORS:
        emf_only += [
            {"data_type": "FLOAT[7]", "measure_type": "TRANSFORM", "target_frame": f"{s}_sensor", "reference_frame": "hub"},
            {"data_type": "FLOAT", "measure_type": "CUSTOM", "target_frame": f"{s}_sensor", "reference_frame": "hub",
             "custom_label": "relative_emf_residual"},
            {"data_type": "UINT32", "measure_type": "STATUS_FLAGS", "target_frame": f"{s}_sensor",
             "bit_mapping": flags({0: "is_emf_saturated"})},
        ]
    return {
        "protocol_name": "RGMP", "protocol_version": "2.0.0", "device_id": dev_id,
        "device_type": "Smartgloves II", "timestamp_epoch": "device_boot",
        "device_info": {"friendly_name": f"Smartgloves II ({hand})", "hand": hand},
        "static_data": [],
        # emf_only deliberately first: clients must find "default" by name
        "groups": [{"name": "emf_only", "expected_rate_hz": RATE, "streams": emf_only},
                   {"name": "default", "expected_rate_hz": RATE, "streams": default}],
    }


# ── motion model ──────────────────────────────────────────────────────────────

FIST = {"thumb": math.radians(70), "index": math.radians(190), "middle": math.radians(195),
        "ring": math.radians(185), "little": math.radians(170)}


RAMP = 0.3   # seconds for open<->fist transitions; 0 = instant jump (stress test)


def curl_fraction(t: float) -> float:
    def ramp(x):
        return 1.0 if RAMP <= 0 else max(0.0, min(1.0, x / RAMP))
    if t < 4:
        return 0.0
    if t < 8:
        return ramp(t - 4)
    if t < 8 + RAMP:
        return 1.0 - ramp(t - 8)
    return 0.5 - 0.5 * math.cos(2 * math.pi * (t - 8 - RAMP) / 4.0)


def finger_pose(hand: str, f: str, frac: float):
    # Both gloves use the same right-handed frame (+X right, +Y toward wrist, +Z down);
    # the left hand is the mirror image across the hand midline (x -> -x), so axial
    # vectors transform as (ax, ay, az) -> (ax, -ay, -az).
    mirror = -1.0 if hand == "left" else 1.0
    if f == "thumb":
        axis = (-0.4, 0.5 * mirror, 0.75 * mirror)
        offset = q_axis_angle((0, 0, 1), mirror * math.radians(35))
        base = (-0.045 * mirror, -0.02, 0.015)
    else:
        axis = (-1.0, 0.0, 0.05 * mirror)   # flexion: fingertip (-Y) rotates toward the palm (+Z)
        offset = q_axis_angle((0, 0, 1), mirror * math.radians({"index": 4, "middle": 0, "ring": -4, "little": -8}[f]))
        base = ({"index": 0.025, "middle": 0.008, "ring": -0.01, "little": -0.027}[f] * mirror, -0.08, 0.0)
    q = q_mul(q_axis_angle(axis, FIST[f] * frac), offset)
    tip = q_rot(q, (0.0, -0.025, 0.0))
    return (base[0] + tip[0], base[1] + tip[1], base[2] + tip[2]), q


def frame_values(hand: str, t: float, t_boot: float) -> tuple[bytes, bytes]:
    frac = curl_fraction(t)
    rnd = random.random
    def q_out(q):   # random sign flips: q and -q are the same rotation
        return tuple(-c for c in q) if rnd() < 0.15 else q
    sensor_vals = []
    for s in SENSORS:
        if s == "arm":
            p, q = (0.0, 0.09, 0.01), q_axis_angle((1, 0, 0), math.radians(5))
        else:
            p, q = finger_pose(hand, s.replace("_finger", ""), frac)
        sensor_vals.append((p, q_out(q)))

    side = -1.0 if hand == "left" else 1.0
    ang = 2 * math.pi * t / 10.0
    arm_coil_p = (0.5 + 0.15 * math.cos(ang), side * 0.2 + 0.15 * math.sin(ang), 0.3)
    arm_coil_q = q_axis_angle((0, 0, 1), 0.3 * math.sin(ang))
    hub_ned = q_axis_angle((0, 0, 1), 0.2 * t)

    coil_flags = (1 << 6) | ((1 << 9) if t > 2.0 else 0)
    hub_flags = (1 << 5) if t > 3.0 else 0

    # default group, same order as definition()
    d = b""
    for p, q in sensor_vals:
        d += struct.pack("<7f", *p, *q) + struct.pack("<3f", 0, 0, 0) + struct.pack("<3f", 0, 0, 9.81) + struct.pack("<I", 0)
    d += struct.pack("<4f", *q_out(hub_ned)) + struct.pack("<3f", 0, 0, 0) + struct.pack("<3f", 0, 0, 9.81)
    d += struct.pack("<I", hub_flags)
    d += struct.pack("<7f", *arm_coil_p, *q_out(arm_coil_q)) + struct.pack("<f", 1.5) + struct.pack("<I", coil_flags)
    # emf_only group
    e = b""
    for p, q in sensor_vals:
        e += struct.pack("<7f", *p, *q) + struct.pack("<f", 2.0) + struct.pack("<I", 0)
    return d, e


# ── RGMP server ───────────────────────────────────────────────────────────────

class Rgmp:
    def __init__(self, port: int, loop: float):
        self.port, self.loop = port, loop
        self.clients: list[socket.socket] = []
        self.lock = threading.Lock()
        self.t0 = time.time()

    def _frame(self, prefix: int, payload: bytes) -> bytes:
        return struct.pack("<II", prefix, len(payload)) + payload

    def definitions(self) -> bytes:
        return b"".join(self._frame(1, json.dumps(definition(i, h)).encode())
                        for i, h in ((LEFT_ID, "left"), (RIGHT_ID, "right")))

    def accept_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", self.port))
        srv.listen()
        print(f"[fake] RGMP listening on 127.0.0.1:{self.port}")
        while True:
            c, addr = srv.accept()
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                c.sendall(self.definitions())
            except OSError:
                continue
            with self.lock:
                self.clients.append(c)
            print(f"[fake] RGMP client {addr} connected")

    def stream_loop(self) -> None:
        period, nxt = 1.0 / RATE, time.time()
        while True:
            t = (time.time() - self.t0) % self.loop
            ts_us = int((time.time() - self.t0) * 1e6)
            out = b""
            for dev, hand in ((LEFT_ID, "left"), (RIGHT_ID, "right")):
                d, e = frame_values(hand, t, self.t0)
                out += self._frame(2, struct.pack("<IIQ", dev, 1, ts_us) + d)   # default = group 1
                out += self._frame(2, struct.pack("<IIQ", dev, 0, ts_us) + e)   # emf_only = group 0
            with self.lock:
                for c in list(self.clients):
                    try:
                        c.sendall(out)
                    except OSError:
                        self.clients.remove(c)
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))


# ── RCSP server ───────────────────────────────────────────────────────────────

class Rcsp:
    def __init__(self, port: int, rgmp: Rgmp, buttons: bool):
        self.port, self.rgmp, self.buttons = port, rgmp, buttons
        self.subs: list[tuple[socket.socket, int]] = []
        self.lock = threading.Lock()

    @staticmethod
    def send(c, ptype, payload):
        body = json.dumps(payload).encode()
        c.sendall(bytes([0xDC, 1, 8, ptype]) + len(body).to_bytes(4, "little") + body)

    def handle(self, c: socket.socket) -> None:
        buf = bytearray()
        try:
            while True:
                chunk = c.recv(65536)
                if not chunk:
                    return
                buf += chunk
                while len(buf) >= 8:
                    plen = int.from_bytes(buf[4:8], "little")
                    if len(buf) < 8 + plen:
                        break
                    cmd = json.loads(bytes(buf[8:8 + plen]))
                    del buf[:8 + plen]
                    self.reply(c, cmd)
        except OSError:
            return

    def reply(self, c, cmd) -> None:
        name, tid, args = cmd.get("Command"), cmd.get("TrackId"), cmd.get("Arguments") or {}
        ok = lambda resp=None: self.send(c, 2, {"TrackId": tid, "Status": "Ok", "Version": 1,
                                                  **({"Response": resp} if resp is not None else {})})
        if name == "ListDevices":
            ok({"Devices": [{"DeviceId": i, "DeviceType": "Smartgloves", "Connections": ["Serial"],
                             "Updatable": True, "IsBootloader": False} for i in (LEFT_ID, RIGHT_ID)]})
        elif name == "GetDeviceInfo":
            ok({"DeviceType": "Smartgloves", "Smartgloves": {"IsLeft": args.get("DeviceId") == LEFT_ID}})
        elif name == "DeviceSubscribe":
            with self.lock:
                self.subs.append((c, args.get("DeviceId")))
            ok()
        elif name == "SetColorLed":
            hand = "left" if args.get("DeviceId") == LEFT_ID else "right"
            print(f"[fake] LED {hand}: rgb({args.get('Red')},{args.get('Green')},{args.get('Blue')})")
            ok()
        else:
            self.send(c, 3, {"TrackId": tid, "Status": "Error", "Version": 1,
                             "Error": {"Code": "Unknown command", "Message": name}})

    def accept_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", self.port))
        srv.listen()
        print(f"[fake] RCSP listening on 127.0.0.1:{self.port}")
        while True:
            c, _ = srv.accept()
            threading.Thread(target=self.handle, args=(c,), daemon=True).start()

    def button_loop(self) -> None:
        script = [(1.5, "DoublePress"), (5.0, "LongPress"), (8.5, "SinglePress")]
        fired = set()
        while True:
            t = (time.time() - self.rgmp.t0) % self.rgmp.loop
            cycle = int((time.time() - self.rgmp.t0) // self.rgmp.loop)
            for at, kind in script:
                if t >= at and (cycle, at) not in fired:
                    fired.add((cycle, at))
                    targets = (LEFT_ID, RIGHT_ID) if kind != "SinglePress" else (RIGHT_ID,)
                    with self.lock:
                        for c, dev in list(self.subs):
                            if dev in targets:
                                try:
                                    self.send(c, 4, {"Publisher": "DeviceEvents", "Topic": "ButtonPushed",
                                                     "EventData": {"DeviceId": dev, "ButtonPushType": kind}})
                                except OSError:
                                    self.subs.remove((c, dev))
                    print(f"[fake] t={t:.1f}s button {kind}")
            time.sleep(0.02)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rgmp-port", type=int, default=12276)
    ap.add_argument("--rcsp-port", type=int, default=45451)
    ap.add_argument("--loop", type=float, default=20.0)
    ap.add_argument("--buttons", action="store_true", help="send scripted glove button events")
    ap.add_argument("--ramp", type=float, default=0.3, help="open<->fist transition time (s); 0 = instant")
    a = ap.parse_args()
    global RAMP
    RAMP = a.ramp
    rgmp = Rgmp(a.rgmp_port, a.loop)
    rcsp = Rcsp(a.rcsp_port, rgmp, a.buttons)
    threading.Thread(target=rgmp.accept_loop, daemon=True).start()
    threading.Thread(target=rgmp.stream_loop, daemon=True).start()
    threading.Thread(target=rcsp.accept_loop, daemon=True).start()
    if a.buttons:
        threading.Thread(target=rcsp.button_loop, daemon=True).start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
