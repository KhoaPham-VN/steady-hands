#!/usr/bin/env python3
"""
coil_axes_check.py - measure the Coil -> robot axis mapping from where the operator stands NOW,
for position AND orientation, and print the --coil-to-robot value to restart the bridge with.
Read only: it listens to glove_bridge.py's UDP JSON and uses the raw Coil-frame fields
(hands.<side>.wrist_pos / wrist_quat), so the answer does not depend on the bridge's current mapping.

Close hands_sim.py / egg_sim.py first (they use the same port), keep the bridge running, then:
  python coil_axes_check.py                 # right hand, bridge on UDP 9870
  python coil_axes_check.py --hand left

Five moves, each: hold the start pose, press Enter, do the move, hold, press Enter.
  1  move the hand about 20 cm FORWARD (away from you, towards where the camera looks)
  2  move the hand about 20 cm to YOUR LEFT
  3  move the hand about 20 cm UP
  4  palm down, turn the hand about 60 deg so the fingers point more to YOUR LEFT
     (turning about the vertical axis, like a steering wheel lying flat; counter-clockwise from above)
  5  palm down, bend so the FINGERTIPS point DOWN about 45 deg (tilt the whole hand, nose down)
Robot frame (what the sims use): X forward, Y left, Z up. Moves 1-3 give the mapping; moves 4-5 check
that the orientation agrees with it (expected: move 4 = +Z rotation, move 5 = +Y rotation).
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time

from glove_bridge import mat_vec, parse_axis_map, q_conj, q_mul, q_normalize   # read only

AX = "xyz"


class Listener(threading.Thread):
    def __init__(self, port, hand):
        super().__init__(daemon=True)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(("0.0.0.0", port))
        self.s.settimeout(0.5)
        self.hand, self.lock, self.hist = hand, threading.Lock(), []

    def run(self):
        while True:
            try:
                m = json.loads(self.s.recvfrom(65536)[0])
            except (socket.timeout, ConnectionResetError, ValueError, OSError):
                continue
            h = (m.get("hands") or {}).get(self.hand) or {}
            if h.get("coil_ok") and h.get("wrist_pos") and h.get("wrist_quat"):
                with self.lock:
                    self.hist.append((time.time(), h["wrist_pos"], h["wrist_quat"]))
                    self.hist = self.hist[-200:]

    def mean_pose(self, sec=0.4):
        """Mean position and (sign-aligned) mean orientation over the last sec seconds."""
        with self.lock:
            now = time.time()
            rec = [r for r in self.hist if now - r[0] <= sec]
        if not rec:
            return None
        p = [sum(r[1][k] for r in rec) / len(rec) for k in range(3)]
        q0 = rec[-1][2]
        acc = [0.0] * 4
        for r in rec:
            sg = 1.0 if sum(a * b for a, b in zip(r[2], q0)) >= 0 else -1.0
            acc = [a + sg * b for a, b in zip(acc, r[2])]
        return p, q_normalize(acc)


def rot_axis_angle(q):
    x, y, z, w = q
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    ang = 2 * math.degrees(math.acos(min(1.0, w)))
    n = math.sqrt(x * x + y * y + z * z)
    return ([x / n, y / n, z / n] if n > 1e-9 else [0.0, 0.0, 0.0]), ang


def fmt(v):
    return "(" + ", ".join(f"{c:+.3f}" for c in v) + ")"


def dominant(v):
    k = max(range(3), key=lambda i: abs(v[i]))
    share = abs(v[k]) / (math.sqrt(sum(c * c for c in v)) or 1)
    return ("-" if v[k] < 0 else "") + AX[k], share


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hand", choices=["left", "right"], default="right")
    ap.add_argument("--port", type=int, default=9870, help="bridge UDP port (close the sims first)")
    ap.add_argument("--current", default="y,x,-z", help="the bridge's current --coil-to-robot (for the comparison)")
    a = ap.parse_args()
    try:
        L = Listener(a.port, a.hand)
    except OSError as e:
        print(f"cannot listen on UDP {a.port}: {e} - close hands_sim.py / egg_sim.py first")
        return
    L.start()
    print(f"waiting for {a.hand}-hand Coil data on UDP {a.port} ...")
    t0 = time.time()
    while L.mean_pose() is None:
        if time.time() - t0 > 10:
            print("no Coil data: is the bridge running, the Coil on, and does its status line show pos=...?")
            return
        time.sleep(0.2)
    moves = [("forward", "move the hand about 20 cm FORWARD (away from you)"),
             ("left", "move the hand about 20 cm to YOUR LEFT"),
             ("up", "move the hand about 20 cm UP"),
             ("yaw_left", "palm down, turn the hand ~60 deg so the fingers point more to YOUR LEFT (about vertical)"),
             ("tips_down", "palm down, tilt the hand so the FINGERTIPS point DOWN ~45 deg")]
    res = {}
    for i, (key, text) in enumerate(moves, 1):
        input(f"\n[{i}/5] hold the START pose still and press Enter ")
        p0, q0 = L.mean_pose()
        input(f"      now {text}, hold still, press Enter ")
        p1, q1 = L.mean_pose()
        if key in ("forward", "left", "up"):
            d = [b - c for b, c in zip(p1, p0)]
            res[key] = d
            ax, share = dominant(d)
            print(f"      Coil-frame change {fmt(d)} m, |d| {math.sqrt(sum(c * c for c in d)) * 100:.1f} cm "
                  f"-> mostly coil {ax} ({share * 100:.0f} % of the move)")
        else:
            dq = q_normalize(q_mul(q1, q_conj(q0)))           # world-frame (Coil-frame) turn, like the bridge
            axis, ang = rot_axis_angle(dq)
            res[key] = (axis, ang)
            print(f"      Coil-frame turn {ang:.0f} deg about {fmt(axis)}")

    print("\n================ result ================")
    toks, weak = [], []
    for key, robot in (("forward", "X"), ("left", "Y"), ("up", "Z")):
        ax, share = dominant(res[key])
        toks.append(ax)
        if share < 0.8 or math.sqrt(sum(c * c for c in res[key])) < 0.08:
            weak.append(f"robot {robot} ({key}: {share * 100:.0f} % on one axis)")
    spec = ",".join(toks)
    print(f"position: robot X,Y,Z = coil {spec}")
    if weak:
        print("  WARNING: not clean, repeat these moves more carefully / longer: " + "; ".join(weak))
    try:
        M, _ = parse_axis_map(spec)
    except ValueError as e:
        print(f"  NOT a valid mapping: {e}")
        print("  The three moves do not form a rotation. If all three directions were reversed, that is the")
        print("  Coil's half-space (EMF) ambiguity, not the mapping: restart the driver with --coil-half-space.")
        return
    ok = True
    for key, want, name in (("yaw_left", [0, 0, 1], "+Z (turn left about vertical)"),
                            ("tips_down", [0, 1, 0], "+Y (fingertips down)")):
        axis, ang = res[key]
        r = mat_vec(M, axis)
        dot = sum(x * y for x, y in zip(r, want))
        good = dot > 0.8 and ang > 20
        ok &= good
        print(f"orientation check {key}: {ang:.0f} deg about robot {fmt(r)}, expected {name}: "
              f"{'OK' if good else 'MISMATCH'} (alignment {dot:+.2f})")
    try:
        Mc, _ = parse_axis_map(a.current)
        print(f"\nwith the CURRENT mapping ({a.current}) the sim would see:")
        for key, want in (("forward", [1, 0, 0]), ("left", [0, 1, 0]), ("up", [0, 0, 1])):
            r = mat_vec(Mc, res[key])
            n = math.sqrt(sum(c * c for c in r)) or 1
            dot = sum(x * y for x, y in zip(r, want)) / n
            print(f"  {key:8s} -> {fmt(r)}  " + ("correct" if dot > 0.8 else "REVERSED" if dot < -0.8 else "wrong axis"))
    except ValueError:
        pass
    if ok and not weak:
        print(f"\nposition and orientation agree. Restart the bridge with:\n"
              f"  python glove_bridge.py --no-rcsp --rate 83.33 --udp 127.0.0.1:9870 --coil-to-robot={spec}")
        print("(write it with '=': a value starting with '-' would otherwise be read as an option)")
    else:
        print("\nposition and orientation do NOT agree yet: repeat the check before changing anything.")


if __name__ == "__main__":
    main()
