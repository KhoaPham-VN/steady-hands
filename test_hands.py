#!/usr/bin/env python3
"""
test_hands.py - test hands_sim.py without gloves. Uses its own ports, so it can run while the
real driver / solver / bridge are running (they use 12276, 12277, 45451, 9870).

Part A (deterministic): a fake two-glove stream (bridge format, coil_ok false, wrist fields null)
drives hands_sim.py --led; fake_rgmp_server.py on RCSP port 45461 records the LED commands.
  A1 both hands open -> both LEDs green
  A2 per-hand calibration: raw 0.10 = open, 0.90 = fist -> raw 0.50 maps to 0.50
  A3 object off, angle limit 80 %: curl 0.5 -> green, 0.7 -> yellow, 1.0 -> red, joints stop at 80 %
  A4 hands independent: left fist (red) while right stays at 0.3 (green); LED commands per glove
  A5 object on, force cap 10 N: grip settles at 10 N +-10 %; "limit force 0" -> grip far above 10 N
  A6 right glove data stops -> right LED off ("no data")
  A7 "q" -> hands_sim exits and turns both LEDs off
Part C (Coil, fake gloves with ee_pos + ee_quat): engage without a jump (position and palms up),
  axes, scale 0.3, position filter, repeated frames (jitter still shown), clutch; wrist rotation:
  axes for both hands (not mirrored), palm up -> palm down, rotation filter, rotation clutch,
  rot off / on / reset; force cap while moving and while rotating; reach-and-hold task.
Part B (chain): fake_rgmp_server.py (RGMP 12286) -> glove_bridge.py --no-rcsp -> hands_sim.py:
  real-time ratio 0.95-1.05, both hands receive data, fingers move, open/fist calibration works.
Part D (Coil chain, bridge engaged): wrists follow positions and the bridge's ee_quat rotation.
Part E (pick-up): free object on the table; scripted grasp-and-lift through the fake gloves with the
  right hand (can), the left hand (ball), both hands (can, two-hand assist); open while lifted ->
  dropped; pick reset; result files; real-time ratio.

  python test_hands.py            # about 4 minutes
  python test_hands.py --viewer   # watch part A in the MuJoCo window
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
FINGERS = ("thumb", "index", "middle", "ring", "little")
SIM_PORT, CMD_PORT, STATE_PORT, RGMP_PORT, RCSP_PORT = 9880, 9884, 9882, 12286, 45461


def qa(axis, deg):
    """Quaternion (x, y, z, w) for a rotation of deg about axis (bridge order)."""
    import math
    h = math.radians(deg) / 2
    n = sum(a * a for a in axis) ** 0.5
    return [math.sin(h) * a / n for a in axis] + [math.cos(h)]


def qm(a, b):
    """a x b for (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]


def rot_vec(q_wxyz, v):
    """Rotate vector v by the (w, x, y, z) quaternion the sim reports."""
    w, x, y, z = q_wxyz
    qv = [x, y, z, w]
    r = qm(qm(qv, list(v) + [0.0]), [-x, -y, -z, w])
    return r[:3]


class FakeGloves(threading.Thread):
    """Bridge-format messages for both gloves. Without .coil: coil_ok false and null wrist fields.
    With .coil: ee_pos = .base[side] + tremor (10 Hz sine, .tremor_mm per axis) + white noise, and
    ee_quat = .rot[side] (x, y, z, w; robot frame), optionally with a 10 Hz roll tremor
    (.rot_tremor_deg). .dup_every = n sends every n-th frame twice (same ts_us, like the real
    right glove)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.curl = {"left": 0.0, "right": 0.0}
        self.present = {"left": True, "right": True}
        self.coil, self.engaged = False, False
        self.base = {"left": [1.00, 0.20, -0.10], "right": [1.00, -0.20, -0.10]}
        self.rot = {"left": [0.0, 0.0, 0.0, 1.0], "right": [0.0, 0.0, 0.0, 1.0]}
        self.tremor_mm, self.noise_mm, self.rot_tremor_deg = 0.0, 0.0, 0.0
        self.dup_every = 0
        self.running, self.seq = True, 0

    def turn(self, side, axis, deg):
        """Turn that glove by deg about a ROBOT-frame axis (world frame: applied on the left)."""
        self.rot[side] = qm(qa(axis, deg), self.rot[side])

    def run(self):
        import math, random
        t0 = time.time()
        while self.running:
            self.seq += 1
            hands = {}
            tt = time.time() - t0
            for side in ("left", "right"):
                if not self.present[side]:
                    continue
                c = self.curl[side]
                ee = eq = None
                if self.coil:
                    ee = [b + self.tremor_mm / 1000 * math.sin(2 * math.pi * 10 * tt + k)
                          + random.gauss(0, self.noise_mm / 1000) for k, b in enumerate(self.base[side])]
                    eq = qm(qa((1, 0, 0), self.rot_tremor_deg * math.sin(2 * math.pi * 10 * tt)), self.rot[side]) \
                        if self.rot_tremor_deg else list(self.rot[side])
                    if self.seq % 7 == 0:
                        eq = [-v for v in eq]               # same orientation, other sign (the SDK does this)
                hands[side] = {"ts_us": int(tt * 1e6), "calibrated": True,
                               "curl": {f: c for f in FINGERS}, "grip": c, "pinch_m": 0.09 - 0.06 * c,
                               "coil_ok": ee is not None,
                               "wrist_pos": [ee[1], ee[0], -ee[2]] if ee else None, "wrist_quat": None,
                               "wrist_dpos": None, "wrist_dquat": None, "ee_pos": ee, "ee_quat": eq,
                               "ee_dpos": None, "ee_dquat": None, "hub_quat_ned": [0, 0, 0, 1], "heading_ok": True}
            msg = {"t": time.time(), "seq": self.seq, "engaged": self.engaged, "hands": hands}
            data = json.dumps(msg).encode()
            self.sock.sendto(data, ("127.0.0.1", SIM_PORT))
            time.sleep(0.012)
            if self.dup_every and self.seq % self.dup_every == 0:
                msg["seq"] = self.seq = self.seq + 1        # the bridge resends the same glove frame
                self.sock.sendto(json.dumps(msg).encode(), ("127.0.0.1", SIM_PORT))
                time.sleep(0.012)


class State(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(("127.0.0.1", STATE_PORT))
        self.s.settimeout(0.5)
        self.last, self.history = None, []

    def run(self):
        while True:
            try:
                self.last = json.loads(self.s.recvfrom(65536)[0])
                self.history.append(self.last)
            except (socket.timeout, ConnectionResetError):
                pass


def cmd(text):
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(text.encode(), ("127.0.0.1", CMD_PORT))


def start_sim(work, name, viewer, led=True, coil=False):
    log = os.path.join(work, f"{name}.log")
    args = [PY, os.path.join(HERE, "hands_sim.py"), "--port", str(SIM_PORT), "--cmd-port", str(CMD_PORT),
            "--state-udp", f"127.0.0.1:{STATE_PORT}", "--rcsp", f"127.0.0.1:{RCSP_PORT}"]
    if not coil:
        args.append("--no-coil")
    if led:
        args.append("--led")
    if not viewer:
        args.append("--headless")
    p = subprocess.Popen(args, cwd=work, stdout=open(log, "w"), stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    return p, log


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--keep", action="store_true")
    a = ap.parse_args()
    work = tempfile.mkdtemp(prefix="steady_hands_test_")
    shutil.copy(os.path.join(HERE, "hands_scene.xml"), work)   # hands_calib.json is written in work, not here
    print(f"work dir: {work}")
    checks = []

    def check(name, ok, detail=""):
        checks.append(bool(ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}")

    fake_log = os.path.join(work, "fake.log")
    fake = subprocess.Popen([PY, "-u", os.path.join(HERE, "fake_rgmp_server.py"), "--rgmp-port", str(RGMP_PORT),
                             "--rcsp-port", str(RCSP_PORT), "--ramp", "0.3"], cwd=HERE,
                            stdout=open(fake_log, "w"), stderr=subprocess.STDOUT)
    time.sleep(2.0)
    st = State(); st.start()

    # ── part A ──
    print("\n== part A: fake two-glove stream ==")
    # a bad calibration already on disk (left index inverted, like a failed "fist L"): must be ignored
    bad = {"bridge": {"left": {f: [0.0, 1.0] for f in FINGERS}, "right": {f: [0.0, 1.0] for f in FINGERS}}}
    bad["bridge"]["left"]["index"] = [0.2909, 0.2754]
    json.dump(bad, open(os.path.join(work, "hands_calib.json"), "w"))
    gl = FakeGloves(); gl.start()
    sim, sim_log = start_sim(work, "hands_A", a.viewer)

    def wait_for(cond, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if st.last and cond(st.last):
                return True
            time.sleep(0.05)
        return False

    def settle(sec):
        time.sleep(sec)
        return st.last

    try:
        ok = wait_for(lambda s: s["left"]["glove"] is not None and s["right"]["glove"] is not None, 30)
        check("hands_sim started, both gloves seen", ok, f"see {sim_log}" if not ok else "")
        time.sleep(6.0)                                       # LED worker lists the gloves
        s = settle(0.5)
        check("A1 open: both LEDs green, fingers straight",
              s["left"]["led"] == "green" and s["right"]["led"] == "green"
              and max(abs(v) for v in s["left"]["deg"].values()) < 5,
              f"L {s['left']['led']} R {s['right']['led']} deg L {s['left']['deg']}")

        gl.curl = {"left": 0.5, "right": 0.5}; s = settle(0.6)
        check("A0 inverted index calibration on disk is ignored (warning, raw value used)",
              "ignoring the open/fist values of index" in open(sim_log).read()
              and abs(s["left"]["target"]["index"] - 0.5) < 0.01,
              f"left index target {s['left']['target']['index']:.3f} for raw 0.5")

        gl.curl = {"left": 0.10, "right": 0.10}; time.sleep(0.7); cmd("open both")
        gl.curl = {"left": 0.90, "right": 0.90}; time.sleep(0.7); cmd("fist both")
        gl.curl = {"left": 0.50, "right": 0.50}; s = settle(0.5)
        check("A2 calibration: raw 0.50 -> 0.50 (open 0.10, fist 0.90, mean of last frames)",
              all(abs(s[h]["target"]["index"] - 0.5) < 0.01 for h in ("left", "right")),
              f"targets L {s['left']['target']['index']:.3f} R {s['right']['target']['index']:.3f}")
        gl.curl = {"left": 0.30, "right": 0.10}; time.sleep(0.7); cmd("open L")
        gl.curl = {"left": 0.28, "right": 0.90}; time.sleep(0.7); cmd("fist L")
        gl.curl = {"left": 0.60, "right": 0.60}; s = settle(0.5)
        check("A2b bad fist L (0.30 -> 0.28) rejected: left uses raw 0.60, right keeps its calibration (0.625)",
              "REJECTED" in open(sim_log).read() and abs(s["left"]["target"]["index"] - 0.60) < 0.01
              and abs(s["right"]["target"]["index"] - 0.625) < 0.01,
              f"L {s['left']['target']['index']:.3f} R {s['right']['target']['index']:.3f}")
        cmd("calib reset both"); time.sleep(0.3)

        cmd("obj off"); cmd("limit force 0"); cmd("limit angle 80")
        res = {}
        for v in (0.5, 0.7, 1.0):
            gl.curl = {"left": v, "right": v}
            s = settle(1.2)
            res[v] = (s["left"]["led"], s["left"]["angle_pct"], max(s["left"]["deg"].values()))
        check("A3 angle limit 80%: 0.5 green, 0.7 yellow, 1.0 red; index stops at 80% (224 deg)",
              res[0.5][0] == "green" and res[0.7][0] == "yellow" and res[1.0][0] == "red"
              and res[1.0][2] <= 0.8 * 280 + 3,
              " | ".join(f"{k}: {v[0]} {v[1]:.1f}% max {v[2]:.0f} deg" for k, v in res.items()))

        gl.curl = {"left": 1.0, "right": 0.3}; s = settle(1.2)
        check("A4 hands independent: left red, right green",
              s["left"]["led"] == "red" and s["right"]["led"] == "green",
              f"L {s['left']['led']} {s['left']['angle_pct']:.0f}%  R {s['right']['led']} {s['right']['angle_pct']:.0f}%")

        gl.curl = {"left": 0.0, "right": 0.0}; time.sleep(0.8)
        cmd("obj on"); cmd("limit angle 100"); cmd("limit force 10"); time.sleep(0.3)
        for k in range(1, 26):                                 # 0.3 s fist
            gl.curl = {"left": k / 25, "right": k / 25}; time.sleep(0.012)
        s = settle(3.0)
        fl, fr = s["left"]["force_n"], s["right"]["force_n"]
        check("A5 force cap 10 N: grip settles at 9-11 N, LED red (at the cap), first-contact peak < 25 N",
              9 <= fl <= 11 and 9 <= fr <= 11 and s["left"]["led"] == "red" and s["left"]["peak_n"] < 25,
              f"L {fl:.2f} N R {fr:.2f} N, peak L {s['left']['peak_n']:.1f} N, LED {s['left']['led']}")
        cmd("limit force 0"); s = settle(2.0)
        check("A5b no force cap: grip well above 10 N", s["left"]["force_n"] > 20,
              f"L {s['left']['force_n']:.1f} N, LED {s['left']['led']} (angle {s['left']['angle_pct']:.0f}%)")

        gl.present["right"] = False; s = settle(1.2)
        check("A6 right glove silent -> right LED off, left still live",
              s["right"]["led"] == "off" and s["right"]["glove"] is None and s["left"]["led"] != "off",
              f"R {s['right']['led']} L {s['left']['led']}")
        gl.present["right"] = True; time.sleep(0.5)
        cmd("s"); time.sleep(0.3)
        n_before = len(open(fake_log).read().splitlines())
        cmd("q")
        try:
            sim.wait(10)
            exited = True
        except subprocess.TimeoutExpired:
            exited = False
        time.sleep(0.3)
        tail = open(fake_log).read().splitlines()[n_before:]
        check("A7 q: hands_sim exits, both LEDs off",
              exited and any("LED left: rgb(0,0,0)" in l for l in tail) and any("LED right: rgb(0,0,0)" in l for l in tail),
              f"exited {exited}, last LED lines {[l for l in tail if 'LED' in l][-2:]}")
    finally:
        if sim.poll() is None:
            sim.kill()
        gl.running = False
    led_lines = [l for l in open(fake_log).read().splitlines() if "LED" in l]
    check("LED commands per glove: left and right both got red and green",
          all(any(f"LED {h}: rgb({c})" in l for l in led_lines) for h in ("left", "right")
              for c in ("255,0,0", "0,255,0")),
          f"{len(led_lines)} LED commands")
    out_a = open(sim_log).read()
    check("part A: no Python traceback", "Traceback" not in out_a)

    # ── part C: Coil positioning ──
    print("\n== part C: Coil positioning (fake gloves with Coil positions, tremor) ==")
    gl = FakeGloves(); gl.coil = True; gl.start()
    home = {"left": [0.0, 0.13, 0.30], "right": [0.0, -0.13, 0.30]}
    simc, log_c = start_sim(work, "hands_C", a.viewer, led=True, coil=True)
    dist = lambda p, q: sum((x - y) ** 2 for x, y in zip(p, q)) ** 0.5

    def move(side, axis, delta, seconds=1.0):
        steps = int(seconds / 0.012)
        b0 = gl.base[side][axis]
        for k in range(1, steps + 1):
            gl.base[side][axis] = b0 + delta * k / steps
            time.sleep(0.012)

    def turn(turns, seconds=1.0):
        """turns: {side: (robot axis, deg)} applied together, smoothly."""
        steps = int(seconds / 0.012)
        for _ in range(steps):
            for side, (axis, deg) in turns.items():
                gl.turn(side, axis, deg / steps)
            time.sleep(0.012)

    def ang(q1, q2):
        import math
        return math.degrees(2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(q1, q2))))))

    ident = [1.0, 0.0, 0.0, 0.0]

    try:
        wait_for(lambda s: s["left"]["glove"] is not None and s["right"]["glove"] is not None, 30)
        # the gloves start in arbitrary orientations: the simulated hands must still start palm up
        gl.rot = {"left": qa((0.3, 1.0, 0.2), 70), "right": qa((1.0, -0.4, 0.5), -50)}
        time.sleep(1.0)
        s = settle(1.0)
        check("C1 Coil data but not engaged: wrists stay home",
              all(dist(s[h]["wrist"], home[h]) < 1e-4 for h in ("left", "right")) and not s["left"]["engaged"],
              f"L {s['left']['wrist']} engaged {s['left']['engaged']}")
        cmd("filter off"); time.sleep(0.2)
        gl.engaged = True; s = settle(0.6)
        check("C2 engage: no jump (wrists within 1 mm of home, palms still up within 1 deg although the "
              "gloves are turned 70 / 50 deg)",
              all(dist(s[h]["wrist"], home[h]) < 0.001 and ang(s[h]["quat"], ident) < 1.0 for h in ("left", "right"))
              and s["left"]["engaged"],
              f"L off by {1000 * dist(s['left']['wrist'], home['left']):.2f} mm, palm turned L "
              f"{ang(s['left']['quat'], ident):.2f} R {ang(s['right']['quat'], ident):.2f} deg")

        # ---- wrist rotation ----
        res = {}
        for k, axis in enumerate(((1, 0, 0), (0, 1, 0), (0, 0, 1))):
            cmd("rot reset"); time.sleep(0.3)
            turn({"left": (axis, 40), "right": (axis, 40)}, 0.8); s = settle(0.5)
            res["XYZ"[k]] = (s["left"]["rpy"], s["right"]["rpy"])
        ok = all(abs(l[k] - (40 if k == i else 0)) < 2 and abs(r[k] - (40 if k == i else 0)) < 2
                 for i, (l, r) in enumerate(res.values()) for k in range(3))
        check("C3r rotation axes: both gloves +40 deg about X, Y, Z -> both hands roll / pitch / yaw +40 "
              "(same sign on both hands = not mirrored)", ok,
              " | ".join(f"{a}: L {[round(v) for v in l]} R {[round(v) for v in r]}" for a, (l, r) in res.items()))

        cmd("rot reset"); time.sleep(0.3)
        # palm up -> palm down by pronation: the left forearm rolls about +X, the right about -X
        half = {}
        turn({"left": ((1, 0, 0), 90), "right": ((1, 0, 0), -90)}, 0.8); s = settle(0.5)
        for h, sg in (("left", 1), ("right", -1)):
            half[h] = rot_vec(s[h]["quat"], (0, sg, 0))[2]           # thumb direction, z
        turn({"left": ((1, 0, 0), 90), "right": ((1, 0, 0), -90)}, 0.8); s = settle(0.5)
        pd = {h: (rot_vec(s[h]["quat"], (0, 0, 1))[2], rot_vec(s[h]["quat"], (0, sg, 0))[1])
              for h, sg in (("left", 1), ("right", -1))}
        check("C3p palm up -> palm down (pronation): at 90 deg both thumbs point up; at 180 both palms face "
              "down and both thumbs point to the middle (left thumb -Y, right thumb +Y)",
              half["left"] > 0.95 and half["right"] > 0.95 and pd["left"][0] < -0.95 and pd["right"][0] < -0.95
              and pd["left"][1] < -0.95 and pd["right"][1] > 0.95,
              f"thumb z at 90: L {half['left']:.2f} R {half['right']:.2f}; palm normal z at 180: "
              f"L {pd['left'][0]:.2f} R {pd['right'][0]:.2f}; thumb y: L {pd['left'][1]:.2f} R {pd['right'][1]:.2f}")
        cmd("rot reset"); time.sleep(0.3)

        w0 = settle(0.3)["left"]["wrist"]; r0 = st.last["right"]["wrist"]
        move("left", 0, 0.10); move("left", 1, 0.05); move("left", 2, 0.05); s = settle(0.5)
        d = [s["left"]["wrist"][k] - w0[k] for k in range(3)]
        check("C3 axes at scale 1: left glove +10 cm X, +5 cm Y, +5 cm Z -> left wrist the same; right still",
              abs(d[0] - 0.10) < 0.002 and abs(d[1] - 0.05) < 0.002 and abs(d[2] - 0.05) < 0.002
              and dist(s["right"]["wrist"], r0) < 0.001,
              f"dX {d[0] * 1000:.1f} dY {d[1] * 1000:.1f} dZ {d[2] * 1000:.1f} mm, right moved "
              f"{1000 * dist(s['right']['wrist'], r0):.2f} mm")

        before = settle(0.3)["left"]["wrist"]
        cmd("scale 0.3"); s = settle(0.4)
        jump = dist(s["left"]["wrist"], before)
        w1, g1 = s["left"]["wrist"][0], s["left"]["raw_pos"][0]
        move("left", 0, -0.10); s = settle(0.5)
        dx, gx = s["left"]["wrist"][0] - w1, s["left"]["raw_pos"][0] - g1
        check("C4 scale 0.3: no jump on the switch; 10 cm of hand travel -> 3 cm (ghost moves 10 cm)",
              jump < 0.0005 and abs(dx + 0.030) < 0.001 and abs(gx + 0.100) < 0.002,
              f"switch step {jump * 1000:.3f} mm, wrist {dx * 1000:.1f} mm, ghost {gx * 1000:.1f} mm, "
              f"scale {s['left']['scale']}")

        gl.tremor_mm, gl.noise_mm = 1.0, 0.3
        cmd("filter off"); s = settle(4.0)
        j_off = s["left"]["jitter"]
        cmd("filter 3"); s = settle(4.0)
        j_on = s["left"]["jitter"]
        check("C5 filter: off -> raw = filtered; on (3 Hz) -> filtered jitter < half the raw (1 mm 10 Hz tremor)",
              j_off["raw"] and j_on["raw"] and abs(j_off["raw"] - j_off["filt"]) < 1e-6
              and j_on["filt"] < 0.5 * j_on["raw"],
              f"off raw {j_off['raw']:.2f} filt {j_off['filt']:.2f} | 3 Hz raw {j_on['raw']:.2f} "
              f"filt {j_on['filt']:.2f} mm (sim x0.3: {j_on['filt'] * 0.3:.2f} mm)" if j_off["raw"] and j_on["raw"]
              else f"jitter {j_off} {j_on}")

        # the bridge resent a glove frame (same ts_us) about once a second on the real right glove:
        # the jitter read-out showed "--" for that hand. Now repeats are skipped.
        gl.dup_every = 80
        cmd("jitter reset"); s = settle(4.0)
        jl, jr = s["left"]["jitter"], s["right"]["jitter"]
        gl.dup_every = 0
        check("C5b repeated glove frames (1 per second): jitter still shown for both hands, cumulative "
              "(>= 2 s of still data)",
              jl["raw"] is not None and jr["raw"] is not None and jl["raw_hf"] is not None
              and jl["still_s"] >= 2 and jr["still_s"] >= 2,
              f"L raw {jl['raw']} filt {jl['filt']} (>3 Hz {jl['raw_hf']} / {jl['filt_hf']}) {jl['still_s']:.1f} s | "
              f"R raw {jr['raw']} {jr['still_s']:.1f} s")
        gl.tremor_mm, gl.noise_mm = 0.0, 0.0

        import statistics
        cmd("rot reset"); time.sleep(0.3)
        gl.rot_tremor_deg = 2.0
        sd = {}
        for mode in ("off", "3"):
            cmd(f"rotfilter {mode}"); time.sleep(0.5)
            n0 = len(st.history); time.sleep(2.5)
            rolls = [h["left"]["rpy"][0] for h in st.history[n0:]]
            sd[mode] = statistics.pstdev(rolls) if len(rolls) > 20 else None
        gl.rot_tremor_deg = 0.0
        cmd("rotfilter off"); time.sleep(0.3)
        turn({"left": ((0, 1, 0), 25)}, 0.5); s = settle(0.4)
        exact = abs(s["left"]["rpy"][1] - 25) < 0.5
        cmd("rotfilter 3"); time.sleep(0.2)
        check("C5r rotation filter: 2 deg 10 Hz roll tremor -> roll std with 3 Hz filter < half of filter off; "
              "filter off follows exactly (25 deg pitch within 0.5 deg)",
              sd["off"] and sd["3"] is not None and sd["3"] < 0.5 * sd["off"] and exact,
              f"roll std off {sd['off']:.2f} deg, 3 Hz {sd['3']:.2f} deg; pitch {s['left']['rpy'][1]:.2f} deg"
              if sd["off"] and sd["3"] is not None else f"{sd}")

        held = settle(0.3)["left"]["wrist"]
        gl.engaged = False; time.sleep(0.2)
        move("left", 1, 0.05); s = settle(0.4)
        still = dist(s["left"]["wrist"], held) < 0.0005
        before = s["left"]["wrist"]
        gl.engaged = True; s = settle(0.5)
        check("C6 clutch: disengaged hand holds while the glove moves 5 cm; re-engage without a jump",
              still and dist(s["left"]["wrist"], before) < 0.002 and not False,
              f"held {still}, re-engage step {1000 * dist(s['left']['wrist'], before):.2f} mm")

        q_held = settle(0.3)["left"]["quat"]
        gl.engaged = False; time.sleep(0.2)
        turn({"left": ((0, 0, 1), 90)}, 0.6); s = settle(0.4)
        rot_still = ang(s["left"]["quat"], q_held)
        q_before = s["left"]["quat"]; yaw0 = s["left"]["rpy"][2]
        gl.engaged = True; s = settle(0.5)
        step = ang(s["left"]["quat"], q_before)
        turn({"left": ((0, 0, 1), 30)}, 0.5); s = settle(0.5)
        dyaw = s["left"]["rpy"][2] - yaw0
        check("C6r rotation clutch: glove turns 90 deg while disengaged -> hand holds; re-engage without a "
              "jump; then +30 deg yaw -> hand +30 deg",
              rot_still < 0.5 and step < 1.0 and abs(dyaw - 30) < 2,
              f"held (moved {rot_still:.2f} deg), re-engage step {step:.2f} deg, yaw change {dyaw:.1f} deg")

        cmd("rot off"); time.sleep(0.2)
        turn({"left": ((1, 0, 0), 30), "right": ((0, 1, 0), 30)}, 0.4); s = settle(0.4)
        off_ok = all(ang(s[h]["quat"], ident) < 0.1 for h in ("left", "right"))
        cmd("rot on"); s = settle(0.4)
        on_step = max(ang(s[h]["quat"], ident) for h in ("left", "right"))
        turn({"left": ((0, 1, 0), 20), "right": ((0, 1, 0), 20)}, 0.4); s = settle(0.5)
        check("C6o rot off -> palms up while the gloves turn; rot on -> no jump, then follows (+20 deg pitch)",
              off_ok and on_step < 1.0 and abs(s["left"]["rpy"][1] - 20) < 2 and abs(s["right"]["rpy"][1] - 20) < 2,
              f"off: palms up {off_ok}; on: step {on_step:.2f} deg; pitch L {s['left']['rpy'][1]:.1f} "
              f"R {s['right']['rpy'][1]:.1f}")
        cmd("rot reset"); time.sleep(0.3)

        cmd("scale 1"); cmd("home"); time.sleep(0.5)
        cmd("obj on"); cmd("limit force 10"); time.sleep(0.6)
        for k in range(1, 26):
            gl.curl = {"left": k / 25, "right": k / 25}; time.sleep(0.012)
        import math
        t0, forces = time.time(), []
        while time.time() - t0 < 3.5:
            ph = 2 * math.pi * 0.5 * (time.time() - t0)
            gl.base["left"][0] = 1.0 + 0.03 * math.sin(ph); gl.base["right"][2] = -0.10 + 0.03 * math.sin(ph)
            if time.time() - t0 > 2.0 and st.last:
                forces.append((st.last["left"]["force_n"], st.last["right"]["force_n"],
                               st.last["left"]["led"], st.last["right"]["led"]))
            time.sleep(0.012)
        fl = [f[0] for f in forces]; fr = [f[1] for f in forces]
        leds = [f[2] for f in forces] + [f[3] for f in forces]
        red_share = leds.count("red") / len(leds) if leds else 0
        # red from 97 %: a grip held at the cap (98.9-99.7 % while moving) must show red all the time
        check("C7 force cap 10 N while both hands move: grip 9-11 N, LEDs red in every sample",
              forces and 9 <= min(fl) and max(fl) <= 11 and 9 <= min(fr) and max(fr) <= 11
              and red_share == 1.0,
              f"L {min(fl):.2f}-{max(fl):.2f} N, R {min(fr):.2f}-{max(fr):.2f} N, LED red in "
              f"{red_share * 100:.0f} % of {len(leds)} samples" if forces else "no samples")

        # grip at the cap while both wrists rotate: palm up -> palm down -> up, then flexion +-40 deg
        cmd("rot reset"); time.sleep(0.3)
        forces, mounted, t0 = [], True, time.time()
        seq = [({"left": ((1, 0, 0), 180), "right": ((1, 0, 0), -180)}, 2.0),
               ({"left": ((1, 0, 0), -180), "right": ((1, 0, 0), 180)}, 2.0),
               ({"left": ((0, 1, 0), 40), "right": ((0, 1, 0), 40)}, 1.0),
               ({"left": ((0, 1, 0), -80), "right": ((0, 1, 0), -80)}, 1.5),
               ({"left": ((0, 0, 1), 40), "right": ((0, 0, 1), -40)}, 1.0)]
        min_up = {"left": 1.0, "right": 1.0}
        for turns, secs in seq:
            steps = int(secs / 0.012)
            for _ in range(steps):
                for side, (axis, deg) in turns.items():
                    gl.turn(side, axis, deg / steps)
                if time.time() - t0 > 0.5 and st.last:
                    s = st.last
                    forces.append((s["left"]["force_n"], s["right"]["force_n"], s["left"]["led"], s["right"]["led"]))
                    for h in ("left", "right"):
                        min_up[h] = min(min_up[h], rot_vec(s[h]["quat"], (0, 0, 1))[2])
                time.sleep(0.012)
        fl = [f[0] for f in forces]; fr = [f[1] for f in forces]
        leds = [f[2] for f in forces] + [f[3] for f in forces]
        red_share = leds.count("red") / len(leds) if leds else 0
        check("C7r force cap 10 N while both wrists rotate (palm up -> down -> up, flexion +-40, deviation 40): "
              "grip 9-11 N, LEDs red in every sample, the hands really turned palm down",
              forces and 9 <= min(fl) and max(fl) <= 11 and 9 <= min(fr) and max(fr) <= 11 and red_share == 1.0
              and min_up["left"] < -0.9 and min_up["right"] < -0.9,
              f"L {min(fl):.2f}-{max(fl):.2f} N, R {min(fr):.2f}-{max(fr):.2f} N, LED red in {red_share * 100:.0f} % "
              f"of {len(leds)} samples, lowest palm-normal z L {min_up['left']:.2f} R {min_up['right']:.2f}"
              if forces else "no samples")
        cmd("rot reset"); time.sleep(0.3)

        # C8: scripted operator for the reach-and-hold task (closed loop on the reported wrist and target)
        cmd("task"); time.sleep(0.5)
        t0, done = time.time(), False
        while time.time() - t0 < 90:
            s = st.last
            if s and s.get("last_task"):
                done = True
                break
            if s and s.get("task_active"):
                for h in ("left", "right"):
                    tk = s[h].get("task")
                    if not tk or tk["done"]:
                        continue
                    err = [t - w for t, w in zip(tk["target"], s[h]["wrist"])]
                    for k in range(3):
                        gl.base[h][k] += max(-0.002, min(0.002, 4.0 * err[k] * 0.012 / s[h]["scale"]))
            time.sleep(0.012)
        res = st.last.get("last_task") if st.last else None
        summ = {h: res[h]["summary"] for h in ("left", "right")} if res else None
        check("C8 reach and hold, both hands: 5 of 5 targets held each, result file written",
              done and summ["left"]["held"] == 5 and summ["right"]["held"] == 5
              and os.path.isdir(os.path.join(work, "hands_tasks")) and os.listdir(os.path.join(work, "hands_tasks")),
              f"{summ}" if summ else "task did not finish")
    finally:
        cmd("q")
        try:
            simc.wait(10)
        except subprocess.TimeoutExpired:
            simc.kill()
        gl.running = False
    check("part C: no Python traceback", "Traceback" not in open(log_c).read())

    # ── part E: pick-up (free object on a table, scripted grasp and lift through the fake gloves) ──
    print("\n== part E: pick-up (free object, scripted grasp-and-lift per hand and with both hands) ==")
    gl = FakeGloves(); gl.coil = True; gl.start()
    sime, log_e = start_sim(work, "hands_E", a.viewer, led=True, coil=True)
    TABLE_Z, PX = 0.20, 0.05
    REST = {"can": 0.05, "cube": 0.025, "ball": 0.03, "egg": 0.0317}
    HALF = {"can": 0.03, "cube": 0.025, "ball": 0.03, "egg": 0.022}

    def glide(targets, seconds):
        """targets: {side: robot-frame glove position}; moves the fake gloves there smoothly."""
        steps = max(1, int(seconds / 0.012))
        b0 = {h: list(gl.base[h]) for h in targets}
        for k in range(1, steps + 1):
            for h, tgt in targets.items():
                gl.base[h] = [b + (t - b) * k / steps for b, t in zip(b0[h], tgt)]
            time.sleep(0.012)

    def grasp_and_lift(sides, shape, dx=0.0, lift=0.08):
        """Hands palm-in beside the object, close, lift 8 cm, hold 3.5 s. Returns the last pick result."""
        cmd("home"); time.sleep(0.6)                       # palms up at home; re-anchored at the gloves' pose
        ref = {h: list(gl.base[h]) for h in sides}
        cmd(f"pick {shape}"); time.sleep(0.3)
        cmd("pick"); time.sleep(0.2)
        home_h = {"left": [0.0, 0.13, 0.30], "right": [0.0, -0.13, 0.30]}
        goal, appr = {}, {}
        for h in sides:
            sg = 1 if h == "left" else -1
            g = [PX + dx, sg * (HALF[shape] + 0.012), TABLE_Z + REST[shape]]
            goal[h] = [r + (gv - hv) for r, gv, hv in zip(ref[h], g, home_h[h])]
            appr[h] = [goal[h][0], goal[h][1] + sg * 0.06, goal[h][2]]
        turn({h: ((1, 0, 0), 90 if h == "left" else -90) for h in sides}, 0.8)   # palms face the object
        glide(appr, 0.8); glide(goal, 0.6); time.sleep(0.4)
        for k in range(1, 26):
            gl.curl = {h: (k / 25 if h in sides else 0.0) for h in ("left", "right")}; time.sleep(0.016)
        time.sleep(1.2)
        glide({h: [goal[h][0], goal[h][1], goal[h][2] + lift] for h in sides}, 0.8)
        grips = []
        t0 = time.time()
        while time.time() - t0 < 3.5:
            s = st.last
            if s and s["pick"]["lift_m"] is not None and s["pick"]["lift_m"] > 0.05:
                grips.append(max(s[h]["force_n"] for h in sides))
            time.sleep(0.05)
        s = st.last
        return s, grips

    def release_and_home(sides):
        gl.curl = {"left": 0.0, "right": 0.0}; time.sleep(0.8)
        glide({h: [b + (0.06 if h == "left" else -0.06) * (k == 1) for k, b in enumerate(gl.base[h])] for h in sides}, 0.5)
        gl.rot = {"left": [0.0, 0.0, 0.0, 1.0], "right": [0.0, 0.0, 0.0, 1.0]}

    try:
        wait_for(lambda s: s["left"]["glove"] is not None and s["right"]["glove"] is not None, 30)
        gl.engaged = True; time.sleep(1.0)
        cmd("obj on"); cmd("limit force 10"); time.sleep(0.3)
        cmd("pick can"); s = settle(0.8)
        pk = s["pick"]
        check("E1 pick can: object on the table in the middle at rest, bars off, state 'on table'",
              pk["on"] and abs(pk["lift_m"]) < 0.002 and abs(pk["pos"][1]) < 0.002 and not s["ball_on"]
              and pk["state"] == "on table",
              f"pos {pk['pos']} lift {pk['lift_m']} state {pk['state']} bars {'on' if s['ball_on'] else 'off'}")

        res = {}
        for name, sides, shape, dx in (("right, can", ["right"], "can", 0.0), ("left, ball", ["left"], "ball", 0.0),
                                       ("both, can", ["left", "right"], "can", 0.0)):
            s, grips = grasp_and_lift(sides, shape, dx)
            last = s["pick"]["last"] or {}
            res[name] = (last, grips, s)
            # settled grip = the last 1.5 s of the hold (30 samples at 20 Hz); when the lift stops, the
            # object shifts in the hand and the grip can spike briefly (ball: 16 N for ~0.1 s), like the
            # first-contact overshoot of the force cap (measured 16.4 and 18.0 N in two runs) -> bounded at
            # 2.5x the cap, settled within 8.5-11.5 N
            settled = grips[-30:]
            ok = (last.get("result") == "SUCCESS" and set(last.get("hands", [])) == set(sides)
                  and len(grips) >= 30 and 8.5 <= min(settled) and max(settled) <= 11.5 and max(grips) < 25)
            check(f"E2 grasp and lift ({name}): SUCCESS (5 cm for 2 s), settled grip on the object 8.5-11.5 N "
                  f"(10 N cap), any spike < 25 N" + ("; two-hand assist attached" if len(sides) == 2
                                                       else "; pure physics, no assist"),
                  ok and bool(last.get("assisted")) == (len(sides) == 2),
                  f"result {last.get('result')} in {last.get('time_s')} s, hands {last.get('hands')}, assisted "
                  f"{last.get('assisted')}, max height {last.get('max_height_cm')} cm, settled grip "
                  f"{min(settled):.2f}-{max(settled):.2f} N, peak while lifted {max(grips):.2f} N, LED "
                  f"{[s[h]['led'] for h in sides]}" if len(grips) >= 30 else f"result {last}, {len(grips)} lifted samples")
            if name == "right, can":
                gl.curl = {"left": 0.0, "right": 0.0}; time.sleep(1.5)     # open the hand while it is lifted
                s2 = st.last["pick"]
                check("E3 open the hand while lifted -> state 'dropped', object falls back down",
                      s2["state"] == "dropped" and s2["lift_m"] < 0.02,
                      f"state {s2['state']} lift {s2['lift_m']}")
            release_and_home(sides)
        cmd("pick reset"); s = settle(0.8)
        check("E4 pick reset: object back in the middle, on the table",
              abs(s["pick"]["lift_m"]) < 0.002 and abs(s["pick"]["pos"][1]) < 0.002 and s["pick"]["state"] == "on table",
              f"pos {s['pick']['pos']} state {s['pick']['state']}")
        files = [f for f in os.listdir(os.path.join(work, "hands_tasks")) if f.endswith("_pick.json")] \
            if os.path.isdir(os.path.join(work, "hands_tasks")) else []
        check("E5 one result file per trial (hands_tasks/*_pick.json)", len(files) >= 3, f"{len(files)} files")

        # ---- breakable egg (egg_sim.py's 30 N threshold), standing in its cup ----
        cmd("pick egg"); s = settle(1.0)
        pk = s["pick"]
        check("E7 pick egg: whole egg standing still in its cup in the middle of the table",
              pk["shape"] == "egg" and not pk["broken"] and abs(pk["lift_m"]) < 0.002 and abs(pk["pos"][1]) < 0.002
              and pk["state"] == "on table", f"pos {pk['pos']} lift {pk['lift_m']} state {pk['state']}")
        # a small egg has to sit near the finger roots (palm 2 cm behind it), not in the middle of the palm
        s, grips = grasp_and_lift(["right"], "egg", dx=-0.02)
        last = s["pick"]["last"] or {}
        check("E8 egg, force cap 10 N, right hand: SUCCESS, not broken, peak grip below the 30 N break threshold",
              last.get("result") == "SUCCESS" and last.get("broken") is False and s["pick"]["egg_peak_n"] < 30,
              f"result {last.get('result')}, broken {last.get('broken')}, peak {s['pick']['egg_peak_n']} N, "
              f"held at {min(grips[-30:]):.2f}-{max(grips[-30:]):.2f} N" if len(grips) >= 30 else f"result {last}")
        release_and_home(["right"])
        cmd("limit force 0"); time.sleep(0.2)
        s, grips = grasp_and_lift(["right"], "egg", dx=-0.02)     # hard squeeze, no cap
        last = s["pick"]["last"] or {}
        check("E9 egg, force cap off, hard squeeze: BROKEN (state BROKEN, trial result BROKEN)",
              s["pick"]["broken"] and s["pick"]["state"] == "BROKEN" and last.get("result") == "BROKEN",
              f"state {s['pick']['state']}, trial {last.get('result')}, peak {s['pick']['egg_peak_n']} N")
        release_and_home(["right"])
        cmd("limit force 10"); cmd("pick reset"); s = settle(1.0)
        check("E10 pick reset after breaking: a whole egg back in the cup",
              not s["pick"]["broken"] and s["pick"]["state"] == "on table" and abs(s["pick"]["lift_m"]) < 0.002,
              f"broken {s['pick']['broken']} state {s['pick']['state']} lift {s['pick']['lift_m']}")
    finally:
        cmd("q")
        try:
            sime.wait(10)
        except subprocess.TimeoutExpired:
            sime.kill()
        gl.running = False
    oute = open(log_e).read()
    m = re.search(r"realtime ratio over \d+ s: min ([\d.]+)\s+mean ([\d.]+)\s+max ([\d.]+)", oute)
    check("E6 real-time ratio 0.95-1.05 during the pick-up trials, no Python traceback",
          m and 0.95 <= float(m.group(1)) and float(m.group(3)) <= 1.05 and "Traceback" not in oute,
          m.group(0) if m else "no ratio line")

    # ── part B ──
    print("\n== part B: fake_rgmp_server -> glove_bridge --no-rcsp -> hands_sim ==")
    calib_copy = os.path.join(work, "calib_copy.json")
    if os.path.exists(os.path.join(HERE, "calib.json")):
        shutil.copy(os.path.join(HERE, "calib.json"), calib_copy)
    bridge_log = os.path.join(work, "bridge.log")
    bridge = subprocess.Popen([PY, os.path.join(HERE, "glove_bridge.py"), "--no-rcsp", "--rate", "83.33",
                               "--rgmp-port", str(RGMP_PORT), "--udp", f"127.0.0.1:{SIM_PORT}",
                               "--calib", calib_copy, "--quiet"], cwd=HERE, stdout=open(bridge_log, "w"),
                              stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    st.history.clear()
    simb, logb = start_sim(work, "hands_B", False, led=False)
    try:
        time.sleep(22)
        cmd("q")
        simb.wait(15)
    except subprocess.TimeoutExpired:
        simb.kill()
    finally:
        bridge.kill()
        fake.kill()
    outb = open(logb).read()
    m = re.search(r"realtime ratio over \d+ s: min ([\d.]+)\s+mean ([\d.]+)\s+max ([\d.]+)", outb)
    check("B real-time ratio 0.95-1.05", m and 0.95 <= float(m.group(1)) and float(m.group(3)) <= 1.05,
          m.group(0) if m else "no ratio line")
    hist = [h for h in st.history if h["left"]["glove"] and h["right"]["glove"]]
    rng = {side: (max(h[side]["deg"]["index"] for h in hist) - min(h[side]["deg"]["index"] for h in hist))
           if hist else 0 for side in ("left", "right")}
    check("B both hands receive bridge data and the fingers move (index range > 60 deg)",
          hist and rng["left"] > 60 and rng["right"] > 60,
          f"{len(hist)} state samples with both hands; index range L {rng['left']:.0f} R {rng['right']:.0f} deg")
    check("part B: no Python traceback", "Traceback" not in outb)

    # ── part D: Coil chain ──
    print("\n== part D: fake_rgmp_server (Coil) -> glove_bridge --no-rcsp (engaged) -> hands_sim (Coil) ==")
    fake2 = subprocess.Popen([PY, "-u", os.path.join(HERE, "fake_rgmp_server.py"), "--rgmp-port", str(RGMP_PORT),
                              "--rcsp-port", str(RCSP_PORT)], cwd=HERE, stdout=open(os.path.join(work, "fake2.log"), "w"),
                             stderr=subprocess.STDOUT)
    time.sleep(2.0)
    bridge2 = subprocess.Popen([PY, os.path.join(HERE, "glove_bridge.py"), "--no-rcsp", "--rate", "83.33",
                                "--rgmp-port", str(RGMP_PORT), "--udp", f"127.0.0.1:{SIM_PORT}", "--calib", calib_copy,
                                "--quiet"], cwd=HERE, stdout=open(os.path.join(work, "bridge2.log"), "w"),
                               stderr=subprocess.STDOUT, stdin=subprocess.PIPE, text=True)
    st.history.clear()
    simd, logd = start_sim(work, "hands_D", False, led=False, coil=True)
    try:
        time.sleep(4)
        bridge2.stdin.write("e\n"); bridge2.stdin.flush()          # ENGAGE in the bridge window
        time.sleep(16)
        cmd("q")
        simd.wait(15)
    except subprocess.TimeoutExpired:
        simd.kill()
    finally:
        for p in (bridge2, fake2):
            p.kill()
    outd = open(logd).read()
    m = re.search(r"realtime ratio over \d+ s: min ([\d.]+)\s+mean ([\d.]+)\s+max ([\d.]+)", outd)
    check("D real-time ratio 0.95-1.05", m and 0.95 <= float(m.group(1)) and float(m.group(3)) <= 1.05,
          m.group(0) if m else "no ratio line")
    eng = [h for h in st.history if h["left"].get("engaged") and h["right"].get("engaged")]
    span = {side: max(max(h[side]["wrist"][k] for h in eng) - min(h[side]["wrist"][k] for h in eng)
                      for k in range(3)) if eng else 0 for side in ("left", "right")}
    check("D both wrists follow the fake Coil data once engaged (movement > 2 cm)",
          eng and span["left"] > 0.02 and span["right"] > 0.02,
          f"{len(eng)} engaged samples; largest axis range L {span['left'] * 1000:.0f} mm R {span['right'] * 1000:.0f} mm")
    # fake_rgmp_server turns each Coil sensor +-0.3 rad (17 deg) about the Coil's z axis (= robot -Z)
    # and flips quaternion signs: the hands must yaw smoothly through the bridge's ee_quat
    yaw = {side: [h[side]["rpy"][2] for h in eng] for side in ("left", "right")}
    yrng = {side: (max(v) - min(v)) if v else 0 for side, v in yaw.items()}
    ystep = max((abs(b - a) for v in yaw.values() for a, b in zip(v, v[1:])), default=99)
    other = max((abs(h[side]["rpy"][k]) for h in eng for side in ("left", "right") for k in (0, 1)), default=99)
    check("D wrist rotation through the bridge (ee_quat): yaw range 20-40 deg, no jumps (< 5 deg between "
          "samples), roll and pitch stay < 3 deg",
          eng and 20 <= yrng["left"] <= 40 and 20 <= yrng["right"] <= 40 and ystep < 5 and other < 3,
          f"yaw range L {yrng['left']:.1f} R {yrng['right']:.1f} deg, largest step {ystep:.2f} deg, "
          f"largest roll/pitch {other:.2f} deg")
    check("part D: no Python traceback", "Traceback" not in outd)

    n_fail = checks.count(False)
    print(f"\n{len(checks) - n_fail} of {len(checks)} checks passed" + (f", {n_fail} FAILED" if n_fail else ""))
    if a.keep or n_fail:
        print(f"logs kept in {work}")
    else:
        shutil.rmtree(work, ignore_errors=True)
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
