#!/usr/bin/env python3
"""
synth_operator.py - scripted "operator" for testing egg_sim.py without gloves.

Sends glove_bridge.py-format JSON (right hand only) at 83.33 Hz to UDP :9870 and steers its fake
wrist so the gripper visits a list of waypoints (closed loop: egg_sim.py reports the gripper target
back over --state-port). Pinch is converted to pinch_m with the same open/closed calibration as
egg_sim.py (pinch_calib.json if present, else egg_sim defaults).

  terminal 1:  python egg_sim.py --headless --state-udp 127.0.0.1:9872 --log --operator synth
  terminal 2:  python synth_operator.py --scenario place --trials 2

Scenarios
  place   gentle grasp (travel fraction --grip-frac, default 0.55), carry to target, release
  crush   same path, but full pinch at the grasp (breaks the egg with assist off)
  wander  slow random wandering, no grasp (filter / scaling tests)
Options --tremor-mm (10 Hz sine) and --noise-mm (white noise) add jitter to the wrist signal.
Standard library only.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import socket
import time

PINCH_DEFAULT = {"open": 0.09, "closed": 0.03}
BASE_EE = (1.45, -0.03, -0.13)     # a plausible hand position in the robot frame (m)
RATE = 83.33


def waypoints(scenario, g):
    e, t = (0.50, -0.10), (0.50, 0.20)
    if scenario == "wander":
        return []
    grasp = 1.0 if scenario == "crush" else g
    return [  # (x, y, z, travel fraction, hold seconds)
        (e[0], e[1], 0.62, 0.0, 0.3),
        (e[0], e[1], 0.475, 0.0, 0.3),
        (e[0], e[1], 0.475, grasp, 0.8),
        (e[0], e[1], 0.60, grasp, 0.2),
        (t[0], t[1], 0.60, grasp, 0.2),
        (t[0], t[1], 0.478, grasp, 0.4),
        (t[0], t[1], 0.478, 0.0, 0.8),
        (t[0], t[1], 0.60, 0.0, 30.0),
    ]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--udp", default="127.0.0.1:9870", help="where egg_sim.py listens")
    ap.add_argument("--state-port", type=int, default=9872, help="where egg_sim.py --state-udp sends state")
    ap.add_argument("--cmd", default="127.0.0.1:9871", help="egg_sim.py command port (for 'r' between trials)")
    ap.add_argument("--scenario", default="place", choices=["place", "crush", "wander"])
    ap.add_argument("--grip-frac", type=float, default=0.55, help="pinch travel fraction at the grasp")
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--duration", type=float, default=0, help="wander: seconds to run")
    ap.add_argument("--tremor-mm", type=float, default=0.0, help="10 Hz sine amplitude per axis (mm)")
    ap.add_argument("--noise-mm", type=float, default=0.0, help="white noise std per axis (mm)")
    ap.add_argument("--speed", type=float, default=0.15, help="max hand speed (m/s)")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    random.seed(a.seed)
    pinch = dict(PINCH_DEFAULT)
    if os.path.exists("pinch_calib.json"):
        pinch.update(json.load(open("pinch_calib.json")).get("right", {}))

    host, port = a.udp.rsplit(":", 1)
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (host, int(port))
    chost, cport = a.cmd.rsplit(":", 1)
    st_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    st_sock.bind(("127.0.0.1", a.state_port))
    st_sock.setblocking(False)

    hand = list(BASE_EE)
    frac = 0.0
    seq, t0 = 0, time.time()
    period = 1.0 / RATE
    next_t = time.perf_counter()
    state = None
    trial, wi, hold_since = 0, 0, None
    wps = waypoints(a.scenario, a.grip_frac)
    started = False
    sent_reset_at = time.time()
    wander_goal, results = None, []
    out.sendto(b"r", (chost, int(cport)))
    print(f"[synth] scenario {a.scenario}, {a.trials} trial(s), pinch calib {pinch}")
    while True:
        while True:
            try:
                state = json.loads(st_sock.recvfrom(65536)[0])
            except (BlockingIOError, ConnectionResetError):
                break
        now = time.time()
        if state and a.scenario != "wander":
            if state["trial_active"]:
                started = True
            elif started and now - sent_reset_at > 1.0:
                results.append(state["result"])
                print(f"[synth] trial {state['trial_no']}: {json.dumps(state['result'])}", flush=True)
                trial += 1
                if trial >= a.trials:
                    break
                out.sendto(b"r", (chost, int(cport)))
                sent_reset_at, started, wi, hold_since = now, False, 0, None
        if a.scenario == "wander" and a.duration and now - t0 > a.duration:
            break

        # controller: move the fake hand so the gripper target approaches the waypoint
        vel = [0.0, 0.0, 0.0]
        if state and a.scenario != "wander" and wi < len(wps):
            x, y, z, f, hold = wps[wi]
            err = [x - state["target"][0], y - state["target"][1], z - state["target"][2]]
            dist = math.sqrt(sum(e * e for e in err))
            frac += max(-2 * period, min(2 * period, f - frac))
            if dist < 0.003 and abs(f - frac) < 1e-6:
                hold_since = hold_since or now
                if now - hold_since >= hold:
                    wi, hold_since = wi + 1, None
            else:
                hold_since = None
            vel = [4.0 * e for e in err]
        elif a.scenario == "wander":
            if wander_goal is None or random.random() < 0.004:
                wander_goal = [BASE_EE[0] + random.uniform(-0.1, 0.1), BASE_EE[1] + random.uniform(-0.1, 0.1),
                               BASE_EE[2] + random.uniform(-0.05, 0.05)]
            vel = [1.0 * (g - h) for g, h in zip(wander_goal, hand)]
        sp = math.sqrt(sum(v * v for v in vel))
        if sp > a.speed:
            vel = [v * a.speed / sp for v in vel]
        hand = [h + v * period for h, v in zip(hand, vel)]

        tt = time.time() - t0
        jit = [a.tremor_mm / 1000 * math.sin(2 * math.pi * 10 * tt + k) + random.gauss(0, a.noise_mm / 1000)
               for k in range(3)]
        ee = [h + j for h, j in zip(hand, jit)]
        pinch_m = pinch["open"] - frac * (pinch["open"] - pinch["closed"])
        wrist = [ee[1], ee[0], -ee[2]]
        seq += 1
        msg = {"t": time.time(), "seq": seq, "engaged": True, "synthetic": True, "hands": {"right": {
            "ts_us": int(tt * 1e6), "calibrated": True,
            "curl": {f: round(frac, 4) for f in ("thumb", "index", "middle", "ring", "little")},
            "grip": round(frac, 4), "pinch_m": round(pinch_m, 5), "coil_ok": True,
            "wrist_pos": [round(v, 5) for v in wrist], "wrist_quat": [0, 0, 0, 1],
            "wrist_dpos": None, "wrist_dquat": None,
            "ee_pos": [round(v, 5) for v in ee], "ee_quat": [0, 0, 0, 1],
            "ee_dpos": [round(v - b, 5) for v, b in zip(ee, BASE_EE)], "ee_dquat": [0, 0, 0, 1],
            "hub_quat_ned": [0, 0, 0, 1], "heading_ok": True}}}
        out.sendto(json.dumps(msg).encode(), dst)
        next_t += period
        delay = next_t - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.perf_counter()
    print(f"[synth] done: {sum(1 for r in results if r and r['success'])} of {len(results)} succeeded, "
          f"{sum(1 for r in results if r and r['broken'])} broken")


if __name__ == "__main__":
    main()
