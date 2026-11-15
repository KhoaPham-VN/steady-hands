#!/usr/bin/env python3
"""
test_glove_only.py - scripted test of egg_sim.py --no-coil (glove-only mode), no gloves needed.

Plays a fake glove: bridge-format messages with coil_ok false and null wrist fields (as glove_bridge.py
sends them without a Coil Pro), plus some frames where the hand is missing or pinch_m is null.
Drives egg_sim.py through its command port: "grasp", pinch, "carry", open. Optionally starts
fake_rgmp_server.py as an RCSP stand-in and checks the LED commands egg_sim.py sends (--led).

  condition                  expected
  glove_only, assist off     gentle pinch: success;  full pinch: egg broken, LED red
  glove_only, assist on      full pinch: not broken, success, peak < 15 N, LED green
  logs                       condition "glove_only" with assist off/on; report.py --glove-only
                             counts them, the default report.py leaves them out

  python test_glove_only.py            # about 1.5 minutes
  python test_glove_only.py --no-led   # skip the RCSP/LED part (e.g. while the real driver runs)
  python test_glove_only.py --viewer --keep --dir go_test
Uses UDP ports 9880-9883. The LED part needs TCP 45451/12276 free (no real driver running).
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
SIM_PORT, CMD_PORT, STATE_PORT = 9880, 9881, 9882


class FakeGlove(threading.Thread):
    """Sends right-hand messages without Coil data at 83.33 Hz; pinch set via .frac (0 open .. 1 closed)."""

    def __init__(self, pinch_open=0.09, pinch_closed=0.03):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frac, self.running, self.seq = 0.0, True, 0
        self.po, self.pc = pinch_open, pinch_closed

    def run(self):
        t0 = time.time()
        while self.running:
            self.seq += 1
            pinch = round(self.po - self.frac * (self.po - self.pc), 5)
            hand = {"ts_us": int((time.time() - t0) * 1e6), "calibrated": True,
                    "curl": {f: self.frac for f in ("thumb", "index", "middle", "ring", "little")},
                    "grip": self.frac, "pinch_m": pinch, "coil_ok": False,
                    "wrist_pos": None, "wrist_quat": None, "wrist_dpos": None, "wrist_dquat": None,
                    "ee_pos": None, "ee_quat": None, "ee_dpos": None, "ee_dquat": None,
                    "hub_quat_ned": [0, 0, 0, 1], "heading_ok": True}
            msg = {"t": time.time(), "seq": self.seq, "engaged": False, "hands": {"right": hand}}
            if self.seq % 97 == 0:
                msg["hands"] = {}                    # glove dropped out for a frame
            elif self.seq % 89 == 0:
                hand["pinch_m"] = None               # missing pinch
            self.sock.sendto(json.dumps(msg).encode(), ("127.0.0.1", SIM_PORT))
            time.sleep(0.012)


class State(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(("127.0.0.1", STATE_PORT))
        self.s.settimeout(0.5)
        self.last = None

    def run(self):
        while True:
            try:
                self.last = json.loads(self.s.recvfrom(65536)[0])
            except (socket.timeout, ConnectionResetError):
                pass


def cmd(text):
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(text.encode(), ("127.0.0.1", CMD_PORT))


def wait(cond, timeout, st):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if st.last and cond(st.last):
            return True
        time.sleep(0.05)
    return False


def near(p, q, tol=1e-3):
    return all(abs(a - b) < tol for a, b in zip(p, q))


def run_trial(st, glove, frac):
    """One glove-only pick and place; returns the end record (or None)."""
    trial_before = st.last["trial_no"] if st.last else 0
    cmd("r")
    if not wait(lambda s: s["trial_no"] > trial_before and s["trial_active"], 10, st):
        return None
    glove.frac = 0.0
    cmd("grasp")
    egg = st.last["egg"]
    if not wait(lambda s: near(s["target"], [egg[0], egg[1], 0.475]), 20, st):
        return None
    for k in range(1, 41):                           # close over 0.5 s
        glove.frac = frac * k / 40
        time.sleep(0.0125)
    time.sleep(1.0)
    if not st.last["trial_active"]:
        return st.last["result"]                     # broke while squeezing
    cmd("carry")
    wait(lambda s: near(s["target"], [0.5, 0.2, 0.478]) or not s["trial_active"], 25, st)
    if st.last["trial_active"]:
        time.sleep(0.5)
        glove.frac = 0.0                             # open: release
        wait(lambda s: not s["trial_active"], 10, st)
    time.sleep(0.3)
    return st.last["result"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--no-led", action="store_true", help="skip the RCSP / LED checks")
    ap.add_argument("--dir", default=None)
    ap.add_argument("--keep", action="store_true")
    a = ap.parse_args()
    work = os.path.abspath(a.dir) if a.dir else tempfile.mkdtemp(prefix="steady_glove_only_")
    os.makedirs(work, exist_ok=True)
    trials = os.path.join(work, "trials")
    shutil.rmtree(trials, ignore_errors=True)
    print(f"work dir: {work}")
    checks = []

    def check(name, ok, detail=""):
        checks.append((name, bool(ok)))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}")

    fake = fake_log = None
    if not a.no_led:
        fake_log = os.path.join(work, "fake_rcsp.log")
        fake = subprocess.Popen([PY, "-u", os.path.join(HERE, "fake_rgmp_server.py")], cwd=HERE,
                                stdout=open(fake_log, "w"), stderr=subprocess.STDOUT)
        time.sleep(2.5)

    st = State(); st.start()
    glove = FakeGlove(); glove.start()
    sim_log = os.path.join(work, "egg_sim.log")
    args = [PY, os.path.join(HERE, "egg_sim.py"), "--no-coil", "--log", "--operator", "gtest",
            "--trials-dir", trials, "--port", str(SIM_PORT), "--cmd-port", str(CMD_PORT),
            "--state-udp", f"127.0.0.1:{STATE_PORT}", "--quiet"]
    if not a.viewer:
        args.append("--headless")
    if not a.no_led:
        args.append("--led")
    sim = subprocess.Popen(args, cwd=HERE, stdout=open(sim_log, "w"), stderr=subprocess.STDOUT,
                           stdin=subprocess.DEVNULL)
    try:
        if not wait(lambda s: True, 30, st):
            check("egg_sim.py started and sends state", False, f"see {sim_log}")
            raise SystemExit
        time.sleep(1.0 if a.no_led else 6.0)          # LED worker needs a moment to list gloves
        print("\n== glove only, assist off ==")
        cmd("a off")
        r = run_trial(st, glove, 0.55)
        check("off, gentle pinch: success", r and r["success"] and not r["broken"],
              f"{r and {k: r[k] for k in ('reason', 'peak_grip_n', 'placement_error_mm', 'time_s')}}")
        r = run_trial(st, glove, 1.0)
        check("off, full pinch: egg broken", r and r["broken"], f"{r and (r['reason'], r['peak_grip_n'])}")
        check("logged condition glove_only / assist off",
              r and r.get("condition") == "glove_only" and r.get("assist") == "off", str(r and r.get("condition")))
        print("\n== glove only, assist on ==")
        cmd("a on")
        time.sleep(0.3)
        r = run_trial(st, glove, 1.0)
        check("on, full pinch: not broken, success, peak < 15 N",
              r and r["success"] and not r["broken"] and r["peak_grip_n"] < 15,
              f"{r and (r['reason'], r['peak_grip_n'], r['placement_error_mm'])}")
        check("logged condition glove_only / assist on",
              r and r.get("condition") == "glove_only" and r.get("assist") == "on", "")
        print("\n== keyboard nudges ==")
        t0 = list(st.last["target"])
        cmd("z+ 4"); time.sleep(0.3); cmd("y-"); time.sleep(0.3)
        t1 = st.last["target"]
        check("z+ 4 and y- move the target 20 mm up and 5 mm right",
              abs((t1[2] - t0[2]) - 0.020) < 1e-4 and abs((t1[1] - t0[1]) + 0.005) < 1e-4,
              f"dz {1000 * (t1[2] - t0[2]):.1f} mm, dy {1000 * (t1[1] - t0[1]):.1f} mm")
    finally:
        sim.terminate()
        try:
            sim.wait(10)
        except subprocess.TimeoutExpired:
            sim.kill()
        glove.running = False
        if fake:
            time.sleep(0.3)
            fake.terminate()

    out = open(sim_log).read()
    check("no Python traceback in egg_sim output", "Traceback" not in out)
    if not a.no_led:
        led = [l for l in open(fake_log).read().splitlines() if "LED right" in l]
        print(f"  LED commands received by the fake RCSP server: {len(led)}")
        for l in led[:12]:
            print("    " + l)
        check("LED red sent when the egg broke", any("rgb(255,0,0)" in l for l in led))
        check("LED green sent with assist on", any("rgb(0,255,0)" in l for l in led))
    print("\n== report.py ==")
    rg = subprocess.run([PY, os.path.join(HERE, "report.py"), "--dir", trials, "--glove-only",
                         "--out", os.path.join(work, "results_glove_only")], cwd=HERE, capture_output=True, text=True)
    rows = {}
    path = os.path.join(work, "results_glove_only.csv")
    if os.path.exists(path):
        rows = {r["condition"]: r for r in csv.DictReader(open(path, encoding="utf-8"))}
    check("report.py --glove-only: off 2 trials (1 broken), on 1 trial (0 broken)",
          rows.get("glove_only-off", {}).get("trials") == "2" and rows["glove_only-off"]["broken"] == "1"
          and rows.get("glove_only-on", {}).get("trials") == "1" and rows["glove_only-on"]["broken"] == "0",
          rg.stderr[-200:])
    rd = subprocess.run([PY, os.path.join(HERE, "report.py"), "--dir", trials,
                         "--out", os.path.join(work, "results_default")], cwd=HERE, capture_output=True, text=True)
    check("default report.py leaves glove-only trials out", "No complete trials" in rd.stdout, "")
    n_fail = sum(1 for _, ok in checks if not ok)
    print(f"\n{len(checks) - n_fail} of {len(checks)} checks passed" + (f", {n_fail} FAILED" if n_fail else ""))
    if a.keep or a.dir or n_fail:
        print(f"logs kept in {work}")
    else:
        shutil.rmtree(work, ignore_errors=True)
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
