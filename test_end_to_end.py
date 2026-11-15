#!/usr/bin/env python3
"""
test_end_to_end.py - scripted pick-and-place through the whole Steady Hands chain, no gloves needed.

For each assist condition it starts egg_sim.py (headless, logging), lets synth_operator.py drive the
gripper through a gentle pick-and-place and a full-pinch "crush" (with 1 mm 10 Hz tremor and 0.5 mm
noise on the wrist signal), then runs report.py on the logs and checks the results:

  condition  scenario  expected
  off        place     success, peak grip below the break threshold
  off        crush     egg broken
  on         place     success, peak grip below the yellow zone (15 N)
  on         crush     NOT broken, success, peak grip below 15 N
  grip       crush     NOT broken (grip limiter only)          [skip with --no-grip]
  report.py            off 1 of 2 broken, on 0 of 2 broken, on 2 of 2 success,
                       assisted jitter < raw jitter with assist on, equal with assist off

Uses its own UDP ports (9880-9882), so it does not clash with a running bridge on 9870.
Takes about 2 minutes. Exit code 0 = all checks passed.

  python test_end_to_end.py
  python test_end_to_end.py --viewer          # watch it in the MuJoCo window
  python test_end_to_end.py --keep --dir e2e  # keep logs and results in .\\e2e
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
SIM_PORT, CMD_PORT, STATE_PORT = 9880, 9881, 9882


def start_sim(mode, trials_dir, logfile, viewer):
    args = [PY, os.path.join(HERE, "egg_sim.py"), "--assist", mode, "--log", "--operator", "e2e",
            "--trials-dir", trials_dir, "--port", str(SIM_PORT), "--cmd-port", str(CMD_PORT),
            "--state-udp", f"127.0.0.1:{STATE_PORT}", "--quiet"]
    if not viewer:
        args.append("--headless")
    fh = open(logfile, "w")
    p = subprocess.Popen(args, cwd=HERE, stdout=fh, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    t0 = time.time()
    while time.time() - t0 < 30:
        time.sleep(0.3)
        if p.poll() is not None:
            raise RuntimeError(f"egg_sim.py exited early, see {logfile}")
        with open(logfile) as r:
            if "listening" in r.read():
                return p, fh
    p.kill()
    raise RuntimeError(f"egg_sim.py did not start within 30 s, see {logfile}")


def run_synth(scenario, seed):
    args = [PY, os.path.join(HERE, "synth_operator.py"), "--scenario", scenario, "--trials", "1",
            "--tremor-mm", "1", "--noise-mm", "0.5", "--seed", str(seed),
            "--udp", f"127.0.0.1:{SIM_PORT}", "--cmd", f"127.0.0.1:{CMD_PORT}", "--state-port", str(STATE_PORT)]
    out = subprocess.run(args, cwd=HERE, capture_output=True, text=True, timeout=120).stdout
    result = None
    for line in out.splitlines():
        if line.startswith("[synth] trial"):
            result = json.loads(line.split(": ", 1)[1])
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--viewer", action="store_true", help="show the MuJoCo window")
    ap.add_argument("--dir", default=None, help="work directory (default: a temporary one)")
    ap.add_argument("--keep", action="store_true", help="keep the work directory")
    ap.add_argument("--no-grip", action="store_true", help="skip the grip-limiter-only condition")
    a = ap.parse_args()
    sys.path.insert(0, HERE)
    from assist import ASSIST_PARAMS
    from egg_sim import SIM_PARAMS

    work = os.path.abspath(a.dir) if a.dir else tempfile.mkdtemp(prefix="steady_e2e_")
    os.makedirs(work, exist_ok=True)
    trials = os.path.join(work, "trials")
    if os.path.isdir(trials):
        shutil.rmtree(trials)
    print(f"work dir: {work}")
    plan = [("off", "place", 1), ("off", "crush", 2), ("on", "place", 1), ("on", "crush", 2)]
    if not a.no_grip:
        plan.append(("grip", "crush", 2))
    checks = []

    def check(name, ok, detail=""):
        checks.append((name, bool(ok), detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}")

    brk, yellow = SIM_PARAMS["break_n"], ASSIST_PARAMS["yellow_n"]
    for mode in dict.fromkeys(m for m, _, _ in plan):
        print(f"\n== assist {mode} ==")
        sim, fh = start_sim(mode, trials, os.path.join(work, f"egg_sim_{mode}.log"), a.viewer)
        try:
            for m, scen, seed in plan:
                if m != mode:
                    continue
                r = run_synth(scen, seed)
                if r is None:
                    check(f"{mode}/{scen} finished", False, "no trial result from synth_operator")
                    continue
                d = (f"reason={r['reason']} peak={r['peak_grip_n']:.1f} N "
                     f"placement={r['placement_error_mm']:.1f} mm time={r['time_s']:.1f} s")
                if mode == "off" and scen == "place":
                    check("off/place: success, below break threshold", r["success"] and r["peak_grip_n"] < brk, d)
                elif mode == "off" and scen == "crush":
                    check("off/crush: egg broken", r["broken"], d)
                elif mode == "on" and scen == "place":
                    check(f"on/place: success, peak < {yellow:.0f} N", r["success"] and r["peak_grip_n"] < yellow, d)
                elif mode == "on" and scen == "crush":
                    check(f"on/crush: not broken, success, peak < {yellow:.0f} N",
                          (not r["broken"]) and r["success"] and r["peak_grip_n"] < yellow, d)
                elif mode == "grip":
                    check("grip/crush: not broken", not r["broken"], d)
                if r.get("condition") != mode:
                    check(f"{mode}/{scen}: logged condition", False, f"log says {r.get('condition')}")
        finally:
            sim.terminate()
            try:
                sim.wait(10)
            except subprocess.TimeoutExpired:
                sim.kill()
            fh.close()

    print("\n== report.py ==")
    out_prefix = os.path.join(work, "results")
    rp = subprocess.run([PY, os.path.join(HERE, "report.py"), "--dir", trials, "--out", out_prefix],
                        cwd=HERE, capture_output=True, text=True)
    check("report.py ran", rp.returncode == 0, rp.stderr.strip()[-300:])
    rows = {}
    if os.path.exists(out_prefix + ".csv"):
        with open(out_prefix + ".csv", newline="", encoding="utf-8") as f:
            rows = {r["condition"]: r for r in csv.DictReader(f)}
    off, on = rows.get("off"), rows.get("on")
    if off and on:
        check("report: off 1 of 2 broken", off["trials"] == "2" and off["broken"] == "1",
              f"off broken {off['broken']} of {off['trials']}")
        check("report: on 0 of 2 broken, 2 of 2 success",
              on["trials"] == "2" and on["broken"] == "0" and on["success"] == "2",
              f"on broken {on['broken']}, success {on['success']} of {on['trials']}")
        jr, ja = float(on["jitter_raw_mm_mean"]), float(on["jitter_assisted_mm_mean"])
        check("report: assist on lowers wrist jitter", ja < jr, f"raw {jr:.2f} mm -> assisted {ja:.2f} mm")
        jr0, ja0 = float(off["jitter_raw_mm_mean"]), float(off["jitter_assisted_mm_mean"])
        check("report: assist off leaves jitter unchanged", abs(jr0 - ja0) < 1e-9, f"raw {jr0:.2f} = {ja0:.2f} mm")
        check("report: on peak grip < off peak grip",
              float(on["peak_grip_n_max"]) < float(off["peak_grip_n_max"]),
              f"on max {float(on['peak_grip_n_max']):.1f} N vs off max {float(off['peak_grip_n_max']):.1f} N")
    else:
        check("report: off and on rows present", False, str(list(rows)))
    if os.path.exists(out_prefix + ".md"):
        print("\n" + open(out_prefix + ".md", encoding="utf-8").read().split("## Learning")[0])

    n_fail = sum(1 for _, ok, _ in checks if not ok)
    print(f"{len(checks) - n_fail} of {len(checks)} checks passed" + ("" if not n_fail else f", {n_fail} FAILED"))
    if a.keep or a.dir or n_fail:
        print(f"logs and results kept in {work}")
    else:
        shutil.rmtree(work, ignore_errors=True)
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
