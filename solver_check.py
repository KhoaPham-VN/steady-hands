#!/usr/bin/env python3
"""
solver_check.py - watch what rkk-hand-solver reports per finger joint, live, for both hands.

Use it before calibrating hands_sim.py --source solver: open and close each hand a few times while
it runs. For every finger it prints the three joint bend angles (degrees between successive bones of
the OpenXR joints) and the 0..1 curl hands_sim.py computes from them (sum / full joint range). At the
end it prints each finger's curl range over the run and whether it is big enough to calibrate
(fist - open >= hands_sim.MIN_SPAN).

  python solver_check.py                    # both hands, 20 s, 2 lines per second
  python solver_check.py --hand left --seconds 30
Needs rkk-hand-solver --emit-openxr running (TCP 12277). Reads only; changes nothing.
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time

import numpy as np

import hand_viewer as hv
import hands_sim as hs


def bends(P, chain):
    pts = [np.asarray(P[n], dtype=float) for n in chain]
    out = []
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        u, v = b - a, c - b
        nu, nv = np.linalg.norm(u), np.linalg.norm(v)
        out.append(0.0 if nu < 1e-6 or nv < 1e-6 else
                   math.degrees(math.acos(max(-1.0, min(1.0, float(u @ v / (nu * nv)))))))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--solver", default="127.0.0.1:12277")
    ap.add_argument("--hand", choices=["left", "right", "both"], default="both")
    ap.add_argument("--seconds", type=float, default=20.0)
    a = ap.parse_args()
    host, port = a.solver.rsplit(":", 1)
    threading.Thread(target=hv.rgmp_loop, args=(host, int(port)), daemon=True).start()
    sides = list(hs.SIDES) if a.hand == "both" else [a.hand]
    full = {f: sum(hs.JOINT_RANGE[j][1] for j in hs.FINGER_JOINTS[f]) for f in hs.FINGERS}
    rng = {s: {f: [9.9, -9.9] for f in hs.FINGERS} for s in sides}
    t0, last = time.time(), 0.0
    time.sleep(2.0)
    print("curl = sum of the 3 bends / full range (thumb 200, fingers 280 deg); open/close the hands now")
    while time.time() - t0 < a.seconds + 2:
        with hv.LOCK:
            H = {k: dict(v) for k, v in hv.STATE["hands"].items()}
        now = time.time()
        for s in sides:
            h = H.get(s)
            if not h or now - h.get("t", 0) > 0.5:
                continue
            P = dict(zip(h["names"], h["pos"]))
            for f, chain in hs.SolverSource.CHAINS.items():
                b = bends(P, chain)
                c = min(1.0, max(0.0, sum(b) / full[f]))
                rng[s][f][0], rng[s][f][1] = min(rng[s][f][0], c), max(rng[s][f][1], c)
        if now - last > 0.5:
            last = now
            for s in sides:
                h = H.get(s)
                if not h or now - h.get("t", 0) > 0.5:
                    print(f"{s:5s}: no solver data")
                    continue
                P = dict(zip(h["names"], h["pos"]))
                cells = []
                for f, chain in hs.SolverSource.CHAINS.items():
                    b = bends(P, chain)
                    cells.append(f"{hs.SHORT[f]} {b[0]:3.0f}/{b[1]:3.0f}/{b[2]:3.0f}={sum(b) / full[f]:.2f}")
                print(f"{s:5s} " + "  ".join(cells), flush=True)
        time.sleep(0.02)
    print(f"\ncurl range over the run (needs fist - open >= {hs.MIN_SPAN} to calibrate):")
    for s in sides:
        for f in hs.FINGERS:
            lo, hi = rng[s][f]
            if lo > hi:
                print(f"  {s:5s} {f:6s} no data")
                continue
            ok = hi - lo >= hs.MIN_SPAN
            print(f"  {s:5s} {f:6s} {lo:.3f} .. {hi:.3f}  range {hi - lo:.3f}  "
                  + ("ok" if ok else "TOO SMALL (if you opened and closed this hand during the run, "
                                     "the solver barely moves this finger; use --source bridge)"))


if __name__ == "__main__":
    main()
