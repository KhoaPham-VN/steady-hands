#!/usr/bin/env python3
"""
eval_offline.py - run a recorded glove stream through JitterFilter (and MotionScaler) offline and
print wrist jitter before / after plus the added lag. Needs no operator; repeatable.

Input: JSONL files of either kind
  * bridge messages, e.g. recorded with   python udp_monitor.py --record rec.jsonl
  * egg_sim.py trial logs (trials/*.jsonl; uses the raw hand data in each frame)

  python eval_offline.py docs\\organizers_replay.jsonl
  python eval_offline.py rec.jsonl --hand right --cutoffs 1,2,3,4,5 --betas 0,5,20
  python eval_offline.py trials\\*.jsonl --t0 5 --t1 60

Definitions (PLAN_STEADY_HANDS.md section 7.3)
  jitter   RMS (mm) of the 3-D wrist position minus its 1 Hz low-pass trend (zero-phase), over samples
           where the trend moves slower than 2 cm/s. Contains hand tremor AND sensor noise.
  lag      delay of the filtered signal behind the raw one, by cross-correlation (ms)
The position is hands.<hand>.ee_pos (robot frame); time is ts_us (glove clock). Repeated glove
frames are dropped; gaps > 0.1 s (e.g. coil_ok false) split the stream into segments.
Standard library only.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import sys

from assist import ASSIST_PARAMS, JitterFilter

TREND_HZ = 1.0
STILL_MPS = 0.02
EDGE_S = 0.5          # trimmed at both ends of each segment (low-pass edge effects)
GAP_S = 0.1


# ── jitter metric (also used by report.py) ────────────────────────────────────────────────
def _lp(xs, ts, fc):
    y = [xs[0]]
    for i in range(1, len(xs)):
        dt = max(ts[i] - ts[i - 1], 1e-4)
        a = 1.0 / (1.0 + 1.0 / (2 * math.pi * fc * dt))
        y.append(y[-1] + a * (xs[i] - y[-1]))
    return y


def zero_phase_lp(xs, ts, fc=TREND_HZ):
    fwd = _lp(xs, ts, fc)
    tr = [-t for t in reversed(ts)]
    return list(reversed(_lp(list(reversed(fwd)), tr, fc)))


def split_segments(ts, pos, gap=GAP_S):
    """Split (ts, pos) lists where time jumps by more than `gap` or goes backwards."""
    segs, cur = [], ([], [])
    for t, p in zip(ts, pos):
        if cur[0] and not (0 < t - cur[0][-1] <= gap):
            segs.append(cur)
            cur = ([], [])
        cur[0].append(t)
        cur[1].append(p)
    if cur[0]:
        segs.append(cur)
    return segs


def jitter_mm(ts, pos, still_mps=STILL_MPS, trend_hz=TREND_HZ, edge_s=EDGE_S, still_from=None,
              band_hz=None):
    """Returns (jitter_mm or None, n_still_samples, n_samples).

    still_from: positions whose trend decides which samples count as "still" (default: pos itself).
    Pass the RAW positions when scoring a filtered signal, so both are scored on the same samples.
    band_hz: if set, only the part of the residual above this frequency is counted (diagnostic)."""
    sq, n_still, n_all = 0.0, 0, 0
    ref_all = still_from if still_from is not None else pos
    for (st, sp), (_, sr) in zip(split_segments(ts, pos), split_segments(ts, ref_all)):
        if len(st) < 10 or st[-1] - st[0] < 2 * edge_s + 0.5:
            continue
        trend = [zero_phase_lp([p[k] for p in sp], st, trend_hz) for k in range(3)]
        rtrend = None if still_from is None else [zero_phase_lp([p[k] for p in sr], st, trend_hz)
                                                  for k in range(3)]
        resid = [[sp[i][k] - trend[k][i] for i in range(len(st))] for k in range(3)]
        if band_hz:
            resid = [[r - lo for r, lo in zip(resid[k], zero_phase_lp(resid[k], st, band_hz))] for k in range(3)]
        for i in range(1, len(st) - 1):
            if st[i] - st[0] < edge_s or st[-1] - st[i] < edge_s:
                continue
            n_all += 1
            dt = st[i + 1] - st[i - 1]
            speed = lambda tr: math.sqrt(sum(((tr[k][i + 1] - tr[k][i - 1]) / dt) ** 2 for k in range(3)))
            # still = this signal's trend is still (and, with still_from, the reference's trend too)
            if speed(trend) < still_mps and (rtrend is None or speed(rtrend) < still_mps):
                n_still += 1
                sq += sum(resid[k][i] ** 2 for k in range(3))
    return (math.sqrt(sq / n_still) * 1000 if n_still else None), n_still, n_all


def lag_ms(ts, raw, filt, max_shift=40):
    """Delay of filt behind raw (cross-correlation over all segments, parabolic refinement)."""
    cs = [0.0] * (max_shift + 1)
    dts = []
    for (st, sr), (_, sf) in zip(split_segments(ts, raw), split_segments(ts, filt)):
        n = len(st) - max_shift
        if n < 50:
            continue
        dts += [st[i + 1] - st[i] for i in range(len(st) - 1)]
        for k in range(3):
            a = [p[k] for p in sr]
            b = [p[k] for p in sf]
            ma, mb = sum(a) / len(a), sum(b) / len(b)
            a = [x - ma for x in a]
            b = [x - mb for x in b]
            for s in range(max_shift + 1):
                cs[s] += sum(a[i] * b[i + s] for i in range(n))
    if not dts:
        return None
    k = max(range(len(cs)), key=cs.__getitem__)
    kf = float(k)
    if 0 < k < max_shift:
        y0, y1, y2 = cs[k - 1], cs[k], cs[k + 1]
        den = y0 - 2 * y1 + y2
        if den:
            kf = k + 0.5 * (y0 - y2) / den
    dts.sort()
    return kf * dts[len(dts) // 2] * 1000


# ── input ─────────────────────────────────────────────────────────────────────────────────
def load_stream(paths, hand="right", t0=None, t1=None):
    """Returns ts (s, from glove ts_us, starting at 0) and ee_pos lists; duplicates removed."""
    ts, pos, last_us = [], [], None
    first = None
    for path in paths:
        with open(path) as fh:
            for line in fh:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if "hands" in r:
                    h = (r.get("hands") or {}).get(hand)
                elif "hand" in r and isinstance(r["hand"], dict):
                    h = r["hand"]
                else:
                    continue
                if not h or not h.get("coil_ok") or h.get("ee_pos") is None or h.get("ts_us") is None:
                    continue
                us = h["ts_us"]
                if us == last_us:
                    continue
                last_us = us
                if first is None:
                    first = us
                t = (us - first) / 1e6
                if (t0 is not None and t < t0) or (t1 is not None and t > t1):
                    continue
                ts.append(t)
                pos.append([float(v) for v in h["ee_pos"]])
    return ts, pos


def run_filter(ts, pos, min_cutoff, beta):
    out = []
    for st, sp in split_segments(ts, pos):
        f = JitterFilter(min_cutoff=min_cutoff, beta=beta)
        out += [f(p, t) for t, p in zip(st, sp)]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help="JSONL files (wildcards allowed)")
    ap.add_argument("--hand", default="right", choices=["right", "left"])
    ap.add_argument("--cutoffs", default="1,2,3,4,5", help="min_cutoff values to try (Hz)")
    ap.add_argument("--betas", default="0", help="beta values to try")
    ap.add_argument("--t0", type=float, default=None, help="start time (s from first sample)")
    ap.add_argument("--t1", type=float, default=None, help="end time (s)")
    ap.add_argument("--still", type=float, default=STILL_MPS, help="still-hand speed threshold (m/s)")
    a = ap.parse_args()
    paths = sorted({p for f in a.files for p in (glob.glob(f) or [f])})
    ts, pos = load_stream(paths, a.hand, a.t0, a.t1)
    if len(ts) < 100:
        sys.exit(f"only {len(ts)} usable {a.hand}-hand samples in {paths}")
    dur = ts[-1] - ts[0]
    j_raw, n_still, n_all = jitter_mm(ts, pos, a.still)
    scale = ASSIST_PARAMS["scale_precision"]
    print(f"files: {', '.join(paths)}")
    print(f"{a.hand} hand: {len(ts)} samples, {dur:.1f} s, {len(ts) / max(dur, 1e-6):.1f} Hz, "
          f"{len(split_segments(ts, pos))} segment(s); still (< {a.still * 100:.0f} cm/s) "
          f"{n_still} of {n_all} samples ({100 * n_still / max(n_all, 1):.0f} %)")
    if j_raw is None:
        sys.exit("no still samples: cannot compute jitter")
    print(f"raw jitter: {j_raw:.3f} mm   (in gripper space at precision scale {scale}: {j_raw * scale:.3f} mm)")
    for b in (3.0, 8.0):
        jb = jitter_mm(ts, pos, a.still, band_hz=b)[0]
        print(f"  part of the raw jitter above {b:.0f} Hz: {jb:.3f} mm RMS = {100 * (jb / j_raw) ** 2:.0f} % of its power")
    print()
    cur = (ASSIST_PARAMS["filter_min_cutoff_hz"], ASSIST_PARAMS["filter_beta"])
    print("Each row scores raw and filtered on the same samples: those where BOTH trends are still.")
    print(f"{'min_cutoff':>10} {'beta':>5} {'samples':>7} {'raw mm':>7} {'filt mm':>8} {'reduction':>9} "
          f"{'lag ms':>7} {'gripper mm @' + str(scale):>16}")
    for fc in [float(x) for x in a.cutoffs.split(",")]:
        for beta in [float(x) for x in a.betas.split(",")]:
            filt = run_filter(ts, pos, fc, beta)
            j, n_s, _ = jitter_mm(ts, filt, a.still, still_from=pos)
            jr, _, _ = jitter_mm(ts, pos, a.still, still_from=filt)
            lag = lag_ms(ts, pos, filt)
            mark = "  <- assist.py" if (fc, beta) == cur else ""
            if j is None or jr is None:
                print(f"{fc:10.1f} {beta:5.1f}  no common still samples")
                continue
            print(f"{fc:10.1f} {beta:5.1f} {n_s:7d} {jr:7.3f} {j:8.3f} {100 * (1 - j / jr):8.1f}% "
                  f"{lag if lag is not None else float('nan'):7.1f} {j * scale:16.3f}{mark}")
    print("\nlag is measured on this recording's own motion; the phase-2 synthetic test (0.2 Hz sine) "
          "is in test_assist.py")


if __name__ == "__main__":
    main()
