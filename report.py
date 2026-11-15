#!/usr/bin/env python3
"""
report.py - Steady Hands results table from egg_sim.py trial logs (trials/*.jsonl).

  python report.py                          # all trials in trials/ -> results.md, results.csv, results_trials.csv
  python report.py --dir trials --operator AB
  python report.py --since 2026-11-15T08:00 --out results_final
  python report.py --glove-only             # only glove-only trials (egg_sim.py --no-coil) -> results_glove_only.*
Glove-only trials (condition "glove_only") are left out of the default report.

Per trial (PLAN_STEADY_HANDS.md section 8)
  broken          peak grip force above the break threshold
  success         not broken, and the egg centre within 20 mm of the target centre at the end
  peak grip (N)   largest pad normal force on the egg during the trial
  placement (mm)  horizontal egg-centre to target-centre distance at the end
  time (s)        reset to end
  path ratio      gripper path length / sum of straight segments between start, each gripper
                  open/close event, and end (as in demo_qa.py)
  jitter (mm)     eval_offline.jitter_mm on the raw wrist position (hand.ee_pos) and on the
                  assisted one (filtered_in: after the jitter filter, before scaling)
Excluded and listed: trials without an end record, "aborted" trials, and trials whose assist mode
changed while they ran. Counts are printed as "k of n"; mean / min / max for each metric.
No significance test (10 trials per condition are too few). Standard library only.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
from collections import defaultdict

from eval_offline import jitter_mm

ORDER = ["off", "grip", "on", "glove_only-off", "glove_only-grip", "glove_only-on"]
LABEL = {"off": "Assist off", "on": "Assist on", "grip": "Grip limiter only",
         "glove_only-off": "Glove only, assist off", "glove_only-on": "Glove only, assist on",
         "glove_only-grip": "Glove only, grip limiter"}


def path_ratio(pos, travel, open_travel):
    if len(pos) < 2:
        return float("nan")
    step = sum(math.dist(a, b) for a, b in zip(pos, pos[1:]))
    closed = [t >= open_travel for t in travel]
    events = [0] + [i for i in range(1, len(closed)) if closed[i] != closed[i - 1]] + [len(pos) - 1]
    ref = sum(math.dist(pos[a], pos[b]) for a, b in zip(events, events[1:]))
    return step / ref if ref > 1e-6 else float("nan")


def analyse(path):
    header, frames, end = None, [], None
    with open(path) as fh:
        for line in fh:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("type") == "header":
                header = r
            elif r.get("type") == "end":
                end = r
            elif "sim_t" in r:
                frames.append(r)
    row = {"file": os.path.basename(path)}
    if header is None:
        return row | {"excluded": "no header"}
    sim = header["params"]["sim"]
    cond = header["condition"]
    if cond == "glove_only":                     # keyboard position, glove pinch: kept apart
        cond = f"glove_only-{header.get('assist', 'off')}"
    row |= {"condition": cond, "operator": header.get("operator") or "anon",
            "start_time": header.get("start_time"), "trial_no": header.get("trial_no"),
            "frames": len(frames)}
    if end is None:
        return row | {"excluded": "no end record"}
    if end.get("reason") == "aborted":
        return row | {"excluded": "aborted"}
    if end.get("condition_changed"):
        return row | {"excluded": "assist changed during trial"}
    peak = max([end.get("peak_grip_n", 0.0)] + [f.get("grip_peak_n", 0.0) for f in frames])
    broken = bool(end["broken"]) or peak > sim["break_n"]
    ts = [f["hand"]["ts_us"] / 1e6 for f in frames if f["hand"].get("ts_us") is not None
          and f["hand"].get("ee_pos") is not None]
    raw = [f["hand"]["ee_pos"] for f in frames if f["hand"].get("ts_us") is not None
           and f["hand"].get("ee_pos") is not None]
    filt = [f.get("filtered_in") or f["hand"]["ee_pos"] for f in frames
            if f["hand"].get("ts_us") is not None and f["hand"].get("ee_pos") is not None]
    # both scored on the same samples: where the raw AND the assisted trends are still
    j_raw = jitter_mm(ts, raw, still_from=filt)[0] if len(ts) > 20 else None
    j_ass = jitter_mm(ts, filt, still_from=raw)[0] if len(ts) > 20 else None
    row |= {"excluded": "", "reason": end["reason"], "broken": broken,
            "success": bool(end["success"]) and not broken,
            "peak_grip_n": round(peak, 2), "placement_error_mm": end["placement_error_mm"],
            "time_s": end["time_s"],
            "path_ratio": round(path_ratio([f["gripper"] for f in frames], [f["travel"] for f in frames],
                                           sim["open_travel_m"]), 3),
            "jitter_raw_mm": round(j_raw, 3) if j_raw is not None else None,
            "jitter_assisted_mm": round(j_ass, 3) if j_ass is not None else None}
    return row


def stats(vals):
    v = [x for x in vals if x is not None and not (isinstance(x, float) and math.isnan(x))]
    if not v:
        return None
    return {"n": len(v), "mean": sum(v) / len(v), "min": min(v), "max": max(v)}


def fmt(s, nd=1):
    if s is None:
        return "–"
    return f"{s['mean']:.{nd}f} ({s['min']:.{nd}f}–{s['max']:.{nd}f})"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default="trials")
    ap.add_argument("--operator", default=None, help="only this operator")
    ap.add_argument("--since", default=None, help="only trials started at/after this ISO time, e.g. 2026-11-15T08:00")
    ap.add_argument("--out", default=None,
                    help="output prefix (default results, or results_glove_only with --glove-only)")
    ap.add_argument("--glove-only", action="store_true",
                    help="report only glove-only trials (egg_sim.py --no-coil); by default they are left out")
    a = ap.parse_args()
    a.out = a.out or ("results_glove_only" if a.glove_only else "results")

    rows = [analyse(p) for p in sorted(glob.glob(os.path.join(a.dir, "*.jsonl")))]
    rows = [r for r in rows if "condition" not in r
            or r["condition"].startswith("glove_only") == a.glove_only]
    if a.operator:
        rows = [r for r in rows if r.get("operator") == a.operator]
    if a.since:
        rows = [r for r in rows if (r.get("start_time") or "") >= a.since]
    kept = [r for r in rows if r.get("excluded") == ""]
    dropped = [r for r in rows if r.get("excluded")]
    by = defaultdict(list)
    for r in kept:
        by[r["condition"]].append(r)
    conds = [c for c in ORDER if c in by] + [c for c in by if c not in ORDER]

    metrics = [("peak_grip_n", "Peak grip force (N)", 1, None),
               ("placement_error_mm", "Placement error (mm), placed trials", 1, "placed"),
               ("time_s", "Time (s)", 1, None),
               ("path_ratio", "Path ratio", 2, None),
               ("jitter_raw_mm", "Jitter, raw wrist (mm)", 2, None),
               ("jitter_assisted_mm", "Jitter, assisted wrist (mm)", 2, None)]
    summary = {}
    for c in conds:
        rs = by[c]
        s = {"trials": len(rs), "operators": len({r["operator"] for r in rs}),
             "broken": sum(r["broken"] for r in rs), "success": sum(r["success"] for r in rs)}
        for key, _, _, only in metrics:
            s[key] = stats([r[key] for r in rs if only is None or r["reason"] == only])
        summary[c] = s

    # ---- markdown
    L = ["# Steady Hands results", ""]
    if not kept:
        L.append(f"No complete trials in `{a.dir}`.")
    else:
        ops = sorted({r["operator"] for r in kept})
        L.append(f"{len(kept)} trials, {len(ops)} operator(s) ({', '.join(ops)}), from `{a.dir}`. "
                 "Cells show mean (min–max). No significance test: too few trials.")
        L += ["", "| | " + " | ".join(LABEL.get(c, c) for c in conds) + " |",
              "| --- | " + " | ".join("---" for _ in conds) + " |"]
        L.append("| Trials (operators) | " + " | ".join(f"{summary[c]['trials']} ({summary[c]['operators']})"
                                                      for c in conds) + " |")
        L.append("| Broken | " + " | ".join(f"{summary[c]['broken']} of {summary[c]['trials']}" for c in conds) + " |")
        L.append("| Success | " + " | ".join(f"{summary[c]['success']} of {summary[c]['trials']}" for c in conds) + " |")
        for key, label, nd, _ in metrics:
            L.append(f"| {label} | " + " | ".join(
                fmt(summary[c][key], nd) + (f" n={summary[c][key]['n']}" if summary[c][key] and
                                            summary[c][key]['n'] != summary[c]['trials'] else "")
                for c in conds) + " |")
        # learning-effect check: first vs second half of each operator's trials per condition
        L += ["", "## Learning check (first half vs second half of each operator's trials per condition)", "",
              "| Condition | Operator | Success, first half | Success, second half | Time (s), first | Time (s), second |",
              "| --- | --- | --- | --- | --- | --- |"]
        for c in conds:
            per_op = defaultdict(list)
            for r in by[c]:
                per_op[r["operator"]].append(r)
            for op, rs in sorted(per_op.items()):
                rs.sort(key=lambda r: r["start_time"] or "")
                h = len(rs) // 2
                if h == 0:
                    continue
                f, s = rs[:h], rs[h:]
                mt = lambda x: sum(r["time_s"] for r in x) / len(x)
                L.append(f"| {LABEL.get(c, c)} | {op} | {sum(r['success'] for r in f)} of {len(f)} | "
                         f"{sum(r['success'] for r in s)} of {len(s)} | {mt(f):.1f} | {mt(s):.1f} |")
        L += ["", "## Trials", "", "| File | Condition | Operator | Result | Peak N | Placement mm | Time s | "
              "Path ratio | Jitter raw mm | Jitter assisted mm |", "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
        for r in kept:
            res = "broken" if r["broken"] else ("success" if r["success"] else r["reason"])
            L.append(f"| {r['file']} | {r['condition']} | {r['operator']} | {res} | {r['peak_grip_n']} | "
                     f"{r['placement_error_mm']} | {r['time_s']} | {r['path_ratio']} | {r['jitter_raw_mm']} | "
                     f"{r['jitter_assisted_mm']} |")
    if dropped:
        L += ["", "## Excluded", ""] + [f"- {r['file']}: {r['excluded']}" for r in dropped]
    md = "\n".join(L) + "\n"
    with open(a.out + ".md", "w", encoding="utf-8") as fh:
        fh.write(md)

    # ---- csv: one row per condition, and one row per trial
    with open(a.out + ".csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        head = ["condition", "trials", "operators", "broken", "success"]
        for key, _, _, _ in metrics:
            head += [f"{key}_n", f"{key}_mean", f"{key}_min", f"{key}_max"]
        w.writerow(head)
        for c in conds:
            s = summary[c]
            row = [c, s["trials"], s["operators"], s["broken"], s["success"]]
            for key, _, _, _ in metrics:
                st = s[key]
                row += [st["n"], round(st["mean"], 4), round(st["min"], 4), round(st["max"], 4)] if st else ["", "", "", ""]
            w.writerow(row)
    cols = ["file", "condition", "operator", "start_time", "trial_no", "frames", "excluded", "reason", "broken",
            "success", "peak_grip_n", "placement_error_mm", "time_s", "path_ratio", "jitter_raw_mm",
            "jitter_assisted_mm"]
    with open(a.out + "_trials.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(md)
    print(f"wrote {a.out}.md, {a.out}.csv, {a.out}_trials.csv")


if __name__ == "__main__":
    main()
