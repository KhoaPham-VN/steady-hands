#!/usr/bin/env python3
"""
udp_monitor.py - receive glove_bridge.py JSON and show it; template for the robot side.

  python3 udp_monitor.py                 # listen on 0.0.0.0:9870, live bars
  python3 udp_monitor.py --record demo1.jsonl   # also log every message (for replay / datasets)

Copy `to_robot()` into your robot controller and replace the example mappings.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time

FINGERS = ("thumb", "index", "middle", "ring", "little")


def to_robot(hand: dict, engaged: bool) -> dict:
    """Example mappings. Every robot differs - check its SDK for ranges and joint order."""
    c = {f: (hand["curl"][f] or 0.0) for f in FINGERS}
    cmd = {
        # (a) generic 6-value dexterous hand, 0 = open .. 1 = closed
        #     (Inspire-style hands use 6 actuators; check the real order/direction/range
        #      in the Inspire integration from Aalto Robotics Club before sending!)
        "hand6": [c["little"], c["ring"], c["middle"], c["index"], c["thumb"], c["thumb"]],
        # (b) parallel gripper, 0 = open .. 1 = closed
        "gripper": hand["grip"] or 0.0,
        # (c) arm end-effector: motion since the clutch engaged, already in the ROBOT frame
        #     (X forward, Y left, Z up; metres, times the bridge's --scale). Add it to the
        #     robot's end-effector pose captured at the moment you engaged.
        "ee_delta_pos": hand.get("ee_dpos") if engaged else None,
        "ee_delta_quat": hand.get("ee_dquat") if engaged else None,
    }
    return cmd


def bar(v, width=10):
    if v is None:
        return "?" * width
    n = int(round(max(0.0, min(1.0, v)) * width))
    return "#" * n + "." * (width - n)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9870)
    ap.add_argument("--record", default=None, help="append every message to this .jsonl file")
    a = ap.parse_args()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", a.port))
    rec = open(a.record, "a") if a.record else None
    print(f"listening on UDP :{a.port}", file=sys.stderr)
    last_print, n, t_rate = 0.0, 0, time.time()
    while True:
        data, _ = s.recvfrom(65536)
        msg = json.loads(data)
        n += 1
        if rec:
            rec.write(data.decode() + "\n")
        now = time.time()
        if now - last_print < 0.1:
            continue
        last_print = now
        rate = n / max(now - t_rate, 1e-3)
        if now - t_rate > 2:
            n, t_rate = 0, now
        lines = [f"\033[H\033[J{'ENGAGED' if msg['engaged'] else 'idle'}   seq={msg['seq']}   {rate:.0f} msg/s"]
        for side in ("left", "right"):
            h = msg["hands"].get(side)
            if not h:
                lines.append(f"\n{side}: no data")
                continue
            lines.append(f"\n{side}  calibrated={h['calibrated']}  coil_ok={h['coil_ok']}  heading_ok={h['heading_ok']}"
                         f"  pinch={h['pinch_m']} m")
            for f in FINGERS:
                v = h["curl"][f]
                lines.append(f"  {f:<7}{bar(v)} {v if v is not None else '-'}")
            if h["wrist_pos"]:
                lines.append("  wrist  pos " + " ".join(f"{x:+.3f}" for x in h["wrist_pos"]) + "   (coil frame)")
            if h.get("ee_pos"):
                lines.append("  robot  pos " + " ".join(f"{x:+.3f}" for x in h["ee_pos"])
                             + "   (X fwd, Y left, Z up)")
            if h.get("ee_dpos"):
                lines.append("  robot  move " + " ".join(f"{x:+.3f}" for x in h["ee_dpos"]) + "   since engage")
            cmd = to_robot(h, msg["engaged"])
            lines.append("  -> hand6 " + " ".join(f"{x:.2f}" for x in cmd["hand6"]) + f"  gripper {cmd['gripper']:.2f}")
        print("\n".join(lines), flush=True)


if __name__ == "__main__":
    main()
