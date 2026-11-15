#!/usr/bin/env python3
"""
replay_csv.py - play recorded Smartgloves II CSV files back as if the gloves were live.

Acts like `rokoko-sdk`: serves RGMP v2 on TCP 12276, so everything downstream works unchanged
(rkk-hand-solver --no-driver, glove_bridge.py, hand_viewer.py, the relay, Isaac).

  python replay_csv.py --left fran_fac_sim_01_ypos_left.csv --right fran_fac_sim_01_ypos_right.csv --loop
  rkk-hand-solver --emit-openxr --listen 0.0.0.0:12277 --no-driver      (in another window)

Do NOT run the real rokoko-sdk at the same time (both want port 12276).

CSV layout: the Rokoko export of one glove = RGMP 'default' group (TRANSFORM/IMU/flags per
sensor, hub orientation, arm pose vs Coil Pro) followed by the 'emf_only' group.
Timestamps: the "device local" column steps 12 ms per frame (83.33 Hz) and is used for pacing.
Status flag bit names are inferred from the data: bit 9 turns on exactly when the arm pose
vs Coil Pro becomes non-zero (is_coil_pro_present), bit 10 later (emf_flip_decided), hub bit 11
later (is_heading_reliable). Standard library only.
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import socket
import struct
import threading
import time

FLAG_MAPS = {
    "sensor": {"0": "valid_data"},
    "hub": {"0": "valid_data", "11": "is_heading_reliable"},
    "coil": {"0": "valid_data", "9": "is_coil_pro_present", "10": "emf_flip_decided"},
}


def load_glove(path):
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    header, data = rows[0], [r for r in rows[1:] if len(r) >= len(rows[0]) - 1 and r[1]]
    m = re.search(r"\((left|right)\)\s*\(id = (0x[0-9a-fA-F]+)\)", header[0])
    hand, dev_id = (m.group(1), int(m.group(2), 16)) if m else ("left" if "left" in path else "right", 1)
    cols = header[3:]
    # split into groups at the second occurrence of the first column name
    second = cols.index(cols[0], 1)
    groups = []
    for gname, gcols, start in (("default", cols[:second], 3), ("emf_only", cols[second:], 3 + second)):
        streams, layout, i = [], [], 0
        seen_arm_transform = False
        while i < len(gcols):
            name = gcols[i]
            base = re.sub(r"\[\d+\]$", "", name)
            n = 1
            while i + n < len(gcols) and re.sub(r"\[\d+\]$", "", gcols[i + n]) == base and gcols[i + n] != gcols[i]:
                n += 1
            target, kind = base.split(" ", 1)
            st = {"target_frame": target}
            if kind == "STATUS_FLAGS":
                st.update(data_type="UINT32", measure_type="STATUS_FLAGS")
                if target == "hub":
                    st["bit_mapping"] = FLAG_MAPS["hub"]
                elif target == "arm_sensor" and seen_arm_transform and gname == "default":
                    st["bit_mapping"] = FLAG_MAPS["coil"]
                    st["reference_frame"] = "coil_pro"
                else:
                    st["bit_mapping"] = FLAG_MAPS["sensor"]
            elif kind == "relative_emf_residual":
                st.update(data_type="FLOAT", measure_type="CUSTOM", custom_label="relative_emf_residual",
                          reference_frame="coil_pro" if (gname == "default" and target == "arm_sensor") else "hub")
            else:
                st.update(data_type=f"FLOAT[{n}]" if n > 1 else "FLOAT", measure_type=kind)
                if kind == "TRANSFORM":
                    if gname == "default" and target == "arm_sensor" and any(
                            s.get("target_frame") == "arm_sensor" and s["measure_type"] == "TRANSFORM" for s in streams):
                        st["reference_frame"] = "coil_pro"
                        seen_arm_transform = True
                    else:
                        st["reference_frame"] = "hub"
                elif kind == "ORIENTATION" and target == "hub":
                    st["reference_frame"] = "LTP_NED"
            streams.append(st)
            layout.append((start + i, n, st["data_type"] == "UINT32"))
            i += n
        groups.append((gname, streams, layout))
    definition = {
        "protocol_name": "RGMP", "protocol_version": "2.0.0", "device_id": dev_id,
        "device_type": "Smartgloves II", "timestamp_epoch": "device_boot",
        "device_info": {"friendly_name": f"Smartgloves II ({hand}) [replay]", "hand": hand},
        "static_data": [],
        "groups": [{"name": g, "expected_rate_hz": 250 / 3, "streams": s} for g, s, _ in groups],
    }
    frames = []
    for r in data:
        ts = int(float(r[1]))
        packed = []
        for gid, (_, _, layout) in enumerate(groups):
            body = bytearray()
            for col, n, is_int in layout:
                vals = r[col:col + n]
                if is_int:
                    body += struct.pack("<I", int(float(vals[0] or 0)))
                else:
                    body += struct.pack(f"<{n}f", *[float(v or 0) for v in vals])
            packed.append((gid, bytes(body)))
        frames.append((ts, packed))
    return hand, dev_id, definition, frames


class Replay:
    def __init__(self, gloves, port, speed, loop):
        self.gloves, self.port, self.speed, self.loop = gloves, port, speed, loop
        self.clients, self.lock = [], threading.Lock()

    @staticmethod
    def frame(prefix, payload):
        return struct.pack("<II", prefix, len(payload)) + payload

    def defs(self):
        return b"".join(self.frame(1, json.dumps(g[2]).encode()) for g in self.gloves)

    def accept_loop(self):
        srv = socket.socket()
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", self.port))
        srv.listen()
        print(f"[replay] serving RGMP on 127.0.0.1:{self.port}", flush=True)
        while True:
            c, a = srv.accept()
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                c.sendall(self.defs())
            except OSError:
                continue
            with self.lock:
                self.clients.append(c)
            print(f"[replay] client {a} connected", flush=True)

    def send(self, data):
        with self.lock:
            for c in list(self.clients):
                try:
                    c.sendall(data)
                except OSError:
                    self.clients.remove(c)

    def run(self):
        threading.Thread(target=self.accept_loop, daemon=True).start()
        n = min(len(g[3]) for g in self.gloves)
        dur = (self.gloves[0][3][n - 1][0] - self.gloves[0][3][0][0]) / 1e6
        print(f"[replay] {n} frames per glove, {dur:.1f} s, speed x{self.speed}, loop={self.loop}", flush=True)
        offset_us, lap = 0, 0
        while True:
            t_start = time.perf_counter()
            t0 = [g[3][0][0] for g in self.gloves]
            for k in range(n):
                due = (self.gloves[0][3][k][0] - t0[0]) / 1e6 / self.speed
                wait = t_start + due - time.perf_counter()
                if wait > 0:
                    time.sleep(wait)
                out = b""
                for gi, (_, dev, _, frames) in enumerate(self.gloves):
                    ts, packed = frames[k]
                    for gid, body in packed:
                        out += self.frame(2, struct.pack("<IIQ", dev, gid, ts + offset_us) + body)
                self.send(out)
                if k % 830 == 0:
                    print(f"[replay] lap {lap + 1}  t = {due * self.speed:5.1f} / {dur:.1f} s", flush=True)
            if not self.loop:
                print("[replay] finished (use --loop to repeat)", flush=True)
                time.sleep(2)
                return
            lap += 1
            offset_us += int(dur * 1e6) + 12000      # keep timestamps strictly increasing


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--left", required=True)
    ap.add_argument("--right", required=True)
    ap.add_argument("--port", type=int, default=12276)
    ap.add_argument("--speed", type=float, default=1.0, help="0.5 = half speed, 2 = double")
    ap.add_argument("--loop", action="store_true")
    a = ap.parse_args()
    gloves = []
    for path in (a.left, a.right):
        hand, dev, definition, frames = load_glove(path)
        print(f"[replay] loaded {hand} glove 0x{dev:08x}: {len(frames)} frames, "
              f"groups {[g['name'] + '(' + str(len(g['streams'])) + ')' for g in definition['groups']]}", flush=True)
        gloves.append((hand, dev, definition, frames))
    Replay(gloves, a.port, a.speed, a.loop).run()


if __name__ == "__main__":
    main()
