#!/usr/bin/env python3
"""rgmp_recenter.py - make rkk-hand-solver's OpenXR hands wrist-relative again.

Isaac's RokokoGloveDevice expects the xr_wrist joint at (0,0,0) in the joints_openxr
group. On our setup the solver sends world positions (~0.7 m away). This relay
subtracts the wrist position from every joint position of joints_openxr (orientations
untouched) and passes everything else through unchanged.

  python3 rgmp_recenter.py --upstream 192.0.2.10:12277 --listen 0.0.0.0:12278
  then point Isaac at --host 127.0.0.1 --port 12278
Stdlib only. The removed wrist offset is printed every 5 s (useful later as a Coil anchor).
"""
import argparse, json, re, socket, struct, threading, time

GROUP = "joints_openxr"


def recv_exact(s, n):
    b = bytearray()
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c:
            raise ConnectionError("closed")
        b += c
    return bytes(b)


def parse_axes(spec):
    """'x,y,z' = identity; e.g. '-y,x,z' means out_x = -in_y, out_y = in_x, out_z = in_z."""
    m = []
    for tok in spec.replace(" ", "").lower().split(","):
        row = [0.0, 0.0, 0.0]
        row["xyz".index(tok.lstrip("+-"))] = -1.0 if tok.startswith("-") else 1.0
        m.append(row)
    if len(m) != 3:
        raise SystemExit(f"--axes needs 3 entries, got {spec!r}")
    return m


class Relay:
    def __init__(self, up, listen, follow=False, scale=1.0, axes="x,y,z", max_reach=0.6,
                 coil_port=None, yaw_deg=0.0):
        self.up, self.listen = up, listen
        self.follow, self.scale, self.max_reach = follow, scale, max_reach
        self.coil_port = coil_port
        self.yaw = yaw_deg * 3.141592653589793 / 180.0
        self.pose = {}          # hand -> (ee_pos in robot frame, time) from glove_bridge
        self.last_d = {}        # hand -> last robot-frame offset (held when Coil drops out)
        self.hand_of = {}       # device_id -> "left"/"right"
        self.axes = parse_axes(axes)
        self.ref = {}           # device_id -> wrist position captured at "zero"
        self.rezero = True
        self.clients, self.lock = [], threading.Lock()
        self.defs = {}      # device_id -> raw definition frame bytes
        self.layout = {}    # device_id -> (group_id, [pos offsets], wrist offset)
        self.last_wrist = {}

    def broadcast(self, data):
        with self.lock:
            for c in list(self.clients):
                try:
                    c.sendall(data)
                except OSError:
                    self.clients.remove(c)

    def on_definition(self, payload):
        d = json.loads(payload)
        dev = d["device_id"]
        for gid, g in enumerate(d.get("groups", [])):
            if g.get("name") != GROUP:
                continue
            offs, wrist, off = [], None, 16          # 16-byte data-frame header
            for st in g["streams"]:
                dims = re.findall(r"\d+", "".join(re.findall(r"\[([^\]]*)\]", st["data_type"])) or "1")
                size = 8 if st["data_type"].lstrip().startswith(("DOUBLE", "INT64", "UINT64")) else 4
                for k in dims:
                    size *= int(k)
                if st.get("measure_type") == "TRANSFORM" and size == 28:
                    offs.append(off)
                    if st.get("target_frame") == "xr_wrist":
                        wrist = off
                off += size
            if wrist is not None:
                self.layout[dev] = (gid, offs, wrist)
                self.hand_of[dev] = (d.get("device_info") or {}).get("hand", "?")
                print(f"[relay] {d.get('device_info', {}).get('hand', '?')} hand {dev}: "
                      f"recentering {len(offs)} joints (group {gid})", flush=True)

    def fix(self, payload):
        dev, gid = struct.unpack_from("<II", payload, 0)
        lay = self.layout.get(dev)
        if not lay or lay[0] != gid:
            return payload
        _, offs, w = lay
        b = bytearray(payload)
        wx, wy, wz = struct.unpack_from("<3f", b, w)
        tx = ty = tz = 0.0                      # where the wrist should end up (xr_base)
        if self.coil_port:
            import math
            hand = self.hand_of.get(dev, "?")
            P = self.pose.get(hand)
            if P and time.time() - P[1] < 0.3:
                if self.rezero or hand not in self.ref:
                    self.ref[hand] = P[0]
                    self.zeroed = getattr(self, "zeroed", set()) | {hand}
                r0 = self.ref[hand]
                d = [(P[0][i] - r0[i]) * self.scale for i in range(3)]          # robot frame: X fwd, Y left, Z up
                d = [sum(self.axes[i][j] * d[j] for j in range(3)) for i in range(3)]
                c, s_ = math.cos(self.yaw), math.sin(self.yaw)
                d = [c * d[0] - s_ * d[1], s_ * d[0] + c * d[1], d[2]]
                n = (d[0] ** 2 + d[1] ** 2 + d[2] ** 2) ** 0.5
                if n > self.max_reach:
                    d = [v * self.max_reach / n for v in d]
                self.last_d[hand] = d
            d = self.last_d.get(hand, [0.0, 0.0, 0.0])
            # robot frame -> OpenXR xr_base (X right, Y up, -Z forward)
            tx, ty, tz = -d[1], d[2], -d[0]
        elif self.follow:
            if self.rezero or dev not in self.ref:
                self.ref[dev] = (wx, wy, wz)
            r0 = self.ref[dev]
            d = [(wx - r0[0]) * self.scale, (wy - r0[1]) * self.scale, (wz - r0[2]) * self.scale]
            d = [sum(self.axes[i][j] * d[j] for j in range(3)) for i in range(3)]
            n = (d[0] ** 2 + d[1] ** 2 + d[2] ** 2) ** 0.5
            if n > self.max_reach:
                d = [v * self.max_reach / n for v in d]
            tx, ty, tz = d
        for o in offs:
            x, y, z = struct.unpack_from("<3f", b, o)
            struct.pack_into("<3f", b, o, x - wx + tx, y - wy + ty, z - wz + tz)
        self.last_wrist[dev] = (wx, wy, wz)
        if self.rezero and self.coil_port:
            # done once every connected hand has captured a Coil reference
            if getattr(self, "zeroed", set()) >= set(self.hand_of.values()):
                self.rezero, self.zeroed = False, set()
        elif self.rezero and self.follow:
            self.pending = getattr(self, "pending", set()) | {dev}
            if self.pending >= set(self.layout):
                self.rezero, self.pending = False, set()
        return bytes(b)

    def upstream_loop(self):
        host, port = self.up.rsplit(":", 1)
        while True:
            try:
                s = socket.create_connection((host, int(port)), timeout=5)
                s.settimeout(None)
                print(f"[relay] connected to solver {self.up}", flush=True)
                while True:
                    hdr = recv_exact(s, 8)
                    prefix, n = struct.unpack("<II", hdr)
                    p = recv_exact(s, n)
                    if prefix == 1:
                        self.on_definition(p)
                        self.defs[json.loads(p)["device_id"]] = hdr + p
                        self.broadcast(hdr + p)
                    elif prefix == 2:
                        p = self.fix(p)
                        self.broadcast(struct.pack("<II", 2, len(p)) + p)
                    else:
                        if prefix == 3:
                            dev = struct.unpack_from("<I", p)[0]
                            self.defs.pop(dev, None); self.layout.pop(dev, None)
                        self.broadcast(hdr + p)
            except (OSError, ConnectionError) as e:
                print(f"[relay] solver {self.up}: {e}; retrying in 1 s", flush=True)
                self.defs.clear(); self.layout.clear()
                time.sleep(1)

    def accept_loop(self):
        host, port = self.listen.rsplit(":", 1)
        srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, int(port))); srv.listen()
        print(f"[relay] serving recentered hands on {self.listen}", flush=True)
        while True:
            c, a = srv.accept()
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.lock:
                try:
                    for d in list(self.defs.values()):
                        c.sendall(d)
                    self.clients.append(c)
                except OSError:
                    continue
            print(f"[relay] client {a} connected", flush=True)

    def coil_loop(self):
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        u.bind(("0.0.0.0", self.coil_port))
        print(f"[relay] listening for glove_bridge Coil poses on UDP :{self.coil_port}", flush=True)
        seen = set()
        while True:
            data, addr = u.recvfrom(65536)
            try:
                msg = json.loads(data)
            except ValueError:
                continue
            for hand, h in (msg.get("hands") or {}).items():
                if h.get("coil_ok") and h.get("ee_pos"):
                    self.pose[hand] = (tuple(h["ee_pos"]), time.time())
                    if hand not in seen:
                        seen.add(hand)
                        print(f"[relay] Coil pose arriving for {hand} hand from {addr[0]}", flush=True)

    def stdin_loop(self):
        import sys
        for _ in sys.stdin:
            self.rezero = True
            print("[relay] re-zeroed: robot hands return to their start pose, "
                  "follow your motion from here", flush=True)

    def run(self):
        if self.coil_port:
            print(f"[relay] COIL FOLLOW mode: scale {self.scale}, axes {self.axes}, yaw {self.yaw:.2f} rad, "
                  f"max reach {self.max_reach} m. Press Enter here to re-zero.", flush=True)
            threading.Thread(target=self.coil_loop, daemon=True).start()
            threading.Thread(target=self.stdin_loop, daemon=True).start()
        elif self.follow:
            print(f"[relay] FOLLOW mode: scale {self.scale}, axes {self.axes}, max reach "
                  f"{self.max_reach} m. Press Enter here to re-zero.", flush=True)
            threading.Thread(target=self.stdin_loop, daemon=True).start()
        threading.Thread(target=self.upstream_loop, daemon=True).start()
        threading.Thread(target=self.accept_loop, daemon=True).start()
        while True:
            time.sleep(5)
            for dev, w in self.last_wrist.items():
                print(f"[relay] device {dev} wrist offset removed: "
                      + " ".join(f"{v:+.3f}" for v in w), flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--upstream", required=True, help="solver host:port, e.g. 192.0.2.10:12277")
    ap.add_argument("--listen", default="0.0.0.0:12278")
    ap.add_argument("--follow", action="store_true",
                    help="robot wrists follow your hand motion (relative to the pose at start / last Enter)")
    ap.add_argument("--scale", type=float, default=1.0, help="hand motion -> robot motion multiplier")
    ap.add_argument("--axes", default="x,y,z", help="remap motion axes, e.g. '-z,x,y'")
    ap.add_argument("--max-reach", type=float, default=0.6, help="clamp wrist travel (m)")
    ap.add_argument("--follow-coil", type=int, default=None, metavar="UDP_PORT",
                    help="wrists follow the Coil Pro pose sent by glove_bridge.py (robot frame) on this UDP port")
    ap.add_argument("--yaw-deg", type=float, default=0.0,
                    help="rotate hand motion about the vertical axis (deg) if forward is off by an angle")
    a = ap.parse_args()
    Relay(a.upstream, a.listen, a.follow, a.scale, a.axes, a.max_reach, a.follow_coil, a.yaw_deg).run()
