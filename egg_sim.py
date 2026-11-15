#!/usr/bin/env python3
"""
egg_sim.py - Steady Hands: drive a simulated two-finger gripper in MuJoCo from glove_bridge.py
and pick up an egg. Logs one JSONL file per trial into trials/.

Chain:  glove_bridge.py --rate 83.33 --udp 127.0.0.1:9870  -->  egg_sim.py (this)  --> MuJoCo viewer

  python egg_sim.py                                   # viewer, practice mode (nothing logged)
  python egg_sim.py --log --operator AB               # log every trial to trials/
  python egg_sim.py --headless --duration 20          # no window (tests)

Mapping (position and pinch only; the gripper always points down)
  * While the bridge says "engaged", the gripper target moves by the right wrist's motion in the
    ROBOT frame (bridge field hands.right.ee_pos: X fwd, Y left, Z up). The anchor (hand pose <->
    gripper pose) is taken on the first engaged frame after a reset, and again after each
    re-engage (clutch). Keep the bridge's --scale at 1.0.
  * hands.right.pinch_m (thumb-index sensor distance, metres) sets finger travel:
    pinch_open -> 0 (open), pinch_closed -> 0.04 m (closed). Calibrate with "po" / "pc".
  * The target is clamped to a box above the table so the gripper cannot go through it.

Assist (assist.py; mode set with --assist or the command "a"):
  off   plain direct teleop: scale 1.0, no filter, 60 N force cap, no colour cue
  on    grip limiter 5 N + motion scaling (0.3 within 8 cm of egg/target, 1.0 beyond 10 cm)
        + 1-euro jitter filter + colour cue (green / yellow >= 15 N / red broken)
        + blue ball at the raw one-to-one hand position
  grip  grip limiter only (optional third condition)
  Switching re-anchors filter and scaler at the current pose, so the gripper does not jump.
  A trial during which the mode changed is flagged ("condition_changed") in its log.

Trial: starts at reset (egg at (0.50, -0.10), gripper 20 cm above it). Ends when
  * the egg broke (grip force > BREAK_N)                         -> failure
  * the egg was lifted, then rests on the table for 0.5 s with the gripper open
                                                                 -> success if within 20 mm of target
  * the egg fell off the table, or 60 s passed                   -> failure

Glove-only mode (--no-coil): for when the Coil Pro is missing. The glove drives only the pinch;
the gripper position comes from the keyboard. Wrist fields from the bridge are ignored (they are
null when coil_ok is false). Motion scaling and the filter are inactive; "a" still switches the
grip limiter (60 N / 5 N) and the colour cue. Trials are logged with condition "glove_only"
(the assist state is in the "assist" field), so report.py keeps them apart from full trials.
  x+ x- y+ y- z+ z- [N]   move the gripper N x 5 mm (default 1); x = away from the camera,
                          y+ = screen left, z+ = up. Also: fw bk lf rt up dn
  MuJoCo window keys      arrow up/down = x+/x-, arrow left/right = y+/y-, PageUp/PageDown = z+/z-
  grasp (or g)            move over the egg, then descend to grasp height (0.475 m)
  carry (or c)            lift to 0.60 m, move over the target, descend to place height (0.478 m)
  home                    back to the start position;   stop   cancels an automatic move
Glove LED (--led): the right glove's ring LED shows the same cue as the egg (assist on: green
< 15 N, yellow 15-30 N; red once broken; off otherwise). Needs RCSP (port 45451), so start the
bridge with --no-rcsp so that only egg_sim.py drives the LED. Test the LED first: led_test.py.

Terminal commands (type + Enter in THIS window; the MuJoCo window keeps its own keys):
  r            reset = start a new trial (an unfinished trial is logged as "aborted")
  l            toggle logging (off = practice attempts)
  op NAME      set the operator name written into the log
  po / pc      calibrate pinch: hold the hand OPEN / thumb touching index, then type it
  a            toggle assist off <-> on;  a on | a off | a grip  sets it
  p / t / auto motion scaling with assist on: fixed precision / fixed travel / proximity rule (default)
  s            print status
  q            quit
The same commands can be sent as UDP text to --cmd-port (default 9871) from another window or script:
  python -c "import socket;socket.socket(2,2).sendto(b'r',('127.0.0.1',9871))"

Needs: pip install mujoco==4.0.1  (Python 3.10+)
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import queue
import socket
import sys
import threading
import time

import numpy as np
import mujoco

from assist import ASSIST_PARAMS, GripLimiter, JitterFilter, MotionScaler

# ── scene / task parameters (written into every trial log; assist parameters live in assist.py) ──
SIM_PARAMS = {
    "break_n": 30.0,            # egg breaks above this grip force (Koprivica et al. 2021: 32.41 N mean)
    "finger_kp": 3000.0,        # finger position-servo stiffness (N/m) - difficulty setting
    "travel_max_m": 0.04,       # full finger travel
    "home": [0.50, -0.10, 0.62],  # gripper palm at reset: 20 cm above the egg's centre
    "egg_start": [0.50, -0.10],
    "target_xy": [0.50, 0.20],
    "box_min": [0.25, -0.35, 0.472],  # clamp for the gripper target (palm centre)
    "box_max": [0.75, 0.35, 0.80],
    "success_radius_m": 0.020,
    "rest_time_s": 0.5,
    "lift_detect_m": 0.020,     # egg must have risen this much before "rests on table" counts
    "open_travel_m": 0.016,     # command below this = gripper open (pads touch the egg at ~0.018)
    "timeout_s": 60.0,
    "fingertip_below_palm_m": 0.07,   # proximity for motion scaling is measured from the finger tips
    "holding_n": 0.5,           # grip force above this = egg held (egg proximity ignored while carrying)
}
MODES = ("off", "on", "grip")   # grip = grip limiter only (optional third condition)
GLOVE_ONLY = {               # keyboard control in --no-coil mode (logged in the header)
    "step_m": 0.005,
    "auto_speed_mps": 0.08,
    "grasp_z": 0.475,
    "carry_z": 0.60,
    "place_z": 0.478,
}
KEY_MOVES = {265: ("x", 1), 264: ("x", -1), 263: ("y", 1), 262: ("y", -1), 266: ("z", 1), 267: ("z", -1)}
LED_RGB = {"green": (0, 255, 0), "yellow": (255, 180, 0), "red": (255, 0, 0), "off": (0, 0, 0)}
PINCH_CALIB_FILE = "pinch_calib.json"
PINCH_DEFAULT = {"open": 0.09, "closed": 0.03}   # metres; replace with "po"/"pc" on real gloves
STATE_EVERY = 1


def vec(v):
    return [round(float(x), 5) for x in v]


# ── bridge listener ───────────────────────────────────────────────────────────────────────
class BridgeListener(threading.Thread):
    """Keeps the latest bridge message for one hand."""

    def __init__(self, port: int, hand: str):
        super().__init__(daemon=True)
        self.hand = hand
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.latest = None          # (msg, hand_dict, n)
        self.n = 0
        self.pinch_hist = collections.deque(maxlen=42)   # ~0.5 s
        self.running = True

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65536)
                msg = json.loads(data)
            except (socket.timeout, ConnectionResetError):
                continue
            except (ValueError, OSError):
                continue
            h = (msg.get("hands") or {}).get(self.hand)
            if not h:
                continue
            with self.lock:
                self.n += 1
                self.latest = (msg, h, self.n)
                if h.get("pinch_m") is not None:
                    self.pinch_hist.append(h["pinch_m"])

    def get(self):
        with self.lock:
            return self.latest

    def pinch_mean(self):
        with self.lock:
            return sum(self.pinch_hist) / len(self.pinch_hist) if self.pinch_hist else None


def command_reader(q: queue.Queue, port: int | None):
    def stdin_loop():
        for line in sys.stdin:
            q.put(line.strip())

    def udp_loop():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", port))
        while True:
            data, _ = s.recvfrom(4096)
            for line in data.decode(errors="replace").splitlines():
                q.put(line.strip())

    threading.Thread(target=stdin_loop, daemon=True).start()
    if port:
        threading.Thread(target=udp_loop, daemon=True).start()


# ── glove ring LED over RCSP (reuses glove_bridge.RcspWorker; glove_bridge.py is not modified) ──
class GloveLed:
    def __init__(self, host: str, port: int, hand: str):
        self.hand, self.state, self.last_set = hand, None, 0.0
        self.ok = False
        try:                                        # probe once so a failure prints the exact error
            socket.create_connection((host, port), timeout=2.0).close()
        except OSError as e:
            print(f"[led] RCSP {host}:{port} not reachable: {type(e).__name__}: {e} - "
                  f"LED disabled, on-screen colour only")
            return
        from glove_bridge import RcspWorker
        self.worker = RcspWorker(host, port, queue.Queue(), lambda m: print(f"[led] {m}"))
        self.worker.start()
        self.ok = True
        self.t0, self.warned = time.time(), False

    def has_glove(self) -> bool:
        return self.ok and self.hand in self.worker.dev_hand.values()

    def set(self, name: str):
        if not self.ok:
            return
        if not self.has_glove():
            if not self.warned and time.time() - self.t0 > 5.0:
                self.warned = True
                print(f"[led] RCSP connected={self.worker.connected} but no {self.hand} glove listed "
                      f"(found {self.worker.dev_hand}) - LED not set")
            return
        now = time.time()
        if name != self.state and now - self.last_set > 0.1:
            self.worker.set_led(self.hand, LED_RGB[name])
            self.state, self.last_set = name, now

    def close(self):
        if self.has_glove():
            self.worker.set_led(self.hand, LED_RGB["off"])
            time.sleep(0.4)                         # let the worker send it


# ── the simulation ────────────────────────────────────────────────────────────────────────
class EggSim:
    def __init__(self, args):
        self.args = args
        self.m = mujoco.MjModel.from_xml_path(args.scene)
        self.d = mujoco.MjData(self.m)
        m = self.m
        gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
        bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        self.egg_geom, self.pads, self.table = gid("egg"), [gid("pad_l"), gid("pad_r")], gid("table")
        self.egg_body, self.grip_body = bid("egg"), bid("gripper")
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "egg")
        self.egg_qadr, self.egg_vadr = m.jnt_qposadr[jid], m.jnt_dofadr[jid]
        self.egg_rgba0 = m.geom_rgba[self.egg_geom].copy()
        self.f6 = np.zeros(6)
        m.actuator_gainprm[:, 0] = SIM_PARAMS["finger_kp"]
        m.actuator_biasprm[:, 1] = -SIM_PARAMS["finger_kp"]
        self.pinch = dict(PINCH_DEFAULT)
        if os.path.exists(PINCH_CALIB_FILE):
            try:
                self.pinch.update(json.load(open(PINCH_CALIB_FILE)).get(args.hand, {}))
            except (ValueError, OSError):
                pass
        self.logging = args.log
        self.operator = args.operator
        self.trial_no = 0
        self.log_fh = None
        self.mode = args.assist
        self.scale_mode = "auto"        # auto (proximity) | precision | travel
        self.limiter = GripLimiter()
        self.scaler = MotionScaler(1.0)
        self.filter = JitterFilter()
        self.precision = False
        self.filtered_in = None
        self.no_coil = args.no_coil
        self.auto_path = []
        self.auto_name = None
        self.last_tick = time.perf_counter()
        self.led = GloveLed(args.rcsp_host, args.rcsp_port, args.hand) if args.led else None
        self.reset()

    # ---- trial bookkeeping -----------------------------------------------------------
    def reset(self):
        if getattr(self, "trial_active", False):
            self.end_trial("aborted")
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        sx, sy = SIM_PARAMS["egg_start"]
        d.qpos[self.egg_qadr:self.egg_qadr + 3] = [sx, sy, 0.4225]
        d.mocap_pos[0] = SIM_PARAMS["home"]
        d.ctrl[:] = 0.0
        mujoco.mj_forward(m, d)
        m.geom_rgba[self.egg_geom] = self.egg_rgba0
        self.target = np.array(SIM_PARAMS["home"], dtype=float)
        self.auto_path, self.auto_name = [], None
        self.anchor_in = None       # hand ee_pos at anchor time
        self.anchor_out = None      # gripper target at anchor time
        self.raw_target = self.target.copy()
        self.raw_anchor_in = None
        self.raw_anchor_out = None
        self.reanchor = True
        self.precision = False
        self.scaler.scale = 1.0
        self.travel = 0.0
        self.trial_mode = self.mode
        self.condition_changed = False
        self.frange = self.limiter.forcerange(self.mode != "off")
        self.cap = self.frange[1]
        self.grip = 0.0
        self.peak = 0.0
        self.frame_peak = 0.0
        self.broken = False
        self.lifted = False
        self.rest_since = None
        self.egg_rest_z = None
        self.t0_sim = d.time
        self.t0_wall = time.time()
        self.result = None
        self.trial_active = True
        self.trial_no += 1
        self.frames = 0
        if self.logging:
            self.open_log()

    def condition(self):
        return "glove_only" if self.no_coil else self.trial_mode

    def params(self):
        p = {"sim": SIM_PARAMS, "assist": ASSIST_PARAMS, "pinch_calib": self.pinch,
             "scale_mode": self.scale_mode, "led": bool(self.led and self.led.ok)}
        if self.no_coil:
            p["glove_only"] = GLOVE_ONLY
        return p

    def set_mode(self, mode: str):
        if mode == self.mode:
            return
        self.mode = mode
        self.reanchor = True            # filter + scaler restart at the current pose: no jump
        self.frange = self.limiter.forcerange(mode != "off")
        self.cap = self.frange[1]
        if self.trial_active and mode != self.trial_mode:
            self.condition_changed = True
        print(f"[sim] assist {mode.upper()}"
              + ("  (changed during a trial: this trial is flagged)" if self.condition_changed else ""))

    def open_log(self):
        os.makedirs(self.args.trials_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        cond = self.condition() + (f"-{self.trial_mode}" if self.no_coil else "")
        name = f"{stamp}_{cond}_{self.operator or 'anon'}.jsonl"
        self.log_path = os.path.join(self.args.trials_dir, name)
        self.log_fh = open(self.log_path, "w", buffering=1 << 16)
        header = {"type": "header", "condition": self.condition(), "assist": self.trial_mode,
                  "operator": self.operator,
                  "start_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "start_unix": time.time(),
                  "trial_no": self.trial_no, "hand": self.args.hand, "scene": self.args.scene,
                  "params": self.params()}
        self.log_fh.write(json.dumps(header) + "\n")
        print(f"[sim] logging trial {self.trial_no} -> {self.log_path}")

    def end_trial(self, reason: str):
        if not self.trial_active:
            return
        self.trial_active = False
        egg = self.d.xpos[self.egg_body]
        err = math.dist(egg[:2], SIM_PARAMS["target_xy"])
        success = (not self.broken) and reason == "placed" and err <= SIM_PARAMS["success_radius_m"]
        self.result = {"type": "end", "reason": reason, "broken": self.broken, "success": success,
                       "peak_grip_n": round(self.peak, 3), "placement_error_mm": round(err * 1000, 1),
                       "time_s": round(self.d.time - self.t0_sim, 3), "egg_final": vec(egg),
                       "frames": self.frames, "condition": self.condition(), "assist": self.trial_mode,
                       "condition_changed": self.condition_changed}
        tag = "SUCCESS" if success else ("BROKEN" if self.broken else reason.upper())
        print(f"[sim] trial {self.trial_no} ended: {tag}  peak {self.peak:.1f} N  "
              f"placement error {err * 1000:.0f} mm  time {self.result['time_s']:.1f} s")
        if self.log_fh:
            self.log_fh.write(json.dumps(self.result) + "\n")
            self.log_fh.close()
            self.log_fh = None
            if reason == "aborted" and self.frames == 0:
                os.remove(self.log_path)        # nothing happened in this trial
                print(f"[sim] (empty trial, log removed)")

    # ---- physics helpers -------------------------------------------------------------
    def grip_force(self):
        m, d = self.m, self.d
        per_pad = dict.fromkeys(self.pads, 0.0)
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 == self.egg_geom or g2 == self.egg_geom:
                other = g2 if g1 == self.egg_geom else g1
                if other in per_pad:
                    mujoco.mj_contactForce(m, d, i, self.f6)
                    per_pad[other] += self.f6[0]
        return max(per_pad.values())

    def step(self):
        self.m.actuator_forcerange[:, 0] = self.frange[0]     # opening: always the full force
        self.m.actuator_forcerange[:, 1] = self.frange[1]     # closing: GripLimiter cap
        mujoco.mj_step(self.m, self.d)
        g = self.grip_force()
        self.grip = g
        self.frame_peak = max(self.frame_peak, g)
        if self.trial_active:
            self.peak = max(self.peak, g)
            if g > SIM_PARAMS["break_n"] and not self.broken:
                self.broken = True
                self.m.geom_rgba[self.egg_geom] = [0.9, 0.05, 0.05, 1]
                self.end_trial("broken")

    def egg_colour(self):
        """Force cue (assist on only): green < yellow_n <= yellow < break_n; red once broken."""
        if self.broken:
            return
        if self.mode == "on":
            rgba = [0.1, 0.8, 0.1, 1] if self.grip < ASSIST_PARAMS["yellow_n"] else [0.95, 0.85, 0.05, 1]
            self.m.geom_rgba[self.egg_geom] = rgba
        else:
            self.m.geom_rgba[self.egg_geom] = self.egg_rgba0

    def draw_markers(self, viewer):
        """Assist on: a blue ball at the raw one-to-one hand position (palm centre), next to the gripper."""
        scn = viewer.user_scn
        scn.ngeom = 0
        if self.mode != "on" or self.raw_anchor_in is None:
            return
        mujoco.mjv_initGeom(scn.geoms[0], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.012, 0, 0]),
                            np.asarray(self.raw_target, dtype=float), np.eye(3).flatten(),
                            np.array([0.2, 0.4, 1.0, 0.6], dtype=np.float32))
        scn.ngeom = 1

    # ---- glove -> gripper ------------------------------------------------------------
    def choose_scale(self):
        """Motion-scaling rule. Assist off / grip-only: always 1.0."""
        p = ASSIST_PARAMS
        if self.mode != "on":
            return 1.0
        if self.scale_mode == "precision":
            self.precision = True
        elif self.scale_mode == "travel":
            self.precision = False
        else:   # auto: precision near the egg (unless carrying it) or near the target
            tip = self.target - np.array([0, 0, SIM_PARAMS["fingertip_below_palm_m"]])
            dists = [math.dist(tip, [SIM_PARAMS["target_xy"][0], SIM_PARAMS["target_xy"][1], 0.401])]
            if self.grip < SIM_PARAMS["holding_n"]:
                dists.append(math.dist(tip, self.d.xpos[self.egg_body]))
            dmin = min(dists)
            if self.precision and dmin > p["precision_exit_m"]:
                self.precision = False
            elif not self.precision and dmin < p["precision_enter_m"]:
                self.precision = True
        return p["scale_precision"] if self.precision else p["scale_travel"]

    def apply_frame(self, msg, h):
        engaged = bool(msg.get("engaged"))
        ee = h.get("ee_pos") if h.get("coil_ok") else None
        box_min, box_max = SIM_PARAMS["box_min"], SIM_PARAMS["box_max"]
        if self.no_coil:
            pass                            # glove-only: position comes from the keyboard (tick)
        elif engaged and ee is not None:
            ee = np.asarray(ee, dtype=float)
            t = h["ts_us"] / 1e6 if h.get("ts_us") is not None else None
            use_filter = self.mode == "on"
            if self.anchor_in is None:              # (re-)engage: raw one-to-one marker anchors here
                self.raw_anchor_in, self.raw_anchor_out = ee.copy(), self.target.copy()
                self.reanchor = True
            if self.reanchor:                       # engage, reset or assist switch: restart, no jump
                self.filter.reset()
                f = self.filter(ee.tolist(), t) if use_filter else ee.tolist()
                self.scaler.scale = self.choose_scale()
                self.scaler.reset(f, self.target.tolist())
                self.anchor_in, self.anchor_out = ee.copy(), self.target.copy()
                self.reanchor = False
            else:
                f = self.filter(ee.tolist(), t) if use_filter else ee.tolist()
                s = self.choose_scale()
                if s != self.scaler.scale:          # re-anchor at the last input and the CLAMPED output
                    self.scaler.scale = s
                    self.scaler.reset(self.scaler.last_in, self.target.tolist())
            self.filtered_in = f
            self.target = np.clip(np.asarray(self.scaler(f)), box_min, box_max)
            self.raw_target = np.clip(self.raw_anchor_out + (ee - self.raw_anchor_in), box_min, box_max)
        else:
            self.anchor_in = None           # clutch out: hold, re-anchor on re-engage
        p = h.get("pinch_m")
        if p is not None:
            span = self.pinch["open"] - self.pinch["closed"]
            frac = (self.pinch["open"] - p) / span if abs(span) > 1e-6 else 0.0
            self.travel = min(max(frac, 0.0), 1.0) * SIM_PARAMS["travel_max_m"]
        self.d.mocap_pos[0] = self.target
        self.d.ctrl[:] = self.travel

    # ---- glove-only mode: keyboard position, automatic moves, LED ---------------------
    def nudge(self, axis: str, sign: int, count: int = 1):
        if not self.no_coil:
            print("[sim] keyboard moves work only with --no-coil (the hand drives the gripper)")
            return
        self.auto_path, self.auto_name = [], None
        k = "xyz".index(axis)
        self.target[k] += sign * count * GLOVE_ONLY["step_m"]
        self.target = np.clip(self.target, SIM_PARAMS["box_min"], SIM_PARAMS["box_max"])

    def start_auto(self, name: str):
        if not self.no_coil:
            print("[sim] grasp / carry / home work only with --no-coil")
            return
        g, (x, y, z) = GLOVE_ONLY, self.target
        if name == "grasp":
            ex, ey = self.d.xpos[self.egg_body][:2]
            zt = max(z, 0.55)
            path = [(x, y, zt), (ex, ey, zt), (ex, ey, g["grasp_z"])]
        elif name == "carry":
            tx, ty = SIM_PARAMS["target_xy"]
            path = [(x, y, g["carry_z"]), (tx, ty, g["carry_z"]), (tx, ty, g["place_z"])]
        else:
            hx, hy, hz = SIM_PARAMS["home"]
            path = [(x, y, hz), (hx, hy, hz)]
        self.auto_path = [np.clip(np.array(p, dtype=float), SIM_PARAMS["box_min"], SIM_PARAMS["box_max"])
                          for p in path]
        self.auto_name = name
        print(f"[sim] auto {name}: " + " -> ".join("(" + ", ".join(f"{v:.3f}" for v in p) + ")"
                                                for p in self.auto_path))

    def led_colour(self) -> str:
        """Same rule as the on-screen cue."""
        if self.broken:
            return "red"
        if self.mode == "on":
            return "green" if self.grip < ASSIST_PARAMS["yellow_n"] else "yellow"
        return "off"

    def tick(self):
        """Called once per main-loop iteration."""
        now = time.perf_counter()
        dt, self.last_tick = min(now - self.last_tick, 0.05), now
        if self.no_coil:
            if self.auto_path:
                goal = self.auto_path[0]
                delta = goal - self.target
                dist = float(np.linalg.norm(delta))
                step = GLOVE_ONLY["auto_speed_mps"] * dt
                if dist <= step:
                    self.target = goal.copy()
                    self.auto_path.pop(0)
                    if not self.auto_path:
                        print(f"[sim] auto {self.auto_name} done")
                        self.auto_name = None
                else:
                    self.target = self.target + delta * (step / dist)
            self.d.mocap_pos[0] = self.target
        if self.led:
            self.led.set(self.led_colour())

    def shutdown(self):
        if self.led:
            self.led.close()

    def check_trial(self):
        if not self.trial_active:
            return
        d = self.d
        egg = d.xpos[self.egg_body]
        if self.egg_rest_z is None and d.time - self.t0_sim > 0.3:
            self.egg_rest_z = float(egg[2])
        if self.egg_rest_z is not None and egg[2] > self.egg_rest_z + SIM_PARAMS["lift_detect_m"]:
            self.lifted = True
        speed = float(np.linalg.norm(d.qvel[self.egg_vadr:self.egg_vadr + 3]))
        on_table = egg[2] < 0.40 + 0.035 and abs(egg[0] - 0.5) < 0.3 and abs(egg[1]) < 0.4
        resting = (self.lifted and on_table and speed < 0.01 and self.grip < 0.2
                   and self.travel < SIM_PARAMS["open_travel_m"])
        if resting:
            if self.rest_since is None:
                self.rest_since = d.time
            elif d.time - self.rest_since >= SIM_PARAMS["rest_time_s"]:
                self.end_trial("placed")
                return
        else:
            self.rest_since = None
        if egg[2] < 0.35:
            self.end_trial("dropped")
        elif d.time - self.t0_sim > SIM_PARAMS["timeout_s"]:
            self.end_trial("timeout")

    def log_frame(self, msg, h):
        self.frames += 1
        if not self.log_fh:
            return
        raw_hand = {k: v for k, v in h.items() if k != "finger_quat"}
        rec = {"t": msg.get("t"), "seq": msg.get("seq"), "engaged": msg.get("engaged"), "hand": raw_hand,
               "sim_t": round(self.d.time - self.t0_sim, 4), "assist": self.condition() == "on",
               "raw_target": vec(self.raw_target), "target": vec(self.target), "travel": round(self.travel, 5),
               "cap_n": self.cap, "gripper": vec(self.d.xpos[self.grip_body]),
               "egg": vec(self.d.xpos[self.egg_body]), "grip_n": round(self.grip, 3),
               "grip_peak_n": round(self.frame_peak, 3)}
        rec.update(self.extra_log())
        self.log_fh.write(json.dumps(rec) + "\n")

    def extra_log(self):
        return {"mode": self.mode, "scale": self.scaler.scale, "precision": self.precision,
                "filtered_in": vec(self.filtered_in) if self.filtered_in is not None else None,
                "forcerange": list(self.frange)}

    # ---- commands --------------------------------------------------------------------
    def command(self, cmd: str, listener: BridgeListener) -> bool:
        if not cmd:
            return True
        parts = cmd.split(maxsplit=1)
        c = parts[0].lower()
        if c == "q":
            return False
        if c == "r":
            self.reset()
            print(f"[sim] reset - trial {self.trial_no} started ({self.condition()}, "
                  f"{'LOGGING' if self.logging else 'practice'})")
        elif c == "l":
            self.logging = not self.logging
            print(f"[sim] logging {'ON' if self.logging else 'OFF (practice)'} - applies from the next reset")
        elif c == "op":
            self.operator = parts[1].strip() if len(parts) > 1 else None
            print(f"[sim] operator = {self.operator}")
        elif c in ("po", "pc"):
            p = listener.pinch_mean()
            if p is None:
                print("[sim] no pinch data from the bridge yet")
            else:
                self.pinch["open" if c == "po" else "closed"] = round(p, 4)
                all_cal = {}
                if os.path.exists(PINCH_CALIB_FILE):
                    try:
                        all_cal = json.load(open(PINCH_CALIB_FILE))
                    except (ValueError, OSError):
                        all_cal = {}
                all_cal[self.args.hand] = self.pinch
                json.dump(all_cal, open(PINCH_CALIB_FILE, "w"), indent=2)
                print(f"[sim] pinch calibration {self.pinch} saved to {PINCH_CALIB_FILE}")
        elif c == "s":
            print(self.status_line())
        elif not self.extra_command(c, parts):
            print(f"[sim] unknown command {cmd!r} (r, l, op NAME, po, pc, a [on|off|grip], p, t, auto, s, q)")
        return True

    def extra_command(self, c, parts) -> bool:
        if c == "a":
            arg = parts[1].strip().lower() if len(parts) > 1 else None
            if arg is None:
                self.set_mode("off" if self.mode == "on" else "on")
            elif arg in MODES:
                self.set_mode(arg)
            else:
                print(f"[sim] a takes one of {MODES}")
            return True
        if c in ("p", "t", "auto"):
            self.scale_mode = {"p": "precision", "t": "travel", "auto": "auto"}[c]
            print(f"[sim] motion scaling: {self.scale_mode} (used only with assist on)")
            return True
        alias = {"fw": "x+", "bk": "x-", "lf": "y+", "rt": "y-", "up": "z+", "dn": "z-"}
        mv = alias.get(c, c)
        if len(mv) == 2 and mv[0] in "xyz" and mv[1] in "+-":
            try:
                count = int(parts[1]) if len(parts) > 1 else 1
            except ValueError:
                count = 1
            self.nudge(mv[0], 1 if mv[1] == "+" else -1, max(1, min(count, 100)))
            return True
        if c in ("grasp", "g", "carry", "c", "home"):
            self.start_auto({"g": "grasp", "c": "carry"}.get(c, c))
            return True
        if c == "stop":
            self.auto_path, self.auto_name = [], None
            print("[sim] auto move stopped")
            return True
        if c == "key" and len(parts) > 1:            # from the MuJoCo window's key callback
            axis, sign = KEY_MOVES[int(parts[1])]
            self.nudge(axis, sign)
            return True
        return False

    def extra_help(self):
        return "a [on|off|grip], p, t, auto" + (
            ", x+/x-/y+/y-/z+/z- [N], grasp, carry, home, stop" if self.no_coil else "")

    def status_line(self):
        st = "running" if self.trial_active else (
            "SUCCESS" if self.result and self.result["success"] else
            ("BROKEN" if self.broken else (self.result or {}).get("reason", "-")))
        return (f"[sim] trial {self.trial_no} {st}  t {self.d.time - self.t0_sim:5.1f}s  "
                f"grip {self.grip:5.1f} N  peak {self.peak:5.1f} N  travel {self.travel * 1000:4.1f} mm  "
                f"target {' '.join(f'{x:+.3f}' for x in self.target)}  pinch {self.pinch}")

    def overlay(self):
        st = "running" if self.trial_active else (
            "SUCCESS" if self.result and self.result["success"] else
            ("EGG BROKEN" if self.broken else (self.result or {}).get("reason", "-")))
        if self.no_coil:
            scale = "keyboard (no Coil)" + (f", auto {self.auto_name}" if self.auto_name else "")
        else:
            scale = (f"x{self.scaler.scale:.1f} ({'precision' if self.precision else 'travel'})"
                     if self.mode == "on" else "x1.0")
        cap = f"{self.cap:.0f} N"
        left = "Assist\nMotion\nGrip cap\nTrial\nTime\nGrip force\nPeak\nState"
        right = (f"{self.mode.upper()}{' (grip limit only)' if self.mode == 'grip' else ''}\n{scale}\n{cap}\n"
                 f"{self.trial_no}{' (logged)' if self.log_fh else ''}\n"
                 f"{self.d.time - self.t0_sim:.1f} s\n{self.grip:.1f} N\n{self.peak:.1f} N\n{st}")
        return left, right


# ── main loop ─────────────────────────────────────────────────────────────────────────────
def run(sim_cls, args):
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:
        pass
    sim = sim_cls(args)
    listener = BridgeListener(args.port, args.hand)
    listener.start()
    cmds: queue.Queue = queue.Queue()
    command_reader(cmds, args.cmd_port)
    state_sock, state_addr = None, None
    if args.state_udp:
        host, port = args.state_udp.rsplit(":", 1)
        state_sock, state_addr = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (host, int(port))

    viewer = None
    if not args.headless:
        import mujoco.viewer
        def on_key(keycode):                       # runs in the viewer thread: hand over via the queue
            if sim.no_coil and keycode in KEY_MOVES:
                cmds.put(f"key {keycode}")

        viewer = mujoco.viewer.launch_passive(sim.m, sim.d, key_callback=on_key,
                                              show_left_ui=False, show_right_ui=False)
        # operator's point of view: behind the robot looking +X, robot-left (+Y) on the screen's left
        viewer.cam.lookat[:] = [0.5, 0.05, 0.45]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.0, 25.0, -30.0
    print(f"[sim] listening for bridge JSON on UDP :{args.port} ({args.hand} hand); "
          f"commands: r l op po pc s q (+ {sim.extra_help() if hasattr(sim, 'extra_help') else ''})")
    print(f"[sim] trial {sim.trial_no} started ({sim.condition()}, {'LOGGING' if sim.logging else 'practice'})")

    dt = sim.m.opt.timestep
    wall0, sim0 = time.perf_counter(), sim.d.time
    last_n, last_sync = 0, 0.0
    rep_wall, rep_sim, rep_n = wall0, sim.d.time, 0
    t_end = wall0 + args.duration if args.duration else None
    ratios = []
    running = True
    while running:
        now = time.perf_counter()
        if t_end and now > t_end:
            break
        if viewer is not None and not viewer.is_running():
            break
        while not cmds.empty():
            c = cmds.get()
            if c.strip().lower() == "r":
                # re-sync the real-time clock after a reset
                running = sim.command(c, listener)
                wall0, sim0 = time.perf_counter(), sim.d.time
                rep_wall, rep_sim = wall0, sim.d.time
            else:
                running = sim.command(c, listener) and running
        if not running:
            break
        got = listener.get()
        if got and got[2] != last_n:
            last_n = got[2]
            msg, h, _ = got
            sim.apply_frame(msg, h)
            sim.log_frame(msg, h)
            sim.frame_peak = 0.0
            if state_sock:
                st = {"target": vec(sim.target), "gripper": vec(sim.d.xpos[sim.grip_body]),
                      "egg": vec(sim.d.xpos[sim.egg_body]), "grip_n": round(sim.grip, 2),
                      "trial_active": sim.trial_active, "trial_no": sim.trial_no,
                      "result": sim.result}
                state_sock.sendto(json.dumps(st).encode(), state_addr)
        sim.tick()
        # step physics until simulated time catches up with the wall clock
        target_sim = sim0 + (time.perf_counter() - wall0)
        n = 0
        while sim.d.time < target_sim and n < 100:
            sim.step()
            n += 1
        if n:
            sim.check_trial()
        if sim.d.time < target_sim - 0.1:           # fell far behind: do not try to catch up
            wall0, sim0 = time.perf_counter(), sim.d.time
        sim.egg_colour()
        if viewer is not None and now - last_sync > 1 / 60:
            last_sync = now
            l, r = sim.overlay()
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, l, r))
            sim.draw_markers(viewer) if hasattr(sim, "draw_markers") else None
            viewer.sync()
        if now - rep_wall >= 1.0:
            ratio = (sim.d.time - rep_sim) / (now - rep_wall)
            ratios.append(ratio)
            rate = (listener.n - rep_n) / (now - rep_wall)
            if not args.quiet:
                print(f"[sim] realtime {ratio:.3f}  bridge {rate:5.1f} msg/s  grip {sim.grip:5.1f} N  "
                      f"travel {sim.travel * 1000:4.1f} mm  target {' '.join(f'{x:+.3f}' for x in sim.target)}"
                      f"{'' if got else '  (no bridge data)'}", flush=True)
            rep_wall, rep_sim, rep_n = now, sim.d.time, listener.n
            if args.exit_after and sim.trial_no >= args.exit_after and not sim.trial_active:
                break
        if sim.d.time >= sim0 + (time.perf_counter() - wall0):
            time.sleep(0.0005)
    if sim.trial_active and sim.log_fh:
        sim.end_trial("aborted")
    sim.shutdown()
    listener.running = False
    if viewer is not None:
        viewer.close()
    if ratios:
        r = ratios[1:] or ratios
        print(f"[sim] realtime ratio over {len(r)} s: min {min(r):.3f}  mean {sum(r) / len(r):.3f}  max {max(r):.3f}")


def build_parser(desc=None):
    ap = argparse.ArgumentParser(description=desc or __doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default="egg_scene.xml")
    ap.add_argument("--port", type=int, default=9870, help="UDP port of the bridge stream (default 9870)")
    ap.add_argument("--hand", default="right", choices=["right", "left"])
    ap.add_argument("--cmd-port", type=int, default=9871, help="UDP port for text commands (0 = off)")
    ap.add_argument("--state-udp", default=None, help="host:port to send sim state per frame (test operator)")
    ap.add_argument("--assist", default="off", choices=MODES, help="assist mode at start (command 'a' toggles)")
    ap.add_argument("--no-coil", action="store_true",
                    help="glove-only mode: keyboard drives the position, the glove only the pinch")
    ap.add_argument("--led", action="store_true", help="show the force cue on the glove ring LED (RCSP)")
    ap.add_argument("--rcsp-host", default="127.0.0.1")
    ap.add_argument("--rcsp-port", type=int, default=45451)
    ap.add_argument("--log", action="store_true", help="log trials from the start (otherwise 'l' toggles)")
    ap.add_argument("--operator", default=None)
    ap.add_argument("--trials-dir", default="trials")
    ap.add_argument("--headless", action="store_true", help="no viewer window")
    ap.add_argument("--duration", type=float, default=0, help="quit after this many seconds (0 = never)")
    ap.add_argument("--exit-after", type=int, default=0, help="quit once trial number N has ended")
    ap.add_argument("--quiet", action="store_true", help="no 1 Hz status line")
    return ap


if __name__ == "__main__":
    run(EggSim, build_parser().parse_args())
