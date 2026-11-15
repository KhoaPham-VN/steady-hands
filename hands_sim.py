#!/usr/bin/env python3
"""
hands_sim.py - two simulated hands (left + right) in MuJoCo driven by the two Rokoko gloves,
glove only (no Coil Pro): the wrists stay fixed, the fingers follow the gloves. A soft bar (a
capsule across the palm, like a bottle or handle) is mounted in each hand so the grip force can
be measured. Does not touch egg_sim.py or its scene.

  python hands_sim.py                      # finger data from glove_bridge.py (UDP 9870), viewer
  python hands_sim.py --led                # also drive each glove's ring LED (bridge must run with --no-rcsp)
  python hands_sim.py --source solver      # finger data from rkk-hand-solver's OpenXR joints (TCP 12277)
  python hands_sim.py --write-scene        # (re)write hands_scene.xml and exit
  python hands_sim.py --headless --duration 20
  python hands_sim.py --no-coil            # fixed wrists (no Coil Pro)

Coil positioning (default; needs --source bridge): each wrist follows its glove's Coil position
(hands.<side>.ee_pos, robot frame X fwd / Y left / Z up) with egg_sim.py's clutch: while the bridge
says "engaged" (type e in the bridge window) the wrist moves from an anchor taken on each (re-)engage;
disengaged = the wrist holds. Per hand: JitterFilter (1-euro, assist.py) then MotionScaler
(assist.py). A blue ghost ball shows the raw 1:1 hand position. Each wrist is clamped to its own
box (BOX), so the hands cannot cross.
Wrist rotation (default on with the Coil): each hand turns with its glove's orientation
(hands.<side>.ee_quat = the back-of-hand sensor in the robot frame, all three axes). Same clutch as
the position: on each (re-)engage the glove's orientation and the simulated hand's orientation are
anchored, and the hand turns by how far the glove has turned since then (no jump, nothing mirrored).
Smoothed by a 1-euro filter on the quaternion (slerp, RotFilter). Rotation is never scaled. The
forearm and the bar are part of the hand body, so they turn with it (the bar stays mounted).
  scale [L|R|both] X     motion scale (0.3: 10 cm of hand travel = 3 cm); position only; re-anchors
  filter on|off|HZ       position jitter filter on/off or its cutoff (default from assist.py); re-anchors
  rot on|off             wrist rotation on (re-anchors, no jump) / off (palms up, the old behaviour)
  rot reset              palms back up, re-anchored at the gloves' current orientation
  rotfilter on|off|HZ    rotation filter on/off or its cutoff (default PARAMS rot_filter_min_cutoff_hz)
  jitter reset           restart the cumulative jitter numbers
Pick-up (a table 10 cm under the hands' home, ONE free object in the middle between the hands, real
physics: the hands are free bodies pulled to their targets by a weld, so friction carries the object):
  pick can|cube|ball|egg pick mode with that object (6x10 cm can / 5 cm cube / 6 cm ball, 150 g each;
                         egg = egg_scene.xml's 4.4 x 5.8 cm, 60 g egg standing in an egg cup),
                         bars off, object in the middle; each hand may cross the middle by 8 cm
                         Egg: breaks when a hand's finger + thumb force exceeds egg_sim.py's break_n
                         (30 N): red, state BROKEN, trial fails; yellow from assist.py's yellow_n (15 N);
                         "pick reset" / "pick egg" bring a whole egg back
  pick                   start a trial: lift it 5 cm above the table and hold 2 s = SUCCESS
                         (hands_tasks/<time>_pick.json: time to touch / lift, hands, peak grip, drops)
  pick reset | stop | off   object back in the middle | end the trial | leave pick mode
  pick assist on|off     two-hand grasp-assist (default on): only when BOTH hands press the object
                         (each >= 1 N finger force with >= 2 of fingers/thumb/palm, for 0.15 s) it is
                         linked to both hands by stiff welds; released when either hand drops below
                         0.3 N. One hand alone is always pure physics.
  In pick mode the grip force (overlay, LED, force cap) is the finger + thumb contact force on the object.
  States: on table / grasped / lifted / dropped / fell off the table.
  home                   wrists back home, palms up (each re-anchors on its next engaged frame)
  task / task stop       reach and hold, both hands at once: 5 targets per hand; palm centre inside
                         the 2 cm sphere while gripping the bar at 40-100 % of the force cap,
                         held 3 s. Scored per target: reach time, hold steadiness (mm RMS), breaks,
                         cap overshoots (> 105 %). Result: hands_tasks/<time>_reach_hold.json
Overlay per hand also shows: wrist roll / pitch / yaw (deg, X fwd / Y left / Z up, from palm-up
home), scale, filter cutoff, and the jitter (eval_offline.jitter_mm) CUMULATIVE since the last
filter / scale / engage change or "jitter reset", raw vs filtered (and sim = filtered x scale),
with the seconds of still data, plus the part above 3 Hz (where a 3 Hz filter acts). A 3 s window
was too noisy: on the organizers' recording "filt" read above "raw" in 17 of 62 (left) and 32 of 86
(right) 3 s windows, although over the whole recording the filter lowers it.

Hands: 5 fingers each. Index..little: MCP -15..90 deg, PIP 0..110, DIP 0..80. Thumb: CMC
(opposition) 0..60, MCP 0..60, IP 0..80. Each joint is a position servo. Source "bridge": one
0..1 curl per finger (hands.<side>.curl), mapped through this script's own per-hand open/fist
calibration (hands_calib.json, kept separately per source; calib.json is never touched), drives that finger's 3 joints in
proportion to their ranges. Source "solver": joint angles computed from the solver's OpenXR
joint positions (see SolverSource).

Grip limits (both active, PLAN: two-hand test):
  A  angle limit  "limit angle 80": each finger may close to at most 80 % of its joint range.
                  % of limit = the most-closed finger's actual flexion / the angle limit.
  B  force cap    "limit force 8": the closing torque of that hand's finger joints is capped so the
                  grip force on its object settles at 8 N (the cap adapts to the measured force);
                  it cannot squeeze harder. Opening always has full torque.
                  % of limit = grip force / cap.
  LED per hand (and the object's colour on screen): worse of A and B ->
      green < warn (80 %) <= yellow < 97 % <= red;  off when quitting.

Terminal commands (type + Enter here, or send UDP text to --cmd-port 9874):
  open L|R|both      calibrate OPEN: hold that hand flat, then type it
  fist L|R|both      calibrate FIST: make a fist, then type it
  calib reset L|R|both
  limit angle PCT    (also: limit PCT)        limit force N    (0 = no force cap)
  warn PCT           yellow threshold (default 80)
  obj on|off         mount / remove both objects (off: fingers can close fully; on: each object
                     appears once that hand is open, so it never traps closed fingers)
  peak               reset the peak-force readings (also reset by "limit" and when an object mounts)
  s                  status         q   quit (LEDs off)
Needs: pip install mujoco==4.0.1
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

from assist import ASSIST_PARAMS, JitterFilter, MotionScaler, _alpha   # defaults unchanged
from eval_offline import jitter_mm

FINGERS = ("thumb", "index", "middle", "ring", "little")
SHORT = {"thumb": "T", "index": "I", "middle": "M", "ring": "R", "little": "P"}
SIDES = ("left", "right")
SCENE_FILE = "hands_scene.xml"
CALIB_FILE = "hands_calib.json"
MIN_SPAN = 0.10          # a finger's fist curl must exceed its open curl by this (0.10 = 28 deg of total bend)
CALIB_SAMPLES = 40       # open / fist use the mean of the last ~0.5 s of frames
LED_RGB = {"green": (0, 255, 0), "yellow": (255, 180, 0), "red": (255, 0, 0), "off": (0, 0, 0)}
SCREEN_RGBA = {"green": (0.1, 0.8, 0.1, 0.55), "yellow": (0.95, 0.8, 0.05, 0.55), "red": (0.9, 0.1, 0.1, 0.55),
               "off": (0.95, 0.95, 0.9, 0.55)}           # semi-transparent so the fingers stay visible

PARAMS = {
    "angle_limit_pct": 100.0,   # A: fingers may close to this % of their joint range
    "force_cap_n": 10.0,        # B: grip-force cap per hand (0 = off)
    "warn_pct": 80.0,           # LED yellow from here
    "red_pct": 97.0,            # LED red from here ("at/over the limit"). Was 99: a grip held at the force
                                # cap sits at 98.9-99.7 % while the hands move, so the LED flickered red/yellow
    "joint_kp": 2.0,            # N*m/rad per joint servo (was 1.0: a full fist with no force cap then
                                # squeezed the small egg with only ~22 N, below its 30 N break threshold)
    "joint_torque_max": 2.0,    # N*m per joint actuator
    "torque_per_n": 0.004,      # force cap: starting closing-torque cap per joint, N*m per N of cap
                                # (tested 0.004/0.006/0.008: 0.004 gives the smallest first-contact
                                # overshoot, about 1.3-1.5x the cap, settling within 5 % in 0.5-1 s)
    "sigma_gain": 4.0,          # 1/s, how fast the torque cap adapts to the measured force
    "sigma_min": 0.002,         # N*m, lowest closing-torque cap
    "contact_n": 0.05,          # the hand counts as touching its ball above this force
    "rot_filter_min_cutoff_hz": 3.0,   # wrist rotation 1-euro filter (same cutoff as the position filter)
    "rot_filter_beta": 0.0,            # 0 = plain low-pass, like the position filter
}

# finger geometry: (y offset towards the thumb side, (proximal, middle, distal) lengths, radius)
GEOM = {
    "index":  (0.027, (0.040, 0.025, 0.020), 0.0085),
    "middle": (0.009, (0.044, 0.028, 0.021), 0.0085),
    "ring":   (-0.009, (0.041, 0.027, 0.020), 0.0082),
    "little": (-0.027, (0.032, 0.020, 0.018), 0.0075),
}
JOINT_RANGE = {"mcp": (-15, 90), "pip": (0, 110), "dip": (0, 80),
               "cmc": (0, 60), "tmcp": (0, 60), "ip": (0, 80)}
FINGER_JOINTS = {"thumb": ("cmc", "tmcp", "ip"), "index": ("mcp", "pip", "dip"),
                 "middle": ("mcp", "pip", "dip"), "ring": ("mcp", "pip", "dip"),
                 "little": ("mcp", "pip", "dip")}
HAND_POS = {"left": (0.0, 0.13, 0.30), "right": (0.0, -0.13, 0.30)}
# The grasped object is a soft bar (capsule across the palm, like a bottle or a handle), mounted in
# place. A ball was tried first: the outer fingers sit at its edge, curl past its side and get stuck
# in it (contact pushes sideways, the finger hinges cannot move sideways) -> meaningless forces.
OBJ = {"radius": 0.026, "x": 0.035, "z": 0.040, "half_width": 0.040}

# Coil positioning: each wrist (palm centre) is clamped to its own box so the hands cannot cross
BOX = {"left": ((-0.20, 0.04, 0.15), (0.30, 0.35, 0.55)),
       "right": ((-0.20, -0.35, 0.15), (0.30, -0.04, 0.55))}
# pick mode: each hand may cross the middle by 8 cm to reach the object
BOX_PICK = {"left": ((-0.20, -0.08, 0.12), (0.30, 0.35, 0.55)),
            "right": ((-0.20, -0.35, 0.12), (0.30, 0.08, 0.55))}
# Each hand is a free body pulled to its mocap target by a soft weld (a teleported mocap hand has no
# velocity, so friction could not carry a lifted object); gravity compensated.
HAND_WELD_SOLREF = "0.006 1"
# The commanded pose changes in steps (one per glove frame, 12 ms). A welded hand jerked by those steps
# shook the grip (force-cap test 8.8-11.3 N while moving), so each physics step moves the mocap target
# towards the commanded pose through two first-order stages of this time constant (~12 ms lag).
TARGET_SMOOTH_S = 0.006
# ---- pick-up: a table and ONE free object between the hands ----
TABLE_Z = 0.20                     # table top height (m); the hands' home is 10 cm above it
PICK_XY = (0.05, 0.0)              # object start: middle between the hands, a little in front of the palms
PICK_SHAPES = {                    # 150 g each; "rest" = height of the centre above the table top
    "can":  {"geom": 'type="cylinder" size="0.030 0.050"', "rest": 0.050},
    "cube": {"geom": 'type="box" size="0.025 0.025 0.025"', "rest": 0.025},
    "ball": {"geom": 'type="sphere" size="0.030"', "rest": 0.030},
    # the egg from egg_scene.xml (ellipsoid 4.4 x 5.8 cm, 60 g, condim 6, its friction and colour),
    # standing up in a small egg cup (EGG_CUP) so it cannot roll; breakable (EGG)
    "egg":  {"geom": 'type="ellipsoid" size="0.022 0.022 0.029" condim="6" friction="1 0.02 0.005" '
                     'rgba="1 0.95 0.85 1"', "rest": 0.0317, "mass": 0.060},   # rest: settled in the cup
}
# egg cup: a ring of 8 short capsules on the table, solid and visible only in egg mode
EGG_CUP = {"ring_r": 0.019, "tube_r": 0.006, "n": 8}
# breakable egg: same thresholds as egg_sim.py / assist.py (imported, not copied)
from egg_sim import SIM_PARAMS as _EGG_SIM_PARAMS     # noqa: E402  (egg_sim only defines things on import)
EGG = {"break_n": _EGG_SIM_PARAMS["break_n"],          # a hand's finger + thumb force above this breaks it
       "yellow_n": ASSIST_PARAMS["yellow_n"],          # egg turns yellow from here
       "rgba": (1.0, 0.95, 0.85, 1.0), "yellow_rgba": (0.95, 0.85, 0.05, 1.0), "broken_rgba": (0.9, 0.05, 0.05, 1.0)}
PICK_MASS = 0.150
PICK = {"lift_m": 0.05, "hold_s": 2.0, "touch_n": 0.05, "drop_lift_m": 0.02}
# Two-hand grasp-assist (user's choice, 15 Nov): one hand lifts by pure physics, but with both hands
# the two sets of fingers wrap the front of the object and push it out of the grip as the lift starts
# (7 contact/pose variants tried). So ONLY when both hands press it, the object is linked to both hands
# by soft welds; released as soon as either hand lets go. The grip shown and capped stays the real
# finger + thumb contact force.
ASSIST = {"on": True, "attach_n": 1.0,     # each hand: finger + thumb force on the object at least this
          "min_parts": 2,                  # each hand: at least this many fingers / thumb / palm touching
          "part_n": 0.3,                   # a finger counts as pressing above this force
          "attach_s": 0.15,                # ... both conditions held this long (fingers settled)
          "release_n": 0.3, "release_s": 0.1,   # release when either hand's force stays below this
          "solref": "0.004 1"}             # stiff: 0.02 let the 10 N finger squeeze move the object ~4 cm
JITTER_BLOCK_S = 3.0      # cumulative jitter: scored in 3 s blocks, summed (a partial block counts from 1.1 s)
JITTER_HF_HZ = 3.0        # second jitter line: only the part above this frequency
# reach and hold (both hands at once): palm centre inside the target sphere while the grip on the bar
# is between grip_min_frac and over_frac of the force cap, continuously for hold_s
TASK = {"n_targets": 5, "radius": 0.02, "hold_s": 3.0, "timeout_s": 30.0,
        "grip_min_frac": 0.4, "over_frac": 1.05, "grip_min_n_no_cap": 2.0}
# target offsets from each hand's home (x fwd, y OUTWARD = +y left hand / -y right hand, z up)
TARGET_OFFSETS = [(0.08, 0.0, 0.05), (-0.05, 0.05, 0.10), (0.10, -0.03, -0.04), (0.0, 0.08, 0.0),
                  (0.05, 0.02, 0.12)]


# ── scene ─────────────────────────────────────────────────────────────────────────────────
def build_scene_xml() -> str:
    def hand(side):
        s = 1 if side == "left" else -1           # thumb side is +y for the left hand, -y for the right
        p = side[0].upper()
        hx, hy, hz = HAND_POS[side]
        out = [f'    <body name="{p}_ref" pos="{hx} {hy} {hz}" mocap="true"/>',
               f'    <body name="{p}_hand" pos="{hx} {hy} {hz}">',
               f'      <freejoint name="{p}_free"/>',
               f'      <geom name="{p}_palm" type="box" size="0.045 0.042 0.01" class="hand" mass="0.2"/>',
               f'      <geom name="{p}_forearm" type="capsule" fromto="-0.05 0 -0.004 -0.26 0 -0.03" size="0.03" '
               f'rgba="0.85 0.68 0.58 1"/>',
               f'      <geom name="{p}_obj" type="capsule" size="{OBJ["radius"]}" fromto="{OBJ["x"]} '
               f'{-OBJ["half_width"]} {OBJ["z"]} {OBJ["x"]} {OBJ["half_width"]} {OBJ["z"]}" class="obj"/>']
        for f, (y, lens, r) in GEOM.items():
            names = FINGER_JOINTS[f]
            out.append(f'      <body name="{p}_{f}_1" pos="0.045 {s * y:.4f} 0">')
            for k, (jn, L) in enumerate(zip(names, lens)):
                lo, hi = JOINT_RANGE[jn]
                ind = "  " * k
                out.append(f'      {ind}  <joint name="{p}_{f}_{jn}" axis="0 -1 0" range="{lo} {hi}"/>')
                out.append(f'      {ind}  <geom name="{p}_{f}_g{k}" type="capsule" fromto="0 0 0 {L} 0 0" '
                           f'size="{r}" class="hand"/>')
                if k < 2:
                    out.append(f'      {ind}  <body name="{p}_{f}_{k + 2}" pos="{L} 0 0">')
            out.append("            </body>\n          </body>\n      </body>")
        tl = (0.038, 0.030, 0.025)
        # thumb at palm level: 4 mm under the bar when open, closes onto it from below / the front
        out.append(f'      <body name="{p}_thumb_1" pos="0.0 {s * 0.036} 0.0" euler="0 0 {s * 35}">')
        out.append(f'        <joint name="{p}_thumb_cmc" axis="0 0 {-s}" range="0 60"/>')
        out.append(f'        <geom name="{p}_thumb_g0" type="capsule" fromto="0 0 0 {tl[0]} 0 0" size="0.010" class="hand"/>')
        out.append(f'        <body name="{p}_thumb_2" pos="{tl[0]} 0 0">')
        out.append(f'          <joint name="{p}_thumb_tmcp" axis="0 -1 0" range="0 60"/>')
        out.append(f'          <geom name="{p}_thumb_g1" type="capsule" fromto="0 0 0 {tl[1]} 0 0" size="0.0095" class="hand"/>')
        out.append(f'          <body name="{p}_thumb_3" pos="{tl[1]} 0 0">')
        out.append(f'            <joint name="{p}_thumb_ip" axis="0 -1 0" range="0 80"/>')
        out.append(f'            <geom name="{p}_thumb_g2" type="capsule" fromto="0 0 0 {tl[2]} 0 0" size="0.009" class="hand"/>')
        out.append("          </body>\n        </body>\n      </body>")
        out.append("    </body>")
        # gravity compensation on the hand body only (the weld holds the pose). NOT on the finger links:
        # with it, the force-capped thumb crept along the bar and the grip dipped to 9.2 N after ~9 s
        # (finger gravity as before keeps it seated: 9.65-10.25 N while rotating)
        return "\n".join(out).replace(f'<body name="{p}_hand"', f'<body gravcomp="1" name="{p}_hand"')

    def pick_bodies():
        out = []
        for k, (name, sh) in enumerate(PICK_SHAPES.items()):
            out.append(f'    <body name="pick_{name}" pos="{1.5 + 0.2 * k} 0 -0.5">\n'
                       f'      <freejoint name="pick_{name}_free"/>\n'
                       f'      <geom name="pick_{name}" {sh["geom"]} mass="{sh.get("mass", PICK_MASS)}" class="pick"/>\n'
                       f'    </body>')
        # egg cup ring (static, on the table under the object's start position)
        c = EGG_CUP
        side = 2 * c["ring_r"] * math.tan(math.pi / c["n"])
        for k in range(c["n"]):
            a = 2 * math.pi * k / c["n"]
            x, y = PICK_XY[0] + c["ring_r"] * math.cos(a), PICK_XY[1] + c["ring_r"] * math.sin(a)
            tx, ty = -math.sin(a) * side / 2, math.cos(a) * side / 2
            z = TABLE_Z + c["tube_r"]
            out.append(f'    <geom name="eggcup_{k}" type="capsule" fromto="{x - tx:.5f} {y - ty:.5f} {z} '
                       f'{x + tx:.5f} {y + ty:.5f} {z}" size="{c["tube_r"]}" contype="0" conaffinity="2" '
                       f'rgba="0.75 0.75 0.8 1"/>')
        return "\n".join(out)

    acts = []
    for side in SIDES:
        p = side[0].upper()
        for f in FINGERS:
            for jn in FINGER_JOINTS[f]:
                acts.append(f'    <position name="{p}_{f}_{jn}" joint="{p}_{f}_{jn}" kp="{PARAMS["joint_kp"]}" '
                            f'forcerange="-{PARAMS["joint_torque_max"]} {PARAMS["joint_torque_max"]}" '
                            f'ctrlrange="{math.radians(JOINT_RANGE[jn][0]):.4f} {math.radians(JOINT_RANGE[jn][1]):.4f}"/>')
    return f"""<mujoco model="steady_hands_two_hands">
  <!-- generated by hands_sim.py --write-scene; edit the tables in hands_sim.py, not this file -->
  <compiler angle="degree"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <default>
    <joint damping="0.02" armature="0.0002"/>
    <geom contype="0" conaffinity="0"/>
    <default class="hand">
      <geom contype="1" conaffinity="0" mass="0.01" friction="1 0.02 0.001" rgba="0.93 0.76 0.65 1"/>
    </default>
    <default class="obj">
      <geom contype="0" conaffinity="1" friction="1 0.02 0.001" solref="0.008 1" solimp="0.9 0.95 0.002"
            rgba="0.95 0.95 0.9 1"/>
    </default>
    <!-- collision groups: hand contype 1; pick object contype 2 / conaffinity 1 (hands hit it);
         table and floor conaffinity 2 (only the object rests on them; fingers pass through the table) -->
    <default class="pick">
      <geom contype="2" conaffinity="1" condim="4" friction="1.2 0.02 0.002" solref="0.004 1"
            solimp="0.95 0.99 0.001" rgba="0.85 0.25 0.2 1"/>
    </default>
  </default>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="1 1 0.1" contype="0" conaffinity="2" rgba="0.85 0.85 0.85 1"/>
    <geom name="table" type="box" pos="{PICK_XY[0] + 0.05} 0 {TABLE_Z / 2}" size="0.20 0.32 {TABLE_Z / 2}"
          contype="0" conaffinity="2" friction="0.8 0.01 0.001" rgba="0.55 0.42 0.30 1"/>
    <!-- extra views: in the viewer press ] / [ to cycle cameras, Esc for the free camera -->
    <camera name="side_left" pos="0.04 0.75 0.42" xyaxes="-1 0 0 0 -0.15 1"/>
    <camera name="front" pos="0.55 0 0.55" xyaxes="0 1 0 -0.6 0 0.8"/>
    <!-- reach-and-hold targets (positions and colours set at run time) -->
    <site name="L_target" type="sphere" size="{TASK['radius']}" pos="{HAND_POS['left'][0]} {HAND_POS['left'][1]} {HAND_POS['left'][2]}" rgba="1 0.8 0 0"/>
    <site name="R_target" type="sphere" size="{TASK['radius']}" pos="{HAND_POS['right'][0]} {HAND_POS['right'][1]} {HAND_POS['right'][2]}" rgba="1 0.8 0 0"/>
{hand("left")}
{hand("right")}
{pick_bodies()}
  </worldbody>
  <equality>
    <weld name="L_weld" body1="L_hand" body2="L_ref" solref="{HAND_WELD_SOLREF}"/>
    <weld name="R_weld" body1="R_hand" body2="R_ref" solref="{HAND_WELD_SOLREF}"/>
{chr(10).join(f'    <weld name="assist_{p}_{n}" body1="pick_{n}" body2="{p}_hand" active="false" solref="{ASSIST["solref"]}"/>'
              for n in PICK_SHAPES for p in ("L", "R"))}
  </equality>
  <actuator>
{chr(10).join(acts)}
  </actuator>
</mujoco>
"""


# ── wrist rotation: quaternions (w, x, y, z like MuJoCo; the bridge sends x, y, z, w) ────────
Q_ID = np.array([1.0, 0.0, 0.0, 0.0])


def q_from_xyzw(v):
    q = np.array([v[3], v[0], v[1], v[2]], dtype=float)
    return q / np.linalg.norm(q)


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw])


def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_angle(a, b):
    """Angle (rad) of the rotation between orientations a and b."""
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(a, b)))))


def q_slerp(a, b, t):
    if np.dot(a, b) < 0:
        b = -b
    dot = min(1.0, float(np.dot(a, b)))
    if dot > 0.9995:
        q = a + t * (b - a)
    else:
        th = math.acos(dot)
        q = (math.sin((1 - t) * th) * a + math.sin(t * th) * b) / math.sin(th)
    return q / np.linalg.norm(q)


def q_rpy(q):
    """(roll about X, pitch about Y, yaw about Z) in degrees, Z-Y-X order (yaw, then pitch, then roll)."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


class RotFilter:
    """1-euro filter for an orientation: the speed is the rotation angle per second between the
    filtered and the new orientation, the cutoff is min_cutoff + beta * speed, and the output moves
    towards the new orientation by slerp with the 1-euro alpha (same alpha as assist.OneEuro).
    Same timestamp handling as assist.JitterFilter (a repeated frame changes nothing)."""

    def __init__(self, min_cutoff, beta=0.0, d_cutoff=1.0, rate_hz=None):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.default_dt = 1.0 / (rate_hz or ASSIST_PARAMS["default_rate_hz"])
        self.reset()

    def reset(self):
        self.q, self.t, self.speed = None, None, 0.0

    def __call__(self, q, t=None):
        if self.q is None:
            self.q, self.t = np.array(q, dtype=float), t
            return self.q.copy()
        if t is None or self.t is None:
            dt = self.default_dt
        else:
            dt = t - self.t
            if dt == 0:
                return self.q.copy()             # same glove frame sent twice: no new information
            if not (0 < dt < 0.2):
                dt = self.default_dt
        if t is not None:
            self.t = t
        sp = q_angle(self.q, q) / dt
        self.speed += _alpha(self.d_cutoff, dt) * (sp - self.speed)
        self.q = q_slerp(self.q, np.asarray(q, dtype=float), _alpha(self.min_cutoff + self.beta * self.speed, dt))
        return self.q.copy()


# ── finger data sources ───────────────────────────────────────────────────────────────────
class BridgeSource(threading.Thread):
    """glove_bridge.py UDP JSON: per hand, the five 0..1 curls. Works without the Coil."""
    name = "bridge"

    def __init__(self, port: int):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.latest = {s: None for s in SIDES}     # side -> (time, {finger: curl})
        self.pose = {s: None for s in SIDES}       # side -> (frame no, engaged, ee_pos, ts_us, ee_quat, rx time)
        self.frames = {s: 0 for s in SIDES}
        self.n = 0

    def run(self):
        while True:
            try:
                msg = json.loads(self.sock.recvfrom(65536)[0])
            except (socket.timeout, ConnectionResetError, ValueError, OSError):
                continue
            now = time.time()
            with self.lock:
                self.n += 1
                for side in SIDES:
                    h = (msg.get("hands") or {}).get(side)
                    if not h:
                        continue
                    self.frames[side] += 1
                    ee = h.get("ee_pos") if h.get("coil_ok") else None   # null without the Coil
                    eq = h.get("ee_quat") if h.get("coil_ok") else None  # hub orientation, robot frame (x,y,z,w)
                    self.pose[side] = (self.frames[side], bool(msg.get("engaged")), ee, h.get("ts_us"), eq, now)
                    if not h.get("curl"):
                        continue
                    c = {f: h["curl"].get(f) for f in FINGERS}
                    if all(v is not None for v in c.values()):
                        self.latest[side] = (now, c)

    def get_pose(self, side):
        """(frame no, engaged, ee_pos or None, ts_us, ee_quat or None) of the latest frame, or None if stale."""
        with self.lock:
            p = self.pose[side]
        return p[:5] if p and time.time() - p[5] < 0.5 else None

    def get(self, side):
        """{finger: curl 0..1} or None if no fresh data (> 0.5 s old)."""
        with self.lock:
            v = self.latest[side]
        return v[1] if v and time.time() - v[0] < 0.5 else None


class SolverSource(threading.Thread):
    """rkk-hand-solver --emit-openxr (RGMP on TCP 12277), the stream hand_viewer.py draws.

    Per finger, flexion = sum of the bend angles at its three joints (angle between successive
    bone directions from the OpenXR joint positions), divided by that finger's full joint range
    -> a 0..1 curl, so the rest of the pipeline is the same as for the bridge."""
    name = "solver"
    # the thumb chain starts at the wrist so it has three bends like the fingers; the resting
    # offset this adds is removed by the per-hand open/fist calibration
    CHAINS = {"thumb": ("wrist", "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip"),
              "index": ("index_metacarpal", "index_proximal", "index_intermediate", "index_distal", "index_tip"),
              "middle": ("middle_metacarpal", "middle_proximal", "middle_intermediate", "middle_distal", "middle_tip"),
              "ring": ("ring_metacarpal", "ring_proximal", "ring_intermediate", "ring_distal", "ring_tip"),
              "little": ("little_metacarpal", "little_proximal", "little_intermediate", "little_distal", "little_tip")}

    def __init__(self, host: str, port: int):
        super().__init__(daemon=True)
        import hand_viewer                       # reuse its RGMP reader (it fills hand_viewer.STATE)
        self.hv = hand_viewer
        self.host, self.port = host, port
        self.n = 0

    def run(self):
        self.hv.rgmp_loop(self.host, self.port)

    def get(self, side):
        with self.hv.LOCK:
            st = dict(self.hv.STATE["hands"].get(side) or {})
        if not st or time.time() - st.get("t", 0) > 0.5 or not st.get("pos") or not st.get("names"):
            return None
        P = dict(zip(st["names"], st["pos"]))
        out = {}
        for f, chain in self.CHAINS.items():
            pts = [P.get(n) for n in chain]
            if any(p is None for p in pts):
                return None
            bend = 0.0
            for a, b, c in zip(pts, pts[1:], pts[2:]):
                u = np.subtract(b, a); v = np.subtract(c, b)
                nu, nv = np.linalg.norm(u), np.linalg.norm(v)
                if nu < 1e-6 or nv < 1e-6:
                    continue
                bend += math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(u, v) / (nu * nv))))))
            full = sum(JOINT_RANGE[j][1] for j in FINGER_JOINTS[f])
            out[f] = max(0.0, min(1.0, bend / full))
        self.n += 1
        return out


# ── LEDs (one RCSP connection for both gloves, glove_bridge.RcspWorker) ────────────────────
class GloveLeds:
    def __init__(self, host, port):
        self.ok, self.state, self.last = False, {}, {}
        try:
            socket.create_connection((host, port), timeout=2.0).close()
        except OSError as e:
            print(f"[led] RCSP {host}:{port} not reachable: {type(e).__name__}: {e} - on-screen colour only")
            return
        from glove_bridge import RcspWorker
        self.w = RcspWorker(host, port, queue.Queue(), lambda m: print(f"[led] {m}"))
        self.w.start()
        self.ok, self.t0, self.warned = True, time.time(), set()

    def set(self, side, name):
        if not self.ok:
            return
        if side not in self.w.dev_hand.values():
            if side not in self.warned and time.time() - self.t0 > 5:
                self.warned.add(side)
                print(f"[led] no {side} glove listed over RCSP ({self.w.dev_hand}) - {side} LED not set")
            return
        now = time.time()
        if self.state.get(side) != name and now - self.last.get(side, 0) > 0.1:
            self.w.set_led(side, LED_RGB[name])
            self.state[side], self.last[side] = name, now

    def close(self):
        if self.ok:
            for side in set(self.w.dev_hand.values()):
                self.w.set_led(side, LED_RGB["off"])
            time.sleep(0.5)


# ── simulation ────────────────────────────────────────────────────────────────────────────
class HandsSim:
    def __init__(self, args):
        xml = build_scene_xml()
        if not os.path.exists(args.scene) or (os.path.basename(args.scene) == SCENE_FILE
                                              and open(args.scene).read() != xml):
            open(args.scene, "w").write(xml)              # the generated scene is kept up to date
        self.args = args
        self.m = mujoco.MjModel.from_xml_path(args.scene)
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.p = dict(PARAMS)
        self.joint = {}       # (side, finger) -> [(qpos adr, actuator id, lo rad, hi rad)]
        for side in SIDES:
            P = side[0].upper()
            for f in FINGERS:
                lst = []
                for jn in FINGER_JOINTS[f]:
                    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{P}_{f}_{jn}")
                    aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{P}_{f}_{jn}")
                    lo, hi = (math.radians(v) for v in JOINT_RANGE[jn])
                    lst.append((m.jnt_qposadr[jid], aid, lo, hi))
                self.joint[(side, f)] = lst
        self.ball = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s[0].upper()}_obj") for s in SIDES}
        self.finger_geoms = {}  # geom id -> (side, finger)
        for g in range(m.ngeom):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            parts = name.split("_")
            if len(parts) == 3 and parts[0] in ("L", "R") and parts[1] in FINGERS:
                self.finger_geoms[g] = ("left" if parts[0] == "L" else "right", parts[1])
        self.ball_on = True
        self.mount_pending = {s: False for s in SIDES}
        self.source_name = getattr(args, "source", "bridge")   # calibration is kept per source
        self.calib = {s: {f: [0.0, 1.0] for f in FINGERS} for s in SIDES}
        if os.path.exists(CALIB_FILE):
            try:
                saved = json.load(open(CALIB_FILE)).get(self.source_name, {})
                for s in SIDES:
                    for f in FINGERS:
                        if f in saved.get(s, {}):
                            self.calib[s][f] = list(saved[s][f])
            except (ValueError, OSError):
                pass
        for s in SIDES:
            bad = [f for f in FINGERS if self.calib[s][f] != [0.0, 1.0] and not self.calib_ok(s, f)]
            if bad:
                print(f"[hands] {CALIB_FILE} ({self.source_name}) {s}: ignoring the open/fist values of "
                      + ", ".join(f"{f} ({self.calib[s][f][0]:.3f} -> {self.calib[s][f][1]:.3f})" for f in bad)
                      + f" - fist must be at least {MIN_SPAN} above open; those fingers use raw values. Recalibrate.")
        self.raw_hist = {s: collections.deque(maxlen=CALIB_SAMPLES) for s in SIDES}
        self.glove = {s: None for s in SIDES}            # raw curls from the source
        self.target = {s: {f: 0.0 for f in FINGERS} for s in SIDES}   # calibrated, angle-limited
        self.cmd = {s: {f: 0.0 for f in FINGERS} for s in SIDES}      # after the force limiter
        self.force = {s: 0.0 for s in SIDES}
        self.finger_force = {s: {f: 0.0 for f in FINGERS} for s in SIDES}
        self.peak = {s: 0.0 for s in SIDES}
        self.led = {s: "off" for s in SIDES}
        self.sigma = {s: self.sigma0() for s in SIDES}                # closing-torque cap per hand
        self.f6 = np.zeros(6)
        # ---- Coil positioning (off with --no-coil: fixed wrists as before) ----
        self.coil = not getattr(args, "no_coil", False)
        self.home = {s: np.array(HAND_POS[s], dtype=float) for s in SIDES}
        # each hand is pulled to its mocap target "<L|R>_ref" by a weld (see HAND_WELD_SOLREF)
        self.mocap = {s: int(m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{s[0].upper()}_ref")])
                      for s in SIDES}
        self.hand_body = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{s[0].upper()}_hand") for s in SIDES}
        self.palm_geom = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s[0].upper()}_palm"): s for s in SIDES}
        # ---- pick-up object ----
        self.pick_on = False
        self.pick_shape = "can"
        self.pick_body = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"pick_{n}") for n in PICK_SHAPES}
        self.pick_geom = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"pick_{n}") for n in PICK_SHAPES}
        self.pick_qadr = {n: int(m.jnt_qposadr[m.body_jntadr[b]]) for n, b in self.pick_body.items()}
        self.pick_dadr = {n: int(m.jnt_dofadr[m.body_jntadr[b]]) for n, b in self.pick_body.items()}
        self.pick_park = {n: np.array(m.body_pos[b]) for n, b in self.pick_body.items()}
        self.pick_rgba = {n: m.geom_rgba[g].copy() for n, g in self.pick_geom.items()}
        self.obj_force = {s: 0.0 for s in SIDES}       # finger + thumb force on the pick object
        self.obj_touch = {s: False for s in SIDES}     # any contact (fingers or palm) with it
        self.obj_parts = {s: 0 for s in SIDES}         # fingers / thumb pressing + palm touching
        self.eggcup = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")
                       .startswith("eggcup_")]
        self.egg_broken = False
        self.egg_peak = 0.0
        self.assist_on = ASSIST["on"]
        self.assist_eq = {(n, s): mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, f"assist_{s[0].upper()}_{n}")
                          for n in PICK_SHAPES for s in SIDES}
        self.attached = False
        self.release_t = None
        self.pick_state = "off"
        self.pick_lifted_since = None
        self.pick_was_up = False
        self.pick_trial = None
        self.last_pick = None
        self.hpos = {s: self.home[s].copy() for s in SIDES}          # where the wrist (palm centre) is
        self.sm_pos = {s: [self.home[s].copy(), self.home[s].copy()] for s in SIDES}   # smoothed target stages
        self.sm_quat = {s: [np.array([1.0, 0, 0, 0]), np.array([1.0, 0, 0, 0])] for s in SIDES}
        self.raw_pos = {s: self.home[s].copy() for s in SIDES}       # raw 1:1 position (ghost marker)
        self.anchor_in = {s: None for s in SIDES}
        self.raw_anchor_in = {s: None for s in SIDES}
        self.raw_anchor_out = {s: None for s in SIDES}
        self.reanchor = {s: True for s in SIDES}
        self.engaged = {s: False for s in SIDES}
        self.coil_seen = {s: False for s in SIDES}
        self.last_frame = {s: 0 for s in SIDES}
        self.filter_on = True
        self.cutoff = ASSIST_PARAMS["filter_min_cutoff_hz"]
        self.filt = {s: JitterFilter(min_cutoff=self.cutoff) for s in SIDES}
        self.scaler = {s: MotionScaler(1.0) for s in SIDES}
        self.jbuf = {s: [] for s in SIDES}           # current jitter block: (t, raw ee, filtered ee)
        self.jacc = {s: self.jacc_zero() for s in SIDES}             # completed blocks, summed
        self.jitter = {s: self.jitter_none() for s in SIDES}
        self.last_jitter = 0.0
        # ---- wrist rotation (Coil only) ----
        self.rot_on = self.coil
        self.hquat = {s: Q_ID.copy() for s in SIDES}                 # simulated hand orientation (w,x,y,z)
        self.rot_in = {s: None for s in SIDES}                       # last glove orientation (sign-continuous)
        self.rot_anchor_in = {s: None for s in SIDES}
        self.rot_anchor_out = {s: Q_ID.copy() for s in SIDES}
        self.rot_reanchor = {s: True for s in SIDES}
        self.rot_filter_on = True
        self.rot_cutoff = self.p["rot_filter_min_cutoff_hz"]
        self.rfilt = {s: RotFilter(self.rot_cutoff, self.p["rot_filter_beta"]) for s in SIDES}
        self.target_site = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{s[0].upper()}_target") for s in SIDES}
        self.task = None
        self.last_task = None
        self.task_log_dir = getattr(args, "task_dir", "hands_tasks")
        for s in SIDES:
            self.d.mocap_pos[self.mocap[s]] = self.home[s]
        for n in PICK_SHAPES:
            self.pick_place(n, active=False)                         # all objects parked until "pick ..."
        mujoco.mj_forward(m, self.d)

    # ---- inputs -----------------------------------------------------------------------
    def calib_ok(self, side, f):
        o, fi = self.calib[side][f]
        return fi - o >= MIN_SPAN

    def calibrated(self, side, raw):
        """Map raw curls through open/fist. A finger whose fist is not at least MIN_SPAN above its
        open (inverted or barely moving: a bad calibration) uses its raw value instead."""
        out = {}
        for f in FINGERS:
            o, fi = self.calib[side][f]
            out[f] = min(1.0, max(0.0, (raw[f] - o) / (fi - o))) if self.calib_ok(side, f) \
                else min(1.0, max(0.0, raw[f]))
        return out

    def raw_mean(self, side):
        h = self.raw_hist[side]
        if not h:
            return None
        return {f: sum(r[f] for r in h) / len(h) for f in FINGERS}

    def update_inputs(self, source):
        lim = self.p["angle_limit_pct"] / 100.0
        for side in SIDES:
            raw = source.get(side)
            self.glove[side] = raw
            if raw is None:
                continue                                  # hold the last pose
            self.raw_hist[side].append(raw)
            c = self.calibrated(side, raw)
            self.target[side] = {f: min(c[f], lim) for f in FINGERS}
        if self.coil and hasattr(source, "get_pose"):
            for side in SIDES:
                p = source.get_pose(side)
                if p is None:
                    self.engaged[side] = False
                    continue
                n, eng, ee, ts, q = p
                if n != self.last_frame[side]:               # each glove frame once (the filter needs it)
                    self.last_frame[side] = n
                    self.update_pose(side, eng, ee, ts, q)

    # ---- Coil positioning -------------------------------------------------------------
    def update_pose(self, side, engaged, ee, ts_us, quat=None):
        """Same clutch as egg_sim.py: while the bridge says engaged, the wrist follows the glove's
        Coil position (robot frame) from an anchor taken on each (re-)engage; disengaged = hold.
        Filter (1-euro) then motion scaling; switching either re-anchors, so nothing jumps.
        The orientation (quat = ee_quat, x,y,z,w) uses the same clutch: see update_rot."""
        self.engaged[side] = bool(engaged and ee is not None)
        if ee is not None:
            self.coil_seen[side] = True
        lo, hi = self.box(side)
        if engaged and ee is not None:
            ee = np.asarray(ee, dtype=float)
            t = ts_us / 1e6 if ts_us is not None else None
            if self.anchor_in[side] is None:                 # (re-)engage
                self.raw_anchor_in[side], self.raw_anchor_out[side] = ee.copy(), self.hpos[side].copy()
                self.anchor_in[side] = ee.copy()
                self.reanchor[side] = True
                self.rot_reanchor[side] = True
                self.jitter_reset([side])
            if self.reanchor[side]:
                self.filt[side].reset()
                f = self.filt[side](ee.tolist(), t) if self.filter_on else ee.tolist()
                self.scaler[side].reset(f, self.hpos[side].tolist())
                self.reanchor[side] = False
            else:
                f = self.filt[side](ee.tolist(), t) if self.filter_on else ee.tolist()
            self.hpos[side] = np.clip(np.asarray(self.scaler[side](f)), lo, hi)
            self.raw_pos[side] = np.clip(self.raw_anchor_out[side] + (ee - self.raw_anchor_in[side]), lo, hi)
            tt = t if t is not None else time.time()
            jb = self.jbuf[side]
            # A glove frame the bridge sent twice (same ts_us; the right glove did this ~once a second
            # on 15 Nov) would split the jitter segment (time must increase), and with a few of
            # them no segment was long enough -> "jitter --". Skip repeats.
            if not jb or tt > jb[-1][0]:
                jb.append((tt, ee.tolist(), list(f)))
            if quat is not None:
                self.update_rot(side, quat, t)
        else:
            self.anchor_in[side] = None                      # clutch out: hold, re-anchor on re-engage
        self.d.mocap_pos[self.mocap[side]] = self.hpos[side]

    def box(self, side):
        return BOX_PICK[side] if self.pick_on else BOX[side]

    # ---- pick-up object ---------------------------------------------------------------
    def pick_place(self, name, active):
        """Active: the object stands in the middle of the table, at rest. Parked: frozen out of sight,
        no collisions."""
        m, d = self.m, self.d
        qa, da, g = self.pick_qadr[name], self.pick_dadr[name], self.pick_geom[name]
        if active:
            pos = [PICK_XY[0], PICK_XY[1], TABLE_Z + PICK_SHAPES[name]["rest"] + 0.0005]
            m.geom_contype[g], m.geom_conaffinity[g] = 2, 1
            m.geom_rgba[g] = self.pick_rgba[name]
        else:
            pos = list(self.pick_park[name])
            m.geom_contype[g], m.geom_conaffinity[g] = 0, 0
            m.geom_rgba[g] = (0, 0, 0, 0)
        d.qpos[qa:qa + 7] = pos + [1.0, 0.0, 0.0, 0.0]
        d.qvel[da:da + 6] = 0.0

    def pick_set(self, shape=None):
        """Enter pick mode (bars off) with this shape, object back in the middle."""
        if shape:
            self.pick_shape = shape
        if not self.pick_on or shape:
            self.ball_on = False
            for s in SIDES:
                self.mount_pending[s] = False
                self.m.geom_conaffinity[self.ball[s]] = 0
                self.m.geom_rgba[self.ball[s]] = (0.9, 0.9, 0.9, 0.0)
        if self.attached:
            self.set_attached(False)
        for (n, s), eq in self.assist_eq.items():          # a shape change: no weld left on any object
            self.d.eq_active[eq] = 0
        self.pick_on = True
        self.set_eggcup(self.pick_shape == "egg")
        self.egg_broken, self.egg_peak = False, 0.0           # a whole egg again
        for n in PICK_SHAPES:
            self.pick_place(n, active=(n == self.pick_shape))
        self.pick_state, self.pick_lifted_since, self.pick_was_up = "on table", None, False
        self.pick_dropped = False
        mujoco.mj_forward(self.m, self.d)

    def pick_off(self):
        if self.attached:
            self.set_attached(False)
        self.pick_on, self.pick_state, self.pick_trial = False, "off", None
        self.set_eggcup(False)
        for n in PICK_SHAPES:
            self.pick_place(n, active=False)
        mujoco.mj_forward(self.m, self.d)

    def set_eggcup(self, on):
        for g in self.eggcup:
            self.m.geom_conaffinity[g] = 2 if on else 0
            self.m.geom_rgba[g][3] = 1.0 if on else 0.0

    def egg_check(self):
        """Breakable egg (egg_sim.py's threshold): a hand's finger + thumb force on it above
        EGG['break_n'] breaks it (red, state BROKEN, trial fails). Checked every physics step."""
        g = max(self.obj_force[s] for s in SIDES)
        self.egg_peak = max(self.egg_peak, g)
        geom = self.pick_geom["egg"]
        if self.egg_broken:
            return
        if g > EGG["break_n"]:
            self.egg_broken = True
            self.m.geom_rgba[geom] = EGG["broken_rgba"]
            hand = max(SIDES, key=lambda s: self.obj_force[s])
            print(f"[pick] EGG BROKEN: {hand} hand squeezed {g:.1f} N (> {EGG['break_n']:.0f} N)")
            if self.pick_trial:
                self.pick_finish("BROKEN")
        else:
            self.m.geom_rgba[geom] = EGG["yellow_rgba"] if g >= EGG["yellow_n"] else EGG["rgba"]

    def pick_lift(self):
        """Height (m) of the object's centre above its resting height on the table."""
        b = self.pick_body[self.pick_shape]
        return float(self.d.xpos[b][2]) - (TABLE_Z + PICK_SHAPES[self.pick_shape]["rest"])

    def pick_update(self):
        """on table / grasped / lifted (>= 5 cm; 2 s = SUCCESS) / dropped / fell off the table."""
        if not self.pick_on:
            return
        now, lift = self.d.time, self.pick_lift()
        touching = [s for s in SIDES if self.obj_touch[s]]
        grip = sum(self.obj_force[s] for s in SIDES)
        prev = self.pick_state
        gripped = grip > PICK["touch_n"]
        # "dropped" = the grip let go while the object was more than drop_lift_m up (sticky until it is
        # gripped again or reset); setting it down on the table and then letting go stays "on table".
        # (The palm may still touch the object after it lands, so this uses the grip, not "touching".)
        if self.pick_was_up and not gripped and lift > PICK["drop_lift_m"]:
            self.pick_dropped = True
        self.pick_was_up = gripped
        if gripped:
            self.pick_dropped = False
        if self.pick_shape == "egg" and self.egg_broken:
            st = "BROKEN"
        elif lift < -0.03:
            st = "fell off the table"
        elif lift >= PICK["lift_m"] and touching:
            st = "lifted"
        elif gripped:
            st = "grasped"
        elif getattr(self, "pick_dropped", False):
            st = "dropped"
        else:
            st = "on table"
        if st == "lifted":
            if self.pick_lifted_since is None:
                self.pick_lifted_since = now
        else:
            self.pick_lifted_since = None
        self.pick_state = st
        held = 0.0 if self.pick_lifted_since is None else now - self.pick_lifted_since
        tr = self.pick_trial
        if st != prev:
            print(f"[pick] {self.pick_shape}: {st}  (height {lift * 100:+.1f} cm, touching "
                  f"{'+'.join(touching) or 'nothing'}, grip {grip:.1f} N)")
        if tr:
            if tr["t_touch"] is None and touching:
                tr["t_touch"] = now
            for s in touching:
                tr["hands"].add(s)
            tr["peak_n"] = max(tr["peak_n"], *(self.force[s] for s in SIDES))
            if st in ("dropped", "fell off the table") and prev not in ("dropped", "fell off the table"):
                tr["drops"] += 1
            tr["max_lift"] = max(tr["max_lift"], lift)
            if held >= PICK["hold_s"]:
                self.pick_finish("SUCCESS")
            elif st == "fell off the table" and prev != st:
                self.pick_finish("fell off the table")

    def pick_start(self):
        if not self.pick_on:
            self.pick_set()
        self.pick_trial = {"start": self.d.time, "start_unix": time.time(), "shape": self.pick_shape,
                           "t_touch": None, "hands": set(), "peak_n": 0.0, "drops": 0, "max_lift": 0.0,
                           "params": {"force_cap_n": self.p["force_cap_n"], "angle_limit_pct": self.p["angle_limit_pct"],
                                      "scale": {s: self.scaler[s].scale for s in SIDES}, "rot_on": self.rot_on,
                                      "pick": PICK}}
        print(f"[pick] trial started ({self.pick_shape}): grasp it and hold it {PICK['lift_m'] * 100:.0f} cm "
              f"above the table for {PICK['hold_s']:.0f} s")

    def pick_finish(self, result):
        tr, self.pick_trial = self.pick_trial, None
        if not tr:
            return
        now = self.d.time
        out = {"type": "hands_pick", "result": result, "shape": tr["shape"], "start_unix": tr["start_unix"],
               "time_s": round(now - tr["start"], 2),
               "time_to_touch_s": round(tr["t_touch"] - tr["start"], 2) if tr["t_touch"] is not None else None,
               "time_to_lift_s": round(now - tr["start"] - PICK["hold_s"], 2) if result == "SUCCESS" else None,
               "hands": sorted(tr["hands"]), "assisted": tr.get("assisted", False),
               "broken": self.egg_broken if tr["shape"] == "egg" else None,
               "egg_break_n": EGG["break_n"] if tr["shape"] == "egg" else None,
               "peak_grip_n": round(tr["peak_n"], 2), "drops": tr["drops"],
               "max_height_cm": round(tr["max_lift"] * 100, 1), "params": tr["params"]}
        print(f"[pick] TRIAL {result}: {out['shape']}, {out['time_s']} s, lifted after {out['time_to_lift_s']} s, "
              f"hands {'+'.join(out['hands']) or '-'}, peak grip {out['peak_grip_n']} N, drops {out['drops']}")
        os.makedirs(self.task_log_dir, exist_ok=True)
        path = os.path.join(self.task_log_dir, time.strftime("%Y%m%d-%H%M%S") + "_pick.json")
        json.dump(out, open(path, "w"), indent=1)
        print(f"[pick] result written to {path}")
        self.last_pick = out

    def update_rot(self, side, quat, t):
        """Wrist orientation, same clutch as the position: on (re-)engage / rot on / rot reset /
        rotfilter change, anchor the (filtered) glove orientation and the simulated hand's current
        orientation; then hand = (glove turn since the anchor, in the robot frame) x anchored hand.
        Using only the turn since the anchor means no jump and no mirroring: both hands turn the
        way the gloves turn in the room."""
        if not self.rot_on:
            return
        q = q_from_xyzw(quat)
        prev = self.rot_in[side]
        if prev is not None and np.dot(prev, q) < 0:
            q = -q                                           # same orientation; keep the sign continuous
        self.rot_in[side] = q
        if self.rot_reanchor[side]:
            self.rfilt[side].reset()
            qf = self.rfilt[side](q, t) if self.rot_filter_on else q
            self.rot_anchor_in[side], self.rot_anchor_out[side] = qf.copy(), self.hquat[side].copy()
            self.rot_reanchor[side] = False
        else:
            qf = self.rfilt[side](q, t) if self.rot_filter_on else q
        h = q_mul(q_mul(qf, q_conj(self.rot_anchor_in[side])), self.rot_anchor_out[side])
        self.hquat[side] = h / np.linalg.norm(h)
        self.d.mocap_quat[self.mocap[side]] = self.hquat[side]

    def set_rot(self, mode):
        """mode: 'on' (re-anchor, no jump), 'off' (palms up, rotation ignored), 'reset' (palms up,
        re-anchored at the gloves' current orientation)."""
        if mode in ("off", "reset"):
            for s in SIDES:
                self.hquat[s] = Q_ID.copy()
                self.d.mocap_quat[self.mocap[s]] = self.hquat[s]
        if mode in ("on", "reset"):
            self.rot_on = True
        elif mode == "off":
            self.rot_on = False
        for s in SIDES:
            self.rot_reanchor[s] = True

    def set_rot_filter(self, on=None, cutoff=None):
        if on is not None:
            self.rot_filter_on = on
        if cutoff is not None:
            self.rot_cutoff = cutoff
        self.rfilt = {s: RotFilter(self.rot_cutoff, self.p["rot_filter_beta"]) for s in SIDES}
        for s in SIDES:
            self.rot_reanchor[s] = True                      # restart at the current orientation, no jump

    def set_scale(self, sides, s):
        for side in sides:
            sc = self.scaler[side]
            if s == sc.scale:
                continue
            sc.scale = s
            if sc.last_in is not None:                       # re-anchor at the last input and the CLAMPED output
                sc.reset(sc.last_in, self.hpos[side].tolist())
        self.jitter_reset(sides)

    def set_filter(self, on=None, cutoff=None):
        if on is not None:
            self.filter_on = on
        if cutoff is not None:
            self.cutoff = cutoff
            self.filt = {s: JitterFilter(min_cutoff=cutoff) for s in SIDES}
        for s in SIDES:
            self.reanchor[s] = True                          # restart filter + scaler at the current pose
        self.jitter_reset(SIDES)

    # ---- jitter read-out (cumulative) -------------------------------------------------
    @staticmethod
    def jacc_zero():
        return {k: [0.0, 0] for k in ("raw", "filt", "raw_hf", "filt_hf")}   # sum of squares (mm^2), n

    @staticmethod
    def jitter_none():
        return {"raw": None, "filt": None, "raw_hf": None, "filt_hf": None, "still_s": 0.0}

    def jitter_reset(self, sides):
        for s in sides:
            self.jbuf[s] = []
            self.jacc[s] = self.jacc_zero()
            self.jitter[s] = self.jitter_none()

    @staticmethod
    def jitter_block(buf):
        """Score one block with eval_offline.jitter_mm: raw and filtered on the same still samples,
        whole band and above JITTER_HF_HZ. Returns {key: (sum of squares, n)}."""
        ts = [b[0] for b in buf]
        raw = [b[1] for b in buf]
        fil = [b[2] for b in buf]
        out = {}
        for key, sig, ref, band in (("raw", raw, fil, None), ("filt", fil, raw, None),
                                    ("raw_hf", raw, fil, JITTER_HF_HZ), ("filt_hf", fil, raw, JITTER_HF_HZ)):
            v, n, _ = jitter_mm(ts, sig, still_from=ref, edge_s=0.3, band_hz=band)
            out[key] = ((v * v * n) if v is not None else 0.0, n if v is not None else 0)
        return out

    def update_jitter(self):
        """Cumulative jitter since the last reset (filter / scale / engage change, "jitter reset"):
        completed JITTER_BLOCK_S blocks are summed; the current block counts once it is 1.1 s long."""
        for s in SIDES:
            buf = self.jbuf[s]
            if buf and buf[-1][0] - buf[0][0] >= JITTER_BLOCK_S:
                for k, (sq, n) in self.jitter_block(buf).items():
                    self.jacc[s][k][0] += sq
                    self.jacc[s][k][1] += n
                self.jbuf[s] = buf = []
            tot = {k: list(v) for k, v in self.jacc[s].items()}
            if len(buf) >= 10 and buf[-1][0] - buf[0][0] >= 1.1:
                for k, (sq, n) in self.jitter_block(buf).items():
                    tot[k][0] += sq
                    tot[k][1] += n
            j = {k: (math.sqrt(sq / n) if n else None) for k, (sq, n) in tot.items()}
            j["still_s"] = tot["raw"][1] / ASSIST_PARAMS["default_rate_hz"]
            self.jitter[s] = j

    # ---- reach and hold task (both hands at once) -------------------------------------
    def target_pos(self, side, k):
        s = 1 if side == "left" else -1                      # +y offset = outward for each hand
        dx, dy, dz = TARGET_OFFSETS[k % len(TARGET_OFFSETS)]
        lo, hi = self.box(side)
        return np.clip(self.home[side] + np.array([dx, s * dy, dz]), lo, hi)

    def task_start(self):
        if not self.coil:
            print("[hands] the task needs Coil positioning (run without --no-coil)")
            return
        now = self.d.time
        self.task = {"start_unix": time.time(), "params": self.task_params(), "hands": {}}
        for s in SIDES:
            self.task["hands"][s] = {"k": 0, "t_shown": now, "t_reached": None, "hold_start": None,
                                     "hold_pts": [], "breaks": 0, "over": 0, "over_flag": False,
                                     "peak": 0.0, "results": [], "done": False}
            self.m.site_pos[self.target_site[s]] = self.target_pos(s, 0)
        print(f"[hands] task started: {TASK['n_targets']} targets per hand; put the palm in the sphere, grip the "
              f"bar at {TASK['grip_min_frac'] * 100:.0f}-100 % of the force cap, hold {TASK['hold_s']:.0f} s")

    def task_params(self):
        return {"scale": {s: self.scaler[s].scale for s in SIDES}, "filter_on": self.filter_on,
                "cutoff_hz": self.cutoff, "rot_on": self.rot_on, "rot_filter_on": self.rot_filter_on,
                "rot_cutoff_hz": self.rot_cutoff, "force_cap_n": self.p["force_cap_n"],
                "angle_limit_pct": self.p["angle_limit_pct"], "task": TASK}

    def task_update(self):
        if not self.task:
            return
        now, cap = self.d.time, self.p["force_cap_n"]
        for s in SIDES:
            st = self.task["hands"][s]
            site = self.target_site[s]
            if st["done"]:
                self.m.site_rgba[site] = (0, 0, 0, 0)
                continue
            tgt = self.target_pos(s, st["k"])
            F = self.force[s]
            st["peak"] = max(st["peak"], F)
            grip_ok = (TASK["grip_min_frac"] * cap <= F <= TASK["over_frac"] * cap) if cap > 0 \
                else F >= TASK["grip_min_n_no_cap"]
            over = cap > 0 and F > TASK["over_frac"] * cap
            if over and not st["over_flag"]:
                st["over"] += 1
            st["over_flag"] = over
            inside = float(np.linalg.norm(self.hpos[s] - tgt)) < TASK["radius"]
            if inside and st["t_reached"] is None:
                st["t_reached"] = now
            if inside and grip_ok:
                if st["hold_start"] is None:
                    st["hold_start"], st["hold_pts"] = now, []
                st["hold_pts"].append(self.hpos[s].copy())
                held = now - st["hold_start"]
                self.m.site_rgba[site] = (0.1, 0.9, 0.1, 0.45)
                if held >= TASK["hold_s"]:
                    pts = np.array(st["hold_pts"])
                    steady = float(np.sqrt(((pts - pts.mean(0)) ** 2).sum(1).mean()) * 1000)
                    self.task_record(s, st, "held", steady)
            else:
                if st["hold_start"] is not None:
                    st["breaks"] += 1
                st["hold_start"] = None
                self.m.site_rgba[site] = (1.0, 0.8, 0.0, 0.35) if not inside else (1.0, 0.5, 0.0, 0.45)
                if now - st["t_shown"] > TASK["timeout_s"]:
                    self.task_record(s, st, "timeout", None)
        if all(self.task["hands"][s]["done"] for s in SIDES):
            self.task_finish()

    def task_record(self, side, st, result, steady):
        now = self.d.time
        r = {"target": st["k"] + 1, "pos": [round(float(v), 4) for v in self.target_pos(side, st["k"])],
             "result": result,
             "reach_s": round(st["t_reached"] - st["t_shown"], 2) if st["t_reached"] is not None else None,
             "total_s": round(now - st["t_shown"], 2), "hold_steadiness_mm": round(steady, 2) if steady else None,
             "breaks": st["breaks"], "cap_overshoots": st["over"], "peak_grip_n": round(st["peak"], 2)}
        st["results"].append(r)
        print(f"[hands] {side} target {r['target']}: {result}  reach {r['reach_s']} s  steadiness "
              f"{r['hold_steadiness_mm']} mm  breaks {r['breaks']}  overshoots {r['cap_overshoots']}")
        st.update(k=st["k"] + 1, t_shown=now, t_reached=None, hold_start=None, hold_pts=[], breaks=0,
                  over=0, over_flag=False, peak=0.0)
        if st["k"] >= TASK["n_targets"]:
            st["done"] = True

    def task_summary(self, side, t=None):
        t = t or self.task
        R = t["hands"][side]["results"] if t else []
        held = [r for r in R if r["result"] == "held"]
        mean = lambda xs: sum(xs) / len(xs) if xs else None
        return {"targets": len(R), "held": len(held),
                "mean_reach_s": mean([r["reach_s"] for r in held if r["reach_s"] is not None]),
                "mean_steadiness_mm": mean([r["hold_steadiness_mm"] for r in held]),
                "breaks": sum(r["breaks"] for r in R), "cap_overshoots": sum(r["cap_overshoots"] for r in R)}

    def task_finish(self, stopped=False):
        t = self.task
        self.task = None
        for s in SIDES:
            self.m.site_rgba[self.target_site[s]] = (0, 0, 0, 0)
        out = {"type": "hands_reach_hold", "stopped_early": stopped, "params": t["params"],
               "start_unix": t["start_unix"], "end_unix": time.time()}
        for s in SIDES:
            out[s] = {"summary": self.task_summary(s, t), "targets": t["hands"][s]["results"]}
            sm = out[s]["summary"]
            ms = f"{sm['mean_steadiness_mm']:.2f} mm" if sm["mean_steadiness_mm"] is not None else "-"
            mr = f"{sm['mean_reach_s']:.1f} s" if sm["mean_reach_s"] is not None else "-"
            print(f"[hands] TASK {s}: held {sm['held']} of {sm['targets']}  mean reach {mr}  mean steadiness {ms}  "
                  f"breaks {sm['breaks']}  cap overshoots {sm['cap_overshoots']}")
        os.makedirs(self.task_log_dir, exist_ok=True)
        path = os.path.join(self.task_log_dir, time.strftime("%Y%m%d-%H%M%S") + "_reach_hold.json")
        json.dump(out, open(path, "w"), indent=1)
        print(f"[hands] task result written to {path}")
        self.last_task = out

    def go_home(self):
        for s in SIDES:
            self.hpos[s] = self.home[s].copy()
            self.anchor_in[s] = None                         # next engaged frame re-anchors here
            self.d.mocap_pos[self.mocap[s]] = self.hpos[s]
            self.hquat[s] = Q_ID.copy()                      # palms up; rotation re-anchors with the position
            self.d.mocap_quat[self.mocap[s]] = self.hquat[s]
            self.rot_reanchor[s] = True

    # ---- physics ----------------------------------------------------------------------
    def measure(self):
        m, d = self.m, self.d
        ff = {s: {f: 0.0 for f in FINGERS} for s in SIDES}
        pg = self.pick_geom[self.pick_shape] if self.pick_on else -1
        of = {s: 0.0 for s in SIDES}
        touch = {s: False for s in SIDES}
        palm = {s: False for s in SIDES}
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if pg in (g1, g2):                            # the free pick-up object
                other = g2 if g1 == pg else g1
                hit = self.finger_geoms.get(other)
                if hit:
                    mujoco.mj_contactForce(m, d, i, self.f6)
                    ff[hit[0]][hit[1]] += self.f6[0]      # finger / thumb force: what the force cap limits
                    of[hit[0]] += self.f6[0]
                    touch[hit[0]] = True
                elif other in self.palm_geom:
                    touch[self.palm_geom[other]] = True   # palm contact counts as touching, not as grip force
                    palm[self.palm_geom[other]] = True
                continue
            for ball_side, bg in self.ball.items():
                if bg in (g1, g2):
                    other = g2 if g1 == bg else g1
                    hit = self.finger_geoms.get(other)
                    if hit and hit[0] == ball_side:
                        mujoco.mj_contactForce(m, d, i, self.f6)
                        ff[ball_side][hit[1]] += self.f6[0]
        for s in SIDES:
            self.finger_force[s] = ff[s]
            self.force[s] = sum(ff[s].values())
            self.peak[s] = max(self.peak[s], self.force[s])
            self.obj_force[s], self.obj_touch[s] = of[s], touch[s]
            self.obj_parts[s] = (sum(1 for f in FINGERS if pg >= 0 and ff[s][f] > ASSIST["part_n"])
                                 + (1 if palm[s] else 0))
        if self.pick_on:
            if self.pick_shape == "egg":
                self.egg_check()
            self.assist_update()

    def set_attached(self, on):
        """Two-hand assist: link the object to both hands at their CURRENT relative pose (no jump)."""
        m, d = self.m, self.d
        ob = self.pick_body[self.pick_shape]
        for s in SIDES:
            eq = self.assist_eq[(self.pick_shape, s)]
            if on:
                hb = self.hand_body[s]
                R1 = d.xmat[ob].reshape(3, 3)
                # MuJoCo weld relpose = body2 (hand) pose in body1 (object) frame (checked: the other way
                # round the object jumped 197 mm on attach)
                m.eq_data[eq][0:3] = 0.0                                   # anchor at the hand's origin
                m.eq_data[eq][3:6] = R1.T @ (d.xpos[hb] - d.xpos[ob])
                m.eq_data[eq][6:10] = q_mul(q_conj(d.xquat[ob]), d.xquat[hb])
                m.eq_data[eq][10] = 1.0
            d.eq_active[eq] = 1 if on else 0
        if on != self.attached:
            print(f"[pick] two-hand assist: {'ATTACHED (both hands pressing)' if on else 'released'}")
            if on and self.pick_trial:
                self.pick_trial["assisted"] = True
        self.attached = on

    def assist_update(self):
        dt = self.m.opt.timestep
        if not self.attached:
            if self.assist_on and all(self.obj_force[s] >= ASSIST["attach_n"] and self.obj_parts[s] >= ASSIST["min_parts"]
                                      for s in SIDES):
                self.attach_t = getattr(self, "attach_t", 0.0) + dt
                if self.attach_t >= ASSIST["attach_s"]:
                    self.set_attached(True)
                    self.release_t, self.attach_t = None, 0.0
            else:
                self.attach_t = 0.0
        else:
            if min(self.obj_force[s] for s in SIDES) < ASSIST["release_n"]:
                self.release_t = (self.release_t or 0.0) + dt
                if self.release_t >= ASSIST["release_s"]:
                    self.set_attached(False)
            else:
                self.release_t = None

    def limiter(self, dt):
        """Force cap B at the actuators (like the egg's GripLimiter): each hand's CLOSING joint
        torque is capped at sigma; opening always has the full torque. While the hand touches its
        ball, sigma adapts so the measured grip force converges on the cap; without contact it
        relaxes to sigma0 = cap * torque_per_n, so the fingers cannot wind up before an impact.
        The position targets themselves are the glove's (angle-limited) curls."""
        cap, tmax = self.p["force_cap_n"], self.p["joint_torque_max"]
        for s in SIDES:
            if cap > 0:
                s0 = self.sigma0()
                if self.force[s] > self.p["contact_n"]:
                    e = max(-1.0, min(1.0, (cap - self.force[s]) / cap))
                    self.sigma[s] *= math.exp(self.p["sigma_gain"] * e * dt)
                else:
                    self.sigma[s] += (s0 - self.sigma[s]) * min(1.0, 2.0 * dt)
                self.sigma[s] = min(tmax, max(self.p["sigma_min"], self.sigma[s]))
            else:
                self.sigma[s] = tmax
            for f in FINGERS:
                c = self.target[s][f]
                self.cmd[s][f] = c
                for adr, aid, lo, hi in self.joint[(s, f)]:
                    self.d.ctrl[aid] = max(0.0, lo) + c * (hi - max(0.0, lo))
                    self.m.actuator_forcerange[aid] = (-tmax, self.sigma[s])

    def sigma0(self):
        cap, tmax = self.p["force_cap_n"], self.p["joint_torque_max"]
        return min(tmax, cap * self.p["torque_per_n"]) if cap > 0 else tmax

    def check_mount(self):
        for s in SIDES:
            if self.mount_pending[s] and max(self.flexion(s, f)[1] for f in FINGERS) < 0.25:
                self.mount_pending[s] = False
                self.m.geom_conaffinity[self.ball[s]] = 1
                self.peak[s] = 0.0
                print(f"[hands] {s} object mounted")

    def step(self):
        self.check_mount()
        self.limiter(self.m.opt.timestep)
        for n in PICK_SHAPES:                             # parked objects stay frozen
            if not (self.pick_on and n == self.pick_shape):
                qa, da = self.pick_qadr[n], self.pick_dadr[n]
                self.d.qpos[qa:qa + 3] = self.pick_park[n]
                self.d.qvel[da:da + 6] = 0.0
        a = min(1.0, self.m.opt.timestep / TARGET_SMOOTH_S)
        for s in SIDES:                                   # smooth the commanded pose into the mocap target
            p1, p2 = self.sm_pos[s]
            p1 += a * (self.hpos[s] - p1)
            p2 += a * (p1 - p2)
            self.d.mocap_pos[self.mocap[s]] = p2
            q = self.hquat[s]
            for k in range(2):
                qs = self.sm_quat[s][k]
                if np.dot(qs, q) < 0:
                    qs = -qs
                qs = qs + a * (q - qs)
                self.sm_quat[s][k] = q = qs / np.linalg.norm(qs)
            self.d.mocap_quat[self.mocap[s]] = q
        mujoco.mj_step(self.m, self.d)
        self.measure()

    # ---- read-outs --------------------------------------------------------------------
    def flexion(self, side, f):
        """(degrees summed over the 3 joints, fraction of the finger's full range)."""
        deg, frac = 0.0, []
        for adr, aid, lo, hi in self.joint[(side, f)]:
            q = float(self.d.qpos[adr])
            deg += math.degrees(q)
            frac.append(max(0.0, q) / hi)
        return deg, sum(frac) / len(frac)

    def pcts(self, side):
        lim = self.p["angle_limit_pct"] / 100.0
        a = max(self.flexion(side, f)[1] for f in FINGERS) / lim * 100.0 if lim > 0 else 0.0
        cap = self.p["force_cap_n"]
        fpct = self.force[side] / cap * 100.0 if cap > 0 else 0.0
        return a, fpct

    def update_led(self, leds):
        for s in SIDES:
            worst = max(self.pcts(s))
            name = "red" if worst >= self.p["red_pct"] else ("yellow" if worst >= self.p["warn_pct"] else "green")
            if self.glove[s] is None:
                name = "off"                              # no data for this glove
            self.led[s] = name
            if self.m.geom_conaffinity[self.ball[s]]:           # object mounted: show the cue on it
                self.m.geom_rgba[self.ball[s]] = SCREEN_RGBA[name]
            if leds:
                leds.set(s, name)

    def pos_state(self, side):
        if not self.coil:
            return "fixed wrist (--no-coil)"
        if self.engaged[side]:
            return "Coil, engaged"
        return "Coil, clutch out (hold)" if self.coil_seen[side] else "no Coil data (hold)"

    def overlay(self, side):
        a, fp = self.pcts(side)
        angles = "  ".join(f"{SHORT[f]} {self.flexion(side, f)[0]:4.0f}" for f in FINGERS)
        cap = self.p["force_cap_n"]
        src = "no data" if self.glove[side] is None else "live"
        grip = (f"grip  {self.force[side]:5.1f} N  ({fp:5.1f}% of {cap:.0f} N)" if cap > 0
                else f"grip  {self.force[side]:5.1f} N  (no force cap)")
        lines = [f"{side.upper()} hand ({src})", f"{angles} deg",
                 f"angle {a:5.1f}% of {self.p['angle_limit_pct']:.0f}% limit", grip,
                 f"LED {self.led[side].upper()}"]
        if self.coil:
            j = self.jitter[side]
            fmt = lambda v: "--" if v is None else f"{v:.2f}"
            sc = self.scaler[side].scale
            r, p_, y = q_rpy(self.hquat[side])
            rf = f"{self.rot_cutoff:.1f} Hz" if self.rot_filter_on else "off"
            lines += [f"pos   {self.pos_state(side)}",
                      (f"wrist roll {r:+4.0f}  pitch {p_:+4.0f}  yaw {y:+4.0f} deg  (filter {rf})" if self.rot_on
                       else "wrist rotation off (palm up)"),
                      f"scale x{sc:.2f}   filter {f'{self.cutoff:.1f} Hz' if self.filter_on else 'off'}",
                      f"jitter raw {fmt(j['raw'])}  filt {fmt(j['filt'])}  sim "
                      f"{fmt(None if j['filt'] is None else j['filt'] * sc)} mm  ({j['still_s']:.0f} s still)",
                      f"  >{JITTER_HF_HZ:.0f} Hz raw {fmt(j['raw_hf'])}  filt {fmt(j['filt_hf'])} mm"]
        if self.pick_on:
            lines.append(f"object {'TOUCHING' if self.obj_touch[side] else 'not touching'}  "
                         f"grip on it {self.obj_force[side]:4.1f} N")
        if self.task:
            st = self.task["hands"][side]
            if st["done"]:
                lines.append("task  done")
            else:
                held = 0.0 if st["hold_start"] is None else self.d.time - st["hold_start"]
                dist = float(np.linalg.norm(self.hpos[side] - self.target_pos(side, st["k"]))) * 1000
                lines.append(f"task  {st['k'] + 1}/{TASK['n_targets']}  {dist:4.0f} mm  hold {held:3.1f}/{TASK['hold_s']:.0f} s")
        return lines[0], "\n".join(lines[1:]), ""

    def pick_text(self):
        """One-line object summary (overlay bottom panel, terminal, status)."""
        if not self.pick_on:
            return ""
        touching = [s for s in SIDES if self.obj_touch[s]]
        held = 0.0 if self.pick_lifted_since is None else self.d.time - self.pick_lifted_since
        st = self.pick_state.upper() + (f" {held:.1f}/{PICK['hold_s']:.0f} s" if self.pick_state == "lifted" else "")
        tr = (f"   trial {self.d.time - self.pick_trial['start']:.1f} s" if self.pick_trial else
              (f"   last trial: {self.last_pick['result']} {self.last_pick['time_s']} s" if self.last_pick else ""))
        return (f"{self.pick_shape}: {st}   height {self.pick_lift() * 100:+.1f} cm   touching "
                f"{'+'.join(touching) or 'nothing'}   grip {sum(self.obj_force[s] for s in SIDES):.1f} N" + tr)

    def status(self):
        lines = [f"[hands] limits: angle {self.p['angle_limit_pct']:.0f}%  force cap {self.p['force_cap_n']:.1f} N  "
                 f"warn {self.p['warn_pct']:.0f}%  object {'on' if self.ball_on else 'off'}"]
        if self.pick_on:
            lines.append(f"  pick  {self.pick_text()}")
        for s in SIDES:
            a, fp = self.pcts(s)
            raw = self.glove[s]
            lines.append(f"  {s:5s} LED {self.led[s]:6s} angle {a:5.1f}%  force {self.force[s]:5.2f} N ({fp:5.1f}%)  "
                         f"peak {self.peak[s]:5.2f} N")
            lines.append("        deg   " + "  ".join(f"{SHORT[f]} {self.flexion(s, f)[0]:5.1f}" for f in FINGERS))
            lines.append("        glove " + ("no data" if raw is None else
                                             "  ".join(f"{SHORT[f]} {raw[f]:.2f}" for f in FINGERS)))
            lines.append("        calib " + "  ".join(
                f"{SHORT[f]} {self.calib[s][f][0]:.2f}..{self.calib[s][f][1]:.2f}"
                + ("" if self.calib_ok(s, f) else "(raw!)") for f in FINGERS)
                + f"   [(raw!) = bad open/fist, finger uses raw values; source {self.source_name}]")
            if self.coil:
                j = self.jitter[s]
                lines.append(f"        pos   {self.pos_state(s)}  wrist {' '.join(f'{v:+.3f}' for v in self.hpos[s])}  "
                             f"raw 1:1 {' '.join(f'{v:+.3f}' for v in self.raw_pos[s])}  scale x{self.scaler[s].scale:.2f}  "
                             f"filter {f'{self.cutoff:.1f} Hz' if self.filter_on else 'off'}")
                rnd = lambda v: v if v is None else round(v, 2)
                lines.append(f"        jitter (cumulative, {j['still_s']:.0f} s still) raw {rnd(j['raw'])} "
                             f"filt {rnd(j['filt'])} mm;  >{JITTER_HF_HZ:.0f} Hz raw {rnd(j['raw_hf'])} "
                             f"filt {rnd(j['filt_hf'])} mm")
                lines.append(f"        rot   " + ("off (palm up)" if not self.rot_on else
                             "roll {:+.1f} pitch {:+.1f} yaw {:+.1f} deg".format(*q_rpy(self.hquat[s]))
                             + f"  filter {f'{self.rot_cutoff:.1f} Hz' if self.rot_filter_on else 'off'}"))
        return "\n".join(lines)

    def state(self):
        out = {"limits": {k: self.p[k] for k in ("angle_limit_pct", "force_cap_n", "warn_pct")},
               "ball_on": self.ball_on}
        for s in SIDES:
            a, fp = self.pcts(s)
            out[s] = {"glove": self.glove[s], "target": self.target[s], "cmd": self.cmd[s],
                      "deg": {f: round(self.flexion(s, f)[0], 2) for f in FINGERS},
                      "angle_pct": round(a, 2), "force_n": round(self.force[s], 3), "force_pct": round(fp, 2),
                      "peak_n": round(self.peak[s], 3), "led": self.led[s],
                      "wrist": [round(float(v), 5) for v in self.hpos[s]],
                      "raw_pos": [round(float(v), 5) for v in self.raw_pos[s]],
                      "engaged": self.engaged[s], "scale": self.scaler[s].scale,
                      "jitter": self.jitter[s],
                      "quat": [round(float(v), 6) for v in self.hquat[s]],          # w, x, y, z
                      "rpy": [round(v, 2) for v in q_rpy(self.hquat[s])]}
            if self.task:
                st = self.task["hands"][s]
                out[s]["task"] = {"k": st["k"], "done": st["done"], "target": [round(float(v), 4) for v in
                                  self.target_pos(s, min(st["k"], TASK["n_targets"] - 1))],
                                  "holding": st["hold_start"] is not None}
        out["filter"] = {"on": self.filter_on, "cutoff_hz": self.cutoff}
        out["rot"] = {"on": self.rot_on, "filter_on": self.rot_filter_on, "cutoff_hz": self.rot_cutoff}
        out["pick"] = {"on": self.pick_on, "shape": self.pick_shape, "state": self.pick_state,
                       "lift_m": round(self.pick_lift(), 4) if self.pick_on else None,
                       "pos": [round(float(v), 4) for v in self.d.xpos[self.pick_body[self.pick_shape]]],
                       "touch": {s: self.obj_touch[s] for s in SIDES},
                       "force_n": {s: round(self.obj_force[s], 3) for s in SIDES},
                       "trial_active": self.pick_trial is not None, "last": self.last_pick,
                       "broken": self.egg_broken, "egg_peak_n": round(self.egg_peak, 2)}
        out["hand_actual"] = {s: [round(float(v), 4) for v in self.d.xpos[self.hand_body[s]]] for s in SIDES}
        out["task_active"] = self.task is not None
        out["last_task"] = getattr(self, "last_task", None)
        return out

    # ---- commands ---------------------------------------------------------------------
    def sides_of(self, tok):
        t = (tok or "both").lower()
        return {"l": ["left"], "left": ["left"], "r": ["right"], "right": ["right"]}.get(t, list(SIDES))

    def command(self, line) -> bool:
        parts = line.split()
        if not parts:
            return True
        c = parts[0].lower()
        try:
            if c == "q":
                return False
            if c in ("open", "fist"):
                k = 0 if c == "open" else 1
                for s in self.sides_of(parts[1] if len(parts) > 1 else None):
                    raw = self.raw_mean(s) if self.glove[s] is not None else None
                    if raw is None:
                        print(f"[hands] no {s} glove data - cannot calibrate")
                        continue
                    for f in FINGERS:
                        self.calib[s][f][k] = round(raw[f], 4)
                    print(f"[hands] {s} {c.upper()} (mean of last {len(self.raw_hist[s])} frames) = "
                          + "  ".join(f"{SHORT[f]} {raw[f]:.3f}" for f in FINGERS))
                    if c == "fist":
                        for f in FINGERS:
                            o, fi = self.calib[s][f]
                            verdict = "ok" if self.calib_ok(s, f) else (
                                f"REJECTED: fist must be at least {MIN_SPAN} above open - this finger uses "
                                f"its raw value; check it with solver_check.py / udp_monitor.py")
                            print(f"         {f:6s} open {o:.3f} fist {fi:.3f} span {fi - o:+.3f}  {verdict}")
                self.save_calib()
            elif c == "calib" and len(parts) > 1 and parts[1].lower() == "reset":
                for s in self.sides_of(parts[2] if len(parts) > 2 else None):
                    self.calib[s] = {f: [0.0, 1.0] for f in FINGERS}
                    print(f"[hands] {s} calibration reset")
                self.save_calib()
            elif c == "limit":
                if len(parts) == 2:                       # "limit 80" = angle limit
                    parts = ["limit", "angle", parts[1]]
                kind, val = parts[1].lower(), float(parts[2])
                self.peak = {s: 0.0 for s in SIDES}
                if kind.startswith("a"):
                    self.p["angle_limit_pct"] = min(100.0, max(5.0, val))
                    print(f"[hands] angle limit {self.p['angle_limit_pct']:.0f}% of joint range")
                elif kind.startswith("f"):
                    self.p["force_cap_n"] = max(0.0, val)
                    self.sigma = {s: self.sigma0() for s in SIDES}   # start from the estimate, not full torque
                    print(f"[hands] force cap {self.p['force_cap_n']:.1f} N" + (" (off)" if val <= 0 else ""))
                else:
                    raise ValueError
            elif c == "warn":
                self.p["warn_pct"] = min(self.p["red_pct"] - 1.0, max(1.0, float(parts[1])))
                print(f"[hands] yellow from {self.p['warn_pct']:.0f}% of a limit")
            elif c in ("obj", "bar", "ball"):
                on = parts[1].lower() == "on"
                if on and self.pick_on:
                    self.pick_off()
                    print("[pick] pick mode off: the bars and the object would collide")
                self.ball_on = on
                for s in SIDES:
                    if on:
                        # mounting into closed fingers would trap them inside the object (huge
                        # forces), so each hand's object appears once that hand is open
                        self.mount_pending[s] = True
                    else:
                        self.mount_pending[s] = False
                        self.m.geom_conaffinity[self.ball[s]] = 0
                        self.m.geom_rgba[self.ball[s]] = (0.9, 0.9, 0.9, 0.12)
                print("[hands] objects: each mounts as soon as that hand is open" if on
                      else "[hands] objects removed (fingers can close fully)")
            elif c == "peak":
                self.peak = {s: 0.0 for s in SIDES}
                print("[hands] peak force reset")
            elif c == "scale":                                # scale [L|R|both] X
                if len(parts) == 2:
                    sides, val = list(SIDES), float(parts[1])
                else:
                    sides, val = self.sides_of(parts[1]), float(parts[2])
                val = min(2.0, max(0.05, val))
                self.set_scale(sides, val)
                print(f"[hands] motion scale x{val:.2f} for {', '.join(sides)} (re-anchored, no jump)")
            elif c == "filter":                               # filter on | off | <cutoff Hz>
                arg = parts[1].lower()
                if arg in ("on", "off"):
                    self.set_filter(on=(arg == "on"))
                else:
                    self.set_filter(on=True, cutoff=min(20.0, max(0.2, float(arg))))
                print(f"[hands] jitter filter {f'on, {self.cutoff:.1f} Hz' if self.filter_on else 'off'} "
                      f"(1-euro, beta {ASSIST_PARAMS['filter_beta']})")
            elif c == "rot":                                  # rot on | off | reset
                arg = parts[1].lower()
                if arg not in ("on", "off", "reset"):
                    raise ValueError
                if not self.coil:
                    print("[hands] wrist rotation needs the Coil (run without --no-coil)")
                else:
                    self.set_rot(arg)
                    print({"on": "[hands] wrist rotation on (re-anchored, no jump)",
                           "off": "[hands] wrist rotation off: palms up",
                           "reset": "[hands] palms up, re-anchored at the gloves' current orientation"}[arg])
            elif c == "rotfilter":                            # rotfilter on | off | <cutoff Hz>
                arg = parts[1].lower()
                if arg in ("on", "off"):
                    self.set_rot_filter(on=(arg == "on"))
                else:
                    self.set_rot_filter(on=True, cutoff=min(20.0, max(0.2, float(arg))))
                print(f"[hands] rotation filter {f'on, {self.rot_cutoff:.1f} Hz' if self.rot_filter_on else 'off'} "
                      f"(1-euro on the quaternion, beta {self.p['rot_filter_beta']}); re-anchored, no jump")
            elif c == "pick":                                 # pick | pick can|cube|ball | reset | stop | off
                arg = parts[1].lower() if len(parts) > 1 else "start"
                if arg in PICK_SHAPES:
                    self.pick_set(arg)
                    print(f"[pick] {arg} on the table (bars off); 'pick' starts a trial")
                elif arg == "reset":
                    if self.pick_trial:
                        self.pick_finish("reset")
                    self.pick_set()
                    print(f"[pick] {self.pick_shape} back in the middle of the table")
                elif arg == "start":
                    self.pick_start()
                elif arg == "stop":
                    self.pick_finish("stopped")
                elif arg == "off":
                    self.pick_off()
                    print("[pick] pick mode off (obj on brings the bars back)")
                elif arg == "assist":
                    self.assist_on = parts[2].lower() == "on"
                    if not self.assist_on and self.attached:
                        self.set_attached(False)
                    print(f"[pick] two-hand grasp-assist {'on' if self.assist_on else 'off (pure physics only)'}")
                else:
                    raise ValueError
            elif c == "jitter" and len(parts) > 1 and parts[1].lower() == "reset":
                self.jitter_reset(SIDES)
                print("[hands] jitter numbers restarted")
            elif c == "task":
                arg = parts[1].lower() if len(parts) > 1 else "start"
                if arg == "stop":
                    if self.task:
                        self.task_finish(stopped=True)
                else:
                    self.task_start()
            elif c == "home":
                self.go_home()
                print("[hands] wrists back home, palms up; each re-anchors on its next engaged frame")
            elif c == "s":
                print(self.status())
            else:
                print(f"[hands] unknown command {line!r}: open/fist L|R|both, calib reset, limit angle PCT, "
                      f"limit force N, warn PCT, obj on|off, scale [L|R] X, filter on|off|HZ, rot on|off|reset, "
                      f"rotfilter on|off|HZ, jitter reset, pick [can|cube|ball|egg|reset|stop|off|assist on|off], task [stop], "
                      f"home, peak, s, q")
        except (IndexError, ValueError):
            print(f"[hands] could not parse {line!r}")
        return True

    def save_calib(self):
        allc = {}
        if os.path.exists(CALIB_FILE):
            try:
                allc = json.load(open(CALIB_FILE))
            except (ValueError, OSError):
                allc = {}
        allc[self.source_name] = self.calib
        json.dump(allc, open(CALIB_FILE, "w"), indent=2)
        print(f"[hands] calibration saved to {CALIB_FILE} (source {self.source_name})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["bridge", "solver"], default="bridge")
    ap.add_argument("--no-coil", action="store_true",
                    help="fixed wrists (no Coil Pro); default: wrists follow the gloves' Coil positions")
    ap.add_argument("--task-dir", default="hands_tasks", help="where reach-and-hold results are written")
    ap.add_argument("--port", type=int, default=9870, help="bridge UDP port (source bridge)")
    ap.add_argument("--solver", default="127.0.0.1:12277", help="solver host:port (source solver)")
    ap.add_argument("--scene", default=SCENE_FILE)
    ap.add_argument("--write-scene", action="store_true", help="(re)write the scene file and exit")
    ap.add_argument("--led", action="store_true", help="glove ring LEDs over RCSP (bridge must run with --no-rcsp)")
    ap.add_argument("--rcsp", default="127.0.0.1:45451")
    ap.add_argument("--cmd-port", type=int, default=9874, help="UDP port for text commands (0 = off)")
    ap.add_argument("--state-udp", default=None, help="host:port to send the state as JSON 30 times/s (tests)")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--duration", type=float, default=0, help="quit after this many seconds (0 = never)")
    ap.add_argument("--quiet", action="store_true", help="no 1 Hz line")
    a = ap.parse_args()
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:
        pass
    if a.write_scene:
        open(a.scene, "w").write(build_scene_xml())
        print(f"[hands] wrote {a.scene}")
        return
    sim = HandsSim(a)

    if a.source == "bridge":
        src = BridgeSource(a.port)
    else:
        h, p = a.solver.rsplit(":", 1)
        src = SolverSource(h, int(p))
    src.start()
    rh, rp = a.rcsp.rsplit(":", 1)
    leds = GloveLeds(rh, int(rp)) if a.led else None

    cmds: queue.Queue = queue.Queue()
    threading.Thread(target=lambda: [cmds.put(l.strip()) for l in sys.stdin], daemon=True).start()
    if a.cmd_port:
        def udp_cmds():
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("127.0.0.1", a.cmd_port))
            while True:
                for l in s.recvfrom(4096)[0].decode(errors="replace").splitlines():
                    cmds.put(l.strip())
        threading.Thread(target=udp_cmds, daemon=True).start()
    st_sock = st_addr = None
    if a.state_udp:
        h, p = a.state_udp.rsplit(":", 1)
        st_sock, st_addr = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (h, int(p))

    viewer = None
    if not a.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(sim.m, sim.d, show_left_ui=False, show_right_ui=False)
        # from behind and above: your left hand on the screen's left; ] / [ cycle the side and front cameras
        viewer.cam.lookat[:] = [0.04, 0.0, 0.33]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 0.5, 0.0, -60.0
    print(f"[hands] source {src.name}; commands: open/fist L|R|both, calib reset, limit angle PCT, "
          f"limit force N, warn PCT, obj on|off, s, q")

    dt = sim.m.opt.timestep
    wall0, sim0 = time.perf_counter(), sim.d.time
    rep_wall, rep_sim, last_sync, last_state = wall0, sim.d.time, 0.0, 0.0
    t_end = wall0 + a.duration if a.duration else None
    ratios, running = [], True
    while running:
        now = time.perf_counter()
        if (t_end and now > t_end) or (viewer is not None and not viewer.is_running()):
            break
        while not cmds.empty():
            running = sim.command(cmds.get()) and running
        sim.update_inputs(src)
        target_sim = sim0 + (now - wall0)
        n = 0
        while sim.d.time < target_sim and n < 100:
            sim.step()
            n += 1
        if sim.d.time < target_sim - 0.1:
            wall0, sim0 = time.perf_counter(), sim.d.time
        sim.update_led(leds)
        sim.task_update()
        sim.pick_update()
        if sim.coil and now - sim.last_jitter > 0.5:
            sim.last_jitter = now
            sim.update_jitter()
        if viewer is not None and now - last_sync > 1 / 60:
            last_sync = now
            scn = viewer.user_scn                      # ghost balls: the raw 1:1 hand positions
            scn.ngeom = 0
            if sim.coil:
                for s in SIDES:
                    if sim.raw_anchor_in[s] is not None and scn.ngeom < scn.maxgeom:
                        mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                            np.array([0.012, 0, 0]), np.asarray(sim.raw_pos[s], dtype=float),
                                            np.eye(3).flatten(), np.array([0.2, 0.4, 1.0, 0.5], dtype=np.float32))
                        scn.ngeom += 1
            texts = []
            for side, pos in (("left", mujoco.mjtGridPos.mjGRID_TOPLEFT), ("right", mujoco.mjtGridPos.mjGRID_TOPRIGHT)):
                title, body, led = sim.overlay(side)
                texts.append((mujoco.mjtFontScale.mjFONTSCALE_150, pos, f"{title}\n{body}\n{led}", None))
            if sim.pick_on:
                texts.append((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                              "OBJECT", sim.pick_text()))
            viewer.set_texts(texts)
            viewer.sync()
        if st_sock and now - last_state > 1 / 30:
            last_state = now
            st_sock.sendto(json.dumps(sim.state()).encode(), st_addr)
        if now - rep_wall >= 1.0:
            ratio = (sim.d.time - rep_sim) / (now - rep_wall)
            ratios.append(ratio)
            if not a.quiet:
                parts = []
                for s in SIDES:
                    ap_, fp = sim.pcts(s)
                    parts.append(f"{s[0].upper()} {sim.led[s]:6s} angle {ap_:5.1f}% grip {sim.force[s]:5.2f} N")
                print(f"[hands] realtime {ratio:.3f} | " + " | ".join(parts), flush=True)
            rep_wall, rep_sim = now, sim.d.time
        if sim.d.time >= sim0 + (time.perf_counter() - wall0):
            time.sleep(0.0005)
    if leds:
        leds.close()
    if viewer is not None:
        viewer.close()
    if ratios:
        r = ratios[1:] or ratios
        print(f"[hands] realtime ratio over {len(r)} s: min {min(r):.3f}  mean {sum(r) / len(r):.3f}  max {max(r):.3f}")


if __name__ == "__main__":
    main()
