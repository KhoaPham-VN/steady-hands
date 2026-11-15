# Steady Hands

**Humans slip. Robots catch.**

Steady Hands is assisted teleoperation for fragile objects. An operator wearing Rokoko Smartgloves II controls two simulated five-finger robot hands in MuJoCo, and a force cap in software keeps the grip below the force that would break the object.

Built at Junction 2026 (Espoo, 13–15 November 2026) for the Rokoko Robot Teleoperation Challenge.

## The problem

Your own hand feels an egg and eases off in time. Teleoperation gloves send motion out and no force back, so a robot hand copies a squeeze that is too hard at full strength. The operator finds out only when the object breaks.

## What it does

- Two simulated hands follow both gloves: finger curls from the gloves, wrist position and rotation from the Coil Pro, and a clutch to engage and release without jumps.
- A force cap limits the closing force of the fingers (`limit force 10` sets it to 10 N; `limit force 0` turns it off).
- A force cue replaces the missing sense of touch: the object on screen and the LED ring on each glove go green, yellow (from 80% of the limit) and red (at the limit).
- The test object is a 60 g simulated egg. It turns red and counts as broken when one hand squeezes it with more than 30 N.
- Motion scaling (`scale 0.3`: 10 cm of hand travel moves the robot hand 3 cm) and a 1€ jitter filter at 3 Hz help with fine positioning.
- Each pick-up attempt is saved as JSON in `hands_tasks/`, with time to touch, time to lift, peak grip force and whether the egg broke.

## Result

Right hand, 15 November 2026. Task: lift the egg 5 cm above the table and hold it for 2 s. The logs are in `hands_tasks/`.

| | Cap off | Cap on (10 N) |
| --- | --- | --- |
| Logged egg trials | 1 | 3 |
| Egg broken | 1 of 1 | 0 of 3 |
| Peak grip force | broke at the 30 N threshold | 13.3–21.6 N |

The peak values include a brief overshoot at first contact and stay under the 30 N break threshold.

## How it works

```
Smartgloves II (left + right) + Coil Pro
   |  USB, 83 Hz per glove
Rokoko Device SDK driver (port 12276)
   |
glove_bridge.py: finger curls, wrist position and rotation, clutch
   |  UDP JSON (port 9870)
hands_sim.py (MuJoCo): two five-finger hands, egg physics
   |-- assist.py: force cap, motion scaling, 1-euro jitter filter
   |-- LED colour back to each glove (RCSP, port 45451)
   +-- trial logs in hands_tasks/
```

Everything runs on one laptop with no cloud in the loop. The simulation holds 0.999–1.002× real time.

## Requirements

- Windows laptop (the setup we tested), Python 3.10 or newer (tested with 3.13.5)
- `python -m pip install -r requirements.txt` (MuJoCo 4.0.1 and NumPy)
- For live control: Rokoko Device SDK with driver 1.9.7, Smartgloves II (left and right) and a Coil Pro

The glove tools (`glove_bridge.py`, `udp_monitor.py`, `fake_rgmp_server.py`) use only the Python standard library.

## Run the tests without gloves

```powershell
python test_hands.py        # 48 checks: finger limits, Coil positioning, wrist rotation, pick-up, egg
python test_assist.py       # 7 checks: force cap, motion scaling, jitter filter
python test_end_to_end.py   # 11 checks: gripper prototype with a synthetic operator
python test_glove_only.py   # 11 checks: glove-only mode and LED colours
```

The tests use their own ports, so they can run while the driver and the bridge are running.

## Run the demo with gloves

Open three PowerShell windows in this folder.

| Window | Command |
| --- | --- |
| W1 | `rkk-hand-solver --emit-openxr --listen 0.0.0.0:12277` (starts the driver) |
| W2 | `python glove_bridge.py --no-rcsp --rate 83.33 --udp 127.0.0.1:9870` |
| W3 | `python hands_sim.py --led` |

1. Calibrate the fingers in W3: `obj off`, then `open L`, `fist L`, `open R`, `fist R`, holding each pose still.
2. Type `e` and Enter in W2 to engage the clutch. The simulated hands follow the gloves from where they are.
3. Cap on: in W3 type `limit force 10`, `pick egg`, `pick`, then grasp and lift the egg.
4. Cap off: `limit force 0`, `pick reset`, `pick`, then squeeze. The egg turns red.

`HANDS_COIL_SESSION.md` is the full checklist, with pass criteria for each step, the axis check for a new standing spot (`coil_axes_check.py`) and the table of safe force caps. `HANDS_SESSION.md` covers the same session without the Coil Pro (`python hands_sim.py --no-coil --led`).

## Files

| File | What it does |
| --- | --- |
| `hands_sim.py`, `hands_scene.xml` | The demo: two five-finger hands, grip limits, LED cue, pick-up trials and the breakable egg |
| `assist.py` | `GripLimiter`, `MotionScaler` and `JitterFilter` (1€ filter), with their parameters at the top of the file |
| `glove_bridge.py` | Reads the SDK driver and publishes finger curls, wrist pose and clutch state as UDP JSON |
| `led_test.py` | Cycles a glove's ring LED through green, yellow, red and off |
| `coil_axes_check.py` | Measures the Coil-to-robot axis mapping for where the operator stands |
| `solver_check.py` | Checks the finger range of the SDK hand solver before using `--source solver` |
| `egg_sim.py`, `egg_scene.xml`, `check_scene.py`, `synth_operator.py` | Earlier prototype: a two-finger gripper that picks and places the egg |
| `eval_offline.py`, `report.py` | Jitter measurement on a recorded stream, and the results table for gripper trials |
| `hand_viewer.py` | Browser view of both hands around the Coil Pro |
| `udp_monitor.py`, `fake_rgmp_server.py`, `replay_csv.py` | Stream monitor, fake driver for work without gloves, and playback of a recorded CSV |
| `rgmp_recenter.py` | Relay for driving Isaac Lab on a remote GPU (not used in the demo) |
| `test_*.py` | Automated checks |
| `hands_tasks/` | Logged trials from 15 November 2026 |
| `docs/GLOVE_BRINGUP.md` | Glove, Coil Pro and SDK bring-up notes |

## Data not included

The organizers' glove recording (`data/fran_fac_sim_01_ypos_*.csv`) is Rokoko's and is not in this repository. With your own recording in the same format, `replay_csv.py` plays it to the tools as if gloves were connected.

## Status and next steps

Everything runs in simulation. The next step is the same force cap on a real robot hand, as a force or current limit. After that come laboratories that handle glassware and hazardous samples, and healthcare. Assisted teleoperation can also give cleaner demonstrations for training autonomous robots.
