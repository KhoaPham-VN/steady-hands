# Rokoko glove teleop starter (Smartgloves II + Coil Pro + Device SDK)

Gloves in, one small JSON stream out (finger curls 0..1, grip, pinch, absolute wrist pose
from the Coil Pro, clutch state). Any robot side (Inspire hand, G1, MuJoCo, Isaac) just
listens on UDP. Standard-library Python 3.10+, no installs.

```
rokoko-sdk ──RGMP :12276──► glove_bridge.py ──UDP JSON :9870──► your robot code / udp_monitor.py
           ──RCSP :45451──►  (button = clutch/calibrate, ring LED = state)
```

| File | What it does |
| --- | --- |
| `glove_bridge.py` | Reads the SDK driver, computes curls + wrist pose, publishes UDP JSON (message format in the file header) |
| `udp_monitor.py` | Live view of the stream, `--record x.jsonl` to log demos; `to_robot()` is the template for your robot mapping |
| `fake_rgmp_server.py` | Fake `rokoko-sdk` (spec-compliant RGMP + RCSP) so teammates can build the robot side without gloves |

## Robot frame (measured at the venue)

Your Coil Pro axes: **+x = left, +y = forward, +z = down**. The bridge converts them to the usual
robot frame (**X forward, Y left, Z up**, as in Isaac Lab and ROS) and adds these fields to every message:
`ee_pos`, `ee_quat`, `ee_dpos` (hand motion since you engaged, in metres) and `ee_dquat`.
To drive an arm: when you engage, store the robot's current end-effector pose, then each tick send
`target = stored_pose + ee_dpos`, with rotation `ee_dquat ⊗ stored_rotation`.

- Moved the Coil or your standing spot? Measure the axes again and pass them, e.g. `--coil-to-robot y,x,-z`.
  This says robot X comes from coil y, robot Y from coil x, and robot Z from minus coil z.
  The bridge refuses mappings that would mirror the hand.
- `--scale 0.5` halves robot motion relative to your hand, for finer control.

## 1. Bring-up checklist (about 15 minutes, do it in this order)

1. **License + versions**: `rokoko doctor` should show activated, driver version. Latest documented release is 2026.10.20 (driver 1.9.7); `rokoko outdated` tells you if you're behind.
2. **Firmware**: plug ONE glove in by USB, run `rokoko-sdk` in terminal 1, then `rokoko device list` and `rokoko device update <id>`. Power-cycle the glove after. Repeat for the other glove.
   (The CLI can't update the Coil Pro; if it needs firmware, that goes through Rokoko Studio.)
3. **Stream over USB** (simplest, ~19 ms latency, no router needed):
   ```
   rokoko-sdk --auto-stream-usb
   python3 $ROKOKO_SDK_HOME/examples/python/rgmp_stream.py | head -40
   ```
   USB gloves do NOT stream unless you pass `--auto-stream-usb` (Wi-Fi gloves stream by default).
4. **See the hand in 3D** (optional, needs Node 18+ and pnpm):
   ```
   cd $ROKOKO_SDK_HOME/examples/web && pnpm install
   rkk-ws-bridge                                  # terminal 2
   pnpm --filter rgmp-ws-bridge-web dev           # terminal 3 → http://localhost:5181
   ```
5. **Coil Pro**: power it from the wall with ITS OWN power supply (USB can't run it). Put it on a level, non-metal stand about 1 m high, at least 1 m from walls, in the middle of your teleop area.
   Keep hands more than 50 cm from it (too close saturates the sensors), with best accuracy within about 2 to 3 m.
   Keep it away from metal table frames, radiators, laptops, and **the robot's motors**: put the operator station somewhere else and send commands over the network.
   No watches or bracelets.
   - Shake each hand once (this resolves the EMF up/down flip), and turn each hand around a bit (so the magnetometer heading converges).
   - If the two hands show up on opposite sides of the Coil (about 2.6 m apart), restart the driver with `--coil-half-space y-pos` (or `x-pos`/`x-neg`/`y-neg`, depending on where you stand).
6. **Bridge**:
   ```
   python3 glove_bridge.py --udp 127.0.0.1:9870     # terminal 2
   python3 udp_monitor.py                            # terminal 3
   ```
   Calibrate each glove: **double press** the glove button with your hand flat and open, then **long press** it with your hand in a fist.
   Hold the pose for about 1.5 s after the press; the ring LED turns yellow while it samples.
   Calibration saves to `calib.json`.
   A **single press** toggles ENGAGE (the clutch): the LED goes green and `wrist_dpos`/`wrist_dquat` start from zero.
   Keyboard fallback: `o` = open, `f` = fist, `e` = engage, `q` = quit.

Wi-Fi instead of USB (only if the operator must walk around): `python3 network_configuration.py set --ssid <SSID> --receiver-address auto --receiver-port 14041` per glove over USB. Use a dedicated router on 5 GHz channel 36, 44 or 48 with band steering off, and add a firewall exception.

## 2. First checks on real hardware

This code was tested only against `fake_rgmp_server.py`, a simulated driver built from the specs. The tests covered sign flips, curls past 180°, mirrored hands and restarting from a saved calibration: curl error stayed under 0.01 and wrist pose error was 0.3 mm. **It has not run on real gloves yet.** On hardware:

- [ ] After calibration, every finger goes 0 to 1 from open hand to fist. If one comes out inverted or stuck, restart with `--no-palm-check` and recalibrate. (That check assumes the documented glove frame: +X right, +Y toward the wrist, +Z down.)
- [ ] `coil_ok` is true, and `wrist_pos` changes smoothly as you move your hand. Work out which axis is forward, left and up (move your hand along each one and watch), then write the one fixed rotation from the Coil frame to your robot's base frame.
- [ ] The status line shows about 166 glove frames/s with both gloves (83.33 Hz each).

## 3. Troubleshooting (from the Rokoko SDK docs)

- `cannot reach 127.0.0.1:12276`: the driver isn't running.
- Gloves are listed but no data arrives: on USB, start the driver with `--auto-stream-usb`.
- `RBDP Protocol Error` / `Could not create device`: a stale driver is holding the USB connection. Run `pgrep -x rokoko-sdk`, kill it, and replug the hub.
- Jittery wrist: the hand is bent far back, the two gloves are close together, the arm sensor is bunched up near the hub, or there's metal nearby. Pull the glove fabric (never the sensors) so the arm sensor lies flat.
- `heading_ok` false: rotate the hand around, away from metal.
- If you use the official hand solver for ROS 2 or Isaac, it must run with `--emit-openxr`, or the bridge publishes nothing.

## 4. Official building blocks worth knowing

- `rkk-hand-solver --emit-openxr`: a 26-joint hand in OpenXR order (what Isaac Lab and dex retargeters expect), served on port 12277.
- ROS 2 bridge (github.com/Rokoko/rokoko-ros2-bridge): publishes `/rkk_hand_bridge/hand/joints`; RViz via `publish_markers:=true`.
- Isaac Lab 2.3.0 example (`$ROKOKO_SDK_HOME/examples/python/isaac`): `RokokoGloveDevice` stands in for Isaac's OpenXR device, and `isaac_teleop_record.py` records HDF5 demos.
  It was validated on `Isaac-PickPlace-GR1T2-Abs-v0` and `Isaac-Stack-Cube-Franka-IK-Rel-v0`.
  It uses *fixed* wrist anchors; the docs say real arm travel needs a tracked anchor, and the Coil Pro pose from this bridge can be that anchor.
- `rgmp_pinch_detector.py`: thumb-to-finger pinch events, useful as gesture commands.
