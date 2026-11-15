# Glove-only session (right glove, no Coil Pro)

Tests grip control while the Coil Pro is missing. The glove drives only the pinch; the gripper
position comes from the keyboard. This is **not** Phase 4: trials here are practice, and if
logged they carry condition `glove_only`, which the default `report.py` leaves out.
The full checklist for when the Coil is back is `GLOVE_SESSION.md`.

Run everything in PowerShell from the `rokoko-teleop` folder:
`cd $env:USERPROFILE\OneDrive\Documents\rokoko-teleop`

## 0. Before the glove goes on (1.5 minutes)

```powershell
python test_glove_only.py
```

**Pass:** the last line is `11 of 11 checks passed`. Close any `replay_csv.py` or
`fake_rgmp_server.py` window afterwards: they hold ports 12276 and 45451.

## 1. Driver and bridge

| Window | Command |
| --- | --- |
| W1 | `rkk-hand-solver --emit-openxr --listen 0.0.0.0:12277` (starts the driver, as in TONG_KET 6.1) |
| W2 | `python glove_bridge.py --no-rcsp --rate 83.33 --udp 127.0.0.1:9870` |

`--no-rcsp` is required here: `egg_sim.py` drives the LED, and the bridge would otherwise set it too.

**Pass:** W2's status line shows `right: T.. I.. M..` finger values, and `coil:--`, which is
expected without the Coil.

## 2. LED test (W4, before the sim)

```powershell
python led_test.py
```

**Pass:**
- It prints `PASS`.
- The right ring LED visibly shows green, yellow, red, then off, 2 s each.

**If it fails:** copy the whole `FAIL at ...` line (it contains the exact error) and use step 3
*without* `--led`. The on-screen colour still works.

## 3. Start the sim (W3)

```powershell
python egg_sim.py --no-coil --led        # without --led if step 2 failed
```

**Pass:**
- The MuJoCo window opens and its overlay shows `Motion  keyboard (no Coil)`.
- W3 prints `realtime` between 0.95 and 1.05 and `bridge` about 83 msg/s.
- With `--led`: no `[led] ... not reachable` or `no right glove listed` message.

Keep the MuJoCo window visible and keep an eye on the `realtime` value: in two automated runs the sim
clock appeared to stop, and this was not reproduced in clean runs.

## 4. Pinch calibration (type in W3)

1. Hand open and relaxed: `po`
2. Thumb touching index, held: `pc`
3. Open and close a few times, typing `s` each time.

**Pass:**
- `po` is larger than `pc` by at least 0.02 m.
- `s` shows `travel` near 0 mm with the hand open and near 40 mm when pinched.
- Write both values down.

## 5. Keyboard check

Click the MuJoCo window, then press **Up arrow** 4 times, **Left arrow** 2 times and **PageDown**
2 times. Type `s` in W3.

**Pass:** the target changes from `+0.500 -0.100 +0.620` to `+0.520 -0.090 +0.610`. The gripper
moves away from the camera, to the left and down. (In the terminal the same moves are
`x+ 4`, `y+ 2`, `z- 2`.) Then type `home`.

## 6. Five trials, assist off (practice)

For each trial, in W3:

1. `r`: new trial.
2. `grasp`: the gripper moves over the egg and descends. Wait for `auto grasp done`.
3. The operator pinches to grip the egg.
4. `carry`: lift, move over the green disc, descend. Wait for `auto carry done`.
5. The operator opens the hand.

Fine moves at any point: arrow keys / PageUp / PageDown, or `x+ 2`, `z- 1` etc.

**Pass:**
- Each of the 5 trials ends with `SUCCESS`, `BROKEN` or a placement message in W3, not a timeout.
- `realtime` stays within 0.95–1.05.
- A hard squeeze can break the egg (it turns red).
- Write down the outcomes and peak forces.

## 7. Five trials, assist on (practice)

Type `a on` (the overlay shows `Assist ON`, `Grip cap 5 N`), then run the same five steps.

**Pass:**
- No egg breaks, even at a full pinch.
- Peak about 5–7 N (it can go slightly over 5 N while the hand shakes).
- The egg is green while held. With `--led` the ring LED is green and goes red only if an egg breaks.
- Switching back with `a off` makes the overlay show `60 N` again, and the LED goes off.

## 8. Finish

Type `q` in W3 (with `--led` this turns the LED off). If any trials were logged:
`python report.py --glove-only` writes `results_glove_only.md`.

### Optional: keep a record of these trials

Before step 6, type `op NAME` and `l` in W3. The trials are then logged as
`trials\*_glove_only-off_NAME.jsonl` / `*_glove_only-on_NAME.jsonl`, separate from full trials.
The default `python report.py` ignores them. Type `l` again to stop logging.
