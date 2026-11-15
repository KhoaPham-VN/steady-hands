# Two-hand glove session (`hands_sim.py`, both gloves, no Coil Pro)

Two simulated hands follow the two gloves' fingers; the wrists stay fixed. A soft bar is mounted
in each hand to measure grip force. Both grip limits are active:

- **A, angle limit** (`limit angle 80`): each finger may close to 80% of its joint range.
- **B, force cap** (`limit force 10`): the hand cannot squeeze its bar harder than 10 N.

Each glove's ring LED, and that hand's bar on screen, shows the worse of the two:
green below 80% of a limit, yellow from 80%, red at the limit (97% or more), off on quit.

This does not touch `egg_sim.py` or `egg_scene.xml`, and it is not Phase 4.
Run everything from the `rokoko-teleop` folder:
`cd $env:USERPROFILE\OneDrive\Documents\rokoko-teleop`

## 0. Before the gloves go on (5 minutes)

```powershell
python test_hands.py
```

**Pass:** the last line is `48 of 48 checks passed` (the test also covers Coil positioning and wrist rotation). It
uses its own ports, so it can run while the driver, solver and bridge are already running.

## 1. Driver and bridge (skip whatever is already running)

| Window | Command |
| --- | --- |
| W1 | `rkk-hand-solver --emit-openxr --listen 0.0.0.0:12277` (starts the driver) |
| W2 | `python glove_bridge.py --no-rcsp --rate 83.33 --udp 127.0.0.1:9870` |

`--no-rcsp` is required: `hands_sim.py` drives both LEDs.

**Pass:** W2's status line shows finger values for **both** `left:` and `right:`, with `coil:--`,
which is expected without the Coil.

## 2. LED test, both gloves (W4)

```powershell
python led_test.py --hand left --seconds 1
python led_test.py --hand right --seconds 1
```

**Pass:** both print `PASS`, and each glove's ring LED visibly shows green, yellow, red, off.
If one fails, copy its `FAIL at ...` line and continue without `--led` in step 3.

## 3. Start the sim (W3)

```powershell
python hands_sim.py --no-coil --led
```

(`--no-coil` keeps the wrists fixed. Coil positioning is now the default: see `HANDS_COIL_SESSION.md`.)

**Pass:**
- A window opens showing two hands seen from behind (your left hand on the screen's left).
  `]` / `[` switch to the side and front cameras, Esc goes back.
- The top-left panel says `LEFT hand (live)` and the top-right panel says `RIGHT hand (live)`.
- W3 prints `realtime` between 0.95 and 1.05.
- With both hands relaxed, both LEDs and both bars are green.
- No `[led] ... not reachable` or `no ... glove listed` message.

## 4. Calibrate each hand (W3)

The operator holds each pose still while the driver types:

```
obj off          (lets the simulated fingers close fully)
open L           left hand flat, fingers together
fist L           left hand tight fist
open R
fist R
```

**Pass:**
- Each command prints the five raw values, averaged over the last 0.5 s, and
  `calibration saved to hands_calib.json (source bridge)`.
- After `fist L` / `fist R`, every finger line ends in `ok`. `REJECTED` means that finger's fist
  value was not at least 0.10 above its open value; that finger then uses its raw value. Redo `open` and `fist` for that hand.
- Flat hands: every finger angle in the overlay is under 20 deg.
- Fists: each hand's overlay shows `angle` of 90% or more, and the LED is red.

## 5. Angle limit (A), object off

Type `limit angle 80`. Close each hand slowly, one at a time.

**Pass:**
- The LED goes green, then yellow (from about 64% of the joint range = 80% of the limit), then red at the fist.
- At the fist the simulated fingers stop at 80%: index, middle, ring and little about 224 deg (of 280),
  thumb about 160 deg (of 200), and the overlay shows about `100% of 80% limit`.
- **Independence:** left fist and right open give left red and right green, on the LEDs and on screen.
  Then swap hands.

## 6. Force cap (B), object on

```
limit angle 100
obj on           (each bar appears as soon as that hand is open: open both hands)
limit force 10
```

Grip slowly, then squeeze hard.

**Pass:**
- The overlay grip rises. The LED is yellow from 8 N and red at 10 N.
- Holding a hard fist, the grip settles at 9–11 N and stays there however hard you squeeze.
- `s` shows the peak force. At first contact it can overshoot briefly to about 1.3–1.5 times the cap.
- `limit force 20`: the hard fist settles near 20 N.
- `limit force 0` (cap off): the hard fist goes well above 20 N, and the LED follows the angle only.

## 7. Quit

Type `q` in W3.

**Pass:** both ring LEDs turn off, and W3 prints the `realtime ratio over N s` summary.

## 8. Optional: solver joint angles instead of bridge curls

```powershell
python hands_sim.py --no-coil --source solver --led
```

This source reads `rkk-hand-solver`'s 26 joints per hand (TCP 12277, the stream `hand_viewer.py` draws).
The solver has no calibration of its own, and on 14 November some fingers barely changed between
open and fist. So **check its range first**, then calibrate (calibration is stored per source in
`hands_calib.json`):

```powershell
python solver_check.py --seconds 30      # open and close BOTH hands slowly 3-4 times while it runs
```

**Pass:**
- Every finger in the final table says `ok` (range 0.10 or more).
- A finger marked `TOO SMALL` cannot be calibrated with this source: use `--source bridge` for that session.

Then:

```powershell
python hands_sim.py --no-coil --source solver --led
```

Run steps 4–7 again. When you type `fist L` / `fist R`, each finger prints `ok` or
`REJECTED`. A rejected finger uses its raw solver value (marked `(raw!)` in `s`).

**Pass:** all ten fingers print `ok`, then the same as steps 3–7.
