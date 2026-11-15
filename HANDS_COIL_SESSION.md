# Two-hand session with the Coil Pro (`hands_sim.py`, wrists follow the gloves)

Each simulated wrist follows its glove's Coil position, with the same clutch as `egg_sim.py`:
type `e` in the bridge window to engage or disengage, and the hands never jump.

Per hand the position goes through:
- the **1-euro jitter filter** (`filter on|off|HZ`), then
- **motion scaling** (`scale 0.3`).

A blue **ghost ball** shows where the hand would be at 1:1 with no filter. Finger curls, both grip
limits and the per-glove LEDs work exactly as in `HANDS_SESSION.md`.

Each hand also **rotates with its glove** in all three axes, with the same clutch: on engage the hand
starts from the orientation it has, then turns as the glove turns. The rotation has its own filter
(`rotfilter on|off|HZ`), and `scale` does not apply to it. `rot off` keeps the palms up (the old
behaviour), and `rot reset` puts the palms back up. The forearm and the bar turn with the hand.

The finger source is the bridge (default). `--source solver` has no Coil positions.
This does not touch `egg_sim.py`, and it is not Phase 4.
Run everything from the `rokoko-teleop` folder:
`cd $env:USERPROFILE\OneDrive\Documents\rokoko-teleop`

## 0. Before the gloves go on (5 minutes)

```powershell
python test_hands.py
```

**Pass:** the last line is `48 of 48 checks passed`. It uses its own ports, so it can run beside a
running driver. Part C covers Coil positioning: axes, the 0.3 scale, the filter, the clutch, the grip
cap while moving, and reach and hold. It also covers wrist rotation: the three axes on both hands,
palm up → palm down, the rotation filter and clutch, `rot off|on|reset`, and the grip cap while rotating.
Part E covers the pick-up: a scripted grasp and lift with the right hand (can), the left hand (ball) and
both hands (can), dropping, `pick reset`, and realtime. It also covers the egg: lifted intact at a 10 N
cap, broken by a squeeze with the cap off, and whole again after `pick reset`.

## 1. Driver, Coil and bridge

| Window | Command |
| --- | --- |
| W1 | `rkk-hand-solver --emit-openxr --listen 0.0.0.0:12277` (starts the driver) |
| W2 | `python glove_bridge.py --no-rcsp --rate 83.33 --udp 127.0.0.1:9870` **(type `e` + Enter here to engage / disengage)**. Add `--coil-to-robot=...` if step 4 measured a different mapping for where you stand. |

After switching on the Coil, shake each hand once and turn the hands a few times. The operator stands
**0.5–1.1 m from the Coil**; tracking degrades beyond about 1.1 m.

**Pass:** W2's status line shows `pos=x,y,z` numbers for **both** `left:` and `right:`, not `coil:--`.
If a position jumps or mirrors, restart the driver with `--coil-half-space`, as in `glove-teleop-bringup.md`.

## 2. Start the sim (W3)

```powershell
python hands_sim.py --led
```

If the finger calibration for the bridge source isn't done yet, do step 4 of `HANDS_SESSION.md`
first (`obj off`, `open L`, `fist L`, `open R`, `fist R`, and every finger prints `ok`).

**Pass:**
- Each hand's overlay shows `pos   Coil, clutch out (hold)`, `wrist roll +0  pitch +0  yaw +0 deg  (filter 3.0 Hz)`,
  `scale x1.00` and `filter 3.0 Hz`.
- W3 prints `realtime` between 0.95 and 1.05.
- With relaxed hands, both LEDs are green.

## 3. Engage

Type `e` in **W2**.

**Pass:**
- Both overlays switch to `pos   Coil, engaged`.
- The simulated hands do **not** jump: they start from where they were, palms still up (roll, pitch
  and yaw within ±2 deg), whatever way the real hands are turned.
- A blue ghost ball appears at each hand.

## 4. Axis direction check (one hand at a time)

Move each hand slowly about 10 cm in each direction. Both hands must match:

| Hand moves | Simulated hand must move (camera behind the hands) |
| --- | --- |
| Forward, away from the operator | Away from the camera |
| To the operator's left | To the screen's left |
| Up | Up |

**Pass:** all three directions match for both hands.

**If an axis is reversed: measure, don't guess.** The bridge's default `--coil-to-robot y,x,-z` was
measured at the old standing spot (TONG_KET section 4). It only holds while the operator faces the same
way relative to the Coil. On 15 November, after moving, forward/back and left/right were reversed but up/down
was correct, and the rotations were reversed too. That is the pattern of a 180 deg turn about the vertical.
The mapping is applied in one place, the bridge, to the position (`ee_pos`, `ee_dpos`) and the orientation
(`ee_quat`, `ee_dquat`) together. So the fix is a bridge restart, with no code change.

1. Quit W3 (`q`). The checker uses the same UDP port. Leave the driver and the bridge running.
2. Stand where you will operate, facing the way the camera looks. Then in W3:
   ```powershell
   python coil_axes_check.py --hand right
   ```
   Do the five moves it asks for. For each: hold the start pose, press Enter, move, hold, press Enter.
   - forward 20 cm
   - left 20 cm
   - up 20 cm
   - palm down, turn the fingers about 60 deg to the left (about the vertical)
   - palm down, tilt the fingertips down about 45 deg

   Optionally repeat with `--hand left`: it must give the same answer.
3. Without the tool, you can do the position part with W2's status line. `pos=x,y,z` there is the **raw Coil frame**,
   whatever the mapping. Move 20 cm forward and note which number changes by about ±0.20 and its sign: that
   is robot X. For example, if y drops by 0.20, robot X = `-y`. Do the same for left (robot Y) and up (robot Z).
   The status line doesn't show the orientation, which is why `coil_axes_check.py` exists.

**Pass:**
- The tool prints `position: robot X,Y,Z = coil ...` with no WARNING.
- Both orientation checks say `OK`.
- It ends with `position and orientation agree. Restart the bridge with: ...`.

Then stop W2 (Ctrl+C) and start it with the printed value, **with `=`**: argparse would read a value
starting with `-` as an option. For the 180 deg case the value is `-y,-x,-z`:

```powershell
python glove_bridge.py --no-rcsp --rate 83.33 --udp 127.0.0.1:9870 --coil-to-robot=-y,-x,-z
```

W2's first lines print `Coil -> robot axes: robot X,Y,Z = coil -y,-x,-z`. Restart W3, engage, and repeat
this step's table (positions) and step 7 (rotations). Back at the old standing spot, leave the option out (default `y,x,-z`).
If the tool says `NOT a valid mapping` after all three directions were reversed, that is the Coil's
half-space ambiguity, not the mapping. Restart the driver with `--coil-half-space` instead.

## 5. Scale check (10 cm of hand travel = 3 cm at 0.3)

1. Type `s` and note the left `wrist` x value. Move the left hand 10 cm forward (use a ruler), then type `s` again.
2. Type `scale 0.3`. The hands must not move when you type it. Repeat step 1.
3. Repeat both for the right hand, then type `scale 1`.

**Pass:**
- At scale 1, wrist x changes by 0.085–0.115 m.
- At 0.3, it changes by 0.025–0.035 m, while the blue ghost moves the full 10 cm ahead of the hand.
- There is no jump when the scale changes.

## 6. Filter check and clutch

The jitter numbers are **cumulative**. They add up from the last `filter`, `scale` or engage change
(or `jitter reset`), and the overlay shows how many seconds of still data they cover. The second line
counts only the jitter above 3 Hz, the part a 3 Hz filter acts on. (Before this change, a 3 s window
was used. It was too noisy: on the organizers' recording, "filt" read higher than "raw" in a quarter to a
third of the 3 s windows. On 15 November the left overlay showed filt 5.37 mm against raw 4.71 mm for this reason.)

1. Hold both hands still at grasp height.
2. Type `filter off` and hold still for **20 s**. Note `jitter raw`, `filt` and the `>3 Hz` line.
3. Type `filter 3` and hold still for 20 s. Note them again. (Optional: `filter 1.5`.)
4. Clutch: type `e` in W2, move both hands about 10 cm, then type `e` again.

**Pass:**
- **Both** hands show numbers, not `--`. (The right hand showed `--` before because the bridge sometimes
  sends a right-glove frame twice. Those repeats are now skipped.)
- With the filter off, `raw` equals `filt` on both lines.
- With the filter on (20 s, at least 15 s still):
  - On the `>3 Hz` line, `filt` is clearly lower than `raw`. On the organizers' recording it was about 20 % lower: 0.73 → 0.57 mm.
  - On the main line, `filt` is the same or a few % lower. Most of the hand's jitter is slow wobble below
    3 Hz, and the filter barely touches it (organizers' recording: 4–10 %). That is a finding, not a fail.
  - Write all four numbers down with the seconds of still data: they are the jitter result.
- Clutch: while disengaged the simulated hands hold still, and on re-engage they don't jump.

## 7. Wrist rotation (both hands)

Both hands engaged, camera behind the hands, and `obj off` so the fingers are easy to see. Type
`rot reset` with both palms up, then turn **slowly**. Do each move with both hands together, then each hand alone.
The overlay angles are measured in the room's frame, from palm-up home: roll about the forward axis,
pitch about the left axis, yaw about the up axis.

| Real hands do | Simulated hands must do | Overlay |
| --- | --- | --- |
| Palm up → palm down (turn the forearms so each thumb goes up and over toward the middle) | The same: both thumbs go up at the halfway point, and end pointing toward the middle with the palms down. The left hand must **not** turn the other way (mirrored). | Left roll goes **positive** to about +180, right roll **negative** to about −180 (at ±180 the sign may flip; that is the same angle) |
| Palm down, fingertips bend **down** (wrist flexion) | Fingertips bend down | pitch **positive** (about +30 to +60), both hands |
| Palm down, fingertips bend **up** (extension) | Fingertips bend up | pitch negative, both hands |
| Palm down, fingertips swing to the **left** (deviation) | Fingertips swing to the screen's left | yaw **positive**, both hands |
| Palm down, fingertips swing to the **right** | Fingertips swing to the screen's right | yaw negative, both hands |

Then:
1. **Clutch:** type `e` in W2 (disengage), turn both hands palm down, then type `e` again. The simulated
   hands must not move while disengaged or jump on re-engage, and then follow from where they are.
2. **`rot off`:** both simulated palms go up and stay up while you turn. `rot on`: no jump, and the hands follow again.
   `rot reset` with your palms up: the simulated palms are up and match yours again.
3. **Filter:** hold one hand still with the palm down. Compare the overlay angles with `rotfilter off`
   and `rotfilter 3`. With the filter on they should flicker less. (`rotfilter 1.5` smooths more but lags more.)

**If the rotations are reversed** (for example, fingers to the left turns the simulated hand to the right,
and positions are reversed too): the axis mapping is wrong for where you stand. Do the
`coil_axes_check.py` part of step 4 and restart the bridge with the value it prints. Position and
orientation are fixed together. Don't try to correct the rotation on its own: `hands_sim.py` has no
rotation-only setting on purpose.

**Pass:**
- Palm up → palm down turns both simulated hands the same way as the real hands, and the left is not mirrored.
- Flexion and deviation match, as in the table, for both hands.
- No jump on engage, on re-engage, or on `rot on`.

**Known effect:** the Coil measures the sensor on the back of the hand. Flipping the palm moves that
sensor, so the simulated hand also shifts up to about 4 cm while it turns. That is expected.

## 8. Grip limit and LEDs while the hands move and rotate

```
obj on            (open both hands: each bar mounts once its hand is open)
limit force 10
```

Grip both bars hard while moving both hands slowly around. Keep gripping and turn both palms down,
back up, then bend the wrists up and down. Then:

```
obj off
limit angle 80
```

Make fists while moving.

**Pass:**
- With the force cap, the overlay grip stays at 9–11 N and both LEDs stay red the whole time while squeezing,
  with no flicker to yellow (red starts at 97 %; the held grip sits at 98.9–99.7 % of the cap).
  The same holds **while the wrists rotate**: the bar stays in the hand, palm down too, and the grip
  stays at 9–11 N (in the test, 9.91–10.10 N while palm up → down → up).
- With the angle limit, the fingers stop at 80 % and the LEDs are red at the fist.
- Each LED follows its own hand.

## 9. Reach and hold task (both hands at once)

```
obj on
limit angle 100
limit force 10
scale 1
filter 3
task
```

Each hand gets a yellow 2 cm sphere. Move the palm into it (it turns green while holding), grip the
bar at 4 N or more (40–100 % of the cap) and hold for 3 s. That's 5 targets per hand. The overlay
shows `task k/5`, the distance in mm and the hold time. `task stop` ends early.

**Pass:**
- W3 prints `TASK left: held 5 of 5 ...` and `TASK right: held 5 of 5 ...`.
- A JSON file appears in `hands_tasks\`. Note the mean reach time, the mean steadiness (mm), breaks and cap overshoots.

Then repeat with `scale 0.3`, and optionally with `filter off`, and compare the steadiness numbers
(lower is steadier). Quote them with their target counts.

## 10. Pick up a free object

A table stands 10 cm below the hands' home. One **free** object is on it, between the hands: it has
real physics and is not attached to a hand. It starts as a can, 6 cm wide, 10 cm tall, 150 g. In this
mode, the grip force on the overlay, for the LED and for the force cap is the finger + thumb contact force
on the object. The LEDs follow the same rules (yellow from 80 %, red from 97 %). The bars are switched off.

- **One hand** lifts it with pure physics.
- **Two hands:** pure physics did not work. The two hands' fingers push the object out of the grip as the
  lift starts. So a **two-hand grasp-assist** is used: when both hands press the object (each at least
  1 N, with at least 2 of fingers / thumb / palm touching, for 0.15 s), the object is linked to both
  hands. It is released as soon as either hand lets go. The overlay prints `two-hand assist: ATTACHED`.
  `pick assist off` turns it off.

```
limit angle 100
limit force 10
scale 1
pick can          (bars off, can in the middle of the table)
pick              (starts a timed trial)
```

How to grasp (one hand):
- Turn the palm to face the can (right hand: palm to the left; left hand: palm to the right).
- Bring the palm to its side at mid height, with the can level with the palm centre or slightly
  toward the wrist. Don't hold it at the fingertips.
- Make a fist, then lift.

With both hands, hold it between the two palms the same way, then close both hands. In the scripted test
the cube lifted with both hands only when it sat 2 cm deeper in the hands. At a 5 N cap, the can also
needs to sit 2 cm deeper in the hand.

The bottom-left panel shows `OBJECT`: the shape, the state (ON TABLE / GRASPED / LIFTED x/2 s / DROPPED /
FELL OFF THE TABLE), the height above the table, which hands touch it, the grip and the trial time.
Each hand's panel shows `object TOUCHING` and its own grip on it.

1. **One hand:** grasp and lift the can with the right hand, then `pick reset` and do it with the left.
2. **Drop:** lift it, then open the hand.
3. **Two hands:** `pick reset`, then grasp and lift with both hands.
4. `pick cube`, `pick ball`: repeat step 1 with each shape. `s` prints the status. For `pick egg`, see below.

**Pass:**
- Each lift: the terminal prints `[pick] TRIAL SUCCESS ...` once the object has been 5 cm above the table for 2 s.
  A JSON file appears in `hands_tasks\` (`*_pick.json`, with the time to touch and lift, the hands,
  `assisted`, the peak grip and drops).
- While the object is held at the 10 N cap, the grip shows 9–11 N and the LED is red. When the lift stops,
  the grip may spike briefly: the scripted ball lift reached 16–18 N for about 0.1 s.
- Opening the hand while lifted: the state goes `DROPPED` and the object falls back onto the table, without exploding or flying off.
- Two hands: `two-hand assist: ATTACHED` appears, and the trial JSON says `"assisted": true`.
- No finger passes through the object, and the object doesn't shake or creep while held.
- W3 keeps printing `realtime` between 0.95 and 1.05.

### The egg (`pick egg`)

This is the egg from `egg_scene.xml`: 4.4 × 5.8 cm, 60 g, cream coloured. It stands upright in a small
egg cup in the middle of the table, so it can't roll. It breaks like in `egg_sim.py`, at the same
threshold: when **one hand's** finger + thumb force on it goes above **30 N**, it turns red, the state is
`BROKEN`, and a running trial ends as `TRIAL BROKEN`. It turns yellow from 15 N (`yellow_n` in
`assist.py`). `pick reset` or `pick egg` brings back a whole egg.

The egg is small. Hold it **near the finger roots**, at the front edge of the palm, not in the middle of
the palm. Centred on the palm, the fingers close past it without wrapping it and it drops. In the
scripted test the palm centre was 2 cm behind the egg.

Safe force caps, from scripted lifts (peak grip including the brief overshoot; the egg breaks above 30 N):

| `limit force` | peak, one hand | peak, two hands | egg |
| --- | --- | --- | --- |
| 5 | 5.4 N | 6.8 N | intact |
| 10 | 11.0 N | 14.1 N | intact (**recommended**: more than 2× margin) |
| 15 | 17.8 N | 21.5 N | intact, but a fast real squeeze may overshoot more |
| 20 | 24.8 N | 30.4 N | **breaks** with two hands |
| 0 (off) | 140 N | | **breaks** |

Use 10 N or less. 5 N still lifts the 60 g egg.

```
limit force 10
pick egg
pick
```

1. Grasp the egg with the right hand (palm facing it, egg at the finger roots), lift it 5 cm, hold 2 s.
   `pick reset`, then do it with the left hand, then with both hands.
2. `limit force 0`, `pick reset`, `pick`, then squeeze the egg hard.
3. `pick reset`, then `limit force 10` again.

**Pass:**
- Step 1: `TRIAL SUCCESS` for each hand and for both hands. The egg stays cream or yellow, never red.
  The trial JSON says `"broken": false`.
- Step 2: the egg turns **red**, the overlay shows `BROKEN`, the terminal prints `EGG BROKEN: ... squeezed
  ... N (> 30 N)` and `TRIAL BROKEN`, and the JSON says `"broken": true`.
- Step 3: a whole cream egg stands in the cup again.

`pick off` leaves pick mode. `obj on` also leaves it and brings the bars back.

## 11. Quit

Type `q` in W3.

**Pass:** both ring LEDs turn off, and W3 prints `realtime ratio over N s` with values between 0.95 and 1.05.
