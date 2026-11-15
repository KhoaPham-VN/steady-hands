# Glove session checklist (Steady Hands)

(Glove but no Coil Pro? Use `GLOVE_ONLY_SESSION.md` instead: keyboard position, glove pinch, LED test.)

Everything below needs real gloves and the Coil Pro. Work through it top to bottom. Each step has
the commands, where to type them, and what counts as a pass. If a step fails, stop and note what
you saw; do not edit code during the session.

Written 14 November 2026. All code for phases 1–3 is finished and was tested without gloves
(`replay_csv.py`, `fake_rgmp_server.py`, `test_end_to_end.py`: 11 of 11 checks passed).
Run every command in PowerShell from the `rokoko-teleop` folder:

```powershell
cd $env:USERPROFILE\OneDrive\Documents\rokoko-teleop
```

**Who does what:** the *operator* wears the gloves; the *driver* types at the laptop.

## 0. Before the gloves go on (no gloves needed, 3 minutes)

```powershell
python test_end_to_end.py
```

**Pass:** the last line is `11 of 11 checks passed`. This proves the sim, the assist and
`report.py` still work on this laptop. Then close any `replay_csv.py` or `fake_rgmp_server.py`
windows: they use port 12276, which the real driver needs.

## 1. Start the chain (four windows)

| Window | Command | What it is |
| --- | --- | --- |
| W1 | `rkk-hand-solver --emit-openxr --listen 0.0.0.0:12277` | Solver; starts the glove driver (TONG_KET section 6.1) |
| W2 | `python glove_bridge.py --no-rcsp --rate 83.33 --udp 127.0.0.1:9870,127.0.0.1:9873` | Bridge. **Type `e` + Enter here to engage / disengage (clutch).** |
| W3 | `python egg_sim.py` | The sim; the MuJoCo window opens. **All sim commands are typed here.** |
| W4 | (used in step 6 only) | Recorder |

Notes:
- `--rate 83.33` matters: the bridge defaults to 60 Hz.
- `--no-rcsp` as in the runbook: the glove button and LED have not been tested on real gloves.
  (They worked against `fake_rgmp_server.py`. Optional test at the end, step 11.)
- After switching on the Coil: shake each hand once, then turn the hands a few times.
- The operator stands **0.5–1.1 m from the Coil**. Tracking degrades beyond about 1.1 m.

**Pass:**
- W2's status line shows `right: ... pos=` with numbers, not `coil:--`.
- W3 prints once a second: `realtime` between 0.95 and 1.05, and `bridge` about 83 msg/s.

## 2. Pinch calibration (`po` / `pc`)

The bridge sends `pinch_m`, the thumb-to-index sensor distance in metres. It is not a 0–1 value,
so the sim needs the operator's open and pinched values.

1. Operator holds the right hand open and relaxed. Driver types in W3: `po`
2. Operator touches thumb to index and holds. Driver types in W3: `pc`
3. Operator opens and closes a few times. Driver types `s` in W3 each time.

**Pass:**
- `po` prints a larger value than `pc`, at least 0.02 m apart. Both are saved in `pinch_calib.json`.
- `s` shows `travel` near 0 mm with the hand open and near 40 mm when pinched.
- Write both values down.

## 3. Axis direction and scale check

1. In W2 type `e` (engage). In W3 type `r` (reset; the gripper anchors to the current hand pose).
2. Operator moves the right hand slowly, one direction at a time, about 10 cm each:

| Hand moves | Gripper must move (on screen) |
| --- | --- |
| Forward (away from the operator) | Away from the camera |
| To the operator's left | To the screen's left (towards the green target) |
| Up | Up |

3. For scale: driver types `s`, the operator moves the hand 10 cm forward (use a ruler), and the
   driver types `s` again.

**Pass:**
- All three directions match the table.
- The `target` x value changes by 0.08–0.12 m for 10 cm of hand travel.

**If an axis is reversed:** note which one and stop. The bridge's `--coil-to-robot y,x,-z` mapping
was measured at the team's old standing spot (TONG_KET section 4). It has to be measured again,
not guessed.

## 4. Phase 1: five attempts with assist off

W3 shows `Assist OFF` on the overlay. Logging is off (practice), which is right for this step.

For each attempt: driver types `r` in W3. Operator picks up the egg, puts it on the green disc and
opens the hand. The egg must rest for 0.5 s with the gripper open before the trial counts as placed.

**Pass (plan, phase 1):**
- `realtime` stays within 0.95–1.05 throughout.
- Each of the 5 attempts ends with `SUCCESS`, `BROKEN` or a placement message in W3, not a timeout
  caused by tracking trouble.
- Write down the 5 outcomes and peak forces.

If the gripper is jumpy: move the operator to 0.5–1.1 m from the Coil before anything else. If the
Coil position jumps or mirrors: restart the driver with `--coil-half-space`, as in
`glove-teleop-bringup.md`.

## 5. Phase 2: assist check

Driver types `a` in W3 (assist on; the overlay says `Assist ON`, `Grip cap 5 N`). Run 2–3 attempts with `r`.

**Pass:**
- The egg turns **green** when gripped, and would turn yellow at 15 N or more.
- The overlay shows `x0.3 (precision)` when the finger tips are within about 8 cm of the egg or the
  target. It shows `x1.0 (travel)` while carrying the egg.
- A blue ball shows the raw one-to-one hand position next to the gripper.
- **Full pinch** on the egg does not break it. Peak about 5–7 N, against 30 N or more with assist off.
- **Toggle test:** while the hand is moving, type `a` several times. The gripper must not jump.
  (Toggling mid-trial flags that trial; that is expected.)

Type `a off` when done.

## 6. Jitter recording for filter tuning (W4)

```powershell
# W4
python udp_monitor.py --port 9873 --record docs\team_glove.jsonl
```

Operator stands 0.5–1.1 m from the Coil, right hand at grasp height, for **60 s**: 20 s still,
20 s slow movement, 20 s still. Stop W4 with Ctrl+C. Then:

```powershell
python eval_offline.py docs\team_glove.jsonl --cutoffs 1,2,3,4,5 --betas 0
```

**Pass:**
- The file has more than 4,500 lines.
- `eval_offline.py` prints a table with a "still" share of at least 30%.
- Keep the table: it is the source for the jitter numbers in the pitch.

On the organizers' recording (1.1–1.5 m from the Coil), 3 Hz removed only 3.5% of the jitter and
1 Hz removed 33% at 151 ms lag (TONG_KET section 14). Choose the cutoff in step 8.

## 7. Phase 3 dry run with gloves

In W3:

```
l            (logging ON)
op NAME      (operator name or initials)
a off
r            ... do the trial; then r again for trial 2
a on
r            ... trial 3; r ... trial 4
l            (logging OFF)
```

Then:

```powershell
python report.py --dir trials --out results_dry
mkdir trials\dryrun; Move-Item trials\*.jsonl trials\dryrun\
```

**Pass:**
- `results_dry.md` shows 2 trials per condition.
- One log file checked by hand (send it to Claude) matches its row: result, peak force,
  placement error, time.
- The dry-run logs are moved to `trials\dryrun\`, so they cannot mix into phase 4.
  (`report.py` reads only `trials\*.jsonl`, not subfolders.)

---

## 8. Phase 4 pilot: tune, then freeze (logging OFF)

Practice attempts only (`l` must be off). Tune in this order and write each final value down:

| Parameter | Where | Start | Tune until |
| --- | --- | --- | --- |
| Finger stiffness `finger_kp` | top of `egg_sim.py` (`SIM_PARAMS`) | 3000 N/m | Assist off breaks the egg in **at least 2 of 10** pilot attempts (plan, section 10). If fewer break, raise it. |
| `cap_on_n` / `cap_off_n` | top of `assist.py` | 5 / 60 N | Assist on lifts the egg reliably |
| `scale_precision` | `assist.py` | 0.3 | Operator can place within 20 mm without it feeling stuck |
| `filter_min_cutoff_hz` | `assist.py` | 3.0 Hz | From step 6's table: the most reduction with lag the operator does not notice |

After any change, restart W3 (`q`, then `python egg_sim.py`).

**Freeze:**

```powershell
python assist.py              # prints the final ASSIST_PARAMS: copy into TONG_KET
python test_end_to_end.py     # must still say 11 of 11 checks passed
python test_assist.py         # filter tests assume about 3 Hz: if you chose another cutoff, note which tests fail and why
```

**Pass:**
- The values are written down.
- `test_end_to_end.py` passes.
- From here on, **no parameter changes.** If one has to change after logging starts, move the
  logged trials to `trials\discarded\` and start the logged trials again (plan, phase 4).

## 9. Phase 4: logged trials (per operator)

**Order:** 5 unlogged practice attempts per condition first, then 10 logged trials per condition
in blocks of five: **off ×5, on ×5, on ×5, off ×5**.

```
# practice (logging OFF)
a off      then r ... 5 attempts
a on       then r ... 5 attempts
# logged
op NAME
l          (logging ON; W3 prints "logging trial N -> trials\...")
a off      then r ... 5 trials
a on       then r ... 10 trials
a off      then r ... 5 trials
l          (logging OFF)
```

Rules during logged trials:
- Never type `a` during a trial. Switch only between trials, before `r`. A switch mid-trial flags
  the trial and `report.py` excludes it.
- Type `r` only after the previous trial has ended (`SUCCESS`, `BROKEN`, placed, dropped or
  timeout). An `r` during a trial logs it as aborted, and `report.py` excludes it.
- If tracking fails mid-trial (Coil jump), finish or abort the trial and note its number.

Optional third condition (if time allows): `a grip` = grip limiter only, 10 logged trials.

```powershell
python report.py
```

**Pass:**
- `results.md` shows 10 trials per condition for each operator, and the "Excluded" list is empty
  or explained.
- `results.md`, `results.csv` and `results_trials.csv` exist.
- Every number in the pitch comes from these files, quoted with its trial count.

## 10. Hand the results back

Send Claude:
1. The pinch values (step 2).
2. Axis check result (step 3).
3. Phase 1 outcomes (step 4).
4. `docs\team_glove.jsonl` and the `eval_offline.py` table (step 6).
5. Frozen parameters (step 8).
6. `results.md` (step 9).

## 11. Optional: glove button and LED (RCSP)

Only if time is left. The bridge saves calibration on button presses, so use a copy of
`calib.json`. That file must not change.

```powershell
Copy-Item calib.json calib_rcsp_test.json
python glove_bridge.py --rate 83.33 --udp 127.0.0.1:9870 --calib calib_rcsp_test.json
```

**Pass:**
- A single press on the right glove toggles `ENGAGED` in the bridge window.
- The ring LED turns green when engaged and blue when idle.
- In `egg_sim.py` the gripper holds still while disengaged and does not jump on re-engage.
