# Robot Dog — Project Reference

A quadruped robot dog. Four legs, each a **parallel five-bar linkage** driven by
two servos, plus a per-leg **torso (roll/abduction) servo**. Controlled by a
**Pimoroni Servo 2040** board running **MicroPython**. 12 servos total.

This document is the full state of the project as of the last working session,
written as a handoff. Everything below has been verified on hardware unless
explicitly marked as an assumption to check.

---

## 1. Hardware

### Controller
- **Pimoroni Servo 2040** (RP2040-based, 18 servo channels, headers labelled 1–18).
- Runs **Pimoroni's MicroPython build** (the `pico-...-pimoroni-micropython.uf2`
  image — NOT vanilla MicroPython, or the `servo` library import fails).
- The board's `servo` module maps `servo2040.SERVO_1` … `SERVO_18` to **indices
  0–17**. So header N = cluster index N-1.

### Power
- **6V, 6A** from a DC bench supply into the board's screw terminals (rated 10A,
  reverse-polarity protected).
- The **"Separate USB and Ext. Power" trace on the back of the board IS CUT.**
  Required because 6V > 5V would otherwise damage the RP2040. Consequence: **the
  board needs USB connected to boot** — logic power no longer comes from the
  terminals.
- Dev setup is 3 wires: USB-C to laptop (logic + programming), and +/- from the
  bench supply (servo power). Because the trace is cut, the supply cannot
  back-feed the laptop, so both can stay connected.
- Recommended: set the supply current limit to ~6A so a jammed servo makes the
  supply fold back instead of cooking. A bulk capacitor (1000µF+) across the
  terminals helps absorb startup spikes.
- The board has onboard voltage + current sensing (via the analog mux) — not yet
  used, but ideal for checking real draw during a gait.

### Servo wiring / channel map
```
Header  Index  Role
1       0      Leg 1 LEFT motor
2       1      Leg 1 RIGHT motor
3       2      Leg 2 LEFT motor
4       3      Leg 2 RIGHT motor
5       4      Leg 3 LEFT motor
6       5      Leg 3 RIGHT motor
7       6      Leg 4 LEFT motor
8       7      Leg 4 RIGHT motor
9       8      Leg 1 TORSO (roll)
10      9      Leg 2 TORSO (roll)
11      10     Leg 3 TORSO (roll)
12      11     Leg 4 TORSO (roll)
```
Convention: within each leg pair, **odd header = left motor, even = right.**

> NOTE / TO VERIFY: which physical corner each leg (1–4) occupies, and which end
> of the body is the FRONT, was never nailed down. Needed before a real gait
> (see §6). The single-leg IK was originally derived with "index 0 = right", but
> the current wiring is "index 0 (odd header) = left" — so the IK's R/L
> assignment must be swapped when integrating (see §4).

---

## 2. Servo calibration

### Pulse widths
All servos use a custom `Calibration` reproducing the pulse widths the earlier
PCA9685 setup produced:
```python
cal = Calibration()
cal.apply_two_pairs(610, 2441, -90, 90)   # 610us -> -90deg, 2441us -> +90deg
```
The Pimoroni library works on a **-90..+90** scale. All project code works
internally on a **0–180** scale and converts at the hardware boundary:
`cluster.value(index, angle - 90.0, load)`.

> These pulse numbers were inherited from the PCA board, NOT measured on the
> current servos. They are "good enough" but not verified. If precision matters,
> sweep each servo's real endpoints and update.

### Leg servos
- Home / neutral = **90°** on both motors of a pair.
- At 90/90 the two upper links should sit symmetric, **45° either side of
  vertical, 90° apart.** (TO RE-VERIFY after any reassembly — if a horn is
  reseated wrong, every foot position inherits the error.)
- Safe range: **20°–160°** per motor.

### Torso servos — HAND-CALIBRATED, both ends
The torso horns were seated with offsets, so 90° is NOT vertical. Real values
found by hand:

```
Torso  Index  STAND (vertical)  FLAT (rest)   Safe range
9      8      60                160           60–160
10     9      110               10            10–110
11     10     110               5             5–110
12     11     70                170           70–170
```

Critical facts about the torsos:
- **They physically power on at FLAT**, not stand. Code must seed each at its
  flat angle before/at enable, or the first command snaps them.
- **90° for all = "stand up like a normal dog"** was the DESIGN INTENT, but due
  to horn offsets the real stand angles are 60/110/110/70 above.
- Torsos 9 & 12 are mirrored vs 10 & 11 (opposite sides of the body), so "toward
  flat" is decreasing angle for 9/12 and increasing for 10/11. Working in a
  flat→stand fraction (0.0=flat, 1.0=stand) handles this automatically.
- **If a torso is commanded past its stop, the legs physically clash.** This
  happened twice during bring-up. Every torso command MUST be clamped to its
  safe range above. Never command a torso to 0/180.

> TO VERIFY: the flat values (160/10/5/170) are assumed to be at or just inside
> the mechanical stop. If seeding at flat makes a servo strain, back that value
> off ~5° toward stand.

---

## 3. Leg geometry & inverse kinematics

### Dimensions (mm)
```
L1 (upper link, pulley axis -> knee) = 30
L2 (lower link, knee -> foot ball)   = 120
HIP_SPACING (centre-to-centre of the two pulley axes) = 46
```

### Coordinate frame (per leg)
- Origin: midpoint between the two pulley axes of that leg.
- **+x toward the servo-0/left side**, **+y straight DOWN** (foot depth is a
  positive number).
- Home pose (90/90) puts the foot at **(0, 132.77)**.

### The IK method
A five-bar with both lower links meeting at the foot = **two independent 2-link
arms sharing the foot point.** Solve each side by circle intersection (law of
cosines). No iteration.

For one side: given hip H and foot F, with D = |F−H|:
```
phi = atan2(F-H direction from vertical) + acos((L1² + D² − L2²) / (2·L1·D))
```
The `+acos` branch selects the **knees-splayed-outward** configuration (matches
the build). Then convert link angle φ to a servo command:
```
servoR = degrees(phi_R) + 45
servoL = 135 - degrees(phi_L)
```
This mapping assumes: at 90/90, links are 45° off vertical; increasing the right
servo and decreasing the left both RAISE the foot.

### Reachable workspace (at x=0, WITH the 20–160 servo clamp)
- Depth range roughly **96.5 to 148 mm**.
- Full x extent is ±82 mm, but vertical room shrinks fast off-centre
  (±40 → ~29mm tall; ±80 → ~2mm, unusable).
- `acos` domain gives two hard limits: D > L1+L2 = 150 (fully straight) and
  D < |L2−L1| = 90 (can't fold further). `legIK` returns None outside these.
- **Motion is mushy near full extension** (D near 150): tiny foot moves need huge
  servo swings (arccos derivative blows up). Keep trajectories in the ~110–140
  depth band where response is well-conditioned.

---

## 4. Walking gait (single leg, proven)

Phase-driven, non-blocking. Phase 0.0→1.0 = one step cycle:
- **Stance** (first `STANCE_DUTY` of cycle): foot planted, sweeps front→rear at
  constant depth. This pushes the body. MUST be a flat line (no vertical wobble).
- **Swing** (rest): foot lifts in a sine arc and returns to the front.

Current tuned parameters (verified to stay inside servo limits):
```
STANCE_Y   = 138    # foot depth while planted
LIFT       = 20     # swing arc height
STRIDE     = 22     # half step length (foot swings -22..+22)
STANCE_DUTY= 0.6
CYCLE_MS   = 1200
```
Larger strides work too (verified: stance 132 / lift 18 / stride 40 keeps servos
45–135). Longer stride is the efficient way to add speed; shortening CYCLE_MS
costs servo rate proportionally.

### Known issues / TODO on the gait
- **R/L SWAP:** `leg_walk_servo2040.py` defines `R=0, L=1` (from the old
  single-leg assumption). Current wiring is odd-header=left, so this must become
  `L=0, R=1` per pair when building the 4-leg version.
- **Eased swing profile NOT yet added.** At the stance→swing transition x
  reverses instantly → a servo-rate spike (~538°/s at stride 38) and foot scrub
  at touchdown. Easing the swing endpoints would cut the peak and reduce slip.
  This is the top pending improvement.
- Peak servo rate at aggressive strides may exceed the servos' spec (~0.11s/60°
  needed). 6V (vs 5V) buys ~20% more speed/torque, which helps.

---

## 5. Files in this project

| File | What it is |
|------|-----------|
| `full_test_12.py` | **Main bring-up test.** All 12 servos: seeds torsos at flat + legs at 90, wiggles each leg while flat (unloaded), stands up, lies down. All channels clamped to safe ranges. |
| `leg_walk_servo2040.py` | Single-leg IK + walking gait (servo2040/MicroPython). Needs R/L swap for current wiring. |
| `torso_calibrate_one.py` | Interactive REPL helper to find a torso's true vertical. Set `WHICH`, type angles until vertical. This is how 60/110/110/70 were found. |
| `torso_test.py` | Torso-only test (4 servos), calibrated, seeds at flat and ramps. |
| `leg_test_4legs.py` | 8 leg servos, per-leg wiggle, no IK. Confirms pairing + left/right. |
| `servo_test_4.py` | Simple 4-servo sweep. |
| `robot_dog_leg_ik.ino`, `robot_dog_leg_walk.ino`, `robot_dog_servos.ino` | **OBSOLETE** — original Arduino/PCA9685 versions. Kept for reference only; the project is now MicroPython on the Servo 2040. |

The MicroPython files run via Thonny or VS Code (MicroPico extension). Saving a
file as `main.py` on the board makes it autorun on power-up — only do this after
testing interactively, since a bad `main.py` can lock up the board.

---

## 6. Immediate next steps

1. **Verify leg-to-corner mapping and front direction.** Which of legs 1–4 is
   front-left/front-right/rear-left/rear-right, and which body end is front.
   Everything about a real gait depends on this.
2. **Build the 4-leg coordinated gait.** The single-leg gait already takes phase
   as an argument. A trot = diagonal pairs in antiphase: legs at one diagonal at
   phase p, the other diagonal at p+0.5. Apply the R/L swap (§4).
3. **Fold in the torso servos.** They roll each leg's plane sideways, turning the
   2D foot (x,y) into a 3D position. The five-bar IK is unchanged; the torso
   angle just tilts the plane (a coordinate transform on top). For basic forward
   walking the torsos can simply hold at STAND.
4. **Add eased swing profile** to the gait (§4).
5. **Add collision clamps generally** — the torso safe ranges are the model;
   confirm what physically collides and at how many degrees for each.
6. **Measure real current** during standing and walking via the onboard sensor,
   to confirm 6A is adequate before going untethered.

## 7. Hard safety rules (learned the hard way)

- **Never command a torso outside its calibrated range** (§2). Past the stop =
  legs clash and stall. Clamp every torso command.
- **Torsos start at flat.** Always seed them at their flat angle before enabling,
  and ramp — never snap.
- **Servos have no position feedback.** Code only knows where it last commanded.
  After any manual repositioning with power off, the first powered move must go
  to a known safe angle before anything relies on `current` position.
- **Keep a hand on the servo-power switch** on first run of anything new.
- Power the servo rail OFF when touching the linkage or reflashing; leave USB on.
