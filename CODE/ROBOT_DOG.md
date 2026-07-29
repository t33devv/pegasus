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
- **Never physically disconnect USB** — the board loses power and dies. "Disconnect"
  always means closing VS Code / MicroPico software only.
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
> of the body is the FRONT, was never nailed down. The teleop and balance code make
> assumptions (legs 1&2 = front, 3&4 = back) that need physical verification.

### Sensors

#### VL53L0X — Time-of-Flight distance sensor
- I2C address **0x29**.
- Wiring: VCC→3V3, GND→GND, SDA→SDA, SCL→SCL. XSHUT and GPIO1 unconnected.
- USB power alone is sufficient; 6V servo rail does NOT need to be on.
- Requires a VL53L0X MicroPython driver uploaded to the board (`vl53l0x.py` or
  `VL53L0X.py` — `tof_read.py` tries both import names automatically).
- I2C bus/pin auto-detected at runtime using the same scan logic as `imu_read.py`.

#### MPU6050 — 6-DOF IMU (accelerometer + gyroscope)
- I2C address **0x68** (AD0→GND, the default). Use 0x69 if AD0 is pulled high —
  change `MPU_ADDR` in the relevant file.
- Wiring: VCC→3V3, GND→GND, SDA→SDA, SCL→SCL, AD0→GND.
- USB power alone is sufficient.
- **No external driver needed** — all code uses raw I2C register reads.
- I2C bus/pin auto-detected at runtime.

**Confirmed axis mapping** (sensor as physically mounted on this robot):
```
Physical motion       Accel axis   Gyro axis
Side-to-side (roll)   ax           gy
Front-to-back (pitch) ay           gx
```
If the sensor is remounted, re-verify these with `imu_read.py` before running `balance.py`.

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
- **Indices 8 & 10 are one physical side of the body; 9 & 11 are the other.**
  This is used by the lean/balance code: opposite signs on each side produce a
  lateral lean.
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
- Full x extent is ±82 mm, but vertical room shrinks fast off-centre.
- `legIK` returns None outside the reachable range — all callers must check.
- **Keep trajectories in the ~110–140 mm depth band** where response is
  well-conditioned. Motion near full extension (D→150) is mushy.

---

## 4. Walking gait

Phase-driven, non-blocking. Phase 0.0→1.0 = one step cycle:
- **Swing** (first 40%): foot lifts in an eased sine arc, steps forward.
- **Stance** (last 60%): foot planted at constant depth, sweeps rear→front to push body.

4-leg trot: diagonal pairs in antiphase.
- **Pair A**: legs 1 (idx 0,1, fwd=-1) + leg 4 (idx 6,7, fwd=1) at phase `p`
- **Pair B**: legs 2 (idx 2,3, fwd=1) + leg 3 (idx 4,5, fwd=-1) at phase `p+0.5`

Tuned parameters used across `crawl_gait.py`, `teleop_dog.py`, `balance.py`:
```
STANCE_Y    = 132   # foot depth while planted (mm)
LIFT        =  18   # swing arc height (mm)
STRIDE      =  40   # half-stride length (foot sweeps -40..+40 mm)
STANCE_DUTY = 0.6
CYCLE_MS    = 1200
```

The `fwd` parameter mirrors the x axis for legs facing the opposite end of the
body. For static poses (balance, tripedal) x=0 so fwd doesn't matter.

---

## 5. Files in this project

| File | What it is |
|------|-----------|
| `crawl_gait.py` | 4-leg diagonal-pair trot. Runs a fixed number of cycles then stops. |
| `all_dog_test.py` | Full demo: walk forward, torso twist, tripedal balance, walk backward. |
| `torso_calibrate_one.py` | Interactive REPL helper to find a torso's true vertical angle. |
| `torso_test.py` | Torso-only test — seeds at flat and ramps to stand. |
| `leg_test_4legs.py` | 8 leg servos, per-leg wiggle, no IK. Confirms pairing + left/right. |
| `full_test_12.py` | All 12 servos bring-up test. Stand up, wiggle, lie down. |
| `servo_test_4.py` | Simple 4-servo sweep. |
| `tof_read.py` | **VL53L0X test.** Auto-detects I2C bus, reads distance in mm. Run to verify sensor wiring before using teleop. |
| `imu_read.py` | **MPU6050 test.** Auto-detects I2C bus, prints ax/ay/az (g) and gx/gy/gz (°/s) at 5Hz. Run to verify sensor wiring and confirm axis directions before running balance.py. |
| `balance.py` | **Active balance.** Stands the dog up, calibrates gyro offsets, then continuously adjusts torso servos (roll) and leg depths (pitch) to keep the body level. See §9. |
| `TELEOP/teleop_dog.py` | **Teleop firmware.** Command-driven gait + ToF obstacle avoidance. Upload as `main.py`. See §8. |
| `TELEOP/main.py` | Exact copy of `teleop_dog.py` — upload this directly to avoid renaming. Always keep in sync with `teleop_dog.py`. |
| `TELEOP/bridge.py` | Laptop bridge: HTTP on port 8080, WebSocket on 8765, forwards to serial. `pip install websockets pyserial`. |
| `TELEOP/index.html` | WASD web controller. Hold key to move, release to stop. |
| `robot_dog_leg_ik.ino`, `robot_dog_leg_walk.ino`, `robot_dog_servos.ino` | **OBSOLETE** Arduino/PCA9685 originals. Reference only. |

The MicroPython files run via VS Code + MicroPico extension.

---

## 6. Immediate next steps

1. **Verify leg-to-corner mapping and front direction.** Which of legs 1–4 is
   front-left / front-right / rear-left / rear-right, and which end of the body
   is front. Affects: `fwd` sign correctness in teleop, FRONT_LEGS/BACK_LEGS in
   `balance.py`, and any future 3D gait work.
2. **Verify balance.py axis directions on hardware.** `ROLL_DIR` and `PITCH_DIR`
   may need flipping, and FRONT_LEGS/BACK_LEGS may be swapped depending on
   physical corner mapping above.
3. **Integrate balance into teleop.** `balance.py` is standalone. A future version
   of `teleop_dog.py` could run the IMU compensation loop in parallel with the
   gait so the dog self-levels while walking.
4. **Add eased swing profile** to reduce servo-rate spike at stance→swing
   transition (see §4).
5. **Measure real current** during standing and walking via the onboard sensor,
   to confirm 6A is adequate before going untethered.
6. **Wireless teleop.** RP2040 has no WiFi. An ESP32 or Pi Zero mounted on the
   robot could host `bridge.py` locally and serve over WiFi for untethered driving.

---

## 7. Hard safety rules (learned the hard way)

- **Never command a torso outside its calibrated range** (§2). Past the stop =
  legs clash and stall. Clamp every torso command.
- **Torsos start at flat.** Always seed them at their flat angle before enabling,
  and ramp — never snap.
- **Servos have no position feedback.** Code only knows where it last commanded.
  After any manual repositioning with power off, the first powered move must go
  to a known safe angle before anything relies on current position.
- **Never physically disconnect USB** — the trace is cut so USB is the only logic
  power source. The board dies instantly. "Disconnect" always means software only.
- **Keep a hand on the servo-power switch** on first run of anything new.
- Power the servo rail OFF when touching the linkage or reflashing; leave USB on.

---

## 8. Teleop system

### Architecture
```
Browser (WASD keys)
    ↓  WebSocket  ws://localhost:8765
bridge.py on laptop
    ↓  USB serial  115200 baud
teleop_dog.py on Servo 2040
```

The RP2040 has no WiFi. The laptop bridges between the browser and the board over
the existing USB cable.

### Commands (single ASCII bytes)
| Key | Byte | Behaviour |
|-----|------|-----------|
| W | `f` | Walk forward (blocked if ToF < 10 cm) |
| S | `b` | Walk backward |
| A | `l` | Spin left |
| D | `r` | Spin right |
| Space | `s` | Stop — snap legs to home stance |

### Spin / turn implementation
- `fwd=1` legs (2 & 4, indices 2-3 and 6-7): one physical side
- `fwd=-1` legs (1 & 3, indices 0-1 and 4-5): the other side

Spin drives the two sides in opposite directions while keeping the diagonal phase
structure (pair A at `p`, pair B at `p+0.5`). L/R were swapped once in `CMD_MAP`
after physical testing to get the correct direction.

### ToF obstacle avoidance
- VL53L0X polled every **5 ticks (100 ms)**.
- If distance < **100 mm (10 cm)**: `WALK_FWD` is silently blocked; legs freeze.
- Backward and spin are always allowed.
- If sensor is missing or driver not found: prints warning, teleop continues
  without obstacle avoidance.
- Threshold constant: `OBSTACLE_MM = 100` in `teleop_dog.py`.

### Workflow
1. Open `TELEOP/main.py` in VS Code.
2. Cmd+Shift+P → **"MicroPico: Upload current file to Pico"**.
3. In the MicroPico terminal press **Ctrl+D** to soft-reset — `main.py` autoruns.
4. **Close VS Code** (releases serial port; USB stays in, board stays powered).
5. `python3 TELEOP/bridge.py` in a terminal.
6. Open `http://localhost:8080`.

Stop: press S/Space on the page → Ctrl+C in the terminal.
Reflash: close `bridge.py` → reopen VS Code → MicroPico reconnects.

---

## 9. Balance system (MPU6050)

### What it does
`balance.py` stands the dog up, auto-calibrates gyro offsets (2 s at rest), then
runs a 20 ms control loop that:
- **Roll** (side tilt) → adjusts all 4 torso servos to lean the legs and
  counteract the tilt.
- **Pitch** (front/back tilt) → adjusts front leg depth and back leg depth in
  opposite directions to level the body front-to-back.

This is a **reactive proportional controller**, not learning. It continuously
compensates but does not eliminate steady-state error (a small residual tilt
remains proportional to the gain).

### Sensor fusion — complementary filter
Both axes use the same filter:
```
angle = ALPHA * (angle + gyro_rate * dt) + (1 - ALPHA) * accel_angle
```
- Gyro integrates fast motion accurately but drifts over time.
- Accelerometer gives absolute angle but is noisy.
- `ALPHA = 0.95` balances speed vs smoothness. Increase toward 0.98 if it
  oscillates; decrease toward 0.90 if it reacts too slowly.

### Gyro calibration
At startup, `_CAL_SAMPLES = 100` readings are averaged over ~2 s to measure the
gyro's zero-rate offset. **Keep the dog completely still during this window.**
The offsets are applied to every subsequent reading.

### Tuning constants
```python
ROLL_GAIN  = 0.022   # torso fraction per degree of roll
MAX_LEAN   = 0.25    # max torso fraction offset (same as all_dog_test.py twist max)
ROLL_DIR   = 1       # flip to -1 if roll compensation goes the wrong way

PITCH_GAIN = 1.0     # mm of leg depth change per degree of pitch
MAX_PITCH  = 15.0    # max depth offset (mm) — STANCE_Y ± 15 stays within IK range
PITCH_DIR  = 1       # flip to -1 if pitch compensation goes the wrong way
```

### Leg grouping for pitch (assumed — verify with corner mapping)
```python
FRONT_LEGS = [(0, 1), (2, 3)]   # legs 1 & 2
BACK_LEGS  = [(4, 5), (6, 7)]   # legs 3 & 4
```
If the dog pitches the wrong way, either flip `PITCH_DIR` or swap FRONT/BACK.

### Axis mapping (confirmed for current mounting)
```
Physical motion       Accel axis   Gyro axis
Roll (side-to-side)   ax           gy
Pitch (front/back)    ay           gx
```
If the IMU is remounted, re-run `imu_read.py`, tilt the robot each way, and
confirm which raw axis changes before running `balance.py`.

### Limitations
- Pitch compensation changes leg depth only — it cannot fix large pitch angles
  if the leg hits the IK range limit (~96–148 mm depth).
- No integral term: small steady-state error remains. Adding a PID integral would
  eliminate it but risks wind-up if not tuned carefully.
- `balance.py` is standalone — not yet integrated with `teleop_dog.py`.
