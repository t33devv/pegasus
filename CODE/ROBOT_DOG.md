<!--
  This file is written for AI assistants (Claude, GPT, Gemini, etc.).
  If you are an AI helping with this codebase, read this document first.
  It is the ground truth for hardware, firmware, and software decisions.
  Everything here has been verified on physical hardware unless marked otherwise.
-->

# Pegasus — Project Reference

A quadruped robot dog named **Pegasus**. Four legs, each a **parallel five-bar
linkage** driven by two servos, plus a per-leg **torso (roll/abduction) servo**.
Controlled by a **Pimoroni Servo 2040** board running **MicroPython**. 12 servos
total. Voice-controlled from a laptop over USB serial.

---

## 1. Hardware

### Controller
- **Pimoroni Servo 2040** (RP2040-based, 18 servo channels, headers labelled 1–18).
- Runs **Pimoroni's MicroPython build** (the `pico-...-pimoroni-micropython.uf2`
  image — NOT vanilla MicroPython, or the `servo` library import fails).
- The board's `servo` module maps `servo2040.SERVO_1` … `SERVO_18` to **indices
  0–17**. Header N = cluster index N-1.

### Power
- **6V, 6A** from a DC bench supply into the board's screw terminals (rated 10A,
  reverse-polarity protected).
- The **"Separate USB and Ext. Power" trace on the back of the board IS CUT.**
  Required because 6V > 5V would otherwise damage the RP2040. Consequence: **the
  board needs USB connected to boot** — logic power no longer comes from the terminals.
- Dev setup is 3 wires: USB-C to laptop (logic + programming), and +/- from bench
  supply (servo power). Because the trace is cut, the supply cannot back-feed the
  laptop, so both can stay connected.
- **Never physically disconnect USB** — the board loses power and dies. "Disconnect"
  always means closing VS Code / MicroPico software only.
- Set the supply current limit to ~6A so a jammed servo makes the supply fold back
  instead of cooking. A bulk capacitor (1000µF+) across the terminals helps absorb
  startup spikes.

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
> of the body is FRONT, was never fully nailed down. teleop and balance code assume
> legs 1&2 = front, 3&4 = back — verify physically.

### Sensors

#### VL53L0X — Time-of-Flight distance sensor
- I2C address **0x29**.
- Wiring: VCC→3V3, GND→GND, SDA→SDA, SCL→SCL. XSHUT and GPIO1 unconnected.
- No 6V rail needed; USB power alone is sufficient.
- Requires a VL53L0X MicroPython driver on the board (`vl53l0x.py` or `VL53L0X.py`).
- I2C bus/pin auto-detected at runtime via `_find_i2c(0x29)`.

#### MPU6050 — 6-DOF IMU (accelerometer + gyroscope)
- I2C address **0x68** (AD0→GND, the default).
- Wiring: VCC→3V3, GND→GND, SDA→SDA, SCL→SCL, AD0→GND.
- USB power alone is sufficient. No external driver needed — raw I2C register reads.
- I2C bus/pin auto-detected at runtime via `_find_i2c(0x68)`.
- **Gyro scale: 131.0 LSB/°/s at ±250°/s default range** (register 0x43, 6-byte burst).

**Confirmed axis mapping (sensor as physically mounted):**
```
Physical motion       Accel axis   Gyro axis
Side-to-side (roll)   ax           gy
Front-to-back (pitch) ay           gx
Yaw (spin in place)   —            gz   ← used for IMU-guided turns
```

**Confirmed gz sign for turns:**
- Left turns  → gz_raw is **positive**  → `GZ_SIGN['L'] = 1`
- Right turns → gz_raw is **negative** → `GZ_SIGN['R'] = -1`

If sensor is remounted, verify signs with `diagnose_imu.py` before assuming.

---

## 2. Servo calibration

### Pulse widths
```python
cal = Calibration()
cal.apply_two_pairs(610, 2441, -90, 90)   # 610us -> -90deg, 2441us -> +90deg
```
The Pimoroni library works on a **-90..+90** scale. All project code works
internally on a **0–180** scale and converts at the hardware boundary:
`cluster.value(index, angle - 90.0, load)`.

### Leg servos
- Home / neutral = **90°** on both motors of a pair.
- At 90/90 the two upper links sit symmetric, 45° either side of vertical.
- Safe range: **20°–160°** per motor.

### Torso servos — HAND-CALIBRATED, both ends
```
Torso  Index  STAND (vertical)  FLAT (rest)   Safe range
9      8      60                160           60–160
10     9      110               10            10–110
11     10     110               5             5–110
12     11     70                170           70–170
```
Critical facts:
- **They physically power on at FLAT.** Code must seed them at flat angle before
  enabling, or the first command snaps them.
- `torso_ramp(from_frac, to_frac, ms)` interpolates between flat (0.0) and stand
  (1.0) using these values. Always ramp, never snap.
- Torsos 9 & 12 are mirrored vs 10 & 11 — "toward flat" is decreasing angle for
  9/12 and increasing for 10/11. The ramp function handles this automatically.
- Indices 8 & 10 are one physical side; 9 & 11 are the other. Used for lateral lean.
- **Past the mechanical stop = legs physically clash.** Always clamp to safe range.
  Never command a torso to 0 or 180.

**`_torso_frac` tracking:** The module-level variable `_torso_frac` tracks the
current torso fraction (0.0=flat, 1.0=stand) so `cmd_stand_up` and `cmd_lay_down`
always ramp from the correct current position, not from an assumed starting point.
It is set to `1.0` right after the startup `torso_ramp(0.0, 1.0, 2500)` call in
`main()`.

---

## 3. Leg geometry & inverse kinematics

### Dimensions (mm)
```
L1 (upper link, pulley axis → knee)  = 30
L2 (lower link, knee → foot ball)    = 120
HIP_SPACING (centre-to-centre)       = 46
```

### Coordinate frame (per leg)
- Origin: midpoint between the two pulley axes of that leg.
- +x toward the servo-0/left side, +y straight DOWN (foot depth is positive).
- Home pose (90/90) puts the foot at **(0, 132.77)**. Code uses `STANCE_Y = 132.0`.

### IK workspace at x=0
- Valid fy range: **~87 mm to ~148 mm** (set by L1, L2, HIP_SPACING).
- `legIK()` returns `None` outside the reachable range — all callers must check.
- For the jump command: `LOW_Y = 146.0` (crouched) and `HIGH_Y = 93.0` (lifted)
  are chosen with small margins from these limits.

---

## 4. Walking gait

Phase-driven, non-blocking. Phase 0.0→1.0 = one step cycle:
- **Swing** (first 40%): foot lifts in an eased sine arc, steps forward.
- **Stance** (last 60%): foot planted, sweeps rear→front to push body.

4-leg diagonal trot — diagonal pairs in antiphase:
- **Pair A**: legs 1 (idx 0,1, fwd=-1) + leg 4 (idx 6,7, fwd=1) at phase `p`
- **Pair B**: legs 2 (idx 2,3, fwd=1) + leg 3 (idx 4,5, fwd=-1) at phase `p+0.5`

Tuned parameters:
```python
STANCE_Y    = 132   # foot depth while planted (mm)
LIFT        =  18   # swing arc height (mm)
STRIDE      =  40   # half-stride length (foot sweeps -40..+40 mm)
STANCE_DUTY = 0.6
CYCLE_MS    = 1200
TICK_MS     =  20
```

---

## 5. Serial command protocol

USB serial at **115200 baud**. The board (firmware) listens; the laptop sends.

### Single-byte motion commands
| Byte | Action |
|------|--------|
| `f`  | Walk forward (blocked if ToF < 100 mm) |
| `b`  | Walk backward |
| `l`  | Spin left (continuous until stopped) |
| `r`  | Spin right (continuous until stopped) |
| `s` / Space | Stop — snap to home stance |

**Important CMD_MAP swap:** `ord('l')` maps to `TURN_R` and `ord('r')` maps to
`TURN_L` in the firmware. This physical swap was confirmed by hardware testing —
do not "fix" it. The labels and the motion are correct end-to-end.

### Multi-byte IMU turn commands
```
L<N>\n   — turn left N degrees   e.g. "L90\n"
R<N>\n   — turn right N degrees  e.g. "R45\n"
```
Accumulated in `_cmd_buf` across ticks. Parsed when `\n` is received.
Returns tuple `('imu_turn', 'L'|'R', degrees)` from `read_cmd()`.

### One-shot action commands (single byte)
| Byte | Action |
|------|--------|
| `U`  | Stand up (torso ramp to 1.0 over 1500ms) |
| `D`  | Lay down (torso ramp to 0.0 over 1500ms) |
| `J`  | Jump (smooth dip-and-rise body motion) |
| `T`  | Fast feet (rapid sequential foot lifts, 4 cycles) |
| `1`  | Lift front-right foot (foot index 0), hold 2s |
| `2`  | Lift front-left foot (foot index 1), hold 2s |
| `3`  | Lift back-right foot (foot index 2), hold 2s |
| `4`  | Lift back-left foot (foot index 3), hold 2s |

All one-shot commands are blocking on the board side and return to STOP mode when
done. The `ONE_SHOT` dict in `main()` maps `ord(byte)` → action tuple.

---

## 6. IMU-guided turns

**Goal:** use the MPU6050 gyroscope to accumulate real rotation angle and stop
exactly when the target is reached, instead of relying on open-loop timing.

**Key implementation decisions (hard-won through debugging):**

1. **6-byte burst read from 0x43** (same register as `balance.py`):
   `i2c.readfrom_mem(MPU_ADDR, 0x43, 6)` → gx, gy, gz as signed 16-bit integers.
   Never read gz alone — always read all 6 bytes in one transaction.

2. **Raw integer offset calibration:** 100 samples at startup (robot must be still).
   `gz_offset_raw` is subtracted from each raw reading BEFORE dividing by scale.

3. **5-sample moving average** (`_GZ_SMOOTH_N = 5`) to reduce gait vibration noise.

4. **Signed gz, not abs(gz):** The critical fix. Using `abs(gz)` caused massive
   over-accumulation because gait vibration produces spikes in BOTH directions and
   `abs` counts them all as real rotation. Using `GZ_SIGN[direction] * gz` means
   wrong-direction spikes SUBTRACT from the buffer average rather than add.
   ```python
   gz_dps = GZ_SIGN[direction] * (sum(gz_buf) / _GZ_SMOOTH_N) / _GYRO_SCALE
   ```

5. **Deadband:** Only accumulate if `gz_dps > GZ_MIN_DPS = 3.0`. Filters standstill
   noise without blocking real rotation signal.

6. **Timeout:** `GYRO_TIMEOUT_FACTOR = 4` × estimated open-loop time. Prevents
   infinite loops if the robot gets stuck.

7. **Gait mode swap in IMU turn:** Because CMD_MAP physically swaps L/R labels,
   `imu_guided_turn` must also swap:
   ```python
   gait_mode = TURN_R if direction == 'L' else TURN_L
   ```

---

## 7. Voice control system

**Architecture:**
```
User speaks
  → laptop mic (sounddevice, 16kHz)
  → faster-whisper (offline STT, 'base' model, ~/.cache/huggingface/)
  → parse_commands() — splits on "and"/"then", parses each clause
  → execute_command() — sends serial bytes to board
  → Pegasus moves
```

**Files involved:**
- `TELEOP/voice_control.py` — the main laptop-side controller
- `TELEOP/.env` — stores `OPENAI_API_KEY` (not currently used — faster-whisper is
  fully offline; the key is kept for future use)
- `TELEOP/.venv/` — Python virtual environment with all deps

**Dependencies (install in `.venv`):**
```
pip install faster-whisper pyserial sounddevice soundfile numpy python-dotenv
```

**Running:**
```bash
cd TELEOP
source .venv/bin/activate
python3 voice_control.py
```
Press Enter to start recording, Enter again to stop. Robot responds.

### Parsed actions

| Spoken phrase (examples) | Action sent |
|--------------------------|-------------|
| "forward", "go forward", "ahead" | `f` byte |
| "backward", "back", "reverse"    | `b` byte |
| "left"                           | `l` byte (or `L<N>\n` if degrees given) |
| "right"                          | `r` byte (or `R<N>\n` if degrees given) |
| "stop", "halt", "freeze"         | `s` byte |
| "stand", "stand up", "get up"    | `U` byte |
| "lay", "lay down", "lie down"    | `D` byte |
| "jump", "leap"                   | `J` byte |
| "fast feet", "quick feet"        | `T` byte |
| "lift [front/back] [right/left] foot" | `1`–`4` byte |
| "turn left 90 degrees"           | `L90\n` (IMU-guided) |
| "walk forward 3 meters"          | `f` + sleep + `s` (open-loop) |

Foot index mapping in `_parse_foot()`:
```
0 = front-right   → byte '1'
1 = front-left    → byte '2'
2 = back-right    → byte '3'
3 = back-left     → byte '4'
```

### Compound commands
`parse_commands()` splits on `\band\b` or `\bthen\b` and executes each clause
sequentially. Each clause waits for the previous to finish before starting.

```
"turn left 90 degrees and walk forward 3 meters"
  → L90\n (waits ~5s for IMU turn) → f (3s) → s
```

### Open-loop constants (tune these)
```python
METERS_PER_SECOND  = 0.15   # forward walk speed (m/s)
DEGREES_PER_SECOND = 22.5   # spin rate (°/s) — used only for laptop-side wait estimate
```
`DEGREES_PER_SECOND` on the laptop side is only a courtesy wait estimate — the
board uses the IMU gyro to close the angle loop. Adjust if the laptop prompts for
the next command mid-turn.

---

## 8. One-shot command implementations

### `cmd_stand_up(home)`
Ramps torso from `_torso_frac` → 1.0 over 1500ms. Updates `_torso_frac = 1.0`.

### `cmd_lay_down()`
Ramps torso from `_torso_frac` → 0.0 over 1500ms. Updates `_torso_frac = 0.0`.

### `cmd_jump(home)`
Smooth large-range dip-and-rise. Three phases, all interpolated continuously:
1. Sink: STANCE_Y (132mm) → LOW_Y (146mm) over 700ms — body drops
2. Rise: LOW_Y (146mm) → HIGH_Y (93mm) over 900ms — body lifts high
3. Return: HIGH_Y (93mm) → STANCE_Y (132mm) over 600ms

IK is valid for fy ∈ (87, 148) with fx=0. Both limits have safety margins.

### `cmd_fast_feet(home, cycles=4)`
Lifts each foot in sequence at 60ms up / 60ms down. Cycles through all 4 feet,
repeats `cycles` times. Lift height: STANCE_Y - 20mm = 112mm.

### `cmd_lift_foot(foot_idx, home)`
Sets all legs to home, then lifts the chosen foot to STANCE_Y - 28mm = 104mm.
Holds for 2000ms, returns to home.
`_FOOT_LEGS = [(0,1), (2,3), (4,5), (6,7)]` → foot_idx 0-3 maps to servo pairs.

---

## 9. File reference

| File | What it is |
|------|------------|
| `TELEOP/teleop_dog.py` | **Firmware source.** MicroPython, runs on Servo 2040. |
| `TELEOP/main.py` | Exact copy of `teleop_dog.py`. Always kept in sync via `cp teleop_dog.py main.py`. Upload THIS file to the board (avoids renaming). |
| `TELEOP/voice_control.py` | **Laptop controller.** STT → command parser → serial. |
| `TELEOP/diagnose_imu.py` | Sends an IMU turn command and reads all board serial output. Use to verify gz signs and debug turn accuracy. |
| `TELEOP/test_stt.py` | Records mic audio, prints peak/RMS level, runs through Whisper. Use to verify mic input before robot testing. |
| `TELEOP/.env` | `OPENAI_API_KEY=...` (kept for future use; current STT is offline). |
| `TELEOP/.venv/` | Python virtual environment. |
| `crawl_gait.py` | 4-leg diagonal-pair trot, fixed cycle count. |
| `all_dog_test.py` | Full demo: walk, torso twist, balance, walk back. |
| `balance.py` | Active IMU balance (roll + pitch). Standalone, not yet integrated with teleop. |
| `imu_read.py` | MPU6050 test: prints ax/ay/az and gx/gy/gz at 5Hz. Verify axis directions here. |
| `tof_read.py` | VL53L0X test: reads distance in mm. Verify sensor wiring here. |
| `torso_calibrate_one.py` | Interactive helper to find a torso's true vertical angle. |
| `torso_test.py` | Torso-only test: seeds flat, ramps to stand. |
| `leg_test_4legs.py` | 8 leg servos, per-leg wiggle, no IK. Confirms pairing/direction. |
| `full_test_12.py` | All 12 servos: stand, wiggle, lie down. |
| `robot_dog_leg_ik.ino` etc. | **OBSOLETE** Arduino/PCA9685 originals. Reference only. |

---

## 10. Firmware upload workflow

1. Edit `TELEOP/teleop_dog.py`.
2. Run `cp TELEOP/teleop_dog.py TELEOP/main.py` to sync.
3. In VS Code with MicroPico: right-click `TELEOP/main.py` →
   **"Upload file to Pico"** (or Cmd+Shift+P → "MicroPico: Upload current file to Pico").
4. **Disconnect MicroPico** (click status bar icon, or Cmd+Shift+P → "Disconnect").
   This releases the serial port so `voice_control.py` can connect.
5. Press the reset button on the board (or power-cycle) — `main.py` autoruns.
6. Run `voice_control.py` on the laptop.

Do NOT use "Run current file" for teleop — MicroPico holds the serial port open.

---

## 11. Hard safety rules

- **Never command a torso outside its calibrated range.** Past the stop = legs
  clash and stall. Every torso command is clamped via `TORSO_LIMITS`.
- **Torsos start at flat.** Always seed them at flat angle before enabling and ramp.
- **Servos have no position feedback.** Code only knows the last commanded position.
  After any manual repositioning with power off, move to a known safe angle first.
- **Never physically disconnect USB** — USB is the only logic power source.
  "Disconnect" always means software only.
- **Keep a hand on the servo-power switch** on first run of anything new.
- Power the servo rail OFF when touching the linkage or reflashing; leave USB on.
- **`_torso_frac` must be kept in sync** with the real torso position. If you call
  `torso_ramp()` directly without going through `cmd_stand_up`/`cmd_lay_down`,
  update `_torso_frac` manually afterward.

---

## 12. Known issues / next steps

- **Front/back leg corner mapping unverified.** `FRONT_LEGS`/`BACK_LEGS` in
  `balance.py` and the `fwd` signs in teleop gait are assumed — verify physically.
- **`balance.py` not integrated with teleop.** Running both simultaneously would
  require merging the 20ms control loops. The IMU I2C object is already initialized
  in `teleop_dog.py`; the torso ramp infrastructure is also there.
- **Foot index 0-3 mapping assumed.** The note in `_FOOT_LEGS` says "swap pairs if
  the wrong foot lifts." If `cmd_lift_foot(0, home)` lifts the wrong foot, rotate
  the `_FOOT_LEGS` tuple until it's right.
- **Wireless teleop.** RP2040 has no WiFi. An ESP32 or Pi Zero on the robot could
  host a bridge and serve over WiFi for untethered operation.
- **Compound commands block.** "Turn left and walk forward" executes sequentially
  and the laptop waits for each step. For parallel motion (e.g., walking while
  slowly turning), the firmware gait would need a blended mode.
- **Whisper transcription robustness.** The parser uses `\bforward\b` etc. If
  Whisper mishears a word, try rephrasing. Add synonyms to `parse_command()` as
  needed.
