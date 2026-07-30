# Rex — Quadruped Robot Dog

![Fusion360 CAD](Fusion360%20CAD.png) ![Photo](Photo.png)

Four legs, each a parallel five-bar linkage driven by two servos, plus a roll servo per leg (12 servos total). Controlled by a **Pimoroni Servo 2040** (RP2040) running **Pimoroni's MicroPython**.

---

## Hardware at a glance

| Thing | Detail |
|-------|--------|
| Controller | Pimoroni Servo 2040 (RP2040, 18 servo channels) |
| Servo power | 6V / 6A bench supply into screw terminals |
| Logic power | USB-C to laptop (**power-trace on board is cut** — USB is required to boot) |
| IMU | MPU6050 on I2C (address 0x68) |
| Distance sensor | VL53L0X on I2C (address 0x29) |

**Critical:** the board's USB/external power trace is cut. USB must stay connected at all times — the board loses logic power the instant USB is disconnected physically. "Disconnect" always means closing software, not unplugging the cable.

### Servo channel map

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

---

## CODE folder — file reference

All MicroPython files run via **VS Code + MicroPico extension**. Upload a file with `Cmd+Shift+P → "MicroPico: Upload current file to Pico"`.

| File | What it does |
|------|-------------|
| `crawl_gait.py` | 4-leg diagonal-pair trot for a fixed number of cycles |
| `all_dog_test.py` | Full demo: walk forward → torso twist → tripedal balance → walk backward |
| `torso_calibration.py` | Interactive REPL helper to find each torso's true vertical angle |
| `torso_test.py` | Seeds torsos at flat and ramps to stand — safe first torso check |
| `torso_twist.py` | Lateral body lean / twist test |
| `4_legs_test.py` | 8 leg servos, per-leg wiggle, no IK — confirms pairing and left/right direction |
| `4_servo_test.py` | Simple 4-servo sweep |
| `tripedal_balance.py` | Tripedal static balance pose |
| `balance.py` | Active IMU-driven balance — adjusts torsos (roll) and leg depths (pitch) in a 20 ms loop |
| `imu_read.py` | MPU6050 test — prints ax/ay/az (g) and gx/gy/gz (°/s) at 5 Hz |
| `tof_read.py` | VL53L0X test — reads distance in mm; auto-detects I2C bus |
| `vl53l0x.py` | VL53L0X MicroPython driver — must be on the board |
| `TELEOP/main.py` | Teleop firmware — upload this as `main.py` and it autoruns on boot |
| `TELEOP/teleop_dog.py` | Source copy of the teleop firmware (keep in sync with `main.py`) |
| `TELEOP/bridge.py` | Laptop bridge: HTTP on :8080, WebSocket on :8765, forwards to serial |
| `TELEOP/index.html` | WASD web controller |
| `TELEOP/voice_control.py` | Voice control interface |
| `TELEOP/diagnose_imu.py` | IMU diagnostic tool |

Full technical reference (IK math, gait parameters, calibration values, safety rules): [`CODE/ROBOT_DOG.md`](CODE/ROBOT_DOG.md).

---

## Testing — step by step

### 1. Flash MicroPython

Install **Pimoroni's MicroPython** (the `pico-...-pimoroni-micropython.uf2` image). Vanilla MicroPython will fail — the `servo` library won't import.

### 2. Upload the VL53L0X driver

Upload `CODE/vl53l0x.py` to the board before running anything that uses the ToF sensor.

### 3. Verify sensors

**IMU:**
```
Upload: CODE/imu_read.py
Run in MicroPico terminal — watch ax/ay/az and gx/gy/gz update at 5 Hz.
Tilt the robot side-to-side → ax should change.
Tilt front/back → ay should change.
If nothing changes, check SDA/SCL wiring and I2C address (default 0x68).
```

**Distance sensor:**
```
Upload: CODE/tof_read.py
Run — should print distance in mm.
Wave a hand in front → value drops.
USB power alone is fine; servo rail does not need to be on.
```

### 4. Servo bring-up (no IK)

```
Upload: CODE/4_servo_test.py   → basic sweep of 4 servos
Upload: CODE/4_legs_test.py    → per-leg wiggle for all 8 leg servos
```

Keep a hand on the servo power switch. Watch for any servo that doesn't move or moves the wrong way.

### 5. Torso bring-up

**Do this before any full-dog test.** The torsos power on at their "flat" position, not vertical. Running leg code without seeding the torsos first causes a snap.

```
Upload: CODE/torso_test.py
```

Seeds each torso at its flat angle then ramps to stand. The calibrated stand angles are 60°/110°/110°/70° (not 90°) — see `ROBOT_DOG.md §2` for details.

If a torso strains at flat, back the seed value off ~5° toward stand and update `torso_calibration.py`.

### 6. Full bring-up

```
Upload: CODE/all_dog_test.py
```

Runs: walk forward → torso twist → tripedal balance → walk backward. Good end-to-end sanity check before teleop.

### 7. Active balance

```
Upload: CODE/balance.py
```

Stands up, then runs a 2-second gyro calibration (keep the dog completely still). After that it holds level using a complementary filter on the IMU. If roll or pitch compensation goes the wrong direction, flip `ROLL_DIR` or `PITCH_DIR` in the file.

---

## Teleop

### Setup (one-time)

```bash
cd CODE/TELEOP
pip install websockets pyserial
```

### Running

1. Upload `TELEOP/main.py` to the board.
2. In MicroPico terminal press **Ctrl+D** to soft-reset — `main.py` autoruns.
3. **Close VS Code** (releases the serial port; USB and board stay powered).
4. In a terminal: `python3 CODE/TELEOP/bridge.py`
5. Open `http://localhost:8080` in a browser.

### Controls

| Key | Action |
|-----|--------|
| W | Walk forward (blocked if obstacle < 10 cm) |
| S | Walk backward |
| A | Spin left |
| D | Spin right |
| Space | Stop |

### Stopping

Press S or Space on the page, then Ctrl+C in the bridge terminal.

### Reflashing while in teleop mode

Close `bridge.py` → reopen VS Code → MicroPico reconnects automatically.

---

## Safety rules

- **Never command a torso outside its calibrated range.** Past the mechanical stop = legs clash and stall. Every torso command must be clamped.
- **Always seed torsos at flat before enabling servo power.**
- **Never physically unplug USB** — the board dies instantly. Close software only.
- **Power servo rail off** when touching the linkage or reflashing. Leave USB in.
- Keep a hand on the servo power switch on the first run of anything new.
- Safe servo range per leg motor: **20°–160°**. Never command 0° or 180°.
