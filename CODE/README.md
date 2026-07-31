# CODE

This directory contains all firmware and laptop-side software for Pegasus.

All `*.py` files in `CODE/` run **on the Pimoroni Servo 2040 board** via MicroPython. Files in `TELEOP/` that run on the laptop are noted explicitly below. For full hardware reference (wiring, torso calibration values, IK geometry, serial protocol, safety rules), see `ROBOT_DOG.md`.

---

## Uploading firmware to the board

1. Open VS Code with the **MicroPico** extension installed and connected to the Servo 2040.
2. Right-click the file in the Explorer → **"Upload file to Pico"**, or use `Cmd+Shift+P` → **"MicroPico: Upload current file to Pico"**.
3. `main.py` autoruns on power-up — any other filename must be triggered manually via the MicroPico REPL.

> **Before running any file:** ensure the board has USB-C connected (logic power) and the 6V bench supply is on (servo power). See `ROBOT_DOG.md` for full wiring details and safety rules.

---

## Files in `CODE/`

| File | What it does | How to run |
|------|--------------|------------|
| `4_servo_test.py` | Tests 4 servos on headers 1–4 — confirms wiring and pulse calibration. | Upload to board, run via MicroPico REPL. |
| `4_legs_test.py` | Raw 8-servo leg test (no IK) — wiggles each motor in sequence to confirm pairing and direction. | Upload to board, run via MicroPico REPL. |
| `imu_read.py` | Reads MPU6050 accel + gyro at 5 Hz — use this to verify axis directions and sensor wiring. | Upload to board, run via MicroPico REPL. USB power only (servo rail not needed). |
| `tof_read.py` | Reads VL53L0X distance sensor in mm — use this to verify ToF wiring. Requires `vl53l0x.py` on the board. | Upload to board, run via MicroPico REPL. USB power only. |
| `torso_calibration.py` | Interactive helper — type an angle at the prompt to move one torso servo until the leg hangs vertical. Set `WHICH` to the torso index before uploading. | Upload to board, run via MicroPico REPL. |
| `torso_test.py` | Torso-only test — seeds all torsos at flat, ramps to stand, dips halfway, then lies back down. | Upload to board, run via MicroPico REPL. |
| `torso_twist.py` | Rocks the dog side-to-side by combining torso lean and leg depth changes over `ROCK_COUNT` cycles. | Upload to board, run via MicroPico REPL. |
| `crawl_gait.py` | Diagonal-pair alternating trot for a fixed number of cycles (`GAIT_CYCLES`). | Upload to board, run via MicroPico REPL. |
| `tripedal_balance.py` | Raises each leg in sequence, pre-shifting the CG over the support triangle first. | Upload to board, run via MicroPico REPL. |
| `all_dog_test.py` | Full demo sequence: stand, walk forward, torso twist, tripedal balance, walk backward, twist, lie down. | Upload to board, run via MicroPico REPL. |
| `balance.py` | Active roll + pitch stabilisation using the MPU6050 — continuous 20 ms control loop. Keep the dog still for the first ~2 s of gyro calibration. | Upload to board, run via MicroPico REPL. |
| `vl53l0x.py` | VL53L0X MicroPython driver — must be uploaded to the board alongside any file that uses the ToF sensor. | Upload to board (no need to run directly). |
| `sketch_jul22a.ino` | **Obsolete** Arduino/PCA9685 original — reference only, not used in the current build. | — |

---

## TELEOP system (`TELEOP/`)

The teleop system lets you drive Pegasus in real time from a laptop — via a keyboard/web UI or by voice.

### Step 1 — Upload the firmware

```bash
cp TELEOP/teleop_dog.py TELEOP/main.py
```

Upload `TELEOP/main.py` to the board (right-click → "Upload file to Pico"). `main.py` autoruns on power-up.

### Step 2 — Disconnect MicroPico

Click the MicroPico icon in the VS Code status bar, or `Cmd+Shift+P` → **"MicroPico: Disconnect"**. This releases the serial port so the laptop scripts can connect.

> Do **not** use "Run current file" for teleop — MicroPico holds the serial port open while running.

### Step 3 — Reset the board

Press the reset button on the Servo 2040 (or power-cycle it). The board will stand up, calibrate the IMU (~2 s, keep the dog still), then print `ready` and wait for commands.

### Option A — Web UI (keyboard control)

```bash
cd TELEOP
pip install websockets pyserial          # one-time
python bridge.py                         # auto-detects serial port
```

Open `http://localhost:8080` in a browser. Hold **W / A / S / D** to drive, release to stop. **Space** stops. **E** toggles Walk / Run mode.

```bash
# If auto-detect fails, pass the port manually:
python bridge.py --port /dev/tty.usbmodemXXXX   # macOS
python bridge.py --port COM3                      # Windows
```

### Option B — Voice control

**One-time setup:**

```bash
cd TELEOP
pip install faster-whisper pyserial sounddevice soundfile numpy python-dotenv

# Download the Whisper model (~150 MB, cached to ~/.cache/huggingface/):
python -c "from faster_whisper import WhisperModel; WhisperModel('base', device='cpu', compute_type='int8')"
```

**Running:**

```bash
cd TELEOP
source .venv/bin/activate
python voice_control.py                  # auto-detects serial port
```

Press **Enter** to start recording, **Enter** again to stop. Spoken commands include:

| Say | What happens |
|-----|-------------|
| "forward", "go ahead" | Walk forward |
| "backward", "reverse" | Walk backward |
| "left" / "right" | Spin continuously |
| "turn left 90 degrees" | IMU-guided 90° left turn |
| "walk forward 3 meters" | Walk forward ~3 m (open-loop), then stop |
| "stop", "halt", "freeze" | Stop |
| "stand up", "get up" | Ramp torsos to stand |
| "lay down", "lie down" | Ramp torsos to flat |
| "jump" / "leap" | Jump motion |
| "fast feet", "quick feet" | Rapid sequential foot lifts |
| "lift front right foot" | Lift foot 1 and hold 2 s |

Compound commands work too: `"turn left 90 degrees and walk forward 3 meters"`.

```bash
# Optional: calibrate open-loop speed/turn constants
python voice_control.py --calibrate-speed
python voice_control.py --calibrate-turn
```

---

## Debugging utilities

| File | Purpose | How to run |
|------|---------|------------|
| `TELEOP/diagnose_imu.py` | Sends an IMU turn command and prints all board serial output — use to verify gz signs and turn accuracy. | `python3 TELEOP/diagnose_imu.py L 90` (laptop, board must be running `main.py`). |
| `TELEOP/test_stt.py` | Records audio, prints peak/RMS level, runs through Whisper — use to verify mic input before robot testing. | `python3 TELEOP/test_stt.py` (laptop, inside `.venv`). |

---

## TELEOP file reference

| File | What it is |
|------|-----------|
| `TELEOP/teleop_dog.py` | Firmware source — edit this file. |
| `TELEOP/main.py` | Exact copy of `teleop_dog.py` — always sync with `cp teleop_dog.py main.py`, then upload this to the board. |
| `TELEOP/bridge.py` | Laptop-side bridge: serves the web UI on port 8080 and forwards commands over serial. |
| `TELEOP/index.html` | Web gamepad UI served by `bridge.py`. |
| `TELEOP/voice_control.py` | Laptop-side voice controller: mic → Whisper STT → command parser → serial. |
| `TELEOP/.env` | Stores `OPENAI_API_KEY` (kept for future use; current STT is fully offline). |
| `TELEOP/.venv/` | Python virtual environment with all laptop-side dependencies. |
