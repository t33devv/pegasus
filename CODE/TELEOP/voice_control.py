#!/usr/bin/env python3
"""
voice_control.py — Voice command interface for Pegasus (quadruped robot dog).

Pegasus accepts single ASCII bytes over USB serial at 115200 baud:
    f = walk forward    b = walk backward
    l = turn left       r = turn right
    s = stop

Timed moves (e.g. "forward 3 metres") send the motion byte, wait the
computed duration, then send 's'. Distance and angle conversions are
open-loop and approximate — calibrate METERS_PER_SECOND / DEGREES_PER_SECOND
before trusting them. Use --calibrate-speed and --calibrate-turn.

pip install faster-whisper pyserial sounddevice soundfile numpy python-dotenv

One-time model download (run once, then works offline):
    python -c "from faster_whisper import WhisperModel; WhisperModel('base', device='cpu', compute_type='int8')"
Model is cached in ~/.cache/huggingface/ (~150 MB). Delete that folder to re-download.

Usage:
    python voice_control.py                        # voice control loop
    python voice_control.py --port /dev/tty.usbmodem1101
    python voice_control.py --calibrate-speed      # measure forward speed
    python voice_control.py --calibrate-turn       # measure turn rate
"""

import os
import re
import sys
import time
import tempfile
import threading
from pathlib import Path
from typing import Optional

import numpy as np

# Load TELEOP/.env (or any parent .env) so you can store keys there instead of
# setting them in the shell. python-dotenv is optional — falls back gracefully.
try:
    from dotenv import load_dotenv
    load_dotenv(Path(__file__).parent / ".env")
except ImportError:
    pass

# ─────────────────────────────────────────────────────────────
# TUNABLE CONSTANTS — set these before your first run
# ─────────────────────────────────────────────────────────────

# Serial port connecting to the Servo 2040.
# None = auto-detect the first RP2040/MicroPython USB device found.
# Override example: SERIAL_PORT = "/dev/tty.usbmodem1101"
SERIAL_PORT = None
BAUD_RATE   = 115200

# Open-loop speed estimates — these are rough defaults. Run --calibrate-speed
# and --calibrate-turn to measure real values, then paste them here.
METERS_PER_SECOND  = 0.15   # approximate forward walk speed (m/s)
DEGREES_PER_SECOND = 22.5   # approximate spin rate (°/s)

# Recording
SAMPLE_RATE        = 16000  # Hz — Whisper works well at 16 kHz
MAX_RECORD_SECONDS = 10     # hard cap: stream is stopped after this many seconds


# ─────────────────────────────────────────────────────────────
# SERIAL HELPERS
# ─────────────────────────────────────────────────────────────

def find_serial_port() -> Optional[str]:
    try:
        import serial.tools.list_ports
    except ImportError:
        return None
    for p in serial.tools.list_ports.comports():
        dev  = p.device.lower()
        desc = (p.description or "").lower()
        if "usbmodem" in dev or "micropython" in desc or "rp2040" in desc or "pico" in desc:
            return p.device
    # Fallback: first available port
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if ports else None


def open_serial(port: Optional[str] = None, baud: int = BAUD_RATE):
    import serial
    port = port or find_serial_port()
    if not port:
        sys.exit(
            "[serial] ERROR: could not find Pegasus's serial port.\n"
            "Pass --port /dev/tty.usbmodemXXXX or set SERIAL_PORT at the top of this file."
        )
    ser = serial.Serial(port, baud, timeout=1)
    print(f"[serial] Connected: {port} @ {baud} baud")
    return ser


def send_command(ser, byte: str) -> None:
    """Send a single ASCII byte to Pegasus and log it."""
    ser.write(byte.encode())
    print(f"[serial] Sent: {repr(byte)}")


# ─────────────────────────────────────────────────────────────
# AUDIO RECORDING
# ─────────────────────────────────────────────────────────────

def record_audio() -> str:
    """
    Push-to-key recording: press Enter to start, Enter again to stop.
    Returns the path to a temporary WAV file.
    """
    import sounddevice as sd
    import soundfile as sf

    print("\n[mic] Press Enter to START recording...")
    input()

    frames = []

    def _callback(indata, _frames, _time, _status):
        frames.append(indata.copy())

    # The 'with' block starts the stream; second input() waits for stop.
    # A daemon thread enforces the hard cap so a missed Enter can't hang forever.
    stop_event = threading.Event()

    def _timeout_guard():
        stop_event.wait(MAX_RECORD_SECONDS)

    threading.Thread(target=_timeout_guard, daemon=True).start()

    with sd.InputStream(samplerate=SAMPLE_RATE, channels=1,
                        dtype="float32", callback=_callback):
        print(f"[mic] Recording... Press Enter to STOP (max {MAX_RECORD_SECONDS}s).")

        def _wait_enter():
            input()
            stop_event.set()

        t = threading.Thread(target=_wait_enter, daemon=True)
        t.start()
        stop_event.wait()           # block until Enter pressed or timeout

    if not frames:
        raise RuntimeError("[mic] No audio captured — check your microphone.")

    audio = np.concatenate(frames, axis=0)
    duration = len(audio) / SAMPLE_RATE
    print(f"[mic] Captured {duration:.1f}s of audio.")

    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    sf.write(tmp.name, audio, SAMPLE_RATE)
    return tmp.name


# ─────────────────────────────────────────────────────────────
# SPEECH-TO-TEXT
# Swap this function body to change the STT backend (e.g. Vosk, Faster-Whisper).
# Contract: receives a WAV path, returns a plain-text string.
# ─────────────────────────────────────────────────────────────

def _load_whisper_model():
    """Load faster-whisper once and cache it for the session.
    Device priority: CUDA GPU → Apple Silicon (MPS, via cpu+int8 which is
    still fast on M-series) → CPU. faster-whisper doesn't expose MPS directly
    but int8 on Apple Silicon is already ~10× faster than real-time."""
    from faster_whisper import WhisperModel
    try:
        import torch
        if torch.cuda.is_available():
            device, compute_type = "cuda", "float16"
            print("[stt]    Device: CUDA GPU")
        elif torch.backends.mps.is_available():
            # faster-whisper's CTranslate2 backend doesn't use Metal directly;
            # fall back to CPU int8 which is highly optimised on Apple Silicon.
            device, compute_type = "cpu", "int8"
            print("[stt]    Device: Apple Silicon CPU (int8 — optimised for M-series)")
        else:
            device, compute_type = "cpu", "int8"
            print("[stt]    Device: CPU (int8)")
    except ImportError:
        device, compute_type = "cpu", "int8"
        print("[stt]    Device: CPU (int8)")
    return WhisperModel("base", device=device, compute_type=compute_type)

_whisper_model = None   # loaded on first call, reused for every subsequent call

def transcribe(audio_path: str) -> str:
    """Transcribe audio using faster-whisper — fully offline, no API key.
    First call downloads the 'base' model (~150 MB) to ~/.cache/huggingface/,
    then caches it in memory for the rest of the session."""
    global _whisper_model
    if _whisper_model is None:
        print("[stt]    Loading Whisper model...")
        _whisper_model = _load_whisper_model()
    segments, _ = _whisper_model.transcribe(audio_path, language="en")
    text = " ".join(seg.text for seg in segments).strip()
    print(f"[stt]    Heard: {repr(text)}")
    return text


# ─────────────────────────────────────────────────────────────
# COMMAND PARSER
# Swap this function body to change the parser (e.g. LLM-based).
# Contract: receives a text string, returns a command dict or None.
#   { "action": "forward"|"backward"|"left"|"right"|"stop",
#     "duration": float }   # seconds; 0.0 = run until manually stopped
# ─────────────────────────────────────────────────────────────

# Spoken number words → float
_WORD_NUMS = {
    "a": 1, "an": 1, "half": 0.5,
    "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
    "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10,
    "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14, "fifteen": 15,
    "sixteen": 16, "seventeen": 17, "eighteen": 18, "nineteen": 19,
    "twenty": 20, "thirty": 30, "forty": 40, "fifty": 50,
    "sixty": 60, "seventy": 70, "eighty": 80, "ninety": 90,
}


def _parse_foot(t: str) -> int:
    """Return 0-3 foot index from lowercased text. 0=FR, 1=FL, 2=BR, 3=BL."""
    front = bool(re.search(r'\bfront\b|\bfore\b', t))
    back  = bool(re.search(r'\bback\b|\bhind\b|\brear\b', t))
    right = bool(re.search(r'\bright\b', t))
    left  = bool(re.search(r'\bleft\b', t))
    if front and right: return 0
    if front and left:  return 1
    if back  and right: return 2
    if back  and left:  return 3
    if right: return 0
    if left:  return 1
    return 0   # default: front-right


def _extract_number(text: str) -> Optional[float]:
    """Return the first number found in text (digits take priority over words)."""
    m = re.search(r'\b(\d+(?:\.\d+)?)\b', text)
    if m:
        return float(m.group(1))
    for word, val in _WORD_NUMS.items():
        if re.search(rf'\b{word}\b', text):
            return float(val)
    return None


def parse_command(text: str) -> Optional[dict]:
    """
    Keyword-based command parser.

    Understands: forward / backward / left / right / stop
    plus an optional distance/angle ("3 meters", "90 degrees", "two metres")
    or a bare number interpreted as seconds ("forward 4").

    Returns None if no recognisable action is found.
    """
    t = text.lower()

    # --- one-shot body actions (checked first to avoid keyword collisions) ---
    if re.search(r'\bstop\b|\bhalt\b|\bfreeze\b|\bwait\b', t):
        return {"action": "stop", "duration": 0.0}

    if re.search(r'\bstand\b|\bget\s+up\b', t):
        return {"action": "stand_up", "duration": 0.0}

    if re.search(r'\blay\b|\blie\b|\bsit\s+down\b|\bcrouch\b', t):
        return {"action": "lay_down", "duration": 0.0}

    if re.search(r'\bjump\b|\bleap\b', t):
        return {"action": "jump", "duration": 0.0}

    if re.search(r'\bfast\s+feet\b|\bquick\s+feet\b', t):
        return {"action": "fast_feet", "duration": 0.0}

    if re.search(r'\blift\b|\braise\b|\bpick\s+up\b', t):
        foot_idx = _parse_foot(t)
        print(f"[parse]  lift foot index={foot_idx}")
        return {"action": "lift_foot", "duration": 0.0, "foot_index": foot_idx}

    # --- directional movement ---
    if re.search(r'\bforward\b|\bahead\b|\bstraight\b', t):
        action = "forward"
    elif re.search(r'\bback(?:ward)?\b|\breverse\b', t):
        action = "backward"
    elif re.search(r'\bleft\b', t):
        action = "left"
    elif re.search(r'\bright\b', t):
        action = "right"
    else:
        print(f"[parse]  No known action in: {repr(text)}")
        return None

    # --- magnitude → duration ---
    number   = _extract_number(t)
    duration = 0.0  # 0 = continuous; user must say "stop" to halt

    if number is not None:
        # "meter" / "metre" → distance → open-loop time
        if re.search(r'\bmet(?:er|re)s?\b', t):
            duration = number / METERS_PER_SECOND
            print(f"[parse]  {number} m ÷ {METERS_PER_SECOND} m/s = {duration:.1f}s  (open-loop)")

        # "degree" / "deg" → IMU-guided turn (board closes the loop)
        elif re.search(r'\bdeg(?:ree)?s?\b', t):
            if action in ("left", "right"):
                print(f"[parse]  {number}° → IMU-guided turn")
                return {"action": action, "duration": 0.0, "angle": number}
            # degrees on forward/backward doesn't make sense — ignore unit
            duration = number / DEGREES_PER_SECOND
            print(f"[parse]  {number}° (no IMU for this direction) → {duration:.1f}s")

        else:
            # Bare number — treat as seconds
            duration = number
            print(f"[parse]  {number} (interpreted as seconds)")

    return {"action": action, "duration": duration}


def parse_commands(text: str) -> list:
    """Split on 'and'/'then' and parse each clause independently.

    "turn left 90 degrees and walk forward 3 meters"
      → [{"action":"left","angle":90}, {"action":"forward","duration":20}]
    """
    parts = re.split(r'\s+(?:and|then)\s+', text.strip(), flags=re.IGNORECASE)
    cmds = []
    for part in parts:
        cmd = parse_command(part.strip())
        if cmd:
            cmds.append(cmd)
    if not cmds:
        print(f"[parse]  No known action in: {repr(text)}")
    return cmds


# ─────────────────────────────────────────────────────────────
# COMMAND EXECUTOR
# ─────────────────────────────────────────────────────────────

# Maps parsed action names to Pegasus's single-byte serial commands.
_ACTION_BYTE = {
    "forward":   "f",
    "backward":  "b",
    "left":      "l",
    "right":     "r",
    "stop":      "s",
    "stand_up":  "U",
    "lay_down":  "D",
    "jump":      "J",
    "fast_feet": "T",
}


def execute_command(ser, cmd: dict) -> None:
    action   = cmd["action"]
    duration = cmd["duration"]
    angle    = cmd.get("angle")       # set only for IMU-guided turns

    # One-shot actions: single byte, board handles timing internally
    if action in _ACTION_BYTE and action not in ("forward", "backward", "left", "right", "stop"):
        byte = _ACTION_BYTE[action]
        ser.write(byte.encode())
        print(f"[serial] Sent: {repr(byte)}")
        return

    if action == "lift_foot":
        foot_idx = cmd.get("foot_index", 0)
        byte = str(foot_idx + 1)   # '1'–'4'
        ser.write(byte.encode())
        print(f"[serial] Sent: {repr(byte)}  (foot index {foot_idx})")
        return

    # IMU-guided turn: send L<N>\n or R<N>\n and let the board close the loop
    if action in ("left", "right") and angle is not None:
        direction = "L" if action == "left" else "R"
        msg = f"{direction}{int(round(angle))}\n"
        ser.write(msg.encode())
        print(f"[serial] Sent: {repr(msg)}")
        # Estimated wait so we don't prompt for the next command mid-turn.
        # The board auto-stops when the gyro confirms the angle — this is just
        # a courtesy delay on the laptop side (open-loop estimate × 1.3 safety margin).
        estimated = angle / DEGREES_PER_SECOND
        print(f"[exec]   IMU turn {direction}{int(round(angle))}° — waiting ~{estimated:.1f}s...")
        time.sleep(estimated * 1.3)
        print("[exec]   Done.")
        return

    byte = _ACTION_BYTE[action]

    if action == "stop":
        send_command(ser, byte)
        return

    send_command(ser, byte)

    if duration > 0.0:
        print(f"[exec]   Running '{action}' for {duration:.1f}s...")
        time.sleep(duration)
        send_command(ser, "s")
        print("[exec]   Done.")
    else:
        print(f"[exec]   '{action}' started — say 'stop' or press Ctrl-C to halt.")


# ─────────────────────────────────────────────────────────────
# CALIBRATION HELPERS
# ─────────────────────────────────────────────────────────────

def calibrate_speed(ser, walk_seconds: float = 5.0) -> float:
    """
    Walk Pegasus forward for walk_seconds, then ask you to measure the distance.
    Prints the METERS_PER_SECOND value to paste into this file.

    NOTE: open-loop only — no odometry. Result varies with surface and battery.
    """
    print(f"\n[cal] Forward-speed calibration: Pegasus will walk for {walk_seconds}s.")
    print("[cal] Mark her starting position on the floor.")
    input("[cal] Press Enter when ready...")
    send_command(ser, "f")
    time.sleep(walk_seconds)
    send_command(ser, "s")
    dist = float(input("[cal] Measure the distance travelled (metres): "))
    mps  = dist / walk_seconds
    print(f"\n[cal] *** METERS_PER_SECOND = {mps:.4f} ***")
    print("[cal] Paste that value near the top of voice_control.py.\n")
    return mps


def calibrate_turn(ser, turn_seconds: float = 4.0) -> float:
    """
    Spin Pegasus left for turn_seconds, then ask you to estimate the angle.
    Prints the DEGREES_PER_SECOND value to paste into this file.

    NOTE: open-loop only — actual rate depends on surface friction.
    """
    print(f"\n[cal] Turn-rate calibration: Pegasus will spin left for {turn_seconds}s.")
    print("[cal] Mark her starting heading (e.g. with tape).")
    input("[cal] Press Enter when ready...")
    send_command(ser, "l")
    time.sleep(turn_seconds)
    send_command(ser, "s")
    deg = float(input("[cal] Estimate how many degrees she turned: "))
    dps = deg / turn_seconds
    print(f"\n[cal] *** DEGREES_PER_SECOND = {dps:.2f} ***")
    print("[cal] Paste that value near the top of voice_control.py.\n")
    return dps


# ─────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────

def main():
    import argparse
    p = argparse.ArgumentParser(description="Voice control for Pegasus (quadruped robot)")
    p.add_argument("--port",             default=SERIAL_PORT, help="Serial port (auto-detect if omitted)")
    p.add_argument("--baud",             default=BAUD_RATE,   type=int)
    p.add_argument("--calibrate-speed",  action="store_true", help="Measure forward speed then exit")
    p.add_argument("--calibrate-turn",   action="store_true", help="Measure turn rate then exit")
    args = p.parse_args()

    ser = open_serial(args.port, args.baud)

    if args.calibrate_speed:
        calibrate_speed(ser)
        ser.close()
        return

    if args.calibrate_turn:
        calibrate_turn(ser)
        ser.close()
        return

    print("\nVoice control ready — Ctrl-C to quit and stop Pegasus.")
    print(f"  Speed : {METERS_PER_SECOND} m/s  |  Turn: {DEGREES_PER_SECOND} °/s\n")

    while True:
        try:
            audio_path = record_audio()
            text       = transcribe(audio_path)
            cmds       = parse_commands(text)
            if cmds:
                for cmd in cmds:
                    execute_command(ser, cmd)
            else:
                print("[main]   Unrecognised — try again.")
        except KeyboardInterrupt:
            print("\n[main] Stopping Pegasus and exiting.")
            send_command(ser, "s")
            break
        except Exception as e:
            print(f"[error]  {e}")
            # Keep the loop alive; a bad API call shouldn't kill voice control.

    ser.close()


if __name__ == "__main__":
    main()
