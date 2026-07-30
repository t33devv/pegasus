#!/usr/bin/env python3
"""
diagnose_imu.py — Send an IMU turn command and read back the board's serial output.

Shows what gz values the board is actually seeing during the turn so we can
figure out why the angle is wrong. Run INSTEAD of voice_control.py.

Usage:
    python3 TELEOP/diagnose_imu.py              # auto-detect port, 90° left turn
    python3 TELEOP/diagnose_imu.py L 45         # 45° left turn
    python3 TELEOP/diagnose_imu.py R 90         # 90° right turn
    python3 TELEOP/diagnose_imu.py --port /dev/tty.usbmodem1101 L 90
"""

import sys
import time
import glob
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--port", default=None)
parser.add_argument("direction", nargs="?", default="L", choices=["L", "R"])
parser.add_argument("degrees",   nargs="?", default=90, type=float)
args = parser.parse_args()

import serial
import serial.tools.list_ports

def find_port():
    for p in serial.tools.list_ports.comports():
        if "usbmodem" in p.device.lower() or "pico" in (p.description or "").lower():
            return p.device
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if ports else None

port = args.port or find_port()
if not port:
    sys.exit("Could not find serial port. Pass --port /dev/tty.usbmodemXXXX")

print(f"Port: {port}")
print(f"Command: {args.direction}{int(args.degrees)}°\n")

ser = serial.Serial(port, 115200, timeout=0.2)
time.sleep(0.5)

# Drain any pending output
while ser.in_waiting:
    ser.readline()

cmd = f"{args.direction}{int(args.degrees)}\n"
print(f"Sending: {repr(cmd)}")
ser.write(cmd.encode())

print("\n--- Board output (Ctrl-C to stop) ---")
try:
    deadline = time.time() + 30   # read for up to 30s
    while time.time() < deadline:
        line = ser.readline()
        if line:
            print(line.decode(errors="replace").rstrip())
except KeyboardInterrupt:
    pass

ser.close()
print("\nDone.")
