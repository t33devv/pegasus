#!/usr/bin/env python3
"""
bridge.py — Laptop-side teleop bridge for the robot dog.

  1. Serves index.html on http://localhost:8080
  2. Opens a WebSocket on ws://localhost:8765
  3. Forwards single-byte commands (f/b/l/r/s) to the Servo 2040 via USB serial.

Requirements:
    pip install websockets pyserial

Usage:
    python bridge.py                          # auto-detect serial port
    python bridge.py --port /dev/tty.usbmodemXXXX
    python bridge.py --port COM3              # Windows

IMPORTANT: Close Thonny / VS Code / MicroPico before running this —
they must release the serial port first.
"""

import asyncio
import argparse
import pathlib
import threading
import functools
import http.server
import sys

HERE = pathlib.Path(__file__).parent


def find_serial_port():
    """Auto-detect the Servo 2040 (RP2040) serial port."""
    try:
        import serial.tools.list_ports
    except ImportError:
        return None
    for p in serial.tools.list_ports.comports():
        desc = (p.description or '').lower()
        dev  = p.device.lower()
        if 'usbmodem' in dev or 'micropython' in desc or 'rp2040' in desc or 'pico' in desc:
            return p.device
    candidates = list(serial.tools.list_ports.comports())
    return candidates[0].device if candidates else None


class SerialBridge:
    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0)
        print(f"  serial  : {port} @ {baud} baud")

    def send(self, cmd: str):
        self.ser.write(cmd.encode())


def _serve_static(directory, port):
    """Blocking HTTP server — run in a daemon thread."""
    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *_): pass

    handler = functools.partial(QuietHandler, directory=str(directory))
    with http.server.HTTPServer(('localhost', port), handler) as srv:
        srv.serve_forever()


async def run(args):
    try:
        import websockets
        import serial
    except ImportError as e:
        print(f"Missing dependency: {e}")
        print("Run:  pip install websockets pyserial")
        sys.exit(1)

    # Serial port
    port = args.port or find_serial_port()
    if not port:
        print("ERROR: could not find a serial port.")
        print("Pass --port /dev/tty.usbmodemXXXX (macOS) or --port COMX (Windows).")
        sys.exit(1)

    bridge = SerialBridge(port, args.baud)

    # Static file server (background thread)
    t = threading.Thread(target=_serve_static, args=(HERE, args.http_port), daemon=True)
    t.start()
    print(f"  web ui  : http://localhost:{args.http_port}")

    # WebSocket server
    async def ws_handler(websocket):
        async for message in websocket:
            cmd = str(message).strip()
            if cmd in ('f', 'b', 'F', 'B', 'l', 'r', 's'):
                bridge.send(cmd)

    print(f"  websocket: ws://localhost:{args.ws_port}")
    print("Ready. Open the web UI in a browser and use WASD to drive.")
    print("Ctrl-C to quit.\n")

    async with websockets.serve(ws_handler, 'localhost', args.ws_port):
        await asyncio.Future()  # run forever


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Robot dog teleop bridge')
    parser.add_argument('--port',      default=None,  help='Serial port (auto-detect if omitted)')
    parser.add_argument('--baud',      default=115200, type=int)
    parser.add_argument('--http-port', default=8080,   type=int, dest='http_port')
    parser.add_argument('--ws-port',   default=8765,   type=int, dest='ws_port')
    args = parser.parse_args()

    print("Starting teleop bridge…")
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\nBridge stopped.")
