# teleop_dog.py — Pimoroni Servo 2040 (MicroPython)
#
# Listens for single-byte commands over USB serial and drives the gait:
#   f = walk forward    b = walk backward
#   l = turn left       r = turn right  (torso lean steers the path)
#   s / Space = stop (hold stand)
#
# Workflow (MicroPico in VS Code):
#   1. Cmd+Shift+P → "MicroPico: Upload current file to Pico"
#      — OR right-click this file in Explorer → "Upload file to Pico"
#      Then rename it main.py on the board so it autoruns on power-up.
#   2. Disconnect MicroPico: click the MicroPico icon in the VS Code status bar,
#      or Cmd+Shift+P → "MicroPico: Disconnect". This releases the serial port.
#   3. Reset/power-cycle the board — teleop_dog.py starts automatically.
#   4. Run bridge.py on the laptop.
#
# Do NOT use "Run current file" for teleop — MicroPico keeps the serial port
# open while running, so bridge.py cannot connect.
#
# Ctrl-C stops the loop and disables servos.

import time
import math
import select
import sys
from machine import I2C, Pin
from servo import ServoCluster, Calibration, servo2040

# ---- hardware ----
cal = Calibration()
cal.apply_two_pairs(610, 2441, -90, 90)
pins = list(range(servo2040.SERVO_1, servo2040.SERVO_12 + 1))
cluster = ServoCluster(0, 0, pins, cal)

# ---- geometry ----
L1 = 30.0;  L2 = 120.0;  HS = 46.0
LEG_MIN = 20.0;  LEG_MAX = 160.0

# ---- gait parameters ----
STANCE_Y    = 132.0
LIFT        =  18.0
STRIDE      =  40.0
STANCE_DUTY =   0.6
CYCLE_MS    = 1200
TICK_MS     =  20

# ---- torso calibration ----
TORSO_CAL = [
    ( 9,  8,  60.0, 160.0),
    (10,  9, 110.0,  10.0),
    (11, 10, 110.0,   5.0),
    (12, 11,  70.0, 170.0),
]
TORSO_LIMITS = {
    idx: (min(stand, flat), max(stand, flat))
    for _, idx, stand, flat in TORSO_CAL
}

# ---- ToF obstacle avoidance ----
OBSTACLE_MM  = 100   # block forward motion below this distance (10 cm)
TOF_INTERVAL =   5   # read sensor every N ticks (5 × 20ms = 100ms)

def _find_i2c():
    combos = []
    try:
        combos.append((0, servo2040.SDA, servo2040.SCL))
        combos.append((1, servo2040.SDA, servo2040.SCL))
    except Exception:
        pass
    for bus in (0, 1):
        for sda, scl in ((4, 5), (20, 21), (0, 1)):
            combos.append((bus, sda, scl))
    for bus, sda, scl in combos:
        try:
            i2c = I2C(bus, sda=Pin(sda), scl=Pin(scl))
            found = i2c.scan()
            if len(found) > 20: continue   # stuck/shorted bus
            if 0x29 in found:
                return i2c
        except Exception:
            pass
    return None

def setup_tof():
    """Return (tof_object, read_method_name) or (None, None) if unavailable."""
    i2c = _find_i2c()
    if i2c is None:
        print("ToF: not found — obstacle avoidance disabled")
        return None, None
    try:
        import vl53l0x
        tof = vl53l0x.VL53L0X(i2c)
    except ImportError:
        try:
            from VL53L0X import VL53L0X
            tof = VL53L0X(i2c)
        except Exception as e:
            print("ToF: driver missing —", e)
            return None, None
    start = getattr(tof, "start", None)
    if start: start()
    for name in ("read", "read_range", "ping"):
        if getattr(tof, name, None):
            print("ToF: ready —", name)
            return tof, name
    if hasattr(tof, "range"):
        return tof, "range"
    print("ToF: unknown driver interface")
    return None, None

def tof_read_mm(tof_obj, method):
    try:
        return tof_obj.range if method == "range" else getattr(tof_obj, method)()
    except Exception:
        return 9999   # treat read error as clear path


# ---- modes ----
STOP      = 0
WALK_FWD  = 1
WALK_BACK = 2
TURN_L    = 3
TURN_R    = 4

CMD_MAP = {
    ord('f'): WALK_FWD,
    ord('b'): WALK_BACK,
    ord('l'): TURN_R,
    ord('r'): TURN_L,
    ord('s'): STOP,
    ord(' '): STOP,
}


# ---- IK ----

def legIK(fx, fy):
    dxL = fx - HS / 2;  dxR = fx + HS / 2
    DL  = math.sqrt(dxL * dxL + fy * fy)
    DR  = math.sqrt(dxR * dxR + fy * fy)
    if not (abs(L2 - L1) < DL < L1 + L2): return None
    if not (abs(L2 - L1) < DR < L1 + L2): return None
    cosL = max(-1.0, min(1.0, (L1*L1 + DL*DL - L2*L2) / (2*L1*DL)))
    cosR = max(-1.0, min(1.0, (L1*L1 + DR*DR - L2*L2) / (2*L1*DR)))
    phi_L = math.atan2( dxL, fy) + math.acos(cosL)
    phi_R = math.atan2(-dxR, fy) + math.acos(cosR)
    return 135.0 - math.degrees(phi_L), math.degrees(phi_R) + 45.0


# ---- servo helpers ----

def leg_set(idx, angle, load=True):
    cluster.value(idx, max(LEG_MIN, min(LEG_MAX, float(angle))) - 90.0, load)

def torso_raw(idx, angle, load=True):
    lo, hi = TORSO_LIMITS[idx]
    cluster.value(idx, max(lo, min(hi, float(angle))) - 90.0, load)

def torso_ramp(from_frac, to_frac, ms):
    t0 = time.ticks_ms()
    while True:
        el = time.ticks_diff(time.ticks_ms(), t0)
        if el >= ms: break
        f = from_frac + (to_frac - from_frac) * (el / ms)
        for _, idx, stand, flat in TORSO_CAL:
            torso_raw(idx, flat + (stand - flat) * f, False)
        cluster.load()
        time.sleep_ms(TICK_MS)
    for _, idx, stand, flat in TORSO_CAL:
        torso_raw(idx, flat + (stand - flat) * to_frac, False)
    cluster.load()

def set_home(home):
    for li, ri in ((0, 1), (2, 3), (4, 5), (6, 7)):
        leg_set(li, home[0], False)
        leg_set(ri, home[1], False)
    cluster.load()


# ---- gait helpers ----

def _ease(t):
    return t * t * (3.0 - 2.0 * t)

def foot_pos(phase, direction=1):
    sw = 1.0 - STANCE_DUTY
    if phase < sw:
        t = phase / sw
        x = direction * (-STRIDE + 2.0 * STRIDE * _ease(t))
        y = STANCE_Y - LIFT * math.sin(math.pi * t)
    else:
        t = (phase - sw) / STANCE_DUTY
        x = direction * (STRIDE - 2.0 * STRIDE * t)
        y = STANCE_Y
    return x, y

def apply_leg(li, ri, phase, fwd=1, direction=1):
    fx, fy = foot_pos(phase, direction)
    ik = legIK(fx * fwd, fy)
    if ik is None: return
    leg_set(li, ik[0], False)
    leg_set(ri, ik[1], False)


# ---- serial command read (non-blocking) ----

def read_cmd():
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        b = sys.stdin.read(1)
        return ord(b) if b else None
    return None


# ---- main ----

def main():
    for _, idx, _, flat in TORSO_CAL:
        torso_raw(idx, flat, False)
    home = legIK(0.0, STANCE_Y)
    if home is None:
        print("ERROR: home position unreachable — check L1/L2/HS")
        return
    set_home(home)
    cluster.load()
    cluster.enable_all()
    time.sleep_ms(500)

    print("standing up...")
    torso_ramp(0.0, 1.0, 2500)

    tof_obj, tof_method = setup_tof()
    last_dist_mm = 9999
    tof_counter  = 0

    print("ready — f/b/l/r/s")

    mode = STOP
    t0   = time.ticks_ms()

    while True:
        cmd = read_cmd()
        if cmd is not None:
            new_mode = CMD_MAP.get(cmd, mode)
            if new_mode != mode:
                mode = new_mode
                if mode == STOP:
                    set_home(home)

        # poll ToF every TOF_INTERVAL ticks
        tof_counter += 1
        if tof_counter >= TOF_INTERVAL:
            tof_counter = 0
            if tof_obj is not None:
                last_dist_mm = tof_read_mm(tof_obj, tof_method)

        obstacle = (last_dist_mm < OBSTACLE_MM)

        if mode != STOP:
            el = time.ticks_diff(time.ticks_ms(), t0)
            p  = (el % CYCLE_MS) / CYCLE_MS
            pB = (p + 0.5) % 1.0

            if mode == WALK_FWD and not obstacle:
                apply_leg(0, 1, p,  fwd=-1, direction= 1)
                apply_leg(6, 7, p,  fwd= 1, direction= 1)
                apply_leg(2, 3, pB, fwd= 1, direction= 1)
                apply_leg(4, 5, pB, fwd=-1, direction= 1)
            elif mode == WALK_BACK:
                apply_leg(0, 1, p,  fwd=-1, direction=-1)
                apply_leg(6, 7, p,  fwd= 1, direction=-1)
                apply_leg(2, 3, pB, fwd= 1, direction=-1)
                apply_leg(4, 5, pB, fwd=-1, direction=-1)
            elif mode == TURN_L:
                # right side (fwd=1 legs) forward, left side (fwd=-1 legs) backward
                # if it spins the wrong way, swap TURN_L and TURN_R in CMD_MAP
                apply_leg(0, 1, p,  fwd=-1, direction=-1)
                apply_leg(6, 7, p,  fwd= 1, direction= 1)
                apply_leg(2, 3, pB, fwd= 1, direction= 1)
                apply_leg(4, 5, pB, fwd=-1, direction=-1)
            elif mode == TURN_R:
                apply_leg(0, 1, p,  fwd=-1, direction= 1)
                apply_leg(6, 7, p,  fwd= 1, direction=-1)
                apply_leg(2, 3, pB, fwd= 1, direction=-1)
                apply_leg(4, 5, pB, fwd=-1, direction= 1)

            cluster.load()

        time.sleep_ms(TICK_MS)


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    cluster.disable_all()
    print("servos disabled")
