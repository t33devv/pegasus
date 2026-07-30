# teleop_dog.py — Pimoroni Servo 2040 (MicroPython)
#
# Listens for commands over USB serial and drives the gait:
#   f = walk forward    b = walk backward
#   l = turn left       r = turn right  (continuous, until 's')
#   s / Space = stop (hold stand)
#   L<N>\n = IMU-guided left turn N degrees   e.g. "L90\n"
#   R<N>\n = IMU-guided right turn N degrees  e.g. "R45\n"
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

# ---- IMU (MPU6050) for precise angle turns ----
MPU_ADDR       = 0x68
_GYRO_XOUT     = 0x43   # start of 6-byte gyro block (same as balance.py)
_GYRO_SCALE    = 131.0  # LSB per °/s at ±250°/s default range
_IMU_CAL_N     = 100    # calibration samples (~2 s) — matches balance.py
# Moving-average window to smooth gait vibration before integrating.
_GZ_SMOOTH_N   = 5
# Sign of gz for each turn direction — confirmed from diagnose_imu output:
# R turns produce negative gz_raw; L turns produce positive gz_raw.
# If one direction consistently over/underturns, flip its sign here.
GZ_SIGN        = {'L': 1, 'R': -1}
# Minimum signed gz to accumulate — filters standstill noise.
GZ_MIN_DPS     = 3.0
# Abort if the turn hasn't finished within this multiple of the estimated time.
GYRO_TIMEOUT_FACTOR = 4

def _find_i2c(target_addr):
    """Return an I2C bus object that has target_addr on it, or None."""
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
            if target_addr in found:
                return i2c
        except Exception:
            pass
    return None

def setup_tof():
    """Return (tof_object, read_method_name) or (None, None) if unavailable."""
    i2c = _find_i2c(0x29)
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


def _imu_read_gyro(i2c):
    """Read all 6 gyro bytes in one I2C transaction — same pattern as balance.py.
    Returns (gx_raw, gy_raw, gz_raw) as signed integers (LSB, not yet scaled)."""
    g = i2c.readfrom_mem(MPU_ADDR, _GYRO_XOUT, 6)
    def s16(hi, lo):
        v = (hi << 8) | lo
        return v - 65536 if v > 32767 else v
    return s16(g[0], g[1]), s16(g[2], g[3]), s16(g[4], g[5])

def setup_imu():
    """Find MPU6050, wake it, calibrate gz offset. Returns (i2c, gz_offset_raw) or (None, 0).
    gz_offset_raw is in raw LSB — subtracted before dividing by _GYRO_SCALE.
    Keep Pegasus still during the calibration window."""
    i2c = _find_i2c(MPU_ADDR)
    if i2c is None:
        print("IMU: not found — angle turns fall back to open-loop timing")
        return None, 0
    i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00')   # wake up (clear sleep bit)
    time.sleep_ms(500)   # let the board settle after standing up
    print("IMU: calibrating gz ({} samples, keep still)...".format(_IMU_CAL_N))
    gz_sum = 0
    for _ in range(_IMU_CAL_N):
        _, _, gz_raw = _imu_read_gyro(i2c)
        gz_sum += gz_raw
        time.sleep_ms(20)
    gz_offset_raw = gz_sum // _IMU_CAL_N   # integer raw LSB offset
    print("IMU: ready, gz_offset_raw={}".format(gz_offset_raw))
    return i2c, gz_offset_raw


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
# Single-byte commands (f/b/l/r/s) are returned as ord() integers as before.
# Angle commands L<N>\n / R<N>\n are accumulated over multiple ticks and
# returned as ('imu_turn', 'L'|'R', degrees_float) when \n is received.

_cmd_buf = []

def read_cmd():
    global _cmd_buf
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if not r:
        return None
    b = sys.stdin.read(1)
    if not b:
        return None
    ch = b   # MicroPython sys.stdin.read(1) returns a str

    # Accumulating a buffered angle command
    if _cmd_buf:
        if ch.isdigit():
            _cmd_buf.append(ch)
            return None
        elif ch == '\n':
            direction = _cmd_buf[0]
            try:
                degrees = float(''.join(_cmd_buf[1:]))
            except Exception:
                degrees = 0.0
            _cmd_buf = []
            return ('imu_turn', direction, degrees) if degrees > 0 else None
        else:
            _cmd_buf = []   # unexpected char — abandon and fall through

    # Start of a buffered angle command
    if ch in ('L', 'R'):
        _cmd_buf = [ch]
        return None

    # Single-byte commands (unchanged)
    return ord(ch)


# ---- IMU-guided turn ----

def imu_guided_turn(direction, target_deg, home, imu_i2c, gz_offset_raw):
    """
    Turn by target_deg degrees using gyro integration to know when to stop.
    direction: 'L' (left) or 'R' (right).

    Reads gyro the same way as balance.py — all 6 bytes in one I2C burst from
    0x43, subtracts the raw-integer offset calibrated at startup, then applies
    a _GZ_SMOOTH_N-tick moving average to reduce gait vibration before integrating.
    """
    if imu_i2c is None or target_deg <= 0:
        return

    # TURN_R physically turns left because CMD_MAP already swapped the labels.
    gait_mode = TURN_R if direction == 'L' else TURN_L

    gz_buf      = [0] * _GZ_SMOOTH_N   # raw-LSB ring buffer for smoothing
    buf_idx     = 0
    accumulated = 0.0
    t_prev      = time.ticks_ms()
    t0          = t_prev
    timeout_ms  = int(target_deg / 22.5 * 1000 * GYRO_TIMEOUT_FACTOR)
    print("IMU turn: {}{:.0f}deg".format(direction, target_deg))

    while accumulated < target_deg:
        _, _, gz_raw = _imu_read_gyro(imu_i2c)
        gz_buf[buf_idx] = gz_raw - gz_offset_raw
        buf_idx = (buf_idx + 1) % _GZ_SMOOTH_N

        # Signed: positive = rotating in the commanded direction.
        # Gait vibration spikes in the wrong direction subtract from the
        # buffer average rather than adding (unlike abs which counts both).
        gz_dps = GZ_SIGN[direction] * (sum(gz_buf) / _GZ_SMOOTH_N) / _GYRO_SCALE

        t_now = time.ticks_ms()
        dt    = time.ticks_diff(t_now, t_prev) / 1000.0
        t_prev = t_now
        if gz_dps > GZ_MIN_DPS:
            accumulated += gz_dps * dt

        if time.ticks_diff(t_now, t0) > timeout_ms:
            print("IMU turn: timeout after {:.1f}deg".format(accumulated))
            break

        # One gait tick — mirrors the main loop turn logic exactly
        el = time.ticks_diff(t_now, t0)
        p  = (el % CYCLE_MS) / CYCLE_MS
        pB = (p + 0.5) % 1.0

        if gait_mode == TURN_L:
            apply_leg(0, 1, p,  fwd=-1, direction=-1)
            apply_leg(6, 7, p,  fwd= 1, direction= 1)
            apply_leg(2, 3, pB, fwd= 1, direction= 1)
            apply_leg(4, 5, pB, fwd=-1, direction=-1)
        else:
            apply_leg(0, 1, p,  fwd=-1, direction= 1)
            apply_leg(6, 7, p,  fwd= 1, direction=-1)
            apply_leg(2, 3, pB, fwd= 1, direction=-1)
            apply_leg(4, 5, pB, fwd=-1, direction= 1)

        cluster.load()
        time.sleep_ms(TICK_MS)

    set_home(home)
    print("IMU turn done: {:.1f}deg".format(accumulated))


# ---- one-shot action commands ----

_torso_frac = 0.0   # tracked so stand/lay always ramp from the current position

# Foot index → (left_servo, right_servo) pairs.
# Order: 0=front-right, 1=front-left, 2=back-right, 3=back-left.
# NOTE: left/right is assumed — swap pairs if the wrong foot lifts.
_FOOT_LEGS = [(0, 1), (2, 3), (4, 5), (6, 7)]

def cmd_stand_up(home):
    global _torso_frac
    torso_ramp(_torso_frac, 1.0, 1500)
    _torso_frac = 1.0
    set_home(home)
    print("standing")

def cmd_lay_down():
    global _torso_frac
    torso_ramp(_torso_frac, 0.0, 1500)
    _torso_frac = 0.0
    print("lying down")

def cmd_jump(home):
    # Smooth large-range dip-and-rise: sink body low then lift it high.
    # IK is valid for fy ∈ (87, 148) with fx=0 given L1=30, L2=120, HS=46.
    LOW_Y  = 146.0   # body sunk low  (legs extended toward ground)
    HIGH_Y =  93.0   # body lifted high (legs retracted)
    for y_from, y_to, ms in (
        (STANCE_Y, LOW_Y,   700),   # sink
        (LOW_Y,    HIGH_Y, 900),    # rise
        (HIGH_Y,   STANCE_Y, 600),  # return
    ):
        t0 = time.ticks_ms()
        while True:
            el = time.ticks_diff(time.ticks_ms(), t0)
            if el >= ms:
                break
            y  = y_from + (y_to - y_from) * (el / ms)
            ik = legIK(0.0, y)
            if ik:
                for li, ri in ((0,1),(2,3),(4,5),(6,7)):
                    leg_set(li, ik[0], False)
                    leg_set(ri, ik[1], False)
                cluster.load()
            time.sleep_ms(TICK_MS)
        ik = legIK(0.0, y_to)
        if ik:
            for li, ri in ((0,1),(2,3),(4,5),(6,7)):
                leg_set(li, ik[0], False)
                leg_set(ri, ik[1], False)
            cluster.load()
    set_home(home)
    print("jumped")

def cmd_fast_feet(home, cycles=4):
    # Lift each foot in sequence very quickly — like "fast feet" running drill
    up = legIK(0.0, STANCE_Y - 20.0)
    if up is None:
        return
    for _ in range(cycles):
        for li, ri in _FOOT_LEGS:
            leg_set(li, up[0], False)
            leg_set(ri, up[1], False)
            cluster.load()
            time.sleep_ms(60)
            leg_set(li, home[0], False)
            leg_set(ri, home[1], False)
            cluster.load()
            time.sleep_ms(60)
    print("fast feet done")

def cmd_lift_foot(foot_idx, home):
    up = legIK(0.0, STANCE_Y - 28.0)
    if up is None:
        return
    set_home(home)
    li, ri = _FOOT_LEGS[foot_idx]
    leg_set(li, up[0], False)
    leg_set(ri, up[1], False)
    cluster.load()
    print("lifted foot {}".format(foot_idx + 1))
    time.sleep_ms(2000)
    set_home(home)


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
    global _torso_frac
    _torso_frac = 1.0   # track torso position so stand/lay ramp from correct state

    tof_obj, tof_method = setup_tof()
    imu_i2c, gz_offset_raw = setup_imu()
    last_dist_mm = 9999
    tof_counter  = 0

    print("ready — f/b/l/r/s  |  L<deg>\\n / R<deg>\\n  |  U/D/J/T/1-4 one-shot actions")

    # One-shot action byte map: U=stand_up D=lay_down J=jump T=fast_feet 1-4=lift_foot
    ONE_SHOT = {
        ord('U'): ('stand_up',),
        ord('D'): ('lay_down',),
        ord('J'): ('jump',),
        ord('T'): ('fast_feet',),
        ord('1'): ('lift_foot', 0),
        ord('2'): ('lift_foot', 1),
        ord('3'): ('lift_foot', 2),
        ord('4'): ('lift_foot', 3),
    }

    mode = STOP
    t0   = time.ticks_ms()

    while True:
        cmd = read_cmd()
        if cmd is not None:
            if isinstance(cmd, tuple):
                # IMU-guided angle turn — blocking until target reached
                _, direction, degrees = cmd
                imu_guided_turn(direction, degrees, home, imu_i2c, gz_offset_raw)
                mode = STOP
                t0 = time.ticks_ms()
            elif cmd in ONE_SHOT:
                act = ONE_SHOT[cmd]
                if act[0] == 'stand_up':
                    cmd_stand_up(home)
                elif act[0] == 'lay_down':
                    cmd_lay_down()
                elif act[0] == 'jump':
                    cmd_jump(home)
                elif act[0] == 'fast_feet':
                    cmd_fast_feet(home)
                elif act[0] == 'lift_foot':
                    cmd_lift_foot(act[1], home)
                mode = STOP
                t0 = time.ticks_ms()
            else:
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
