# balance.py — Active roll stabilisation using MPU6050 + torso servos
# Pimoroni Servo 2040 (MicroPython)
#
# The dog stands up, auto-calibrates the gyro (2 s), then continuously adjusts
# the four torso servos to counteract platform roll (side-to-side tilt).
#
# Calibration: automatic at startup — keep the dog STILL for the first 2 s
# after it stands up. No manual steps needed.
#
# Tuning:
#   ROLL_GAIN  — torso fraction per degree of tilt. Increase for stronger
#                response; if it oscillates, decrease it.
#   ALPHA      — complementary filter gyro weight (0.95–0.99). Higher = smoother
#                but slower to respond to fast tilts.
#   ROLL_DIR   — set to -1 if the torsos compensate the wrong way.

import time
import math
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
STANCE_Y = 132.0

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

# ---- balance tuning ----
ROLL_GAIN  = 0.022  # torso fraction per degree of roll
ALPHA      = 0.95   # complementary filter gyro weight — lower = faster response
MAX_LEAN   = 0.25   # hard cap on torso lean fraction offset
ROLL_DIR   = 1      # flip to -1 if roll compensation goes the wrong way

PITCH_GAIN = 1.0    # mm of leg depth change per degree of pitch
MAX_PITCH  = 15.0   # max depth offset (mm) — keeps IK within safe range
PITCH_DIR  = 1      # flip to -1 if pitch compensation goes the wrong way

TICK_MS = 20

# front/back leg grouping for pitch compensation (assumed — swap if wrong)
FRONT_LEGS = [(0, 1), (2, 3)]   # legs 1 & 2
BACK_LEGS  = [(4, 5), (6, 7)]   # legs 3 & 4

# ---- MPU6050 constants ----
MPU_ADDR    = 0x68   # change to 0x69 if AD0 is pulled high
_PWR_MGMT_1 = 0x6B
_ACCEL_XOUT = 0x3B
_GYRO_XOUT  = 0x43
_ACCEL_SCALE = 16384.0   # LSB/g    at ±2 g default
_GYRO_SCALE  = 131.0     # LSB/°/s  at ±250 °/s default
_CAL_SAMPLES = 100       # gyro calibration readings (~2 s)


# ================================================================
# IMU
# ================================================================

def find_i2c():
    combos = []
    try:
        combos.append((0, servo2040.SDA, servo2040.SCL))
        combos.append((1, servo2040.SDA, servo2040.SCL))
    except Exception:
        pass
    for bus in (0, 1):
        for sda, scl in ((4, 5), (20, 21), (0, 1), (2, 3)):
            combos.append((bus, sda, scl))
    for bus, sda, scl in combos:
        try:
            i2c = I2C(bus, sda=Pin(sda), scl=Pin(scl))
            found = i2c.scan()
            if len(found) > 20: continue
            if MPU_ADDR in found:
                print("MPU6050: bus={} sda={} scl={}".format(bus, sda, scl))
                return i2c
        except Exception:
            pass
    return None

def imu_read_raw(i2c):
    a = i2c.readfrom_mem(MPU_ADDR, _ACCEL_XOUT, 6)
    g = i2c.readfrom_mem(MPU_ADDR, _GYRO_XOUT,  6)
    def s16(hi, lo):
        v = (hi << 8) | lo
        return v - 65536 if v > 32767 else v
    return (s16(a[0],a[1]), s16(a[2],a[3]), s16(a[4],a[5]),
            s16(g[0],g[1]), s16(g[2],g[3]), s16(g[4],g[5]))


# ================================================================
# Servo helpers
# ================================================================

def legIK(fx, fy):
    dxL = fx - HS/2;  dxR = fx + HS/2
    DL = math.sqrt(dxL*dxL + fy*fy)
    DR = math.sqrt(dxR*dxR + fy*fy)
    if not (abs(L2-L1) < DL < L1+L2): return None
    if not (abs(L2-L1) < DR < L1+L2): return None
    cosL = max(-1.0, min(1.0, (L1*L1+DL*DL-L2*L2)/(2*L1*DL)))
    cosR = max(-1.0, min(1.0, (L1*L1+DR*DR-L2*L2)/(2*L1*DR)))
    phi_L = math.atan2(dxL, fy) + math.acos(cosL)
    phi_R = math.atan2(-dxR, fy) + math.acos(cosR)
    return 135.0 - math.degrees(phi_L), math.degrees(phi_R) + 45.0

def leg_set(idx, angle):
    cluster.value(idx, max(LEG_MIN, min(LEG_MAX, float(angle))) - 90.0, False)

def torso_raw(idx, angle):
    lo, hi = TORSO_LIMITS[idx]
    cluster.value(idx, max(lo, min(hi, float(angle))) - 90.0, False)

def torso_ramp(from_frac, to_frac, ms):
    t0 = time.ticks_ms()
    while True:
        el = time.ticks_diff(time.ticks_ms(), t0)
        if el >= ms: break
        f = from_frac + (to_frac - from_frac) * (el / ms)
        for _, idx, stand, flat in TORSO_CAL:
            torso_raw(idx, flat + (stand - flat) * f)
        cluster.load()
        time.sleep_ms(TICK_MS)
    for _, idx, stand, flat in TORSO_CAL:
        torso_raw(idx, flat + (stand - flat) * to_frac)
    cluster.load()

def apply_torso_lean(lean_frac):
    """
    Apply a lateral lean offset to all torso servos.
    Positive lean_frac tilts one way; negative tilts the other.
    Indices 8 & 10 are one physical side, 9 & 11 the other —
    same pattern as all_dog_test.py LEAN_LEFT / LEAN_RIGHT.
    torso_raw already clamps to calibrated limits, so no outer clamp here.
    """
    for _, idx, stand, flat in TORSO_CAL:
        sign = -1 if idx in (8, 10) else 1
        frac = 1.0 + sign * lean_frac
        torso_raw(idx, flat + (stand - flat) * frac)


# ================================================================
# Main
# ================================================================

def main():
    # Find IMU
    i2c = find_i2c()
    if i2c is None:
        print("MPU6050 not found — check wiring (VCC->3V3, GND, SDA, SCL)")
        return
    i2c.writeto_mem(MPU_ADDR, _PWR_MGMT_1, bytes([0x00]))  # wake up
    time.sleep_ms(100)

    # Stand up
    for _, idx, _, flat in TORSO_CAL:
        torso_raw(idx, flat)
    home = legIK(0.0, STANCE_Y)
    if home is None:
        print("ERROR: home position unreachable"); return
    for li, ri in ((0,1),(2,3),(4,5),(6,7)):
        leg_set(li, home[0])
        leg_set(ri, home[1])
    cluster.load()
    cluster.enable_all()
    time.sleep_ms(500)

    print("standing up...")
    torso_ramp(0.0, 1.0, 2500)

    # Gyro calibration — sample at rest
    print("keep still — calibrating gyro ({} samples)...".format(_CAL_SAMPLES))
    gy_off = 0
    gx_off = 0
    for _ in range(_CAL_SAMPLES):
        _, _, _, gx, gy, _ = imu_read_raw(i2c)
        gy_off += gy
        gx_off += gx
        time.sleep_ms(TICK_MS)
    gy_off //= _CAL_SAMPLES
    gx_off //= _CAL_SAMPLES
    print("gyro offsets: gx={}  gy={}".format(gx_off, gy_off))

    # Seed both complementary filters from accelerometer
    ax, ay, az, _, _, _ = imu_read_raw(i2c)
    roll_deg  = math.degrees(math.atan2(ax / _ACCEL_SCALE, az / _ACCEL_SCALE))
    pitch_deg = math.degrees(math.atan2(ay / _ACCEL_SCALE, az / _ACCEL_SCALE))
    dt = TICK_MS / 1000.0

    print("balance active — Ctrl-C to stop")
    print("{:>10}  {:>8}  note".format("roll (°)", "lean"))

    try:
        while True:
            t0 = time.ticks_ms()

            ax, ay, az, gx, gy, _ = imu_read_raw(i2c)

            # gyro rates in °/s, offset-corrected
            gy_rate = (gy - gy_off) / _GYRO_SCALE
            gx_rate = (gx - gx_off) / _GYRO_SCALE

            # complementary filter — roll (side tilt, ax axis)
            roll_accel = math.degrees(math.atan2(ax / _ACCEL_SCALE, az / _ACCEL_SCALE))
            roll_deg   = ALPHA * (roll_deg  + gy_rate * dt) + (1.0 - ALPHA) * roll_accel

            # complementary filter — pitch (front/back tilt, ay axis)
            pitch_accel = math.degrees(math.atan2(ay / _ACCEL_SCALE, az / _ACCEL_SCALE))
            pitch_deg   = ALPHA * (pitch_deg + gx_rate * dt) + (1.0 - ALPHA) * pitch_accel

            # roll → torso lean
            lean = max(-MAX_LEAN, min(MAX_LEAN, roll_deg * ROLL_GAIN * ROLL_DIR))
            apply_torso_lean(lean)

            # pitch → front/back leg depth adjustment
            pitch_offset = max(-MAX_PITCH, min(MAX_PITCH, pitch_deg * PITCH_GAIN * PITCH_DIR))
            for li, ri in FRONT_LEGS:
                ik = legIK(0.0, STANCE_Y + pitch_offset)
                if ik:
                    leg_set(li, ik[0])
                    leg_set(ri, ik[1])
            for li, ri in BACK_LEGS:
                ik = legIK(0.0, STANCE_Y - pitch_offset)
                if ik:
                    leg_set(li, ik[0])
                    leg_set(ri, ik[1])
            cluster.load()

            print("roll={:6.1f}° lean={:.3f}  pitch={:6.1f}° depth_off={:+.1f}mm".format(
                roll_deg, lean, pitch_deg, pitch_offset))

            elapsed = time.ticks_diff(time.ticks_ms(), t0)
            wait = TICK_MS - elapsed
            if wait > 0:
                time.sleep_ms(wait)

    except KeyboardInterrupt:
        pass

    print("lying down...")
    torso_ramp(1.0, 0.0, 2000)


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    cluster.disable_all()
    print("servos disabled")
