# Tripedal balance test — Pimoroni Servo 2040 (MicroPython)
#
# For each leg in sequence:
#   1. Shorten the opposite diagonal corner slightly → shifts CG over the
#      remaining support triangle before the lift.
#   2. Raise the target leg (foot well off the ground).
#   3. Hold, then return to neutral.
#
# Diagonal pairs (opposite corners):  leg 1 ↔ leg 4,  leg 2 ↔ leg 3
#
# Leg → motor indices:
#   Leg 1: idx 0 (L), 1 (R)     Leg 2: idx 2 (L), 3 (R)
#   Leg 3: idx 4 (L), 5 (R)     Leg 4: idx 6 (L), 7 (R)
#   Torsos: idx 8–11

import time
import math
from servo import ServoCluster, Calibration, servo2040

cal = Calibration()
cal.apply_two_pairs(610, 2441, -90, 90)
pins = list(range(servo2040.SERVO_1, servo2040.SERVO_12 + 1))
cluster = ServoCluster(0, 0, pins, cal)

# ---- IK ----
L1 = 30.0;  L2 = 120.0;  HS = 46.0
LEG_MIN = 20.0;  LEG_MAX = 160.0

# ---- tuning ----
STANCE_Y      = 132.0   # normal standing depth (mm)
BALANCE_SHORT = 117.0   # opposite corner depth — slightly shorter shifts CG
RAISED_Y      =  100.0  # raised leg depth — foot well clear of the ground
HOLD_MS       =  600    # ms to hold the raised pose
RAMP_MS       =  350    # ms per transition

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

# Raise each leg in turn; opposite diagonal corner gets shortened first.
# Format: (leg to raise, opposite corner leg)
LIFT_SEQUENCE = [(1, 4), (2, 3), (3, 2), (4, 1)]

# Motor index pairs for each leg (left, right)
LEG_MOTORS = {1: (0, 1), 2: (2, 3), 3: (4, 5), 4: (6, 7)}


# ---- helpers ----

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
        time.sleep_ms(20)
    for _, idx, stand, flat in TORSO_CAL:
        torso_raw(idx, flat + (stand - flat) * to_frac, False)
    cluster.load()


def depths_to_angles(d1, d2, d3, d4):
    """Convert per-leg foot depths to 8 servo angles via IK."""
    angles = [0.0] * 8
    for leg, depth in ((1, d1), (2, d2), (3, d3), (4, d4)):
        ik = legIK(0.0, depth)
        if ik is None:
            return None
        li, ri = LEG_MOTORS[leg]
        angles[li], angles[ri] = ik
    return angles

def ramp_legs(from_a, to_a, ms):
    t0 = time.ticks_ms()
    while True:
        el = time.ticks_diff(time.ticks_ms(), t0)
        if el >= ms: break
        t = el / ms
        for i in range(8):
            leg_set(i, from_a[i] + (to_a[i] - from_a[i]) * t, False)
        cluster.load()
        time.sleep_ms(20)
    for i in range(8):
        leg_set(i, to_a[i], False)
    cluster.load()


def balance_one(raise_leg, opposite_leg):
    """Full sequence: shorten opposite corner → raise leg → hold → return."""
    print("  raising leg", raise_leg, "(opposite corner: leg", str(opposite_leg) + ")")

    S = STANCE_Y
    neutral  = depths_to_angles(S, S, S, S)

    # Build depth tuples with raise_leg and opposite_leg swapped in
    def make_depths(raise_d, opp_d):
        d = [S, S, S, S]
        d[raise_leg   - 1] = raise_d
        d[opposite_leg - 1] = opp_d
        return depths_to_angles(*d)

    balanced = make_depths(S,        BALANCE_SHORT)   # opposite shortened
    raised   = make_depths(RAISED_Y, BALANCE_SHORT)   # target lifted

    if None in (neutral, balanced, raised):
        print("    IK out of range — skipping")
        return

    # 1. Shorten opposite corner to pre-shift the CG
    ramp_legs(neutral, balanced, RAMP_MS)
    time.sleep_ms(400)

    # 2. Lift the target leg
    ramp_legs(balanced, raised, RAMP_MS)
    time.sleep_ms(HOLD_MS)

    # 3. Lower back down
    ramp_legs(raised, balanced, RAMP_MS)
    time.sleep_ms(200)

    # 4. Restore opposite corner
    ramp_legs(balanced, neutral, RAMP_MS // 2)
    time.sleep_ms(500)


def main():
    S = STANCE_Y
    neutral = depths_to_angles(S, S, S, S)
    if neutral is None:
        print("ERROR: IK failed at neutral stance")
        return

    # Seed before enabling
    for _, idx, _, flat in TORSO_CAL:
        torso_raw(idx, flat, False)
    for i in range(8):
        leg_set(i, neutral[i], False)
    cluster.load()
    cluster.enable_all()
    time.sleep_ms(500)

    print("standing up...")
    torso_ramp(0.0, 1.0, 2500)
    time.sleep(1)

    print("tripedal balance sequence")
    for raise_leg, opposite_leg in LIFT_SEQUENCE:
        balance_one(raise_leg, opposite_leg)

    print("lying down...")
    torso_ramp(1.0, 0.0, 2500)
    print("done")


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    cluster.disable_all()
    print("servos disabled")
