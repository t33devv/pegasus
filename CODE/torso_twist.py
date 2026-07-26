# Torso twist — Pimoroni Servo 2040 (MicroPython)
#
# Rocks the dog side to side: torsos lean one side out / other side in,
# while the leg motors exaggerate the effect — legs shorten on the torso-out
# side and lengthen on the torso-in side.
#
# Torso pairing (physical mirrors, need opposite frac directions to lean together):
#   Side A (torsos 9 & 12, idx 8 & 11) → legs 1 & 4 (idx 0-1, 6-7)
#   Side B (torsos 10 & 11, idx 9 & 10) → legs 2 & 3 (idx 2-3, 4-5)
#
# Right lean: side A torsos out  + side A legs short / side B torsos in  + side B legs long
# Left lean:  side B torsos out  + side B legs short / side A torsos in  + side A legs long

import time
import math
from servo import ServoCluster, Calibration, servo2040

cal = Calibration()
cal.apply_two_pairs(610, 2441, -90, 90)
pins = list(range(servo2040.SERVO_1, servo2040.SERVO_12 + 1))
cluster = ServoCluster(0, 0, pins, cal)

# ---- IK constants (same as crawl_gait) ----
L1 = 30.0
L2 = 120.0
HS = 46.0
LEG_MIN = 20.0
LEG_MAX = 160.0

# ---- tuning ----
STANCE_Y   = 132.0   # neutral foot depth (mm)
LEG_DIFF   =  15.0   # how much to raise/lower foot during lean (mm)
NEUTRAL    =  0.85   # neutral torso frac
LEAN       =  0.25   # torso lean magnitude (larger = more dramatic)
ROCK_COUNT =   4     # full rock cycles (right + left = 1 cycle)
RAMP_MS    = 1000    # ms per lean direction

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

# Lean targets — within each pair, opposite frac directions produce the same physical lean
LEAN_RIGHT = {8: NEUTRAL-LEAN, 9: NEUTRAL+LEAN, 10: NEUTRAL-LEAN, 11: NEUTRAL+LEAN}
LEAN_LEFT  = {8: NEUTRAL+LEAN, 9: NEUTRAL-LEAN, 10: NEUTRAL+LEAN, 11: NEUTRAL-LEAN}
NEUTRAL_T  = {8: NEUTRAL,      9: NEUTRAL,       10: NEUTRAL,      11: NEUTRAL}


# ---- IK ----

def legIK(fx, fy):
    dxL = fx - HS/2;  dxR = fx + HS/2
    DL = math.sqrt(dxL*dxL + fy*fy);  DR = math.sqrt(dxR*dxR + fy*fy)
    if not (abs(L2-L1) < DL < L1+L2): return None
    if not (abs(L2-L1) < DR < L1+L2): return None
    cosL = max(-1.0, min(1.0, (L1*L1+DL*DL-L2*L2)/(2*L1*DL)))
    cosR = max(-1.0, min(1.0, (L1*L1+DR*DR-L2*L2)/(2*L1*DR)))
    phi_L = math.atan2(dxL, fy) + math.acos(cosL)
    phi_R = math.atan2(-dxR, fy) + math.acos(cosR)
    return 135.0 - math.degrees(phi_L), math.degrees(phi_R) + 45.0


# ---- servo helpers ----

def torso_raw(idx, angle, load=True):
    lo, hi = TORSO_LIMITS[idx]
    cluster.value(idx, max(lo, min(hi, float(angle))) - 90.0, load)

def leg_set(idx, angle, load=True):
    cluster.value(idx, max(LEG_MIN, min(LEG_MAX, float(angle))) - 90.0, load)


# ---- build leg angle arrays from IK ----

def make_leg_angles(depth_a, depth_b):
    """
    depth_a: foot depth for side A legs (1 & 4, idx 0-1, 6-7)
    depth_b: foot depth for side B legs (2 & 3, idx 2-3, 4-5)
    Returns list of 8 servo angles [idx0..idx7], or None if IK fails.
    """
    ika = legIK(0.0, depth_a)
    ikb = legIK(0.0, depth_b)
    if ika is None or ikb is None:
        return None
    angles = [0.0] * 8
    angles[0], angles[1] = ika   # leg 1 (side A)
    angles[2], angles[3] = ikb   # leg 2 (side B)
    angles[4], angles[5] = ikb   # leg 3 (side B)
    angles[6], angles[7] = ika   # leg 4 (side A)
    return angles


# ---- combined torso + leg ramp ----

def ramp_combined(torso_from, torso_to, legs_from, legs_to, ms):
    t0 = time.ticks_ms()
    while True:
        el = time.ticks_diff(time.ticks_ms(), t0)
        if el >= ms:
            break
        t = el / ms
        for _, idx, stand, flat in TORSO_CAL:
            f = torso_from[idx] + (torso_to[idx] - torso_from[idx]) * t
            torso_raw(idx, flat + (stand - flat) * f, False)
        for i in range(8):
            leg_set(i, legs_from[i] + (legs_to[i] - legs_from[i]) * t, False)
        cluster.load()
        time.sleep_ms(20)
    for _, idx, stand, flat in TORSO_CAL:
        torso_raw(idx, flat + (stand - flat) * torso_to[idx], False)
    for i in range(8):
        leg_set(i, legs_to[i], False)
    cluster.load()

def ramp_all(from_frac, to_frac, ms):
    f = {idx: from_frac for _, idx, _, _ in TORSO_CAL}
    t = {idx: to_frac   for _, idx, _, _ in TORSO_CAL}
    ramp_combined(f, t,
                  make_leg_angles(STANCE_Y, STANCE_Y),
                  make_leg_angles(STANCE_Y, STANCE_Y), ms)


# ---- main ----

def main():
    # Compute leg angle targets
    legs_neutral = make_leg_angles(STANCE_Y,            STANCE_Y)
    legs_right   = make_leg_angles(STANCE_Y - LEG_DIFF, STANCE_Y + LEG_DIFF)  # A short, B long
    legs_left    = make_leg_angles(STANCE_Y + LEG_DIFF, STANCE_Y - LEG_DIFF)  # A long, B short

    if None in (legs_neutral, legs_right, legs_left):
        print("ERROR: IK out of range — reduce LEG_DIFF")
        return

    # Seed and enable
    for _, idx, _, flat in TORSO_CAL:
        torso_raw(idx, flat, False)
    for i in range(8):
        leg_set(i, legs_neutral[i], False)
    cluster.load()
    cluster.enable_all()
    time.sleep_ms(500)

    print("standing up...")
    ramp_all(0.0, 1.0, 2500)
    ramp_all(1.0, NEUTRAL, 500)
    time.sleep(1)

    print("rocking —", ROCK_COUNT, "cycles")
    torso_cur = dict(NEUTRAL_T)
    legs_cur  = list(legs_neutral)

    for _ in range(ROCK_COUNT):
        ramp_combined(torso_cur, LEAN_RIGHT, legs_cur, legs_right, RAMP_MS)
        torso_cur = dict(LEAN_RIGHT);  legs_cur = list(legs_right)

        ramp_combined(torso_cur, LEAN_LEFT, legs_cur, legs_left, RAMP_MS)
        torso_cur = dict(LEAN_LEFT);  legs_cur = list(legs_left)

    # Return to neutral
    ramp_combined(torso_cur, NEUTRAL_T, legs_cur, legs_neutral, RAMP_MS // 2)
    time.sleep(1)

    print("lying down...")
    ramp_all(NEUTRAL, 0.0, 2500)
    print("done")


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    cluster.disable_all()
    print("servos disabled")
