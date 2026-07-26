# all_dog_test.py — Full demo sequence, Pimoroni Servo 2040 (MicroPython)
#
# Sequence:
#   1. Stand up
#   2. Walk forward  5 cycles
#   3. Torso twist   3 rocks
#   4. Walk forward  5 cycles
#   5. Tripedal balance — raise legs 1 & 2
#   6. Walk backward 5 cycles
#   7. Torso twist   3 rocks
#   8. Lie down

import time
import math
from servo import ServoCluster, Calibration, servo2040

# ---- hardware ----
cal = Calibration()
cal.apply_two_pairs(610, 2441, -90, 90)
pins = list(range(servo2040.SERVO_1, servo2040.SERVO_12 + 1))
cluster = ServoCluster(0, 0, pins, cal)

# ---- geometry ----
L1 = 30.0;  L2 = 120.0;  HS = 46.0
LEG_MIN = 20.0;  LEG_MAX = 160.0

# ---- gait ----
STANCE_Y    = 132.0
LIFT        =  18.0
STRIDE      =  40.0
STANCE_DUTY =   0.6
CYCLE_MS    = 1200

# ---- torso twist ----
LEG_DIFF     = 15.0    # mm to raise/lower legs during twist
T_NEUTRAL    = 0.85    # torso frac during twist sequence
TWIST_LEAN   = 0.25    # torso lean magnitude
TWIST_ROCKS  = 3       # rock cycles per twist
TWIST_RAMP   = 700     # ms per lean direction

# ---- tripedal balance ----
BALANCE_SHORT = 117.0  # opposite corner depth (mm)
RAISED_Y      = 100.0  # raised leg depth (mm)
BAL_HOLD      = 600    # ms to hold each raised pose
BAL_RAMP      = 350    # ms per transition

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

LEG_MOTORS = {1: (0, 1), 2: (2, 3), 3: (4, 5), 4: (6, 7)}

# Twist lean targets (opposite frac directions per physical pair = same lean)
LEAN_RIGHT = {8: T_NEUTRAL-TWIST_LEAN, 9: T_NEUTRAL+TWIST_LEAN,
              10: T_NEUTRAL-TWIST_LEAN, 11: T_NEUTRAL+TWIST_LEAN}
LEAN_LEFT  = {8: T_NEUTRAL+TWIST_LEAN, 9: T_NEUTRAL-TWIST_LEAN,
              10: T_NEUTRAL+TWIST_LEAN, 11: T_NEUTRAL-TWIST_LEAN}
NEUTRAL_T  = {8: T_NEUTRAL, 9: T_NEUTRAL, 10: T_NEUTRAL, 11: T_NEUTRAL}


# ================================================================
# Core helpers
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

def ramp_combined(torso_from, torso_to, legs_from, legs_to, ms):
    t0 = time.ticks_ms()
    while True:
        el = time.ticks_diff(time.ticks_ms(), t0)
        if el >= ms: break
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


# ================================================================
# Gait
# ================================================================

def _ease(t):
    return t * t * (3.0 - 2.0 * t)

def foot_pos(phase, direction=1):
    sw = 1.0 - STANCE_DUTY
    if phase < sw:
        t  = phase / sw
        x  = direction * (-STRIDE + 2.0 * STRIDE * _ease(t))
        y  = STANCE_Y - LIFT * math.sin(math.pi * t)
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

def walk(cycles, direction=1):
    label = "forward" if direction == 1 else "backward"
    print("walk", label, cycles, "cycles")
    t0 = time.ticks_ms()
    total_ms = cycles * CYCLE_MS
    while True:
        el = time.ticks_diff(time.ticks_ms(), t0)
        if el >= total_ms: break
        p  = (el % CYCLE_MS) / CYCLE_MS
        pB = (p + 0.5) % 1.0
        apply_leg(0, 1, p,  fwd=-1, direction=direction)   # leg 1
        apply_leg(6, 7, p,  fwd=1,  direction=direction)   # leg 4
        apply_leg(2, 3, pB, fwd=1,  direction=direction)   # leg 2
        apply_leg(4, 5, pB, fwd=-1, direction=direction)   # leg 3
        cluster.load()
        time.sleep_ms(20)
    # Snap all legs back to neutral stance
    for i in range(8):
        leg_set(i, HOME_LEGS[i], False)
    cluster.load()
    time.sleep_ms(300)


# ================================================================
# Torso twist
# ================================================================

def do_twist():
    print("torso twist", TWIST_ROCKS, "rocks")
    torso_ramp(1.0, T_NEUTRAL, 400)     # ease torsos down from full stand
    tc = dict(NEUTRAL_T)
    lc = list(LEGS_NEUTRAL)
    for _ in range(TWIST_ROCKS):
        ramp_combined(tc, LEAN_RIGHT, lc, LEGS_TWIST_R, TWIST_RAMP)
        tc = dict(LEAN_RIGHT);  lc = list(LEGS_TWIST_R)
        ramp_combined(tc, LEAN_LEFT,  lc, LEGS_TWIST_L, TWIST_RAMP)
        tc = dict(LEAN_LEFT);   lc = list(LEGS_TWIST_L)
    ramp_combined(tc, NEUTRAL_T, lc, LEGS_NEUTRAL, TWIST_RAMP // 2)
    torso_ramp(T_NEUTRAL, 1.0, 400)     # return torsos to full stand


# ================================================================
# Tripedal balance
# ================================================================

def depths_to_angles(d1, d2, d3, d4):
    angles = [0.0] * 8
    for leg, depth in ((1,d1),(2,d2),(3,d3),(4,d4)):
        ik = legIK(0.0, depth)
        if ik is None: return None
        li, ri = LEG_MOTORS[leg]
        angles[li], angles[ri] = ik
    return angles

def balance_one(raise_leg, opposite_leg):
    print("  raise leg", raise_leg, "/ shorten leg", opposite_leg)
    S = STANCE_Y
    neutral = HOME_LEGS

    def make(rd, od):
        d = [S, S, S, S]
        d[raise_leg    - 1] = rd
        d[opposite_leg - 1] = od
        return depths_to_angles(*d)

    balanced = make(S,        BALANCE_SHORT)
    raised   = make(RAISED_Y, BALANCE_SHORT)
    if balanced is None or raised is None:
        print("    IK out of range — skip"); return

    ramp_legs(neutral,  balanced, BAL_RAMP);  time.sleep_ms(300)
    ramp_legs(balanced, raised,   BAL_RAMP);  time.sleep_ms(BAL_HOLD)
    ramp_legs(raised,   balanced, BAL_RAMP);  time.sleep_ms(200)
    ramp_legs(balanced, neutral,  BAL_RAMP // 2);  time.sleep_ms(400)

def do_balance():
    print("tripedal balance — legs 1 & 2")
    balance_one(1, 4)
    balance_one(2, 3)


# ================================================================
# Precompute static leg angle arrays (after functions are defined)
# ================================================================

HOME_LEGS    = list(legIK(0.0, STANCE_Y)) * 1   # placeholder until below
_h = legIK(0.0, STANCE_Y)
HOME_LEGS = [_h[0], _h[1], _h[0], _h[1], _h[0], _h[1], _h[0], _h[1]]

def _make_twist_legs(depth_a, depth_b):
    ika = legIK(0.0, depth_a);  ikb = legIK(0.0, depth_b)
    a = [0.0]*8
    a[0],a[1] = ika;  a[2],a[3] = ikb
    a[4],a[5] = ikb;  a[6],a[7] = ika
    return a

LEGS_NEUTRAL = _make_twist_legs(STANCE_Y,            STANCE_Y)
LEGS_TWIST_R = _make_twist_legs(STANCE_Y - LEG_DIFF, STANCE_Y + LEG_DIFF)
LEGS_TWIST_L = _make_twist_legs(STANCE_Y + LEG_DIFF, STANCE_Y - LEG_DIFF)


# ================================================================
# Main
# ================================================================

def main():
    # Seed before enabling
    for _, idx, _, flat in TORSO_CAL:
        torso_raw(idx, flat, False)
    for i in range(8):
        leg_set(i, HOME_LEGS[i], False)
    cluster.load()
    cluster.enable_all()
    time.sleep_ms(500)

    print("standing up...")
    torso_ramp(0.0, 1.0, 2500)
    time.sleep(1)

    walk(5)
    do_twist()
    walk(5)
    do_balance()
    walk(5, direction=-1)
    do_twist()

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
