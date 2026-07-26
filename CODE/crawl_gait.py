# Crawl gait — Pimoroni Servo 2040 (MicroPython)
#
# Diagonal-pair alternating gait (trot-style crawl):
#   Pair A — legs 1 & 4 (headers 1-2 and 7-8, idx 0-1 / 6-7): swing together
#   Pair B — legs 2 & 3 (headers 3-4 and 5-6, idx 2-3 / 4-5): 180° offset
# One pair swings (foot lifts and steps forward) while the other pushes in stance.
# Torsos ramp to stand before the gait, return to flat after.
#
# If a leg pair walks backwards instead of forwards, negate the x coordinate
# for that pair in the apply_leg() calls at the bottom of main().
#
# Channel map (header → index → role):
#   1→0 leg1 L   2→1 leg1 R   3→2 leg2 L   4→3 leg2 R
#   5→4 leg3 L   6→5 leg3 R   7→6 leg4 L   8→7 leg4 R
#   9→8 torso1  10→9 torso2  11→10 torso3  12→11 torso4

import time
import math
from servo import ServoCluster, Calibration, servo2040

# ---- hardware ----
cal = Calibration()
cal.apply_two_pairs(610, 2441, -90, 90)
pins = list(range(servo2040.SERVO_1, servo2040.SERVO_12 + 1))
cluster = ServoCluster(0, 0, pins, cal)

# ---- IK geometry ----
L1 = 30.0    # upper link length (mm)
L2 = 120.0   # lower link length (mm)
HS = 46.0    # centre-to-centre of the two pulley axes (mm)

# ---- leg servo limits ----
LEG_MIN = 20.0
LEG_MAX = 160.0

# ---- gait tuning ----
STANCE_Y    = 132.0   # foot depth in stance (mm, +y = down)
LIFT        =  18.0   # swing arc peak height above stance (mm)
STRIDE      =  40.0   # half-stride; foot sweeps -STRIDE..+STRIDE in x (mm)
STANCE_DUTY =   0.6   # fraction of cycle spent in stance (swing = 1 - this)
CYCLE_MS    = 1200    # duration of one full step cycle (ms)
GAIT_CYCLES =   8     # how many cycles to run before stopping

# ---- torso calibration (hand-measured, both ends) ----
# (header, cluster_index, stand_angle, flat_angle)
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


# ---- five-bar IK ----

def legIK(fx, fy):
    """
    Compute (servoL, servoR) in 0-180° for a foot at (fx, fy).
    Coordinate frame: +x toward left/forward motor side, +y downward.
    Origin is the midpoint between the two pulley axes.
    Returns None if the foot is outside the reachable workspace.
    Neutral (0, 132.77) → (90, 90). Stance depth 138 is well within range.
    """
    # Vector from each hip to the foot
    dxL =  fx - HS / 2    # left hip at (+HS/2, 0)
    dxR =  fx + HS / 2    # right hip at (-HS/2, 0)
    DL  = math.sqrt(dxL * dxL + fy * fy)
    DR  = math.sqrt(dxR * dxR + fy * fy)

    reach_min = abs(L2 - L1)   # 90 mm
    reach_max = L1 + L2        # 150 mm
    if not (reach_min < DL < reach_max): return None
    if not (reach_min < DR < reach_max): return None

    cosL = max(-1.0, min(1.0, (L1*L1 + DL*DL - L2*L2) / (2*L1*DL)))
    cosR = max(-1.0, min(1.0, (L1*L1 + DR*DR - L2*L2) / (2*L1*DR)))

    # +acos branch → knees splayed outward (matches the build)
    # Right side uses negated x so both arms measure angle symmetrically from vertical
    phi_L = math.atan2( dxL, fy) + math.acos(cosL)
    phi_R = math.atan2(-dxR, fy) + math.acos(cosR)

    sL = 135.0 - math.degrees(phi_L)
    sR = math.degrees(phi_R) + 45.0
    return sL, sR


# ---- helpers ----

def leg_set(idx, angle, load=True):
    cluster.value(idx, max(LEG_MIN, min(LEG_MAX, float(angle))) - 90.0, load)


def torso_raw(idx, angle, load=True):
    lo, hi = TORSO_LIMITS[idx]
    cluster.value(idx, max(lo, min(hi, float(angle))) - 90.0, load)


def torso_ramp(from_frac, to_frac, ms):
    """Smoothly move all torsos between two flat→stand fractions."""
    t0 = time.ticks_ms()
    while True:
        el = time.ticks_diff(time.ticks_ms(), t0)
        if el >= ms:
            break
        f = from_frac + (to_frac - from_frac) * (el / ms)
        for _, idx, stand, flat in TORSO_CAL:
            torso_raw(idx, flat + (stand - flat) * f, False)
        cluster.load()
        time.sleep_ms(20)
    for _, idx, stand, flat in TORSO_CAL:
        torso_raw(idx, flat + (stand - flat) * to_frac, False)
    cluster.load()


# ---- gait trajectory ----

def _ease(t):
    # smoothstep: zero velocity at both endpoints, cuts the servo-rate spike
    return t * t * (3.0 - 2.0 * t)

def foot_pos(phase):
    """
    Returns (x, y) foot position for normalised phase in [0, 1).
    Swing (first 40%): eased x so the foot accelerates out of stance and
    decelerates into touchdown — eliminates the velocity spike at transitions.
    Stance (last 60%): constant-depth push, linear x sweep.
    """
    sw = 1.0 - STANCE_DUTY                                  # swing duty = 0.4
    if phase < sw:
        t  = phase / sw                                     # 0..1 within swing
        ex = _ease(t)
        x  = -STRIDE + 2.0 * STRIDE * ex                   # eased rear → front
        y  = STANCE_Y - LIFT * math.sin(math.pi * t)       # sine lift arc
    else:
        t = (phase - sw) / STANCE_DUTY                     # 0..1 within stance
        x = STRIDE - 2.0 * STRIDE * t                      # linear front → rear
        y = STANCE_Y
    return x, y


def apply_leg(li, ri, phase, fwd=1):
    """Drive one leg's left/right motors from its current gait phase.
    fwd=-1 mirrors the x axis for legs whose motor plane faces the other way."""
    fx, fy = foot_pos(phase)
    ik = legIK(fx * fwd, fy)
    if ik is None:
        return
    leg_set(li, ik[0], False)
    leg_set(ri, ik[1], False)


# ---- main sequence ----

def main():
    # 1. Pre-position before enabling so nothing snaps on power-up
    print("seeding: torsos at flat, legs at home stance depth")
    for _, idx, _, flat in TORSO_CAL:
        torso_raw(idx, flat, False)
    home = legIK(0.0, STANCE_Y)
    if home is None:
        print("ERROR: home position unreachable — check L1/L2/HS constants")
        return
    for li, ri in ((0, 1), (2, 3), (4, 5), (6, 7)):
        leg_set(li, home[0], False)
        leg_set(ri, home[1], False)
    cluster.load()
    cluster.enable_all()
    time.sleep_ms(500)

    # 2. Stand up
    print("standing up...")
    torso_ramp(0.0, 1.0, 2500)
    time.sleep(1)

    # 3. Crawl gait — diagonal pairs alternating
    print("crawl gait starting:", GAIT_CYCLES, "cycles (~",
          GAIT_CYCLES * CYCLE_MS // 1000, "s) — Ctrl-C to stop early")

    t0 = time.ticks_ms()
    total_ms = GAIT_CYCLES * CYCLE_MS

    while True:
        el = time.ticks_diff(time.ticks_ms(), t0)
        if el >= total_ms:
            break

        p  = (el % CYCLE_MS) / CYCLE_MS    # pair A phase
        pB = (p + 0.5) % 1.0               # pair B, 180° offset

        # Pair A: legs 1 (headers 1-2) and 4 (headers 7-8)
        apply_leg(0, 1, p,  fwd=-1)
        apply_leg(6, 7, p)

        # Pair B: legs 2 (headers 3-4) and 3 (headers 5-6)
        apply_leg(2, 3, pB)
        apply_leg(4, 5, pB, fwd=-1)   # mirrored end of body

        cluster.load()
        time.sleep_ms(20)

    # 4. Return to neutral stand pose
    for li, ri in ((0, 1), (2, 3), (4, 5), (6, 7)):
        leg_set(li, home[0], False)
        leg_set(ri, home[1], False)
    cluster.load()
    time.sleep(1)

    # 5. Lie back down
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
