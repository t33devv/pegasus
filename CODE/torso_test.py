# Torso servo test for Pimoroni Servo 2040 (MicroPython)
# Calibrated per-torso. Servos START AT FLAT and ramp gently to STAND.
#
#   torso 9  (idx 8)  -> leg 1   stand @ 60   flat @ 0
#   torso 10 (idx 9)  -> leg 2   stand @ 110  flat @ 180
#   torso 11 (idx 10) -> leg 3   stand @ 110  flat @ 180
#   torso 12 (idx 11) -> leg 4   stand @ 70   flat @ 0
#
# 'stand' angles were found by hand-calibrating each servo to true vertical.
# Everything works in a flat->stand fraction so mirrored servos move the
# correct opposite directions and no servo ever snaps.

import time
from servo import ServoCluster, Calibration, servo2040

cal = Calibration()
cal.apply_two_pairs(610, 2441, -90, 90)

pins = list(range(servo2040.SERVO_1, servo2040.SERVO_12 + 1))
cluster = ServoCluster(0, 0, pins, cal)

# (torso number, index, stand_angle, flat_angle)
# BOTH ends hand-calibrated. flat is the real physical rest position,
# NOT 0/180 - commanding past these drives the legs into each other.
TORSO_CAL = [
    (9,  8,  60.0, 160.0),
    (10, 9, 110.0,  10.0),
    (11, 10, 110.0,  5.0),
    (12, 11, 70.0, 170.0),
]


# hard limits per index: the safe span between stand and flat, so no
# command can ever push a torso past its calibrated stop into a clash
LIMITS = {}
for _, idx, stand, flat in [
    (9, 8, 60.0, 160.0), (10, 9, 110.0, 10.0),
    (11, 10, 110.0, 5.0), (12, 11, 70.0, 170.0)
]:
    LIMITS[idx] = (min(stand, flat), max(stand, flat))


def raw(index, angle, load=True):
    lo, hi = LIMITS[index]
    if angle < lo:
        angle = lo
    elif angle > hi:
        angle = hi
    cluster.value(index, angle - 90.0, load)


def angle_at(stand, flat, frac):
    # frac 0.0 = flat, 1.0 = stand
    return flat + (stand - flat) * frac


def ramp_all(from_frac, to_frac, duration_ms):
    start = time.ticks_ms()
    while True:
        el = time.ticks_diff(time.ticks_ms(), start)
        if el >= duration_ms:
            break
        t = el / duration_ms
        f = from_frac + (to_frac - from_frac) * t
        for _, idx, stand, flat in TORSO_CAL:
            raw(idx, angle_at(stand, flat, f), False)
        cluster.load()
        time.sleep_ms(20)
    for _, idx, stand, flat in TORSO_CAL:
        raw(idx, angle_at(stand, flat, to_frac), False)
    cluster.load()


def main():
    # seed each servo at its FLAT angle, matching where it physically starts,
    # then enable - so the first motion is a gentle ramp, not a snap
    for _, idx, _, flat in TORSO_CAL:
        raw(idx, flat, False)
    cluster.load()
    for _, idx, _, _ in TORSO_CAL:
        cluster.enable(idx)
    time.sleep(1)
    print("seeded at flat")

    print("standing up (flat -> stand)")
    ramp_all(0.0, 1.0, 2500)
    time.sleep(1)

    print("dip halfway toward flat and back")
    ramp_all(1.0, 0.5, 1500)
    time.sleep_ms(500)
    ramp_all(0.5, 1.0, 1500)
    time.sleep(1)

    print("lying back down")
    ramp_all(1.0, 0.0, 2500)

    print("done")


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    for _, idx, _, _ in TORSO_CAL:
        cluster.disable(idx)
    print("torsos disabled")