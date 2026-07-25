# Calibrate ONE torso servo to find true vertical (stand).
# Run this, then type angles at the prompt until the leg hangs straight down.
#
# Change WHICH below to the torso you're calibrating:
#   idx 8  = torso 9  (leg 1)
#   idx 9  = torso 10 (leg 2)
#   idx 10 = torso 11 (leg 3)
#   idx 11 = torso 12 (leg 4)

import time
from servo import ServoCluster, Calibration, servo2040

WHICH = 10          # <-- set to the index you're calibrating
START_ANGLE = 90.0 # where it begins; the flat start is 0 or 180 (see note)

# soft range so a typo can't slam it. Widen only if you must.
SAFE_MIN = 0.0
SAFE_MAX = 180.0

cal = Calibration()
cal.apply_two_pairs(610, 2441, -90, 90)
cluster = ServoCluster(0, 0, list(range(servo2040.SERVO_1, servo2040.SERVO_12 + 1)), cal)


def go(angle):
    if angle < SAFE_MIN:
        angle = SAFE_MIN
        print("  clamped to", SAFE_MIN)
    elif angle > SAFE_MAX:
        angle = SAFE_MAX
        print("  clamped to", SAFE_MAX)
    cluster.value(WHICH, angle - 90.0)   # 0-180 -> -90..+90
    return angle


def main():
    cluster.enable(WHICH)
    current = go(START_ANGLE)
    print("torso index", WHICH, "at", current, "deg")
    print("type an angle and Enter to move it. 'q' to finish.")
    print("nudge until the leg hangs straight down (vertical).")

    while True:
        s = input("angle> ").strip()
        if s in ("q", "quit", "exit", ""):
            break
        try:
            a = float(s)
        except ValueError:
            print("  number, or q")
            continue
        current = go(a)
        print("  now at", current, "deg")

    print("VERTICAL FOUND AT:", current, "deg  (torso index", str(WHICH) + ")")
    print("write that down, then re-run with WHICH set to the next torso.")


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    cluster.disable(WHICH)
    print("disabled")