# 4-leg hardware test for Pimoroni Servo 2040 (MicroPython)
# Legs paired on headers 1-8:
#   leg 1 -> headers 1,2 (idx 0,1)   leg 2 -> headers 3,4 (idx 2,3)
#   leg 3 -> headers 5,6 (idx 4,5)   leg 4 -> headers 7,8 (idx 6,7)
# Within each pair: LEFT motor first (odd header), RIGHT motor second.
#
# This is a RAW SERVO test - not IK. It moves each motor to confirm
# wiring, pairing, and which side is which. No foot positions here.

import time
from servo import ServoCluster, Calibration, servo2040

cal = Calibration()
cal.apply_two_pairs(610, 2441, -90, 90)   # match the PCA pulse widths

NUM = 8
pins = list(range(servo2040.SERVO_1, servo2040.SERVO_8 + 1))
cluster = ServoCluster(0, 0, pins, cal)

MIN_ANGLE = 20.0
MAX_ANGLE = 160.0

# (leg number, left index, right index)
LEGS = [
    (1, 0, 1),
    (2, 2, 3),
    (3, 4, 5),
    (4, 6, 7),
]


def set_angle(index, angle, load=True):
    if angle < MIN_ANGLE:
        angle = MIN_ANGLE
    elif angle > MAX_ANGLE:
        angle = MAX_ANGLE
    cluster.value(index, angle - 90.0, load)   # 0-180 -> -90..+90


def all_to(angle):
    for i in range(NUM):
        set_angle(i, angle, False)
    cluster.load()


def sweep(index, a, b, duration_ms):
    start = time.ticks_ms()
    while True:
        el = time.ticks_diff(time.ticks_ms(), start)
        if el >= duration_ms:
            break
        set_angle(index, a + (b - a) * (el / duration_ms))
        time.sleep_ms(20)
    set_angle(index, b)


def wiggle(index, label):
    print("   ", label, "-> index", index, "(header", str(index + 1) + ")")
    sweep(index, 90, 65, 400)
    sweep(index, 65, 115, 700)
    sweep(index, 115, 90, 400)
    time.sleep_ms(300)


def main():
    cluster.enable_all()

    print("centering all 8 at 90")
    all_to(90)
    time.sleep(1)

    # leg by leg, left motor then right motor
    for leg, li, ri in LEGS:
        print("LEG", leg)
        wiggle(li, "left")
        wiggle(ri, "right")
        time.sleep_ms(500)     # clear gap between legs

    # each leg's pair moving together, one leg at a time
    print("pairs together, one leg at a time")
    for leg, li, ri in LEGS:
        print("LEG", leg, "both motors")
        for ang in (75, 105, 90):
            set_angle(li, ang, False)
            set_angle(ri, ang, False)
            cluster.load()
            time.sleep_ms(500)
        time.sleep_ms(400)

    print("recentering")
    all_to(90)
    print("done")


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    cluster.disable_all()
    print("servos disabled")