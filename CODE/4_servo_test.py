# 4-servo hardware test for Pimoroni Servo 2040 (MicroPython)
# Servos plugged into headers 1, 2, 3, 4  ->  cluster indices 0, 1, 2, 3

import time
import math
from servo import ServoCluster, Calibration, servo2040

# ---- calibration: reproduce the pulse widths from the PCA setup ----
# 0 deg -> 610 us, 180 deg -> 2441 us, mapped onto Pimoroni's -90..+90 scale.
cal = Calibration()
cal.apply_two_pairs(610, 2441, -90, 90)

# headers 1-4 -> pins [0,1,2,3]
pins = list(range(servo2040.SERVO_1, servo2040.SERVO_4 + 1))
cluster = ServoCluster(0, 0, pins, cal)

NUM = 4
MIN_ANGLE = 20.0
MAX_ANGLE = 160.0


def set_angle(index, angle, load=True):
    # work in the familiar 0-180 scale, convert to -90..+90 for the library
    if angle < MIN_ANGLE:
        angle = MIN_ANGLE
    elif angle > MAX_ANGLE:
        angle = MAX_ANGLE
    cluster.value(index, angle - 90.0, load)


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
        t = el / duration_ms
        set_angle(index, a + (b - a) * t)
        time.sleep_ms(20)
    set_angle(index, b)


def main():
    cluster.enable_all()

    # 1) center everything and settle
    print("centering all four at 90")
    all_to(90)
    time.sleep(1)

    # 2) one servo at a time, so you can confirm header -> servo mapping
    for i in range(NUM):
        print("testing header", i + 1, "(index", str(i) + ")")
        sweep(i, 90, 60, 500)     # nudge one way
        time.sleep_ms(200)
        sweep(i, 60, 120, 800)    # sweep across
        time.sleep_ms(200)
        sweep(i, 120, 90, 500)    # back to center
        time.sleep_ms(400)        # pause so you can tell servos apart

    # 3) all together, a slow synchronized sweep
    print("all four together")
    for _ in range(2):
        for ang in range(90, 130):
            all_to(ang); time.sleep_ms(15)
        for ang in range(130, 50, -1):
            all_to(ang); time.sleep_ms(15)
        for ang in range(50, 91):
            all_to(ang); time.sleep_ms(15)

    all_to(90)
    print("done")


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    cluster.disable_all()
    print("servos disabled")