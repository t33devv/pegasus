# imu_read.py — MPU6050 IMU test for Pimoroni Servo 2040 (MicroPython)
#
# No external driver needed — raw I2C register reads only.
#
# Wiring: VCC->3V3, GND->GND, SDA->SDA, SCL->SCL
#         AD0->GND  (gives address 0x68, the default)
#         AD0->3V3  (gives address 0x69 — change MPU_ADDR below)
# USB power alone is enough; 6V servo supply does NOT need to be on.

import time
from machine import I2C, Pin

MPU_ADDR = 0x68   # change to 0x69 if AD0 is pulled high

# MPU6050 registers
_PWR_MGMT_1 = 0x6B
_ACCEL_XOUT = 0x3B   # start of 6-byte accel block (XH, XL, YH, YL, ZH, ZL)
_GYRO_XOUT  = 0x43   # start of 6-byte gyro  block

# Default full-scale ranges
_ACCEL_SCALE = 16384.0   # LSB / g    at ±2 g
_GYRO_SCALE  = 131.0     # LSB / °/s  at ±250 °/s


def find_bus():
    """Try common Servo 2040 I2C pin combos, return a working I2C or None."""
    combos = []
    try:
        from servo import servo2040
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
        except Exception:
            continue
        if len(found) > 20:
            continue   # stuck/shorted bus
        if MPU_ADDR in found:
            print("FOUND MPU6050: bus", bus, "sda", sda, "scl", scl,
                  "->", [hex(x) for x in found])
            return i2c
        if found:
            print("bus", bus, "sda", sda, "scl", scl,
                  "-> other devices", [hex(x) for x in found])
    return None


def _read_word(i2c, reg):
    """Read a signed 16-bit big-endian value from reg."""
    data = i2c.readfrom_mem(MPU_ADDR, reg, 2)
    val = (data[0] << 8) | data[1]
    return val - 65536 if val > 32767 else val


def _read_xyz(i2c, start_reg):
    data = i2c.readfrom_mem(MPU_ADDR, start_reg, 6)
    vals = []
    for i in range(0, 6, 2):
        v = (data[i] << 8) | data[i + 1]
        vals.append(v - 65536 if v > 32767 else v)
    return vals   # [x, y, z] raw


def main():
    i2c = find_bus()
    if i2c is None:
        print()
        print("MPU6050 not found on any bus/pin combo.")
        print("Check: VCC->3V3, GND->GND, SDA/SCL wired correctly, AD0->GND.")
        print("If every line said nothing: sensor may not be powered.")
        return

    # Wake the MPU6050 — it starts in sleep mode
    i2c.writeto_mem(MPU_ADDR, _PWR_MGMT_1, bytes([0x00]))
    time.sleep_ms(100)
    print("MPU6050 awake. Reading... Ctrl-C to stop.\n")
    print("{:>10} {:>10} {:>10}   {:>10} {:>10} {:>10}".format(
        "aX (g)", "aY (g)", "aZ (g)", "gX °/s", "gY °/s", "gZ °/s"))
    print("-" * 68)

    try:
        while True:
            ax, ay, az = _read_xyz(i2c, _ACCEL_XOUT)
            gx, gy, gz = _read_xyz(i2c, _GYRO_XOUT)

            ax_g  = ax / _ACCEL_SCALE
            ay_g  = ay / _ACCEL_SCALE
            az_g  = az / _ACCEL_SCALE
            gx_ds = gx / _GYRO_SCALE
            gy_ds = gy / _GYRO_SCALE
            gz_ds = gz / _GYRO_SCALE

            print("{:>10.3f} {:>10.3f} {:>10.3f}   {:>10.2f} {:>10.2f} {:>10.2f}".format(
                ax_g, ay_g, az_g, gx_ds, gy_ds, gz_ds))

            time.sleep_ms(200)

    except KeyboardInterrupt:
        pass
    print("stopped")


main()
