# VL53L0X distance reader for Pimoroni Servo 2040 (MicroPython)
# Auto-detects the working I2C bus/pin order and validates it before reading.
#
# Wiring: VCC->3V3, GND->circle(-), SDA->SDA, SCL->SCL. XSHUT/GPIO1 unconnected.
# USB power alone is enough; the 6V servo supply does NOT need to be on.
#
# Needs a VL53L0X driver on the board. See notes at bottom - the import name
# must match the file you uploaded.

import time
from machine import I2C, Pin

TARGET = 0x29   # VL53L0X default address (== 41)


def looks_stuck(found):
    # a scan returning almost every address = stuck/shorted bus, not real devices
    return len(found) > 20


def find_bus():
    """Try common Servo 2040 I2C combinations, return a working I2C or None."""
    combos = []
    # prefer the board's named constants if the servo lib is present
    try:
        from servo import servo2040
        combos.append((0, servo2040.SDA, servo2040.SCL))
        combos.append((1, servo2040.SDA, servo2040.SCL))
    except Exception:
        pass
    # fallback fixed pin pairs, both orders
    for bus in (0, 1):
        for a, b in ((4, 5), (5, 4), (20, 21), (21, 20), (0, 1), (1, 0)):
            combos.append((bus, a, b))

    for bus, sda, scl in combos:
        try:
            i2c = I2C(bus, sda=Pin(sda), scl=Pin(scl))
            found = i2c.scan()
        except Exception:
            continue
        if looks_stuck(found):
            print("bus", bus, "sda", sda, "scl", scl,
                  "-> STUCK (", len(found), "addrs) - wiring short/swap")
            continue
        if TARGET in found:
            print("FOUND VL53L0X: bus", bus, "sda", sda, "scl", scl,
                  "->", [hex(x) for x in found])
            return i2c
        if found:
            print("bus", bus, "sda", sda, "scl", scl,
                  "-> other devices", [hex(x) for x in found])
    return None


def main():
    i2c = find_bus()
    if i2c is None:
        print()
        print("VL53L0X not found on any bus/pin combo.")
        print("If every line said STUCK: SDA/SCL are shorted together, shorted")
        print("to GND/3V3, or a wire is loose. Check power: 3V3->GND should be")
        print("~3.3V. Using the QW/ST connector (has pull-ups) is most reliable.")
        return

    # --- load the driver and read ---
    # If this import fails, rename to match your uploaded file / class.
    try:
        import vl53l0x
        tof = vl53l0x.VL53L0X(i2c)
    except ImportError:
        try:
            from VL53L0X import VL53L0X       # uppercase-file driver
            tof = VL53L0X(i2c)
        except Exception as e:
            print("driver import failed:", e)
            print("Upload the driver to the board and match the filename/class.")
            return

    # different drivers expose different method names - handle the common ones
    start = getattr(tof, "start", None)
    stop = getattr(tof, "stop", None)
    if start:
        start()

    def read_mm():
        for name in ("read", "read_range", "range", "ping"):
            fn = getattr(tof, name, None)
            if fn:
                return fn()
        # some drivers expose a property called 'range'
        return getattr(tof, "range")

    print("reading... Ctrl-C to stop")
    try:
        while True:
            mm = read_mm()
            print(mm, "mm  (", round(mm / 10.0, 1), "cm )")
            time.sleep_ms(200)
    except KeyboardInterrupt:
        pass
    finally:
        if stop:
            stop()
        print("stopped")


main()