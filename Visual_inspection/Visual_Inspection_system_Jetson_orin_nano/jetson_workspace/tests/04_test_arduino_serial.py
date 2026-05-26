#!/usr/bin/env python3
"""
Arduino Serial Test — send commands and see responses.
Usage:
    python3 04_test_arduino_serial.py

Command format the Arduino expects:
    tilt,pan\n   e.g.  90,90\n  (tilt=90, pan=90 = center)
"""

import serial
import glob
import os
import time

# ── Find Arduino ─────────────────────────────────────────────────────────────
def find_arduino():
    ARDUINO_VIDS = {'2341', '1a86', '0403'}
    # Try udev symlink first
    if os.path.exists('/dev/arduino'):
        print("✅ Found via udev symlink: /dev/arduino")
        return '/dev/arduino'
    # Scan all serial ports
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    for port in ports:
        try:
            dev_name = os.path.basename(port)
            check = os.path.realpath(f'/sys/class/tty/{dev_name}')
            for _ in range(8):
                vid_file = os.path.join(check, 'idVendor')
                if os.path.exists(vid_file):
                    with open(vid_file) as f:
                        vid = f.read().strip()
                    if vid in ARDUINO_VIDS:
                        print(f"✅ Found Arduino at {port} (VID={vid})")
                        return port
                    break
                check = os.path.dirname(check)
        except:
            pass
    return None

# ── Main ──────────────────────────────────────────────────────────────────────
port = find_arduino()
if not port:
    print("❌ Arduino not found! Check USB connection.")
    exit(1)

try:
    arduino = serial.Serial(port, 9600, timeout=1)
    time.sleep(2)  # wait for Arduino to reset
    print(f"\n🔌 Connected to Arduino at {port} (9600 baud)")
    print("=" * 50)
    print("Commands:")
    print("  tilt,pan     → e.g.  90,90   (center)")
    print("  tilt,pan     → e.g.  70,120  (tilt=70, pan=120)")
    print("  q            → quit")
    print("  center       → send 90,90 (center both)")
    print("=" * 50)
    print()

    while True:
        try:
            cmd = input("Send > ").strip()

            if cmd.lower() == 'q':
                print("Bye!")
                break

            elif cmd.lower() == 'center':
                cmd = '90,90'

            elif cmd == '':
                continue

            # Send command
            full_cmd = cmd + '\n'
            arduino.write(full_cmd.encode())
            print(f"  ✉ Sent: {cmd}")

            # Read any response from Arduino
            time.sleep(0.1)
            while arduino.in_waiting:
                response = arduino.readline().decode().strip()
                if response:
                    print(f"  ← Arduino: {response}")

        except KeyboardInterrupt:
            print("\nBye!")
            break

except serial.SerialException as e:
    print(f"❌ Serial error: {e}")
    print("   Try: sudo chmod 666 /dev/ttyACM0")

finally:
    try:
        arduino.close()
    except:
        pass
