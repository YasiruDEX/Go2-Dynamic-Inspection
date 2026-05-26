#!/usr/bin/env python3
"""
Test Script 1: Check Camera Devices
====================================
Scans and lists all available video devices on the system.
Shows device names and indices.
"""

import os
import glob
import subprocess

def check_camera_devices():
    """Scan and list all video devices"""
    print("=" * 60)
    print("CAMERA DEVICE DETECTION")
    print("=" * 60)

    # ── Step 1: Check /dev/video* devices ──────────────────────
    print("\n[1] Checking /dev/video* devices...")
    dev_devices = sorted(glob.glob('/dev/video*'))
    if dev_devices:
        print(f"    Found: {dev_devices}")
    else:
        print("    ❌ No /dev/video* devices found!")
        print("       → Check USB connections and try: sudo dmesg | tail -20")

    # ── Step 2: Check /sys/class/video4linux/ ──────────────────
    print("\n[2] Checking /sys/class/video4linux/ ...")
    sys_devices = sorted(glob.glob('/sys/class/video4linux/video*'))
    if not sys_devices:
        print("    ❌ No entries in /sys/class/video4linux/")
        print("       → Camera driver may not be loaded")
    else:
        print(f"    Found {len(sys_devices)} entries\n")
        cameras_found = {}
        for device_path in sys_devices:
            try:
                device_idx = int(device_path.split('video')[-1])
                name_path = os.path.join(device_path, 'name')
                if os.path.exists(name_path):
                    with open(name_path, 'r') as f:
                        device_name = f.read().strip()
                else:
                    device_name = "Unknown"

                print(f"    📹 /dev/video{device_idx}  →  {device_name}")

                base_name = device_name.split(':')[0] if ':' in device_name else device_name
                if base_name not in cameras_found:
                    cameras_found[base_name] = []
                cameras_found[base_name].append(device_idx)

            except Exception as e:
                print(f"    ⚠️  Error reading {device_path}: {e}")

        print("\n" + "=" * 60)
        print("CAMERA SUMMARY")
        print("=" * 60)
        for camera_name, indices in cameras_found.items():
            print(f"\n  📷 {camera_name}")
            print(f"     Devices: {', '.join([f'/dev/video{i}' for i in indices])}")
            if len(indices) > 1:
                print(f"     Note: video{indices[0]} is often metadata, video{indices[1]} is the stream")

    # ── Step 3: Run v4l2-ctl if available ──────────────────────
    print("\n[3] Checking with v4l2-ctl...")
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'],
                                capture_output=True, text=True, timeout=5)
        if result.stdout.strip():
            print(result.stdout)
        else:
            print("    No output from v4l2-ctl")
    except FileNotFoundError:
        print("    v4l2-ctl not installed. Run: sudo apt install v4l-utils")
    except Exception as e:
        print(f"    Error: {e}")

    # ── Step 4: Check for expected cameras ─────────────────────
    print("=" * 60)
    print("EXPECTED CAMERAS")
    print("=" * 60)

    all_names = ""
    for path in sys_devices:
        name_path = os.path.join(path, 'name')
        if os.path.exists(name_path):
            with open(name_path, 'r') as f:
                all_names += f.read().strip().lower() + " "

    if "insta360" in all_names:
        print("  ✅ Insta360 detected")
    else:
        print("  ❌ Insta360 NOT detected")

    if "hd pro webcam" in all_names or "c920" in all_names:
        print("  ✅ Logitech C920 detected")
    else:
        print("  ❌ Logitech C920 NOT detected")

    print()

if __name__ == "__main__":
    check_camera_devices()
