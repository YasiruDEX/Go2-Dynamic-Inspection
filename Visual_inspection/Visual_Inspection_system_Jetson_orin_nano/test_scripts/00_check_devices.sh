#!/bin/bash

# Device Detection Script
# Run this to identify all connected devices

echo "========================================="
echo "STEP 0: Device Detection"
echo "========================================="
echo ""

echo "1. USB Devices Connected:"
echo "-------------------------"
lsusb
echo ""

echo "2. Video Devices (Cameras):"
echo "-------------------------"
v4l2-ctl --list-devices
echo ""

echo "3. Serial Devices (Arduino):"
echo "-------------------------"
echo "Available serial ports:"
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No serial devices found. Make sure Arduino is connected."
echo ""

echo "4. Detailed Video Device Info:"
echo "-------------------------"
for device in /dev/video*; do
    if [ -e "$device" ]; then
        echo "Device: $device"
        v4l2-ctl -d "$device" --all | grep -E "Card type|Driver name|Video input|Format|Width/Height"
        echo ""
    fi
done

echo "========================================="
echo "Summary:"
echo "========================================="
echo "TODO: Update config/camera_calibration.yaml with:"
echo "  - Insta360 device index"
echo "  - Logitech device index"
echo "  - Arduino port"
echo "========================================="
