#!/usr/bin/env python3
"""
Test Script 3: Test Arduino Connection
=======================================
Tests serial connection to Arduino and servo control.
"""

import serial
import time
import sys

def test_arduino():
    """Test Arduino connection and servo control"""
    print("=" * 60)
    print("ARDUINO CONNECTION TEST")
    print("=" * 60)
    
    # Try common ports
    ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
    arduino = None
    
    print("\n🔍 Searching for Arduino...")
    for port in ports:
        try:
            print(f"   Trying {port}...")
            arduino = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            print(f"✅ Arduino found at {port}")
            break
        except:
            continue
    
    if arduino is None:
        print("❌ Arduino not found!")
        print("\nTroubleshooting:")
        print("  1. Check USB connection")
        print("  2. Run: ls /dev/ttyACM*")
        print("  3. Fix permissions: sudo chmod 666 /dev/ttyACM0")
        return False
    
    print("\n" + "=" * 60)
    print("SERVO CONTROL TEST")
    print("=" * 60)
    
    test_positions = [
        (90, 90, "Center"),
        (45, 90, "Pan Left"),
        (135, 90, "Pan Right"),
        (90, 60, "Tilt Up"),
        (90, 120, "Tilt Down"),
        (90, 90, "Center"),
    ]
    
    print("\nTesting servo movements...")
    print("Watch the servos move through test positions\n")
    
    for pan, tilt, description in test_positions:
        print(f"→ {description}: pan={pan}°, tilt={tilt}°")
        
        # Send command
        cmd = f"{tilt},{pan}\n"
        arduino.write(cmd.encode())
        time.sleep(1)
        
        # Read response
        if arduino.in_waiting:
            response = arduino.readline().decode().strip()
            print(f"  Arduino: {response}")
    
    arduino.close()
    print("\n✅ Arduino test complete!")
    return True

if __name__ == "__main__":
    success = test_arduino()
    sys.exit(0 if success else 1)
