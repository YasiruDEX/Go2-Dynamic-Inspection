#!/usr/bin/env python3
"""
Step 6: Arduino Serial Communication Test
==========================================
Test sending pan/tilt commands to Arduino servos via serial.

Arduino sketch should accept commands like:
  PAN:90\n
  TILT:45\n
  MOVE:90,45\n

Usage:
    python3 06_arduino_test.py
"""

import serial
import time
import yaml
from pathlib import Path


def load_config():
    """Load configuration."""
    config_path = Path(__file__).parent.parent / "config" / "camera_calibration.yaml"
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


class ArduinoController:
    """Arduino servo controller via serial."""
    
    def __init__(self, port, baudrate=9600, timeout=1.0):
        """
        Initialize Arduino serial connection.
        
        Args:
            port: Serial port (e.g., /dev/ttyACM0)
            baudrate: Baud rate (default: 9600)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        
    def connect(self):
        """Connect to Arduino."""
        try:
            self.serial_conn = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=self.timeout
            )
            # Wait for Arduino to reset
            time.sleep(2.0)
            print(f"[+] Connected to Arduino on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"[ERROR] Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("[+] Disconnected from Arduino")
    
    def send_command(self, command):
        """
        Send command to Arduino.
        
        Args:
            command: Command string (will add newline)
        
        Returns:
            Response from Arduino (if any)
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("[ERROR] Not connected to Arduino")
            return None
        
        try:
            # Send command with newline
            cmd_bytes = (command + '\n').encode('utf-8')
            self.serial_conn.write(cmd_bytes)
            print(f"[TX] {command}")
            
            # Wait a bit for processing
            time.sleep(0.05)
            
            # Read response if available
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode('utf-8').strip()
                print(f"[RX] {response}")
                return response
            
            return None
            
        except Exception as e:
            print(f"[ERROR] Send failed: {e}")
            return None
    
    def set_pan(self, angle):
        """Set pan servo angle."""
        return self.send_command(f"PAN:{int(angle)}")
    
    def set_tilt(self, angle):
        """Set tilt servo angle."""
        return self.send_command(f"TILT:{int(angle)}")
    
    def move_to(self, pan, tilt):
        """Move both servos to specified angles."""
        return self.send_command(f"MOVE:{int(pan)},{int(tilt)}")
    
    def home(self):
        """Move servos to home position (90, 90)."""
        return self.move_to(90, 90)


def test_arduino_connection():
    """Test Arduino connection and servo movement."""
    config = load_config()
    
    arduino_cfg = config['arduino']
    servo_cfg = config['servo_limits']
    
    port = arduino_cfg['port']
    baudrate = arduino_cfg['baudrate']
    
    print("\n" + "="*60)
    print("Arduino Serial Communication Test")
    print("="*60)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Pan limits: {servo_cfg['pan']['min']}-{servo_cfg['pan']['max']}")
    print(f"Tilt limits: {servo_cfg['tilt']['min']}-{servo_cfg['tilt']['max']}")
    print("="*60 + "\n")
    
    # Create controller
    controller = ArduinoController(port, baudrate)
    
    # Connect
    if not controller.connect():
        print("\n[ERROR] Could not connect to Arduino")
        print("Check:")
        print(f"  1. Arduino is plugged in")
        print(f"  2. Correct port: {port}")
        print(f"  3. User has permission: sudo usermod -a -G dialout $USER")
        return
    
    try:
        # Test sequence
        print("\n[+] Starting test sequence...")
        
        # Home position
        print("\n1. Moving to HOME position (90, 90)")
        controller.home()
        time.sleep(2)
        
        # Pan test
        print("\n2. Testing PAN servo")
        for angle in [0, 45, 90, 135, 180]:
            print(f"   Pan to {angle} degrees")
            controller.set_pan(angle)
            time.sleep(1)
        
        # Return to center
        controller.set_pan(90)
        time.sleep(1)
        
        # Tilt test
        print("\n3. Testing TILT servo")
        for angle in [20, 60, 90, 120, 160]:
            print(f"   Tilt to {angle} degrees")
            controller.set_tilt(angle)
            time.sleep(1)
        
        # Return to center
        controller.set_tilt(90)
        time.sleep(1)
        
        # Combined movement test
        print("\n4. Testing combined MOVE command")
        positions = [
            (45, 60),   # Left-down
            (135, 60),  # Right-down
            (135, 120), # Right-up
            (45, 120),  # Left-up
            (90, 90)    # Center
        ]
        
        for pan, tilt in positions:
            print(f"   Move to Pan={pan}, Tilt={tilt}")
            controller.move_to(pan, tilt)
            time.sleep(1.5)
        
        print("\n[+] Test sequence complete!")
        
    except KeyboardInterrupt:
        print("\n[!] Test interrupted by user")
    
    finally:
        # Return to home
        print("\n[+] Returning to home position")
        controller.home()
        time.sleep(1)
        
        # Disconnect
        controller.disconnect()
    
    print("\n" + "="*60)
    print("Test complete!")
    print("="*60 + "\n")


if __name__ == "__main__":
    test_arduino_connection()
