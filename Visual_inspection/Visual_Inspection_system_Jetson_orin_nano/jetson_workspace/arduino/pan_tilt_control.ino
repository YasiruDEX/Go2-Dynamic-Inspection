/*
 * Pan-Tilt Servo Control for Visual Inspection System
 * ====================================================
 * 
 * Hardware:
 *   - Servo on pin 9:  TILT (20-160 degrees)
 *   - Servo on pin 10: PAN (0-180 degrees)
 * 
 * Serial Commands (9600 baud):
 *   90,90        - Set tilt=90, pan=90 (format: tilt,pan)
 *   45,135       - Set tilt=45, pan=135
 *   HOME         - Move to home position (90, 90)
 *   PAN:90       - Set only pan to 90
 *   TILT:45      - Set only tilt to 45
 * 
 * Upload this to your Arduino Uno R3.
 */

#include <Servo.h>

Servo servo9;   // TILT servo (limited range)
Servo servo10;  // PAN servo (full range)

const int servoPin9  = 9;   // TILT
const int servoPin10 = 10;  // PAN

// Limits for servos
const int TILT_MIN = 20;
const int TILT_MAX = 160;
const int PAN_MIN = 0;
const int PAN_MAX = 180;

// Current positions
int currentTilt = 90;
int currentPan = 90;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);

  // Attach servos with extended pulse width range for better compatibility
  servo9.attach(servoPin9, 500, 2500);    // TILT
  servo10.attach(servoPin10, 500, 2500);  // PAN

  // Initial safe position (center)
  servo9.write(90);
  servo10.write(90);
  currentTilt = 90;
  currentPan = 90;
  
  delay(300);

  Serial.println("Pan-Tilt Controller Ready");
  Serial.println("Commands:");
  Serial.println("  tilt,pan  - Move both (e.g., 90,90)");
  Serial.println("  PAN:90    - Set pan only");
  Serial.println("  TILT:45   - Set tilt only");
  Serial.println("  HOME      - Center position");
  Serial.print("Tilt range: ");
  Serial.print(TILT_MIN);
  Serial.print("-");
  Serial.println(TILT_MAX);
  Serial.print("Pan range: ");
  Serial.print(PAN_MIN);
  Serial.print("-");
  Serial.println(PAN_MAX);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Handle HOME command
    if (input.equals("HOME")) {
      moveToHome();
      return;
    }

    // Handle PAN command
    if (input.startsWith("PAN:")) {
      int angle = input.substring(4).toInt();
      setPan(angle);
      return;
    }

    // Handle TILT command
    if (input.startsWith("TILT:")) {
      int angle = input.substring(5).toInt();
      setTilt(angle);
      return;
    }

    // Handle tilt,pan format
    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      Serial.println("ERROR: Format should be tilt,pan (e.g., 90,90)");
      return;
    }

    int tiltAngle = input.substring(0, commaIndex).toInt();
    int panAngle = input.substring(commaIndex + 1).toInt();

    moveTo(tiltAngle, panAngle);
  }
}

void setTilt(int angle) {
  // Enforce limits
  angle = constrain(angle, TILT_MIN, TILT_MAX);
  
  // INVERT angle for reversed servo mounting (20 becomes 160, etc.)
  int invertedAngle = 180 - angle;
  
  servo9.write(invertedAngle);
  currentTilt = angle;
  
  Serial.print("Tilt: ");
  Serial.println(angle);
}

void setPan(int angle) {
  // Enforce limits
  angle = constrain(angle, PAN_MIN, PAN_MAX);
  
  servo10.write(angle);
  currentPan = angle;
  
  Serial.print("Pan: ");
  Serial.println(angle);
}

void moveTo(int tiltAngle, int panAngle) {
  // Enforce limits
  tiltAngle = constrain(tiltAngle, TILT_MIN, TILT_MAX);
  panAngle = constrain(panAngle, PAN_MIN, PAN_MAX);
  
  servo9.write(tiltAngle);
  servo10.write(panAngle);
  
  currentTilt = tiltAngle;
  currentPan = panAngle;
  
  Serial.print("Tilt: ");
  Serial.print(tiltAngle);
  Serial.print(" | Pan: ");
  Serial.println(panAngle);
}

void moveToHome() {
  moveTo(90, 90);
  Serial.println("HOME position");
}
