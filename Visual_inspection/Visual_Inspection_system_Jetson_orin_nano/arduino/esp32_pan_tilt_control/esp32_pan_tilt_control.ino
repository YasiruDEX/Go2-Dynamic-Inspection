/*
 * Pan-Tilt Servo Control for ESP32
 * =================================
 * 
 * Hardware:
 *   - Servo on GPIO 12 (D12): TILT (20-160 degrees) - replaces Arduino pin 9
 *   - Servo on GPIO 13 (D13): PAN (0-180 degrees)   - replaces Arduino pin 10
 * 
 * Serial Commands (115200 baud):
 *   90,90        - Set tilt=90, pan=90 (format: tilt,pan)
 *   45,135       - Set tilt=45, pan=135
 *   HOME         - Move to home position (90, 90)
 *   PAN:90       - Set only pan to 90
 *   TILT:45      - Set only tilt to 45
 * 
 * Upload this to your ESP32 board using Arduino IDE.
 * 
 * Library Required: ESP32Servo
 * Install via: Tools -> Manage Libraries -> Search "ESP32Servo" -> Install
 */

#include <ESP32Servo.h>

Servo servoTilt;  // TILT servo (limited range)
Servo servoPan;   // PAN servo (full range)

const int TILT_PIN = 12;  // GPIO 12 (D12)
const int PAN_PIN = 13;   // GPIO 13 (D13)

// Limits for servos
const int TILT_MIN = 20;
const int TILT_MAX = 160;
const int PAN_MIN = 0;
const int PAN_MAX = 180;

// Current positions
int currentTilt = 90;
int currentPan = 90;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  // Allow allocation of all timers for ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach servos with standard pulse width range
  servoTilt.setPeriodHertz(50);    // Standard 50Hz servo
  servoPan.setPeriodHertz(50);
  
  servoTilt.attach(TILT_PIN, 500, 2500);  // TILT on GPIO 12
  servoPan.attach(PAN_PIN, 500, 2500);    // PAN on GPIO 13

  // Initial safe position (center)
  servoTilt.write(90);
  servoPan.write(90);
  currentTilt = 90;
  currentPan = 90;
  
  delay(300);

  Serial.println("ESP32 Pan-Tilt Controller Ready");
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
  Serial.print("Tilt Pin: GPIO ");
  Serial.println(TILT_PIN);
  Serial.print("Pan Pin: GPIO ");
  Serial.println(PAN_PIN);
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
  
  servoTilt.write(angle);
  currentTilt = angle;
  
  Serial.print("Tilt: ");
  Serial.println(angle);
}

void setPan(int angle) {
  // Enforce limits
  angle = constrain(angle, PAN_MIN, PAN_MAX);
  
  servoPan.write(angle);
  currentPan = angle;
  
  Serial.print("Pan: ");
  Serial.println(angle);
}

void moveTo(int tiltAngle, int panAngle) {
  // Enforce limits
  tiltAngle = constrain(tiltAngle, TILT_MIN, TILT_MAX);
  panAngle = constrain(panAngle, PAN_MIN, PAN_MAX);
  
  servoTilt.write(tiltAngle);
  servoPan.write(panAngle);
  
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
