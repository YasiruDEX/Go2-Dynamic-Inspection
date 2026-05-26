#include <Servo.h>

Servo servo9;    // Limited servo
Servo servo10;   // Full range servo

const int servoPin9  = 9;
const int servoPin10 = 10;

// Servo 9 limits
const int SERVO9_MIN = 20;
const int SERVO9_MAX = 160;

// Current positions
int current9  = 90;
int current10 = 90;

void moveSmooth(Servo &srv, int &current, int target, int stepDelay = 10) {
  if (current < target) {
    for (int pos = current; pos <= target; pos++) {
      srv.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = current; pos >= target; pos--) {
      srv.write(pos);
      delay(stepDelay);
    }
  }
  current = target;
}

void setup() {
  Serial.begin(9600);

  servo9.attach(servoPin9, 500, 2500);
  servo10.attach(servoPin10, 500, 2500);

  // Initial position
  servo9.write(current9);
  servo10.write(current10);
  delay(300);

  Serial.println("Enter angles as: angle9,angle10");
  Serial.println("Servo9: 20–160 | Servo10: 0–180");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      Serial.println(" Use format: angle9,angle10");
      return;
    }

    int target9  = input.substring(0, commaIndex).toInt();
    int target10 = input.substring(commaIndex + 1).toInt();

    // Apply limits
    target9  = constrain(target9, SERVO9_MIN, SERVO9_MAX);
    target10 = constrain(target10, 0, 180);

    // 🔹 MOVE SERVO 10 FIRST (FULL RANGE)
    moveSmooth(servo10, current10, target10, 8);

    // 🔹 THEN MOVE SERVO 9 (LIMITED RANGE)
    moveSmooth(servo9, current9, target9, 8);

    Serial.print("Done → Servo9: ");
    Serial.print(current9);
    Serial.print(" | Servo10: ");
    Serial.println(current10);
  }
}
