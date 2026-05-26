#include <Servo.h>

Servo servo9;   // Limited servo
Servo servo10;  // Full range servo

const int servoPin9  = 9;
const int servoPin10 = 10;

// Limits for servo on pin 9
const int SERVO9_MIN = 20;
const int SERVO9_MAX = 160;

void setup() {
  Serial.begin(9600);

  servo9.attach(servoPin9, 500, 2500);
  servo10.attach(servoPin10, 500, 2500);

  // Initial safe position
  servo9.write(90);
  servo10.write(90);
  delay(300);

  Serial.println("Enter angles as: angle9,angle10");
  Serial.println("Servo9 range: 20–160 | Servo10 range: 0–180");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      Serial.println(" Format error. Use: angle9,angle10");
      return;
    }

    int angle9  = input.substring(0, commaIndex).toInt();
    int angle10 = input.substring(commaIndex + 1).toInt();

    // Enforce limits
    angle9  = constrain(angle9, SERVO9_MIN, SERVO9_MAX);
    angle10 = constrain(angle10, 0, 180);

    servo9.write(angle9);
    servo10.write(angle10);

    Serial.print("Servo9: ");
    Serial.print(angle9);
    Serial.print(" | Servo10: ");
    Serial.println(angle10);
  }
}
