/*
  Slow Multi-Servo Positioner (Pins 9, 10, 11)
  -----------------------------------------------
  Type a letter followed by an angle into the Serial Monitor and
  press Enter:
    A80   -> moves the servo on pin 9  to 80 degrees
    B45   -> moves the servo on pin 10 to 45 degrees
    C120  -> moves the servo on pin 11 to 120 degrees

  Each servo moves gradually, one degree at a time, instead of
  snapping there instantly. Adjust stepDelay to change the speed.
*/

#include <Servo.h>

Servo servoA, servoB, servoC;

const int pinA = 9;
const int pinB = 10;
const int pinC = 11;

int angleA = 90; // assume all start centered
int angleB = 90;
int angleC = 90;

const int stepDelay = 20; // ms between each 1-degree step (bigger = slower)

void setup() {
  Serial.begin(9600);

  servoA.attach(pinA);
  servoB.attach(pinB);
  servoC.attach(pinC);

  servoA.write(angleA);
  servoB.write(angleB);
  servoC.write(angleC);

  Serial.println("Type a letter (A/B/C) followed by an angle, e.g. A80:");
}

void loop() {
  if (Serial.available() > 0) {
    char letter = Serial.read();

    // Ignore whitespace/newline characters as the "letter"
    if (letter == '\n' || letter == '\r' || letter == ' ') return;

    int targetAngle = Serial.parseInt();

    while (Serial.available() > 0) Serial.read(); // clear leftover input

    if (targetAngle < 0 || targetAngle > 180) {
      Serial.println("Please enter a valid angle between 0 and 180.");
      return;
    }

    letter = toupper(letter);

    switch (letter) {
      case 'A':
        Serial.print("Moving servo A (pin 9) to ");
        Serial.print(targetAngle);
        Serial.println(" degrees...");
        angleA = moveSlowlyTo(servoA, angleA, targetAngle);
        break;

      case 'B':
        Serial.print("Moving servo B (pin 10) to ");
        Serial.print(targetAngle);
        Serial.println(" degrees...");
        angleB = moveSlowlyTo(servoB, angleB, targetAngle);
        break;

      case 'C':
        Serial.print("Moving servo C (pin 11) to ");
        Serial.print(targetAngle);
        Serial.println(" degrees...");
        angleC = moveSlowlyTo(servoC, angleC, targetAngle);
        break;

      default:
        Serial.println("Unknown servo letter. Use A, B, or C.");
        return;
    }

    Serial.println("Done.");
    Serial.println("Type a letter (A/B/C) followed by an angle, e.g. A80:");
  }
}

int moveSlowlyTo(Servo &servo, int currentAngle, int targetAngle) {
  if (targetAngle > currentAngle) {
    for (int pos = currentAngle; pos <= targetAngle; pos++) {
      servo.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = currentAngle; pos >= targetAngle; pos--) {
      servo.write(pos);
      delay(stepDelay);
    }
  }
  return targetAngle;
}
