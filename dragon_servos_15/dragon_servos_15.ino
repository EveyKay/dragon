#include <Servo.h>
#include <avr/pgmspace.h>

// ============================================================
// SERVO-TO-PIN CONFIGURATION
// This is the ONLY section you should need to edit when you
// add, remove, or rewire a servo. Everything below reads from
// this list automatically.
// ============================================================

struct ServoConfig {
  const char* name;   // friendly name, used in Serial output
  uint8_t pin;         // Arduino pin the servo signal wire is on
  int homeAngle;       // resting/neutral angle (0-180)
  int minAngle;         // safe minimum angle for this servo
  int maxAngle;         // safe maximum angle for this servo
};

// Add/remove/edit rows here. Order doesn't matter.
ServoConfig servoConfigs[] = {
  { "eyeLeft",     2, 90,  30, 150 },
  { "eyeRight",    3, 90,  30, 150 },
  { "eyelidLeft",  4, 90,  60, 120 },
  { "eyelidRight", 5, 90,  60, 120 },
  { "jaw",         6, 90,  15, 120 },
  { "neck1",       7, 90,  30, 150 },
  { "neck2",       8, 90,  30, 150 },
  { "neck3",       9, 90,  30, 150 },
};

const uint8_t NUM_SERVOS = sizeof(servoConfigs) / sizeof(servoConfigs[0]);

// ============================================================
// Everything below this line is generic plumbing — you
// shouldn't need to touch it just to change pins/servos.
// ============================================================

Servo servos[NUM_SERVOS];

// Look up a servo's array index by its friendly name.
// Returns -1 if not found.
int servoIndex(const char* name) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (strcmp(servoConfigs[i].name, name) == 0) {
      return i;
    }
  }
  return -1;
}

// Move a named servo to an angle, clamped to its configured safe range.
void moveServo(const char* name, int angle) {
  int idx = servoIndex(name);
  if (idx == -1) {
    Serial.print("Unknown servo: ");
    Serial.println(name);
    return;
  }
  angle = constrain(angle, servoConfigs[idx].minAngle, servoConfigs[idx].maxAngle);
  servos[idx].write(angle);
}

// Smoothly ease a single named servo from wherever it currently is
// to a target angle, one degree at a time — used for single Serial
// Monitor commands so they move like the animations do instead of
// snapping instantly.
void moveServoSmooth(const char* name, int targetAngle) {
  int idx = servoIndex(name);
  if (idx == -1) {
    Serial.print("Unknown servo: ");
    Serial.println(name);
    return;
  }

  int clampedTarget = constrain(targetAngle, servoConfigs[idx].minAngle, servoConfigs[idx].maxAngle);
  int startAngle = servos[idx].read();
  int steps = abs(clampedTarget - startAngle);
  if (steps == 0) steps = 1;

  const int stepDelayMs = 12; // pace per degree of travel

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    int angle = startAngle + t * (clampedTarget - startAngle);
    moveServo(name, angle);
    delay(stepDelayMs);
  }
}

void setup() {
  Serial.begin(9600);

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoConfigs[i].pin);
    servos[i].write(servoConfigs[i].homeAngle);

    Serial.print("Attached '");
    Serial.print(servoConfigs[i].name);
    Serial.print("' on pin ");
    Serial.print(servoConfigs[i].pin);
    Serial.print(" (home angle ");
    Serial.print(servoConfigs[i].homeAngle);
    Serial.println(")");
  }

  Serial.println("Dragon servo setup complete.");
}

void loop() {
  handleSerialCommands();
}

// ============================================================
// Synchronized blink
// eyelidRight: 90 -> 71 -> 90
// eyelidLeft:  90 -> 103 -> 90
// Both move together, step by step, so they reach their closed
// position at the same moment. "Slightly slower than normal"
// speed is set by stepDelayMs below.
// ============================================================
void blinkEyelids() {
  const int startAngle = 90;
  const int rightClosed = 71;
  const int leftClosed = 103;
  const int stepDelayMs = 20; // higher = slower; ~15ms is "normal" servo speed, so 20ms is slightly slower

  int rightSteps = abs(startAngle - rightClosed); // 19
  int leftSteps = abs(leftClosed - startAngle);   // 13
  int steps = max(rightSteps, leftSteps);          // use the larger so both arrive together

  // Closing: startAngle -> closed
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps; // 0.0 -> 1.0
    int rightAngle = startAngle + t * (rightClosed - startAngle);
    int leftAngle = startAngle + t * (leftClosed - startAngle);
    moveServo("eyelidRight", rightAngle);
    moveServo("eyelidLeft", leftAngle);
    delay(stepDelayMs);
  }

  // Opening: closed -> startAngle
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    int rightAngle = rightClosed + t * (startAngle - rightClosed);
    int leftAngle = leftClosed + t * (startAngle - leftClosed);
    moveServo("eyelidRight", rightAngle);
    moveServo("eyelidLeft", leftAngle);
    delay(stepDelayMs);
  }
}

// ============================================================
// Roar animation
// At the same time:
//   neck1: 90 -> 20
//   jaw:   90 -> 20
//   eyelids: full blink (90 -> closed -> 90), timed to happen
//            within the same overall duration as the neck/jaw move
// Type "ror" into the Serial Monitor to trigger it.
// ============================================================
void rorAnimation() {
  const int neckStart = 90, neckEnd = 20;
  const int jawStart = 90, jawEnd = 20;
  const int eyelidStart = 90;
  const int rightClosed = 71;
  const int leftClosed = 103;

  const int steps = 70;       // resolution of the animation (higher = smoother)
  const int stepDelayMs = 14; // a bit quicker than before

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps; // 0.0 -> 1.0 across the whole animation

    // Neck moves faster than the rest of the animation: it finishes
    // its travel by the 35% mark, then holds at its end angle.
    const float neckDuration = 0.35;
    int neckAngle;
    if (t <= neckDuration) {
      float neckPhase = t / neckDuration;
      neckAngle = neckStart + neckPhase * (neckEnd - neckStart);
    } else {
      neckAngle = neckEnd;
    }

    int jawAngle = jawStart + t * (jawEnd - jawStart);

    // Eyelids do a full blink within the same duration, but now with
    // an even longer hold in the middle so they stay closed longer:
    // 0.00-0.20 = closing, 0.20-0.80 = holding closed, 0.80-1.00 = opening
    const float closeEnd = 0.20;
    const float holdEnd = 0.80;
    int rightAngle, leftAngle;
    if (t <= closeEnd) {
      float phase = t / closeEnd;
      rightAngle = eyelidStart + phase * (rightClosed - eyelidStart);
      leftAngle = eyelidStart + phase * (leftClosed - eyelidStart);
    } else if (t <= holdEnd) {
      rightAngle = rightClosed;
      leftAngle = leftClosed;
    } else {
      float phase = (t - holdEnd) / (1.0 - holdEnd);
      rightAngle = rightClosed + phase * (eyelidStart - rightClosed);
      leftAngle = leftClosed + phase * (eyelidStart - leftClosed);
    }

    // neck2 sways side to side slowly while neck1 is dropping, and
    // continues a little bit after neck1 finishes before settling back
    // to 90. 1.5 full cycles across its active window so it always
    // ends back at center (90) rather than cutting off mid-swing.
    const float neck2Duration = 0.55;
    const float neck2Cycles = 1.5;
    const int neck2Amplitude = 20; // swings 70 <-> 110
    int neck2Angle;
    if (t <= neck2Duration) {
      float neck2Phase = t / neck2Duration;
      neck2Angle = 90 + neck2Amplitude * sin(2 * PI * neck2Cycles * neck2Phase);
    } else {
      neck2Angle = 90;
    }

    // neck3 sways side to side too, at the same time as neck2 (same
    // timing/cycles), but with a smaller amplitude since it should
    // only move "a little bit."
    const int neck3Amplitude = 10; // swings 80 <-> 100
    int neck3Angle;
    if (t <= neck2Duration) {
      float neck3Phase = t / neck2Duration;
      neck3Angle = 90 + neck3Amplitude * sin(2 * PI * neck2Cycles * neck3Phase);
    } else {
      neck3Angle = 90;
    }

    moveServo("neck1", neckAngle);
    moveServo("neck2", neck2Angle);
    moveServo("neck3", neck3Angle);
    moveServo("jaw", jawAngle);
    moveServo("eyelidRight", rightAngle);
    moveServo("eyelidLeft", leftAngle);

    delay(stepDelayMs);
  }

  // Jaw wobble: 20 -> 35 -> 20, three times (wider swing = more prominent)
  // Neck1 rises back toward 90 gradually during this phase too, so it
  // doesn't sit fully down the whole time the jaw is wobbling.
  const int wobbleLow = 20, wobbleHigh = 35;
  const int wobbleSteps = 30;       // finer resolution = smoother motion
  const int wobbleStepDelayMs = 5;  // a bit quicker than before
  const int totalWobbleSubSteps = 3 * 2 * (wobbleSteps + 1); // 3 reps, 2 directions each
  int wobbleSubStep = 0;

  for (int rep = 0; rep < 3; rep++) {
    // low -> high
    for (int i = 0; i <= wobbleSteps; i++) {
      float t = (float)i / wobbleSteps;
      int jawAngle = wobbleLow + t * (wobbleHigh - wobbleLow);
      moveServo("jaw", jawAngle);

      float neckT = (float)wobbleSubStep / (totalWobbleSubSteps - 1);
      moveServo("neck1", neckEnd + neckT * (90 - neckEnd));
      wobbleSubStep++;

      delay(wobbleStepDelayMs);
    }
    // high -> low
    for (int i = 0; i <= wobbleSteps; i++) {
      float t = (float)i / wobbleSteps;
      int jawAngle = wobbleHigh + t * (wobbleLow - wobbleHigh);
      moveServo("jaw", jawAngle);

      float neckT = (float)wobbleSubStep / (totalWobbleSubSteps - 1);
      moveServo("neck1", neckEnd + neckT * (90 - neckEnd));
      wobbleSubStep++;

      delay(wobbleStepDelayMs);
    }
  }

  // Neck1 is already back at 90 by the time wobbling finishes.
  // Return jaw (and confirm eyelids) to home, at a faster pace than before.
  const int homeAngle = 90;
  const int returnSteps = 70;      // finer resolution = smoother motion
  const int returnStepDelayMs = 4; // a bit quicker than before

  int jawFrom = wobbleLow; // 20 (wobble always ends back at low)

  for (int i = 0; i <= returnSteps; i++) {
    float t = (float)i / returnSteps;
    int jawAngle = jawFrom + t * (homeAngle - jawFrom);
    moveServo("jaw", jawAngle);
    moveServo("neck1", homeAngle);
    moveServo("eyelidRight", homeAngle);
    moveServo("eyelidLeft", homeAngle);
    delay(returnStepDelayMs);
  }
}

// ============================================================
// Generic helper: move any group of named servos together,
// each starting from wherever it currently is, all arriving
// at their own target at the same time.
// ============================================================
const uint8_t MAX_GROUP_SERVOS = 8;

void moveServosTogether(const char* names[], const int targets[], int count, int steps, int stepDelayMs) {
  int startAngles[MAX_GROUP_SERVOS];
  for (int c = 0; c < count; c++) {
    int idx = servoIndex(names[c]);
    startAngles[c] = (idx != -1) ? servos[idx].read() : targets[c];
  }

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    for (int c = 0; c < count; c++) {
      int angle = startAngles[c] + t * (targets[c] - startAngles[c]);
      moveServo(names[c], angle);
    }
    delay(stepDelayMs);
  }
}

// ============================================================
// Look right / look left / front
// Eyes + neck2 + neck3 turn together, then "front" resets them.
// Type "look right", "look left", or "front" into the Serial Monitor.
// ============================================================
void lookRightAnimation() {
  const char* names[] = { "eyeLeft", "eyeRight", "neck2", "neck3" };
  const int targets[] = { 140, 140, 130, 130 };
  moveServosTogether(names, targets, 4, 80, 10); // finer resolution, same overall speed
}

void lookLeftAnimation() {
  const char* names[] = { "eyeLeft", "eyeRight", "neck2", "neck3" };
  const int targets[] = { 40, 40, 50, 50 };
  moveServosTogether(names, targets, 4, 80, 10); // finer resolution, same overall speed
}

void frontAnimation() {
  const char* names[] = { "eyeLeft", "eyeRight", "neck2", "neck3" };
  const int targets[] = { 90, 90, 90, 90 };
  moveServosTogether(names, targets, 4, 80, 10); // finer resolution, same overall speed
}

// ============================================================
// Ror two
// neck1 and jaw both bob smoothly between 90 (up/closed) and 20
// (down/open), completing two full cycles together — so the head
// goes up-down-up-down and the mouth opens-closes-opens-closes.
// Right in the middle (t = 0.5), where neck1/jaw pass back through
// 90 between the two cycles, the eyelids do one full blink.
// Type "ror two" into the Serial Monitor to trigger it.
// ============================================================
void ror2Animation() {
  const float center = 55.0;    // midpoint between 90 (up) and 20 (down)
  const float amplitude = 35.0; // 55 +/- 35 = 90 and 20
  const int cycles = 2;         // two full up-down / open-close cycles

  const int steps = 140;      // fine resolution for smoothness
  const int stepDelayMs = 10; // ~1.4s total, similar pace to the original roar

  // Blink window: centered on t = 0.5, closing / holding / opening
  // within that window, eyelids stay open the rest of the time.
  const float blinkStart = 0.42;
  const float blinkEnd = 0.58;
  const int eyelidStart = 90;
  const int rightClosed = 71;
  const int leftClosed = 103;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;

    float wave = center + amplitude * cos(2 * PI * cycles * t);
    int neckAngle = (int)(wave + 0.5);
    int jawAngle = (int)(wave + 0.5);

    int rightAngle = eyelidStart;
    int leftAngle = eyelidStart;
    if (t >= blinkStart && t <= blinkEnd) {
      float local = (t - blinkStart) / (blinkEnd - blinkStart); // 0.0 -> 1.0 within the window
      if (local <= 1.0 / 3.0) {
        float phase = local / (1.0 / 3.0);
        rightAngle = eyelidStart + phase * (rightClosed - eyelidStart);
        leftAngle = eyelidStart + phase * (leftClosed - eyelidStart);
      } else if (local <= 2.0 / 3.0) {
        rightAngle = rightClosed;
        leftAngle = leftClosed;
      } else {
        float phase = (local - 2.0 / 3.0) / (1.0 / 3.0);
        rightAngle = rightClosed + phase * (eyelidStart - rightClosed);
        leftAngle = leftClosed + phase * (eyelidStart - leftClosed);
      }
    }

    moveServo("neck1", neckAngle);
    moveServo("jaw", jawAngle);
    moveServo("eyelidRight", rightAngle);
    moveServo("eyelidLeft", leftAngle);

    delay(stepDelayMs);
  }
}

// ============================================================
// Eyes closed / eyes open
// Moves both eyelids together to their fully closed or fully
// open position. Type "eyes closed" or "eyes open" into the
// Serial Monitor.
// ============================================================
void eyesClosedAnimation() {
  const char* names[] = { "eyelidRight", "eyelidLeft" };
  const int targets[] = { 71, 103 };
  moveServosTogether(names, targets, 2, 40, 10);
}

void eyesOpenAnimation() {
  const char* names[] = { "eyelidRight", "eyelidLeft" };
  const int targets[] = { 90, 90 };
  moveServosTogether(names, targets, 2, 40, 10);
}

// ============================================================
// Clip 5 bark
// Servo motion generated directly from the volume envelope of
// clip_05.mp3 (one of the dog-voiced takes) rather than hand-timed
// like the animations above. The recording was sampled in 40ms
// slices; jaw and neck1 track how loud it is at each slice, so the
// mouth snaps open and the head dips on every bark and settles back
// on the quiet stretches between them. Eyelids do one quick blink
// right at the recording's single loudest instant (~3.48s in).
// Type "clip5" into the Serial Monitor to trigger it.
//
// This only drives the servos -- it doesn't play the audio itself.
// If you wire up a sound module later, start clip_05.mp3 at the
// same time you call this function and the two will stay in sync,
// since both run off the same 40ms-per-frame timeline.
// ============================================================
const uint8_t CLIP5_FRAME_MS = 40;
const uint8_t CLIP5_NUM_FRAMES = 130;

// Jaw angle per frame: louder in the recording -> lower angle ->
// mouth more open.
static const uint8_t clip5Jaw[CLIP5_NUM_FRAMES] PROGMEM = {
  84, 85, 82, 58, 43, 37, 38, 34, 30, 25, 22, 24, 28, 35, 42, 49, 55, 60, 64, 69,
  73, 76, 78, 80, 80, 81, 82, 84, 85, 86, 87, 87, 85, 76, 54, 43, 46, 51, 55, 52,
  39, 33, 33, 36, 40, 44, 46, 48, 52, 57, 62, 66, 69, 72, 74, 76, 79, 80, 82, 83,
  82, 82, 83, 81, 81, 81, 74, 50, 38, 32, 26, 23, 22, 22, 22, 22, 22, 20, 21, 22,
  23, 27, 33, 40, 47, 54, 37, 26, 24, 22, 21, 21, 21, 21, 22, 23, 23, 22, 22, 24,
  28, 33, 35, 27, 23, 23, 26, 31, 37, 39, 40, 43, 45, 48, 52, 56, 53, 45, 48, 44,
  48, 52, 55, 55, 60, 61, 65, 68, 72, 76,
};

// Neck1 angle per frame: same shape as the jaw but smoothed with
// extra lag, so the head follows the mouth instead of moving in
// lockstep with it.
static const uint8_t clip5Neck1[CLIP5_NUM_FRAMES] PROGMEM = {
  88, 88, 86, 79, 70, 63, 59, 55, 51, 47, 44, 42, 42, 43, 44, 45, 47, 49, 51, 54,
  56, 58, 61, 63, 65, 67, 68, 70, 72, 73, 75, 76, 77, 77, 72, 66, 63, 62, 62, 62,
  58, 54, 51, 51, 51, 51, 52, 52, 53, 54, 56, 57, 59, 61, 62, 64, 66, 67, 69, 71,
  72, 73, 74, 75, 76, 77, 77, 71, 63, 57, 52, 47, 44, 42, 40, 40, 39, 38, 38, 38,
  38, 38, 39, 40, 42, 44, 45, 43, 42, 41, 40, 39, 38, 38, 38, 38, 38, 38, 38, 38,
  39, 39, 40, 40, 40, 40, 40, 40, 41, 42, 43, 44, 45, 47, 48, 50, 51, 51, 52, 52,
  53, 54, 55, 56, 57, 58, 59, 61, 62, 64,
};

void clip5Animation() {
  const int eyelidStart = 90;
  const int rightClosed = 71;
  const int leftClosed = 103;

  // Blink window: centered on the loudest frame in the recording
  // (frame 87 of 0-129, ~3.48s in) -- closes, holds briefly, opens.
  const int blinkStartFrame = 83;
  const int blinkCloseFrame = 88;
  const int blinkEndFrame = 93;

  for (uint8_t i = 0; i < CLIP5_NUM_FRAMES; i++) {
    int jawAngle = pgm_read_byte(&clip5Jaw[i]);
    int neckAngle = pgm_read_byte(&clip5Neck1[i]);

    // neck2/neck3 sway gently throughout, independent of loudness --
    // the same "background life" role they play during rorAnimation().
    float t = (float)i / (CLIP5_NUM_FRAMES - 1);
    int neck2Angle = 90 + 10 * sin(2 * PI * 1.5 * t);
    int neck3Angle = 90 + 6 * sin(2 * PI * 1.5 * t);

    int rightAngle = eyelidStart;
    int leftAngle = eyelidStart;
    if (i >= blinkStartFrame && i < blinkCloseFrame) {
      float phase = (float)(i - blinkStartFrame) / (blinkCloseFrame - blinkStartFrame);
      rightAngle = eyelidStart + phase * (rightClosed - eyelidStart);
      leftAngle = eyelidStart + phase * (leftClosed - eyelidStart);
    } else if (i >= blinkCloseFrame && i < blinkEndFrame) {
      float phase = (float)(i - blinkCloseFrame) / (blinkEndFrame - blinkCloseFrame);
      rightAngle = rightClosed + phase * (eyelidStart - rightClosed);
      leftAngle = leftClosed + phase * (eyelidStart - leftClosed);
    }

    moveServo("jaw", jawAngle);
    moveServo("neck1", neckAngle);
    moveServo("neck2", neck2Angle);
    moveServo("neck3", neck3Angle);
    moveServo("eyelidRight", rightAngle);
    moveServo("eyelidLeft", leftAngle);

    delay(CLIP5_FRAME_MS);
  }

  // Settle everything back to home once the envelope runs out.
  const char* names[] = { "jaw", "neck1", "neck2", "neck3", "eyelidRight", "eyelidLeft" };
  const int targets[] = { 90, 90, 90, 90, 90, 90 };
  moveServosTogether(names, targets, 6, 40, 10);
}

// ============================================================
// Test 1
// Runs every named animation currently in the system, one after
// another, with about a 3 second pause between each. Handy for
// checking that everything still works after changing pins or
// safe ranges. Type "test1" into the Serial Monitor.
// ============================================================
void test1Animation() {
  Serial.println("[test1] blink");
  blinkEyelids();
  delay(3000);

  Serial.println("[test1] ror");
  rorAnimation();
  delay(3000);

  Serial.println("[test1] ror two");
  ror2Animation();
  delay(3000);

  Serial.println("[test1] look right");
  lookRightAnimation();
  delay(3000);

  Serial.println("[test1] look left");
  lookLeftAnimation();
  delay(3000);

  Serial.println("[test1] front");
  frontAnimation();
  delay(3000);

  Serial.println("[test1] eyes closed");
  eyesClosedAnimation();
  delay(3000);

  Serial.println("[test1] eyes open");
  eyesOpenAnimation();
  delay(3000);

  Serial.println("[test1] clip5");
  clip5Animation();
  delay(3000);

  Serial.println("[test1] complete");
}

// ============================================================
// Serial command handling
// Type into the Serial Monitor: <servoName> <angle>
// Example:   jaw 60
// Press Enter (make sure Serial Monitor line ending is set to
// "Newline" or "Both NL & CR").
// ============================================================
void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.length() == 0) {
    return;
  }

  if (line.equalsIgnoreCase("blink")) {
    Serial.println("Blinking...");
    blinkEyelids();
    Serial.println("Blink done.");
    return;
  }

  if (line.equalsIgnoreCase("ror")) {
    Serial.println("Roaring...");
    rorAnimation();
    Serial.println("Roar done.");
    return;
  }

  if (line.equalsIgnoreCase("ror two")) {
    Serial.println("Roaring (take two)...");
    ror2Animation();
    Serial.println("Ror two done.");
    return;
  }

  if (line.equalsIgnoreCase("look right")) {
    Serial.println("Looking right...");
    lookRightAnimation();
    Serial.println("Look right done.");
    return;
  }

  if (line.equalsIgnoreCase("look left")) {
    Serial.println("Looking left...");
    lookLeftAnimation();
    Serial.println("Look left done.");
    return;
  }

  if (line.equalsIgnoreCase("front")) {
    Serial.println("Returning to front...");
    frontAnimation();
    Serial.println("Front done.");
    return;
  }

  if (line.equalsIgnoreCase("eyes closed")) {
    Serial.println("Closing eyes...");
    eyesClosedAnimation();
    Serial.println("Eyes closed.");
    return;
  }

  if (line.equalsIgnoreCase("eyes open")) {
    Serial.println("Opening eyes...");
    eyesOpenAnimation();
    Serial.println("Eyes open.");
    return;
  }

  if (line.equalsIgnoreCase("clip5")) {
    Serial.println("Playing clip5...");
    clip5Animation();
    Serial.println("Clip5 done.");
    return;
  }

  if (line.equalsIgnoreCase("test1")) {
    Serial.println("Running test1 (all animations)...");
    test1Animation();
    return;
  }

  int spaceIndex = line.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println("Format: <servoName> <angle>   e.g. jaw 60");
    return;
  }

  String name = line.substring(0, spaceIndex);
  String angleStr = line.substring(spaceIndex + 1);
  angleStr.trim();

  if (angleStr.length() == 0) {
    Serial.println("Format: <servoName> <angle>   e.g. jaw 60");
    return;
  }

  int angle = angleStr.toInt();

  int idx = servoIndex(name.c_str());
  if (idx == -1) {
    Serial.print("Unknown servo: ");
    Serial.println(name);
    Serial.print("Valid names: ");
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      Serial.print(servoConfigs[i].name);
      if (i < NUM_SERVOS - 1) Serial.print(", ");
    }
    Serial.println();
    return;
  }

  moveServoSmooth(name.c_str(), angle);

  Serial.print(name);
  Serial.print(" -> ");
  Serial.print(constrain(angle, servoConfigs[idx].minAngle, servoConfigs[idx].maxAngle));
  Serial.println(" degrees");
}
