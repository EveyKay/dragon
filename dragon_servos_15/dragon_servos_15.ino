#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// ============================================================
// SERVO DRIVER (PCA9685, via I2C)
// Board's SCL -> Arduino A5, SDA -> Arduino A4, VCC -> Arduino 5V,
// GND -> Arduino GND. Servo power (V+ terminal, separate from the
// logic VCC pin) comes from the LM2596/battery rail, not the Arduino.
//
// Servos used to be driven directly by the Servo library, but that
// relies on constant timer interrupts to hold position -- which
// intermittently collided with SoftwareSerial's interrupts for the
// DFPlayer, causing random servo glitches (and occasional dropouts
// on whichever servo had the least torque margin) right around
// sound-triggering commands. Routing servos through the PCA9685
// instead removes the Arduino from pulse generation entirely, so
// there's no timer interrupt left for SoftwareSerial to collide with.
// ============================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulse-length calibration, in ticks out of 4096 at 50Hz. These are
// typical values for standard analog hobby servos -- if a servo
// doesn't reach its full physical range (or over-travels), tune
// these two numbers.
// Matches the Arduino Servo library's own default pulse-width range
// (544-2400us) at 50Hz, so 0-180 degrees lands on the exact same
// physical positions all 8 servos were originally calibrated/wired
// against -- switching to the PCA9685 shouldn't have shifted anyone's
// idea of "90 degrees."
const int SERVO_MIN_TICKS = 111; // 544us
const int SERVO_MAX_TICKS = 492; // 2400us

// ============================================================
// SOUND MODULE (DFPlayer Mini)
// DFPlayer TX -> Arduino pin 10 (Arduino RX)
// DFPlayer RX -> Arduino pin 11 (Arduino TX)
// SD card layout: an "mp3" folder in the card's root containing
// 0001.mp3, 0002.mp3, etc. -- playMp3Folder(N) plays 000N.mp3.
// ============================================================
SoftwareSerial dfSerial(10, 11); // RX, TX
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerReady = false;

// ============================================================
// SERVO-TO-PIN CONFIGURATION
// This is the ONLY section you should need to edit when you
// add, remove, or rewire a servo. Everything below reads from
// this list automatically.
// ============================================================

struct ServoConfig {
  const char* name;   // friendly name, used in Serial output
  uint8_t channel;     // PCA9685 output channel (0-15) the servo plugs into
  int homeAngle;       // resting/neutral angle (0-180)
  int minAngle;         // safe minimum angle for this servo
  int maxAngle;         // safe maximum angle for this servo
  int trim;             // degrees added before converting to PCA9685 ticks --
                         // corrects small per-servo mechanical differences so
                         // "90" always means visually centered, without any
                         // animation code needing to know about the offset
};

// Add/remove/edit rows here. Order doesn't matter.
ServoConfig servoConfigs[] = {
  { "eyeLeft",     2, 90,  30, 150,  0 },
  { "eyeRight",    3, 90,  30, 150,  0 },
  { "eyelidLeft",  4, 96,  60, 120,  0 },
  { "eyelidRight", 5, 58,  35, 120,  0 },
  { "jaw",         6, 90,  15, 120,  0 },
  { "neck1",       7, 90,  30, 150,  0 },
  { "neck2",       8, 90,  30, 150, 15 },
  { "neck3",       9, 90,  30, 150, 10 },
};

const uint8_t NUM_SERVOS = sizeof(servoConfigs) / sizeof(servoConfigs[0]);

// ============================================================
// Everything below this line is generic plumbing — you
// shouldn't need to touch it just to change pins/servos.
// ============================================================

// Tracks the last commanded angle for each servo. The PCA9685 has no
// way to report back what it last set a channel to, so (unlike the
// old Servo-library version) this is the only source of truth for
// "where is this servo right now" -- moveServoSmooth() and
// moveServosTogether() both read from this instead of a live sensor.
int lastAngle[NUM_SERVOS];

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

// ============================================================
// Idle mode's ambient background sway
// Since Arduino can't run two things at once, "keep swaying while
// an animation plays" means the animation's own step loop has to
// call this itself on every iteration. idleSwayActive gates all of
// it to a no-op outside idle mode, so blinkEyelids()/
// moveServosTogether() behave exactly as before when called from
// a plain Serial command.
// ============================================================
bool idleSwayActive = false;
unsigned long idleSwayStartMillis = 0;
float idleSwayScale = 1.0; // 1.0 = full idle sway, smaller = lessened during a bigger animation

// While frozen, the sine wave's phase must hold at exactly the value
// it had the instant the freeze began -- not reset to 0 -- so that
// once amplitude fades back up, the position it's fading up around
// still matches wherever it actually was, and doesn't jump. See the
// freeze start/end handling in idleAnimation() for how these are set.
bool idleSwayPhaseFrozen = false;
unsigned long idleSwayFrozenElapsed = 0;

bool servoInGroup(const char* name, const char* names[], int count) {
  for (int i = 0; i < count; i++) {
    if (strcmp(name, names[i]) == 0) return true;
  }
  return false;
}

// Drives whichever of neck1/neck2/neck3 are passed as true -- callers
// pass false for any axis a bigger animation is actively controlling
// itself, so the two motions never fight over the same servo.
void applyIdleSway(bool doNeck1, bool doNeck2, bool doNeck3) {
  if (!idleSwayActive) return;
  unsigned long elapsed = idleSwayPhaseFrozen ? idleSwayFrozenElapsed : (millis() - idleSwayStartMillis);
  if (doNeck1) moveServo("neck1", 90 + idleSwayScale * 14 * sin(2 * PI * elapsed / 4000.0));
  if (doNeck2) moveServo("neck2", 90 + idleSwayScale * 10 * sin(2 * PI * elapsed / 5500.0));
  if (doNeck3) moveServo("neck3", 90 + idleSwayScale * 7 * sin(2 * PI * elapsed / 7000.0));
}

// Move a named servo to an angle, clamped to its configured safe range.
void moveServo(const char* name, int angle) {
  int idx = servoIndex(name);
  if (idx == -1) {
    Serial.print(F("Unknown servo: "));
    Serial.println(name);
    return;
  }
  angle = constrain(angle, servoConfigs[idx].minAngle, servoConfigs[idx].maxAngle);
  lastAngle[idx] = angle; // logical angle, untrimmed -- everything else in the
                          // sketch (sway math, blinks, min/max clamps) keeps
                          // working in this same "90 = center" space

  int physicalAngle = constrain(angle + servoConfigs[idx].trim, 0, 180);
  int ticks = map(physicalAngle, 0, 180, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
  pwm.setPWM(servoConfigs[idx].channel, 0, ticks);
}

// Eases a linear progress value (0.0-1.0) into an exponential
// ease-in-out curve: starts slow, accelerates hard through the
// middle, then decelerates into the target -- closer to how a real
// muscle moves than a constant speed the whole way. Used everywhere
// a linear t was previously fed directly into an interpolation.
//
// steepness controls how sharp the burst through the middle is --
// higher means flatter start/end but a much higher PEAK velocity for
// the same total duration (roughly proportional to steepness itself,
// not just to total time), which is why stretching a movement's
// duration can't fully compensate for a steeper curve. The default
// (28) is used everywhere except flinchAnimation()'s snap-in, which
// needed a much gentler curve to stay within what the servos could
// track on a large, fast move.
float easeInOutExpo(float t, float steepness = 28.0) {
  if (t <= 0.0) return 0.0;
  if (t >= 1.0) return 1.0;
  if (t < 0.5) {
    return 0.5 * pow(2.0, steepness * t - steepness / 2.0);
  } else {
    return 1.0 - 0.5 * pow(2.0, -steepness * t + steepness / 2.0);
  }
}

// Smoothly ease a single named servo from wherever it currently is
// to a target angle, one degree at a time — used for single Serial
// Monitor commands so they move like the animations do instead of
// snapping instantly.
void moveServoSmooth(const char* name, int targetAngle) {
  int idx = servoIndex(name);
  if (idx == -1) {
    Serial.print(F("Unknown servo: "));
    Serial.println(name);
    return;
  }

  int clampedTarget = constrain(targetAngle, servoConfigs[idx].minAngle, servoConfigs[idx].maxAngle);
  int startAngle = lastAngle[idx];
  int steps = abs(clampedTarget - startAngle);
  if (steps == 0) steps = 1;

  const int stepDelayMs = 12; // pace per degree of travel

  for (int i = 0; i <= steps; i++) {
    float t = easeInOutExpo((float)i / steps);
    int angle = startAngle + t * (clampedTarget - startAngle);
    moveServo(name, angle);
    delay(stepDelayMs);
  }
}

void setup() {
  Serial.begin(9600);

  randomSeed(analogRead(A0)); // A0 is unconnected -- floating-pin noise seeds curiousTiltAnimation()'s side pick

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // standard hobby servo frequency

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    moveServo(servoConfigs[i].name, servoConfigs[i].homeAngle);

    Serial.print(F("Attached '"));
    Serial.print(servoConfigs[i].name);
    Serial.print(F("' on PCA9685 channel "));
    Serial.print(servoConfigs[i].channel);
    Serial.print(F(" (home angle "));
    Serial.print(servoConfigs[i].homeAngle);
    Serial.println(F(")"));
  }

  Serial.println(F("Dragon servo setup complete."));

  dfSerial.begin(9600);
  if (dfPlayer.begin(dfSerial)) {
    dfPlayerReady = true;
    dfPlayer.volume(30); // 0 (silent) - 30 (loudest)
    Serial.println(F("DFPlayer ready."));
  } else {
    Serial.println(F("DFPlayer not found -- check wiring and SD card."));
  }
}

// Prints whatever the DFPlayer itself reports (errors, card events,
// end-of-track notifications). Without this, a failed play command
// looks identical to a successful one from the Arduino's side --
// the module just silently doesn't play anything.
void checkDFPlayer() {
  if (!dfPlayerReady || !dfPlayer.available()) {
    return;
  }

  uint8_t type = dfPlayer.readType();
  int value = dfPlayer.read();

  switch (type) {
    case TimeOut:
      Serial.println(F("DFPlayer: timed out talking to module."));
      break;
    case WrongStack:
      Serial.println(F("DFPlayer: got a malformed response."));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("DFPlayer: SD card inserted."));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("DFPlayer: SD card removed."));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("DFPlayer: SD card online."));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("DFPlayer: finished playing track "));
      Serial.println(value);
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayer ERROR: "));
      switch (value) {
        case Busy:
          Serial.println(F("no SD card found."));
          break;
        case Sleeping:
          Serial.println(F("module is sleeping."));
          break;
        case SerialWrongStack:
          Serial.println(F("got a malformed command."));
          break;
        case CheckSumNotMatch:
          Serial.println(F("checksum mismatch."));
          break;
        case FileIndexOut:
          Serial.println(F("track number out of range."));
          break;
        case FileMismatch:
          Serial.println(F("can't find that file -- check it's named correctly in mp3/."));
          break;
        case Advertise:
          Serial.println(F("advertise error."));
          break;
        default:
          Serial.print(F("unknown error code "));
          Serial.println(value);
          break;
      }
      break;
    default:
      break;
  }
}

void loop() {
  checkDFPlayer();
  handleSerialCommands();
}

// ============================================================
// Synchronized blink
// eyelidRight: 58 -> 40 -> 58
// eyelidLeft:  96 -> 114 -> 96
// Both move together, step by step, so they reach their closed
// position at the same moment. "Slightly slower than normal"
// speed is set by stepDelayMs below.
// ============================================================
void blinkEyelids() {
  const int rightOpen = 58, rightClosed = 40;
  const int leftOpen = 96, leftClosed = 114;
  const int stepDelayMs = 20; // higher = slower; ~15ms is "normal" servo speed, so 20ms is slightly slower

  int rightSteps = abs(rightOpen - rightClosed);
  int leftSteps = abs(leftClosed - leftOpen);
  int steps = max(rightSteps, leftSteps);          // use the larger so both arrive together

  // Closing: open -> closed
  for (int i = 0; i <= steps; i++) {
    float t = easeInOutExpo((float)i / steps); // 0.0 -> 1.0
    int rightAngle = rightOpen + t * (rightClosed - rightOpen);
    int leftAngle = leftOpen + t * (leftClosed - leftOpen);
    moveServo("eyelidRight", rightAngle);
    moveServo("eyelidLeft", leftAngle);
    applyIdleSway(true, true, true); // blink never touches the neck, so all three stay free
    delay(stepDelayMs);
  }

  // Opening: closed -> open
  for (int i = 0; i <= steps; i++) {
    float t = easeInOutExpo((float)i / steps);
    int rightAngle = rightClosed + t * (rightOpen - rightClosed);
    int leftAngle = leftClosed + t * (leftOpen - leftClosed);
    moveServo("eyelidRight", rightAngle);
    moveServo("eyelidLeft", leftAngle);
    applyIdleSway(true, true, true);
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
//
// Paired with mp3/0011.mp3 (sounds/clip_11.mp3) -- two closely-spaced
// bursts that read as one sustained roar with a growl in the middle,
// picked to match this animation's ~2.2s single-roar-with-wobble shape.
// ============================================================
void rorAnimation() {
  if (dfPlayerReady) {
    dfPlayer.playMp3Folder(11);
  }

  const int neckStart = 90, neckEnd = 20;
  const int jawStart = 90, jawEnd = 20;
  const int rightOpen = 58, rightClosed = 40;
  const int leftOpen = 96, leftClosed = 114;

  const int steps = 70;       // resolution of the animation (higher = smoother)
  const int stepDelayMs = 14; // a bit quicker than before

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps; // 0.0 -> 1.0 across the whole animation

    // Neck moves faster than the rest of the animation: it finishes
    // its travel by the 35% mark, then holds at its end angle.
    const float neckDuration = 0.35;
    int neckAngle;
    if (t <= neckDuration) {
      float neckPhase = easeInOutExpo(t / neckDuration);
      neckAngle = neckStart + neckPhase * (neckEnd - neckStart);
    } else {
      neckAngle = neckEnd;
    }

    int jawAngle = jawStart + easeInOutExpo(t) * (jawEnd - jawStart);

    // Eyelids do a full blink within the same duration, but now with
    // an even longer hold in the middle so they stay closed longer:
    // 0.00-0.20 = closing, 0.20-0.80 = holding closed, 0.80-1.00 = opening
    const float closeEnd = 0.20;
    const float holdEnd = 0.80;
    int rightAngle, leftAngle;
    if (t <= closeEnd) {
      float phase = easeInOutExpo(t / closeEnd);
      rightAngle = rightOpen + phase * (rightClosed - rightOpen);
      leftAngle = leftOpen + phase * (leftClosed - leftOpen);
    } else if (t <= holdEnd) {
      rightAngle = rightClosed;
      leftAngle = leftClosed;
    } else {
      float phase = easeInOutExpo((t - holdEnd) / (1.0 - holdEnd));
      rightAngle = rightClosed + phase * (rightOpen - rightClosed);
      leftAngle = leftClosed + phase * (leftOpen - leftClosed);
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
      float t = easeInOutExpo((float)i / wobbleSteps);
      int jawAngle = wobbleLow + t * (wobbleHigh - wobbleLow);
      moveServo("jaw", jawAngle);

      float neckT = easeInOutExpo((float)wobbleSubStep / (totalWobbleSubSteps - 1));
      moveServo("neck1", neckEnd + neckT * (90 - neckEnd));
      wobbleSubStep++;

      delay(wobbleStepDelayMs);
    }
    // high -> low
    for (int i = 0; i <= wobbleSteps; i++) {
      float t = easeInOutExpo((float)i / wobbleSteps);
      int jawAngle = wobbleHigh + t * (wobbleLow - wobbleHigh);
      moveServo("jaw", jawAngle);

      float neckT = easeInOutExpo((float)wobbleSubStep / (totalWobbleSubSteps - 1));
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
    float t = easeInOutExpo((float)i / returnSteps);
    int jawAngle = jawFrom + t * (homeAngle - jawFrom);
    moveServo("jaw", jawAngle);
    moveServo("neck1", homeAngle);
    moveServo("eyelidRight", rightOpen);
    moveServo("eyelidLeft", leftOpen);
    delay(returnStepDelayMs);
  }
}

// ============================================================
// Generic helper: move any group of named servos together,
// each starting from wherever it currently is, all arriving
// at their own target at the same time.
// ============================================================
const uint8_t MAX_GROUP_SERVOS = 8;

void moveServosTogether(const char* names[], const int targets[], int count, int steps, int stepDelayMs, float easeSteepness = 28.0) {
  int startAngles[MAX_GROUP_SERVOS];
  for (int c = 0; c < count; c++) {
    int idx = servoIndex(names[c]);
    startAngles[c] = (idx != -1) ? lastAngle[idx] : targets[c];
  }

  // Neck axes this particular call isn't already driving are free
  // for idleSway to keep moving underneath it (a no-op outside idle
  // mode, since applyIdleSway() checks idleSwayActive itself).
  bool freeNeck1 = !servoInGroup("neck1", names, count);
  bool freeNeck2 = !servoInGroup("neck2", names, count);
  bool freeNeck3 = !servoInGroup("neck3", names, count);

  for (int i = 0; i <= steps; i++) {
    float t = easeInOutExpo((float)i / steps, easeSteepness);
    for (int c = 0; c < count; c++) {
      int angle = startAngles[c] + t * (targets[c] - startAngles[c]);
      moveServo(names[c], angle);
    }
    applyIdleSway(freeNeck1, freeNeck2, freeNeck3);
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
// Curious head tilt
// Neck2/neck3 lean together to one side (picked at random each
// time) with a slight curve, while the eyes glance toward the same
// side -- like a dog or cat tilting its head at something curious.
// Holds the pose briefly, then eases back to center. No sound.
// Type "tilt" into the Serial Monitor to trigger it.
// ============================================================
void curiousTiltAnimation() {
  bool tiltRight = random(0, 2) == 0; // picks a side at random each time

  const int neck2Tilt = tiltRight ? 130 : 50;
  const int neck3Tilt = tiltRight ? 115 : 65;
  const int eyeTilt = tiltRight ? 140 : 40;

  const char* names[] = { "neck2", "neck3", "eyeLeft", "eyeRight" };
  const int leanTargets[] = { neck2Tilt, neck3Tilt, eyeTilt, eyeTilt };
  moveServosTogether(names, leanTargets, 4, 60, 12);

  delay(700); // hold the curious pose for a moment

  const int homeTargets[] = { 90, 90, 90, 90 };
  moveServosTogether(names, homeTargets, 4, 60, 12);
}

// ============================================================
// Silent yawn/stretch
// Jaw opens slowly and wide, eyelids squint halfway toward closed,
// neck1 leans back a little -- a lazy stretch moment. No sound,
// unlike the roar animations that also open the jaw wide.
// Type "yawn" into the Serial Monitor to trigger it.
// ============================================================
void yawnAnimation() {
  const char* names[] = { "jaw", "neck1", "eyelidRight", "eyelidLeft" };
  const int openTargets[] = { 25, 110, 49, 105 }; // squint: halfway between each eye's open and closed
  moveServosTogether(names, openTargets, 4, 90, 14); // slow -- a yawn isn't rushed

  delay(600); // hold at the peak of the yawn

  const int homeTargets[] = { 90, 90, 58, 96 };
  moveServosTogether(names, homeTargets, 4, 70, 12);
}

// ============================================================
// Scanning look-around
// Eyes sweep from one side to the other and back, with neck2/neck3
// following a little, before settling back to front -- an alert,
// watchful moment. No sound.
// Type "look around" into the Serial Monitor to trigger it.
// ============================================================
void lookAroundAnimation() {
  const char* names[] = { "eyeLeft", "eyeRight", "neck2", "neck3" };

  const int rightTargets[] = { 140, 140, 125, 125 };
  moveServosTogether(names, rightTargets, 4, 90, 14); // slow sweep to one side
  delay(400); // brief pause, like taking in what's there

  const int leftTargets[] = { 40, 40, 55, 55 };
  moveServosTogether(names, leftTargets, 4, 130, 12); // slower sweep across to the other side
  delay(400);

  const int frontTargets[] = { 90, 90, 90, 90 };
  moveServosTogether(names, frontTargets, 4, 80, 10); // settle back to center
}

// ============================================================
// Startled flinch
// A quick, sharp snap: neck jerks back and eyelids pop wider open,
// like something surprised it. Fast in, brief hold, eases back out
// slower than it snapped in. No sound.
// Type "flinch" into the Serial Monitor to trigger it.
// ============================================================
void flinchAnimation() {
  const char* names[] = { "neck1", "eyelidRight", "eyelidLeft" };
  const int startleTargets[] = { 130, 78, 76 }; // wide eyes: further open than the normal resting position

  // Stretching the duration alone couldn't keep up once
  // easeInOutExpo()'s default curve got steeper -- peak velocity
  // scales with steepness, not just total time, so this needs its
  // own much gentler curve (steepness 10 vs. the default 28) rather
  // than another round of more steps/more delay.
  moveServosTogether(names, startleTargets, 3, 35, 16, 10.0);

  delay(250); // brief startled hold

  const int homeTargets[] = { 90, 58, 96 };
  moveServosTogether(names, homeTargets, 3, 40, 12); // ease back out, slower than the snap in
}

// ============================================================
// Look and hold
// Turns the eyes and neck2/neck3 to a random side and actually
// holds there for several seconds -- unlike lookAroundAnimation(),
// which sweeps through both sides and returns to front on its own
// within about a second, this one lingers, like the dragon noticed
// something and kept watching it. neck1 keeps doing its normal idle
// sway throughout the hold (via applyIdleSway()), since this doesn't
// touch that axis at all. Eases back to center once the hold ends.
// No sound. Type "look hold" into the Serial Monitor to trigger it.
// ============================================================
void lookAndHoldAnimation() {
  bool lookRight = random(0, 2) == 0; // picks a side at random each time

  const char* names[] = { "eyeLeft", "eyeRight", "neck2", "neck3" };
  const int rightTargets[] = { 140, 140, 130, 130 };
  const int leftTargets[] = { 40, 40, 50, 50 };
  const int* sideTargets = lookRight ? rightTargets : leftTargets;

  moveServosTogether(names, sideTargets, 4, 80, 10);

  long holdMs = random(5000, 12000);
  const int holdStepMs = 100;
  for (long waited = 0; waited < holdMs; waited += holdStepMs) {
    if (Serial.available()) break;
    applyIdleSway(true, false, false); // neck1 only -- neck2/neck3 stay held to the side
    delay(holdStepMs);
  }

  const int frontTargets[] = { 90, 90, 90, 90 };
  moveServosTogether(names, frontTargets, 4, 90, 12);
}

// ============================================================
// Ror two
// neck1 and jaw both bob smoothly between 90 (up/closed) and 20
// (down/open), completing two full cycles together — so the head
// goes up-down-up-down and the mouth opens-closes-opens-closes.
// Right in the middle (t = 0.5), where neck1/jaw pass back through
// 90 between the two cycles, the eyelids do one full blink.
// Type "ror two" into the Serial Monitor to trigger it.
//
// Paired with mp3/0024.mp3 (sounds/ror_two.mp3) -- a single clean bark
// from clip_07, sped up 1.5x and duplicated, with the two copies placed
// 700ms apart so their peaks land exactly on this animation's two
// mouth-fully-open instants (t=0.25 and t=0.75 of its 1.4s runtime).
// ============================================================
void ror2Animation() {
  if (dfPlayerReady) {
    dfPlayer.playMp3Folder(24);
  }

  const float center = 55.0;    // midpoint between 90 (up) and 20 (down)
  const float amplitude = 35.0; // 55 +/- 35 = 90 and 20
  const int cycles = 2;         // two full up-down / open-close cycles

  const int steps = 140;      // fine resolution for smoothness
  const int stepDelayMs = 10; // ~1.4s total, similar pace to the original roar

  // Blink window: centered on t = 0.5, closing / holding / opening
  // within that window, eyelids stay open the rest of the time.
  const float blinkStart = 0.42;
  const float blinkEnd = 0.58;
  const int rightOpen = 58, rightClosed = 40;
  const int leftOpen = 96, leftClosed = 114;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;

    float wave = center + amplitude * cos(2 * PI * cycles * t);
    int neckAngle = (int)(wave + 0.5);
    int jawAngle = (int)(wave + 0.5);

    int rightAngle = rightOpen;
    int leftAngle = leftOpen;
    if (t >= blinkStart && t <= blinkEnd) {
      float local = (t - blinkStart) / (blinkEnd - blinkStart); // 0.0 -> 1.0 within the window
      if (local <= 1.0 / 3.0) {
        float phase = easeInOutExpo(local / (1.0 / 3.0));
        rightAngle = rightOpen + phase * (rightClosed - rightOpen);
        leftAngle = leftOpen + phase * (leftClosed - leftOpen);
      } else if (local <= 2.0 / 3.0) {
        rightAngle = rightClosed;
        leftAngle = leftClosed;
      } else {
        float phase = easeInOutExpo((local - 2.0 / 3.0) / (1.0 / 3.0));
        rightAngle = rightClosed + phase * (rightOpen - rightClosed);
        leftAngle = leftClosed + phase * (leftOpen - leftClosed);
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
  const int targets[] = { 40, 114 };
  moveServosTogether(names, targets, 2, 40, 10);
}

void eyesOpenAnimation() {
  const char* names[] = { "eyelidRight", "eyelidLeft" };
  const int targets[] = { 58, 96 };
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
  if (dfPlayerReady) {
    dfPlayer.playMp3Folder(5); // mp3/0005.mp3 -- the recording this animation is synced to
  }

  const int rightOpen = 58, rightClosed = 40;
  const int leftOpen = 96, leftClosed = 114;

  // Blink window: centered on the loudest frame in the recording
  // (frame 87 of 0-129, ~3.48s in) -- closes, holds briefly, opens.
  // 8 frames (320ms) each way, closer to blinkEyelids()'s natural
  // pace, so it eases shut instead of snapping.
  const int blinkStartFrame = 80;
  const int blinkCloseFrame = 88;
  const int blinkEndFrame = 96;

  for (uint8_t i = 0; i < CLIP5_NUM_FRAMES; i++) {
    int jawAngle = pgm_read_byte(&clip5Jaw[i]);
    int neckAngle = pgm_read_byte(&clip5Neck1[i]);

    // neck2/neck3 sway gently throughout, independent of loudness --
    // the same "background life" role they play during rorAnimation().
    // Fades out over the last 20 frames so the sway settles to 90 on
    // its own, instead of oscillating right up to the final frame and
    // then snapping still the instant the settle-to-home phase below
    // takes over.
    float t = (float)i / (CLIP5_NUM_FRAMES - 1);
    const uint8_t swayFadeStartFrame = CLIP5_NUM_FRAMES - 20;
    float swayFade = 1.0;
    if (i >= swayFadeStartFrame) {
      swayFade = 1.0 - (float)(i - swayFadeStartFrame) / (CLIP5_NUM_FRAMES - 1 - swayFadeStartFrame);
    }
    int neck2Angle = 90 + swayFade * 10 * sin(2 * PI * 1.5 * t);
    int neck3Angle = 90 + swayFade * 6 * sin(2 * PI * 1.5 * t);

    int rightAngle = rightOpen;
    int leftAngle = leftOpen;
    if (i >= blinkStartFrame && i < blinkCloseFrame) {
      float phase = easeInOutExpo((float)(i - blinkStartFrame) / (blinkCloseFrame - blinkStartFrame));
      rightAngle = rightOpen + phase * (rightClosed - rightOpen);
      leftAngle = leftOpen + phase * (leftClosed - leftOpen);
    } else if (i >= blinkCloseFrame && i < blinkEndFrame) {
      float phase = easeInOutExpo((float)(i - blinkCloseFrame) / (blinkEndFrame - blinkCloseFrame));
      rightAngle = rightClosed + phase * (rightOpen - rightClosed);
      leftAngle = leftClosed + phase * (leftOpen - leftClosed);
    }

    moveServo("jaw", jawAngle);
    moveServo("neck1", neckAngle);
    moveServo("neck2", neck2Angle);
    moveServo("neck3", neck3Angle);
    moveServo("eyelidRight", rightAngle);
    moveServo("eyelidLeft", leftAngle);

    delay(CLIP5_FRAME_MS);
  }

  // Settle everything back to home once the envelope runs out. Jaw/neck1
  // can still be noticeably off from 90 at this point (the recording's
  // last frame isn't necessarily quiet), so this eases out slower than
  // the 40ms-per-frame pace of the envelope itself, rather than snapping
  // the remaining distance shut.
  const char* names[] = { "jaw", "neck1", "neck2", "neck3", "eyelidRight", "eyelidLeft" };
  const int targets[] = { 90, 90, 90, 90, 58, 96 };
  moveServosTogether(names, targets, 6, 60, 12);
}

// ============================================================
// Channel scan (diagnostic)
// Cycles through PCA9685 channels 0-15 one at a time, wiggling each
// briefly (a small +/-12 degree nudge around center, safe for any
// standard servo regardless of its configured range) and printing
// its number to the Serial Monitor. Watch which physical connector
// moves and match it to the printed channel to figure out actual
// wiring on an unlabeled board. Type "scan" to trigger it.
// ============================================================
void scanChannels() {
  const int midTicks = (SERVO_MIN_TICKS + SERVO_MAX_TICKS) / 2;
  const int wiggleTicks = 30; // ~12 degrees each way -- safe for every configured servo

  for (uint8_t ch = 0; ch < 16; ch++) {
    Serial.print(F("Channel "));
    Serial.println(ch);

    pwm.setPWM(ch, 0, midTicks - wiggleTicks);
    delay(800);
    pwm.setPWM(ch, 0, midTicks + wiggleTicks);
    delay(800);
    pwm.setPWM(ch, 0, midTicks);
    delay(1500);
  }

  Serial.println(F("Scan complete."));
}

// ============================================================
// Idle mode
// Randomly runs "self-returning" animations -- ones that do their
// thing and settle back to home on their own -- with a 10-20 second
// gap between each, to make the dragon look alive when nothing else
// is happening. Deliberately excludes "stateful" animations like
// eyesClosedAnimation() or lookRightAnimation() that move somewhere
// and stay there, since a random pick landing on one of those and
// not revisiting it for a while would look broken/stuck rather than
// alive. Never triggers sound.
//
// The neck sways continuously in the background -- a different
// period on each of the three neck servos so the combined motion
// doesn't look like a robotic uniform wobble, closer to how a real
// animal never holds perfectly still -- while the eyelids blink
// regularly (every 3-6 seconds) and a bigger animation from the pool
// below fires every 10-20 seconds. blinkEyelids() isn't in that pool
// since the regular blinking already covers it; the pool is reserved
// for the more distinctive moments.
//
// Big animations fire whenever their timer comes up, regardless of
// where the sway currently has neck2/neck3. The sway doesn't stop
// while one plays -- moveServosTogether() drives whichever neck axis
// that particular animation isn't itself using (e.g. neck1 keeps
// swaying during curiousTiltAnimation(), which only moves neck2/3),
// just at a lessened amplitude so it reads as background motion
// rather than competing with the animation. Once the animation
// finishes, the neck is explicitly resynced to a clean 90 baseline
// and the ambient sway fades back up to full amplitude from there.
// lookAndHoldAnimation() is one of the animations in the pool --
// unlike the others it lingers to one side for several seconds
// before returning, with neck1 still swaying throughout the hold.
//
// Separately, a "freeze" eases the sway down to a dead stop for a
// couple of seconds every 6-10 seconds (more often than the big
// animations, less often than blinking), then eases back up --
// just a moment of stillness before it keeps moving.
//
// Runs until any Serial input arrives (checked every tick, not
// mid-animation) -- once buttons exist, this same check can be
// swapped for reading a button pin instead.
// Type "idle" into the Serial Monitor to start it.
// ============================================================
typedef void (*AnimationFunc)();
AnimationFunc idleAnimations[] = {
  curiousTiltAnimation,
  yawnAnimation,
  lookAroundAnimation,
  flinchAnimation,
  lookAndHoldAnimation,
};
const uint8_t NUM_IDLE_ANIMATIONS = sizeof(idleAnimations) / sizeof(idleAnimations[0]);

void idleAnimation() {
  Serial.println(F("Entering idle mode -- type anything to stop."));

  const char* neckNames[] = { "neck1", "neck2", "neck3" };
  const int neckHome[] = { 90, 90, 90 };

  idleSwayActive = true;
  idleSwayStartMillis = millis();
  idleSwayScale = 1.0;

  // 30ms (instead of the original 100ms) gives ~20 intermediate steps
  // across the 600ms freeze fade instead of just 6, which was coarse
  // enough to feel like discrete little jumps rather than one smooth
  // motion.
  const int swayStepMs = 30;
  const long fadeMs = 1500;    // fade the ambient sway in after each resync
  const long freezeFadeMs = 600; // fade the sway down into a freeze, and back up out of one

  long sinceResync = 0; // ms since the neck was last at a clean 90 baseline
  long nextBlinkAt = random(3000, 6000);
  long nextBigAnimationAt = random(10000, 20000);

  // "Freeze" -- the dragon just holds still for a few seconds before
  // resuming the ambient sway, like it paused. More often than the
  // big animations, less often than blinking.
  long nextFreezeAt = random(6000, 10000);
  long freezeStartedAt = -1; // -1 = not currently frozen
  long freezeDurationMs = 0;

  while (!Serial.available()) {
    if (freezeStartedAt >= 0 && sinceResync - freezeStartedAt >= freezeDurationMs) {
      freezeStartedAt = -1;
      nextFreezeAt = sinceResync + random(6000, 10000);

      // Resume the live clock exactly where the frozen phase left
      // off, so position is continuous the instant amplitude is back
      // to full -- not reset to a different point in the cycle.
      idleSwayPhaseFrozen = false;
      idleSwayStartMillis = millis() - idleSwayFrozenElapsed;
    }
    if (freezeStartedAt < 0 && sinceResync >= nextFreezeAt) {
      freezeStartedAt = sinceResync;
      freezeDurationMs = random(2000, 4000);

      // Snapshot the current phase and hold it fixed for the whole
      // freeze (fade-down, hold, and fade-up) -- only the amplitude
      // changes during a freeze, never the position it's centered on.
      idleSwayFrozenElapsed = millis() - idleSwayStartMillis;
      idleSwayPhaseFrozen = true;
    }

    if (freezeStartedAt >= 0) {
      // easeInOutExpo() has near-zero rate of change at both ends, so
      // easing the fade itself (not just linearly ramping the scale)
      // makes it visibly slow down right into the stop and slowly
      // pick back up out of it, instead of a constant-rate ramp.
      long sinceFreezeStart = sinceResync - freezeStartedAt;
      if (sinceFreezeStart < freezeFadeMs) {
        idleSwayScale = 1.0 - easeInOutExpo((float)sinceFreezeStart / freezeFadeMs);
      } else if (sinceFreezeStart > freezeDurationMs - freezeFadeMs) {
        // Progress INTO this fade-up window (0 at its start, 1 right
        // as the freeze ends) -- not time-remaining-until-the-end,
        // which was counting the wrong direction and made this ramp
        // straight back down to 0 exactly when the freeze finished.
        float t = (float)(sinceFreezeStart - (freezeDurationMs - freezeFadeMs)) / freezeFadeMs;
        if (t > 1.0) t = 1.0;
        idleSwayScale = easeInOutExpo(t);
      } else {
        idleSwayScale = 0.0;
      }
    } else {
      idleSwayScale = (sinceResync < fadeMs) ? easeInOutExpo((float)sinceResync / fadeMs) : 1.0;
    }
    applyIdleSway(true, true, true);

    if (sinceResync >= nextBlinkAt) {
      blinkEyelids(); // keeps swaying underneath via its own applyIdleSway() calls
      nextBlinkAt = sinceResync + random(3000, 6000);
      if (Serial.available()) break;
    }

    if (sinceResync >= nextBigAnimationAt) {
      // Lessen (not stop) the sway -- moveServosTogether() inside
      // whichever animation runs will keep driving any neck axis it
      // isn't using itself, at this reduced amplitude.
      idleSwayScale = 0.7;

      uint8_t idx = random(0, NUM_IDLE_ANIMATIONS);
      idleAnimations[idx](); // fires from wherever neck2/neck3 currently are

      if (Serial.available()) break;

      // Resync to a clean baseline before the next ambient fade-in.
      moveServosTogether(neckNames, neckHome, 3, 20, 10);

      sinceResync = 0;
      nextBlinkAt = random(3000, 6000);
      nextBigAnimationAt = random(10000, 20000);
      nextFreezeAt = random(6000, 10000);
      freezeStartedAt = -1;
      idleSwayPhaseFrozen = false; // in case a freeze happened to still be active when this fired
      continue;
    }

    delay(swayStepMs);
    sinceResync += swayStepMs;
  }

  idleSwayActive = false;

  // Ease the neck back to exact home in case idle mode stopped
  // mid-sway, so the next command starts from a known position.
  moveServosTogether(neckNames, neckHome, 3, 20, 10);

  Serial.println(F("Idle mode stopped."));
}

// ============================================================
// Test 1
// Runs every named animation currently in the system, one after
// another, with about a 3 second pause between each. Handy for
// checking that everything still works after changing pins or
// safe ranges. Type "test1" into the Serial Monitor.
// ============================================================
void test1Animation() {
  Serial.println(F("[test1] blink"));
  blinkEyelids();
  delay(3000);

  Serial.println(F("[test1] ror"));
  rorAnimation();
  delay(3000);

  Serial.println(F("[test1] ror two"));
  ror2Animation();
  delay(3000);

  Serial.println(F("[test1] look right"));
  lookRightAnimation();
  delay(3000);

  Serial.println(F("[test1] look left"));
  lookLeftAnimation();
  delay(3000);

  Serial.println(F("[test1] front"));
  frontAnimation();
  delay(3000);

  Serial.println(F("[test1] tilt"));
  curiousTiltAnimation();
  delay(3000);

  Serial.println(F("[test1] yawn"));
  yawnAnimation();
  delay(3000);

  Serial.println(F("[test1] look around"));
  lookAroundAnimation();
  delay(3000);

  Serial.println(F("[test1] flinch"));
  flinchAnimation();
  delay(3000);

  Serial.println(F("[test1] look hold"));
  lookAndHoldAnimation();
  delay(3000);

  Serial.println(F("[test1] eyes closed"));
  eyesClosedAnimation();
  delay(3000);

  Serial.println(F("[test1] eyes open"));
  eyesOpenAnimation();
  delay(3000);

  Serial.println(F("[test1] clip5"));
  clip5Animation();
  delay(3000);

  Serial.println(F("[test1] complete"));
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
    Serial.println(F("Blinking..."));
    blinkEyelids();
    Serial.println(F("Blink done."));
    return;
  }

  if (line.equalsIgnoreCase("ror")) {
    Serial.println(F("Roaring..."));
    rorAnimation();
    Serial.println(F("Roar done."));
    return;
  }

  if (line.equalsIgnoreCase("ror two")) {
    Serial.println(F("Roaring (take two)..."));
    ror2Animation();
    Serial.println(F("Ror two done."));
    return;
  }

  if (line.equalsIgnoreCase("look right")) {
    Serial.println(F("Looking right..."));
    lookRightAnimation();
    Serial.println(F("Look right done."));
    return;
  }

  if (line.equalsIgnoreCase("look left")) {
    Serial.println(F("Looking left..."));
    lookLeftAnimation();
    Serial.println(F("Look left done."));
    return;
  }

  if (line.equalsIgnoreCase("front")) {
    Serial.println(F("Returning to front..."));
    frontAnimation();
    Serial.println(F("Front done."));
    return;
  }

  if (line.equalsIgnoreCase("tilt")) {
    Serial.println(F("Tilting head..."));
    curiousTiltAnimation();
    Serial.println(F("Tilt done."));
    return;
  }

  if (line.equalsIgnoreCase("yawn")) {
    Serial.println(F("Yawning..."));
    yawnAnimation();
    Serial.println(F("Yawn done."));
    return;
  }

  if (line.equalsIgnoreCase("look around")) {
    Serial.println(F("Looking around..."));
    lookAroundAnimation();
    Serial.println(F("Look around done."));
    return;
  }

  if (line.equalsIgnoreCase("flinch")) {
    Serial.println(F("Flinching..."));
    flinchAnimation();
    Serial.println(F("Flinch done."));
    return;
  }

  if (line.equalsIgnoreCase("look hold")) {
    Serial.println(F("Looking and holding..."));
    lookAndHoldAnimation();
    Serial.println(F("Look hold done."));
    return;
  }

  if (line.equalsIgnoreCase("eyes closed")) {
    Serial.println(F("Closing eyes..."));
    eyesClosedAnimation();
    Serial.println(F("Eyes closed."));
    return;
  }

  if (line.equalsIgnoreCase("eyes open")) {
    Serial.println(F("Opening eyes..."));
    eyesOpenAnimation();
    Serial.println(F("Eyes open."));
    return;
  }

  if (line.equalsIgnoreCase("clip5")) {
    Serial.println(F("Playing clip5..."));
    clip5Animation();
    Serial.println(F("Clip5 done."));
    return;
  }

  String lowerLine = line;
  lowerLine.toLowerCase();
  if (lowerLine.startsWith("play ")) {
    if (!dfPlayerReady) {
      Serial.println(F("DFPlayer not ready -- check wiring and SD card."));
      return;
    }
    int trackNum = line.substring(5).toInt();
    if (trackNum <= 0) {
      Serial.println(F("Format: play <trackNumber>   e.g. play 5   (plays mp3/0005.mp3)"));
      return;
    }
    Serial.print(F("Playing track "));
    Serial.println(trackNum);
    dfPlayer.playMp3Folder(trackNum);
    return;
  }

  if (line.equalsIgnoreCase("scan")) {
    Serial.println(F("Scanning channels 0-15..."));
    scanChannels();
    return;
  }

  if (line.equalsIgnoreCase("idle")) {
    idleAnimation();
    return;
  }

  if (line.equalsIgnoreCase("test1")) {
    Serial.println(F("Running test1 (all animations)..."));
    test1Animation();
    return;
  }

  int spaceIndex = line.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println(F("Format: <servoName> <angle>   e.g. jaw 60"));
    return;
  }

  String name = line.substring(0, spaceIndex);
  String angleStr = line.substring(spaceIndex + 1);
  angleStr.trim();

  if (angleStr.length() == 0) {
    Serial.println(F("Format: <servoName> <angle>   e.g. jaw 60"));
    return;
  }

  int angle = angleStr.toInt();

  int idx = servoIndex(name.c_str());
  if (idx == -1) {
    Serial.print(F("Unknown servo: "));
    Serial.println(name);
    Serial.print(F("Valid names: "));
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      Serial.print(servoConfigs[i].name);
      if (i < NUM_SERVOS - 1) Serial.print(F(", "));
    }
    Serial.println();
    return;
  }

  moveServoSmooth(name.c_str(), angle);

  Serial.print(name);
  Serial.print(F(" -> "));
  Serial.print(constrain(angle, servoConfigs[idx].minAngle, servoConfigs[idx].maxAngle));
  Serial.println(F(" degrees"));
}
