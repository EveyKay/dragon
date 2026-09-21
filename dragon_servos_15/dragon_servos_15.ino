#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <string.h>
#include <stdlib.h>

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
  { "neck2",       8, 90,  20, 165, 15 }, // range widened from 30-150 after testing found it can safely travel to 20/165 before binding
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

// Timestamp of the last commanded move for each servo, used by
// moveServo()'s hard speed cap below to know how much time actually
// elapsed since the previous command.
unsigned long lastMoveMillis[NUM_SERVOS];

// Hard ceiling on how many degrees any servo is ever allowed to move
// per millisecond, enforced unconditionally in moveServo() itself --
// unlike the eased curves (which shape a well-behaved move), this is
// the backstop that catches a bad steepness value, a curve's own peak
// spike, or any future bug that tries to slam a servo across a big
// distance in one tick, and silently slows the actual commanded angle
// down to this rate instead. 0.2 deg/ms is comfortably under standard
// hobby servo slew rates even under mechanical load.
const float MAX_DEGREES_PER_MS = 0.2;

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
// capSpeed defaults on for every normal call, enforcing
// MAX_DEGREES_PER_MS regardless of what asked for the move; setup()
// passes false for the very first, one-time snap to each servo's home
// position at boot, since there's no known prior position to safely
// ramp from there anyway (the PCA9685 can't report where a servo
// actually is), and the cap would otherwise mistake "no boot history"
// for a real starting position of 0 and immediately clip that first
// move to almost nothing.
void moveServo(const char* name, int angle, bool capSpeed = true) {
  int idx = servoIndex(name);
  if (idx == -1) {
    Serial.print(F("Unknown servo: "));
    Serial.println(name);
    return;
  }
  angle = constrain(angle, servoConfigs[idx].minAngle, servoConfigs[idx].maxAngle);

  unsigned long now = millis();
  if (capSpeed) {
    unsigned long elapsedMs = now - lastMoveMillis[idx];
    int maxDelta = (int)(MAX_DEGREES_PER_MS * elapsedMs);
    if (maxDelta < 1) maxDelta = 1;
    int delta = angle - lastAngle[idx];
    if (delta > maxDelta) angle = lastAngle[idx] + maxDelta;
    else if (delta < -maxDelta) angle = lastAngle[idx] - maxDelta;
  }
  lastMoveMillis[idx] = now; // always refreshed, even when this particular call skipped the cap, so the next call's elapsed time is measured from here

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
// (20) is used everywhere movement is eased through this curve.
//
// This used to go as high as 60, which crammed nearly all of a move's
// travel into a narrow sliver of time near the middle -- MAX_DEGREES_PER_MS
// below is what actually keeps that (or any curve) from commanding a
// servo faster than it can physically track, so this default is free
// to just be picked for how the motion looks rather than doubling as
// the only thing standing between a steep curve and a strained motor.
float easeInOutExpo(float t, float steepness = 20.0) {
  if (t <= 0.0) return 0.0;
  if (t >= 1.0) return 1.0;
  const float ln2 = 0.6931472;
  if (t < 0.5) {
    return 0.5 * expf((steepness * t - steepness / 2.0) * ln2);
  } else {
    return 1.0 - 0.5 * expf((-steepness * t + steepness / 2.0) * ln2);
  }
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

// Amplitude (degrees) and period (ms, full cycle) per neck axis.
// Re-randomized periodically by randomizeSwayVariance() -- always at
// a moment idleSwayScale is at/near 0 (right after a resync or a
// freeze ends), so changing these never causes a visible jump, since
// amplitude*sin(anything) contributes ~nothing at that instant
// regardless of what the new values are.
float neck1SwayAmplitude = 14, neck2SwayAmplitude = 10, neck3SwayAmplitude = 7;
float neck1SwayPeriodMs = 4000, neck2SwayPeriodMs = 5500, neck3SwayPeriodMs = 7000;

void randomizeSwayVariance() {
  neck1SwayAmplitude = random(18, 30);
  neck2SwayAmplitude = random(14, 24);
  neck3SwayAmplitude = random(10, 18);
  // Widened again -- was 3500-14000/5000-17000/6500-20000. Pushing the
  // fast end quicker and the slow end lazier spreads the pace out
  // further reroll to reroll, so consecutive swings can differ sharply
  // in speed instead of drifting only moderately.
  neck1SwayPeriodMs = random(2000, 20000);
  neck2SwayPeriodMs = random(3000, 24000);
  neck3SwayPeriodMs = random(4000, 28000);
}

bool servoInGroup(const char* name, const char* names[], int count) {
  for (int i = 0; i < count; i++) {
    if (strcmp(name, names[i]) == 0) return true;
  }
  return false;
}

// A back-and-forth oscillation between -amplitude and +amplitude, one
// full round trip every periodMs -- built out of easeInOutExpo() itself
// (ping-ponging between the two extremes) instead of a raw sin(), so
// the continuous neck sway shares the exact same curve and steepness
// knob as every point-to-point move in the sketch, rather than being
// the one motion that never actually ran through it. Takes its
// steepness from easeInOutExpo()'s own default rather than a second
// copy of the number, so the two can never drift out of sync.
float easedOscillate(float elapsedMs, float periodMs, float amplitude) {
  float halfPeriod = periodMs / 2.0;
  float legPhase = fmod(elapsedMs, periodMs) / halfPeriod; // 0..2: which leg of the round trip, and how far into it
  if (legPhase < 1.0) {
    return -amplitude + easeInOutExpo(legPhase) * (2.0 * amplitude); // -amplitude -> +amplitude
  } else {
    return amplitude - easeInOutExpo(legPhase - 1.0) * (2.0 * amplitude); // +amplitude -> -amplitude
  }
}

// Drives whichever of neck1/neck2/neck3 are passed as true -- callers
// pass false for any axis a bigger animation is actively controlling
// itself, so the two motions never fight over the same servo.
void applyIdleSway(bool doNeck1, bool doNeck2, bool doNeck3) {
  if (!idleSwayActive) return;
  unsigned long elapsed = idleSwayPhaseFrozen ? idleSwayFrozenElapsed : (millis() - idleSwayStartMillis);
  if (doNeck1) moveServo("neck1", 90 + idleSwayScale * easedOscillate(elapsed, neck1SwayPeriodMs, neck1SwayAmplitude));
  if (doNeck2) moveServo("neck2", 90 + idleSwayScale * easedOscillate(elapsed, neck2SwayPeriodMs, neck2SwayAmplitude));
  if (doNeck3) moveServo("neck3", 90 + idleSwayScale * easedOscillate(elapsed, neck3SwayPeriodMs, neck3SwayAmplitude));
}

// ============================================================
// Idle mode's ambient eye movement
// Real eyes don't drift in a continuous sway the way a neck can --
// they snap to a new point (a saccade) and hold there, so this is a
// separate mechanic from applyIdleSway() rather than reusing its
// sine-wave approach. idleAnimation()'s main loop schedules a new
// random target every couple of seconds; this just interpolates the
// current gaze position toward whatever that target currently is.
// ============================================================
int idleGazeStartAngle = 90;
int idleGazeTargetAngle = 90;
unsigned long idleGazeMoveStartMillis = 0;
long idleGazeMoveDurationMs = 400;

// Drives whichever of eyeLeft/eyeRight are passed as true -- same
// free-axis pattern as applyIdleSway(), so a bigger animation that's
// actively steering the eyes itself (tilt, look around, look hold)
// takes priority and this just doesn't touch them meanwhile.
void applyIdleGaze(bool doEyeLeft, bool doEyeRight) {
  if (!idleSwayActive) return;
  unsigned long elapsed = millis() - idleGazeMoveStartMillis;
  float t = (elapsed >= (unsigned long)idleGazeMoveDurationMs) ? 1.0 : easeInOutExpo((float)elapsed / idleGazeMoveDurationMs);
  int angle = idleGazeStartAngle + t * (idleGazeTargetAngle - idleGazeStartAngle);
  if (doEyeLeft) moveServo("eyeLeft", angle);
  if (doEyeRight) moveServo("eyeRight", angle);
}

// Holds still for durationMs -- used in place of a plain delay() during
// an animation's pose-hold, so whichever neck/eye axes that animation
// isn't using stay free to keep swaying/glancing underneath instead of
// freezing solid for the hold's duration. A flat delay() was exactly
// what curiousTiltAnimation()/yawnAnimation()/lookAroundAnimation()
// used to do here, which is why the ambient motion looked like it
// stopped in lockstep with each animation instead of continuing
// through it.
void idleHold(long durationMs, bool doNeck1, bool doNeck2, bool doNeck3, bool doEyeLeft, bool doEyeRight) {
  const int stepMs = 20; // was 50 -- finer sampling keeps the steep sway curve looking like motion instead of a pop
  for (long waited = 0; waited < durationMs; waited += stepMs) {
    if (Serial.available()) return;
    applyIdleSway(doNeck1, doNeck2, doNeck3);
    applyIdleGaze(doEyeLeft, doEyeRight);
    delay(stepMs);
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
    // The final step (t == 1.0, angle == clampedTarget exactly) skips
    // the speed cap -- if an earlier step got throttled by it partway
    // through the steepest part of the curve, the position can end up
    // trailing a few degrees behind where the curve says it should be,
    // and nothing else ever asks it to close that gap. Letting only
    // this last, guaranteed-exact step bypass the cap means the move
    // always actually finishes at its real target instead of settling
    // short of it.
    moveServo(name, angle, i != steps);
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
    moveServo(servoConfigs[i].name, servoConfigs[i].homeAngle, false); // one-time boot snap -- no known prior position to cap speed against

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
    moveServo("eyelidRight", rightAngle, i != steps); // last step bypasses the speed cap so a closing blink always actually reaches fully closed
    moveServo("eyelidLeft", leftAngle, i != steps);
    applyIdleSway(true, true, true); // blink never touches the neck or eyeballs, so all stay free
    applyIdleGaze(true, true);
    delay(stepDelayMs);
  }

  // Opening: closed -> open
  for (int i = 0; i <= steps; i++) {
    float t = easeInOutExpo((float)i / steps);
    int rightAngle = rightClosed + t * (rightOpen - rightClosed);
    int leftAngle = leftClosed + t * (leftOpen - leftClosed);
    moveServo("eyelidRight", rightAngle, i != steps); // last step bypasses the speed cap so an opening blink always actually reaches fully open
    moveServo("eyelidLeft", leftAngle, i != steps);
    applyIdleSway(true, true, true);
    applyIdleGaze(true, true);
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
    bool isFinalStep = (i == steps); // bypasses the speed cap so this loop always actually lands exactly on its targets, not just close

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

    moveServo("neck1", neckAngle, !isFinalStep);
    moveServo("neck2", neck2Angle, !isFinalStep);
    moveServo("neck3", neck3Angle, !isFinalStep);
    moveServo("jaw", jawAngle, !isFinalStep);
    moveServo("eyelidRight", rightAngle, !isFinalStep);
    moveServo("eyelidLeft", leftAngle, !isFinalStep);

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
      moveServo("jaw", jawAngle, i != wobbleSteps);

      float neckT = easeInOutExpo((float)wobbleSubStep / (totalWobbleSubSteps - 1));
      moveServo("neck1", neckEnd + neckT * (90 - neckEnd), wobbleSubStep != totalWobbleSubSteps - 1);
      wobbleSubStep++;

      delay(wobbleStepDelayMs);
    }
    // high -> low
    for (int i = 0; i <= wobbleSteps; i++) {
      float t = easeInOutExpo((float)i / wobbleSteps);
      int jawAngle = wobbleHigh + t * (wobbleLow - wobbleHigh);
      moveServo("jaw", jawAngle, i != wobbleSteps);

      float neckT = easeInOutExpo((float)wobbleSubStep / (totalWobbleSubSteps - 1));
      moveServo("neck1", neckEnd + neckT * (90 - neckEnd), wobbleSubStep != totalWobbleSubSteps - 1);
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
    // This loop's 4ms-per-step pace is fast enough that the speed cap
    // engages hard through the curve's steep middle -- exactly what was
    // leaving the jaw open after "ror". Bypassing the cap on only the
    // guaranteed-exact final step (same fix as moveServosTogether())
    // means it always actually reaches fully closed.
    bool isFinalStep = (i == returnSteps);
    moveServo("jaw", jawAngle, !isFinalStep);
    moveServo("neck1", homeAngle, !isFinalStep);
    moveServo("eyelidRight", rightOpen, !isFinalStep);
    moveServo("eyelidLeft", leftOpen, !isFinalStep);
    delay(returnStepDelayMs);
  }
}

// ============================================================
// Generic helper: move any group of named servos together,
// each starting from wherever it currently is, all arriving
// at their own target at the same time.
// ============================================================
const uint8_t MAX_GROUP_SERVOS = 8;

void moveServosTogether(const char* names[], const int targets[], int count, int steps, int stepDelayMs, float easeSteepness = 20.0) {
  int startAngles[MAX_GROUP_SERVOS];
  for (int c = 0; c < count; c++) {
    int idx = servoIndex(names[c]);
    startAngles[c] = (idx != -1) ? lastAngle[idx] : targets[c];
  }

  // Neck/eye axes this particular call isn't already driving are
  // free for idleSway/idleGaze to keep moving underneath it (a
  // no-op outside idle mode, since both check idleSwayActive first).
  bool freeNeck1 = !servoInGroup("neck1", names, count);
  bool freeNeck2 = !servoInGroup("neck2", names, count);
  bool freeNeck3 = !servoInGroup("neck3", names, count);
  bool freeEyeLeft = !servoInGroup("eyeLeft", names, count);
  bool freeEyeRight = !servoInGroup("eyeRight", names, count);

  for (int i = 0; i <= steps; i++) {
    float t = easeInOutExpo((float)i / steps, easeSteepness);
    bool isFinalStep = (i == steps);
    for (int c = 0; c < count; c++) {
      int angle = startAngles[c] + t * (targets[c] - startAngles[c]);
      // Last step (t == 1.0, angle == the real target) bypasses the
      // speed cap -- if the cap throttled an earlier, steeper step, the
      // commanded position can trail a few degrees behind the curve's
      // intended one, and nothing downstream ever asks it to close that
      // gap on its own. This is exactly what left the jaw open after a
      // yawn: the cap clipped its fast middle section, and by the last
      // step it was still a few degrees short of home with nothing left
      // to push it the rest of the way. Bypassing the cap on only this
      // guaranteed-exact final step means every move still actually
      // finishes at its real target.
      moveServo(names[c], angle, !isFinalStep);
    }
    applyIdleSway(freeNeck1, freeNeck2, freeNeck3);
    applyIdleGaze(freeEyeLeft, freeEyeRight);
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

  idleHold(700, true, false, false, false, false); // hold the curious pose -- neck1 is the only axis this animation doesn't use, so it's the only one free to keep swaying

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

  idleHold(600, false, true, true, true, true); // hold at the peak -- a yawn never touches neck2/neck3 or the eyes, so all of those stay free

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
  idleHold(400, true, false, false, false, false); // brief pause, like taking in what's there -- neck1 is the only free axis here

  const int leftTargets[] = { 40, 40, 55, 55 };
  moveServosTogether(names, leftTargets, 4, 130, 12); // slower sweep across to the other side
  idleHold(400, true, false, false, false, false);

  const int frontTargets[] = { 90, 90, 90, 90 };
  moveServosTogether(names, frontTargets, 4, 80, 10); // settle back to center
}

// ============================================================
// Quick chomps
// Three fast, shallow snaps of the jaw -- closed to a small open and
// back -- like stretching the jaw rather than a full yawn or roar.
// No sound. Type "chomp" into the Serial Monitor to trigger it.
// ============================================================
void quickChompsAnimation() {
  const char* names[] = { "jaw" };
  const int openTarget[] = { 75 };  // a small, shallow open -- not a full yawn
  const int closeTarget[] = { 90 };

  for (int rep = 0; rep < 3; rep++) {
    moveServosTogether(names, openTarget, 1, 15, 10);  // quick snap open
    moveServosTogether(names, closeTarget, 1, 15, 10); // quick snap closed
  }
}

// ============================================================
// Big side tilt
// Unlike the curious tilt (where neck2 and neck3 lean together by
// comparable amounts), this cranks neck2 almost all the way to its
// safe range limit while neck3 only follows a little -- reading as
// one joint doing a dramatic, near-maximum lean rather than the
// whole neck leaning together. Eyes glance the same way. Picks a
// side at random each time, and holds there noticeably longer than
// the curious tilt before easing back to center. No sound. Type
// "big tilt" into the Serial Monitor to trigger it.
// ============================================================
void bigTiltAnimation() {
  bool tiltRight = random(0, 2) == 0; // picks a side at random each time

  const int neck2Tilt = tiltRight ? 163 : 22; // almost the full 20-165 safe range
  const int neck3Tilt = tiltRight ? 98 : 82;  // only a little follow-through
  const int eyeTilt = tiltRight ? 148 : 32;

  const char* names[] = { "neck2", "neck3", "eyeLeft", "eyeRight" };
  const int leanTargets[] = { neck2Tilt, neck3Tilt, eyeTilt, eyeTilt };
  moveServosTogether(names, leanTargets, 4, 90, 14); // slower than the curious tilt -- it has much further to travel

  idleHold(1400, true, false, false, false, false); // a longer, more deliberate hold than the curious tilt -- neck1 is the only free axis here

  const int homeTargets[] = { 90, 90, 90, 90 };
  moveServosTogether(names, homeTargets, 4, 90, 14);
}

// ============================================================
// Shake
// Neck2 swings between its two tested safe extremes (the same 22/163
// big tilt uses) one and a half times (high-low-high), like a head
// shake, then settles back to center. Paced under the hard speed cap
// despite the huge ~140-degree swing -- fast enough to read as a
// shake, slow enough that the motor can actually keep up with the
// full distance rather than getting throttled by MAX_DEGREES_PER_MS
// mid-swing. No sound. Type "shake" into the Serial Monitor to
// trigger it.
// ============================================================
void shakeAnimation() {
  const char* names[] = { "neck2" };
  const int highTarget[] = { 163 };
  const int lowTarget[] = { 22 };

  const int shakeSteps = 110;     // fine resolution across the full-range swing
  const int shakeStepDelayMs = 7; // close to the floor: 141 degrees at the 0.2 deg/ms hard cap needs at least ~705ms, and this is ~770ms -- pushing much faster wouldn't actually move quicker, just ask for more than the cap allows and rely more heavily on the final step's snap to catch up

  moveServosTogether(names, highTarget, 1, shakeSteps, shakeStepDelayMs);
  moveServosTogether(names, lowTarget, 1, shakeSteps, shakeStepDelayMs);
  moveServosTogether(names, highTarget, 1, shakeSteps, shakeStepDelayMs); // the extra "half" shake

  const int homeTarget[] = { 90 };
  moveServosTogether(names, homeTarget, 1, 40, 12);
}

// ============================================================
// Shake chomp
// Eyes close, then neck2 and jaw oscillate at the same time --
// neck2 swinging between its shake extremes while the jaw does quick
// little chomps -- like the dragon is shaking something in its mouth.
// Built from easedOscillate() (the same continuous ping-pong curve
// idle sway uses) rather than a hand-timed step loop, then settled
// to an exact home through moveServosTogether() so neither axis ever
// trails behind like rorAnimation() once did. No sound. Type "shake
// chomp" into the Serial Monitor to trigger it.
// ============================================================
void shakeChompAnimation() {
  const char* eyeNames[] = { "eyelidRight", "eyelidLeft" };
  const int closedTargets[] = { 40, 114 };
  moveServosTogether(eyeNames, closedTargets, 2, 20, 15); // ease eyes shut first

  const float neck2Center = 92.5, neck2Amplitude = 70.5, neck2PeriodMs = 1540; // same 22/163 extremes and pace as shakeAnimation()
  const float jawCenter = 82.5, jawAmplitude = 7.5, jawPeriodMs = 600;         // a quick little chomp roughly every 300ms

  const unsigned long totalDurationMs = 1.5 * neck2PeriodMs; // 1.5 shakes, matching shakeAnimation()
  const int stepMs = 15;
  unsigned long startMillis = millis();

  while (millis() - startMillis < totalDurationMs) {
    unsigned long elapsed = millis() - startMillis;
    int neck2Angle = (int)(neck2Center + easedOscillate(elapsed, neck2PeriodMs, neck2Amplitude) + 0.5);
    int jawAngle = (int)(jawCenter + easedOscillate(elapsed, jawPeriodMs, jawAmplitude) + 0.5);

    moveServo("neck2", neck2Angle);
    moveServo("jaw", jawAngle);
    applyIdleSway(true, false, true); // neck1/neck3 stay free -- neck2 is driven directly above
    applyIdleGaze(true, true);        // eyeball gaze stays free -- only the eyelids are held shut

    delay(stepMs);
  }

  const char* names[] = { "neck2", "jaw" };
  const int homeTargets[] = { 90, 90 };
  moveServosTogether(names, homeTargets, 2, 30, 10); // land both exactly on home, same fix as shakeAnimation()'s final step

  const int openTargets[] = { 58, 96 };
  moveServosTogether(eyeNames, openTargets, 2, 20, 15); // reopen eyes
}

// ============================================================
// Neck stretch / look up
// Neck1 cranes upward and holds, like sniffing the air or watching
// something overhead, before easing back down to home. No sound.
// Type "look up" into the Serial Monitor to trigger it.
// ============================================================
void neckStretchAnimation() {
  const char* names[] = { "neck1" };
  const int upTarget[] = { 45 }; // lower angle = head up on this axis (opposite of what its number suggests)
  moveServosTogether(names, upTarget, 1, 70, 14); // slow, deliberate stretch upward

  idleHold(1000, false, true, true, true, true); // hold at full stretch -- doesn't touch neck2/neck3 or the eyes, so all stay free

  const int homeTarget[] = { 90 };
  moveServosTogether(names, homeTarget, 1, 70, 12);
}

// ============================================================
// Sleepy droop
// Eyelids ease to a heavy half-closed squint and the head droops
// slightly, like a moment of drowsiness, then eases back up. No jaw
// movement (unlike the yawn) and no sound. Type "sleepy" into the
// Serial Monitor to trigger it.
// ============================================================
void sleepyBlinkAnimation() {
  const char* names[] = { "eyelidRight", "eyelidLeft", "neck1" };
  const int droopTargets[] = { 49, 105, 100 }; // heavy-lidded squint (same halfway points as the yawn), slight downward droop -- higher angle = head down on this axis

  moveServosTogether(names, droopTargets, 3, 30, 25); // slow, heavy-lidded ease

  idleHold(1600, false, true, true, true, true); // hold the droop -- doesn't touch neck2/neck3 or the eyes, so all stay free

  const int homeTargets[] = { 58, 96, 90 };
  moveServosTogether(names, homeTargets, 3, 25, 20);
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
    bool isFinalStep = (i == steps); // bypasses the speed cap so this loop always actually lands on 90, the same fix rorAnimation() needed

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

    moveServo("neck1", neckAngle, !isFinalStep);
    moveServo("jaw", jawAngle, !isFinalStep);
    moveServo("eyelidRight", rightAngle, !isFinalStep);
    moveServo("eyelidLeft", leftAngle, !isFinalStep);

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
// animal never holds perfectly still. The amplitude and period of
// that sway are re-randomized by randomizeSwayVariance() every time
// there's a safe moment to do it invisibly (idleSwayScale genuinely
// at 0: right after a resync, or during a freeze's flat hold phase),
// so the sway itself keeps drifting in size and pace over time
// instead of being one fixed repeating pattern.
//
// The eyeballs get their own separate mechanic (applyIdleGaze()):
// unlike the neck's continuous sway, real eyes snap to a new point
// and hold there, so every couple of seconds they ease to a new
// subtle random gaze target (+/-25 degrees, not a full look-left/
// right turn) over a quick 300-700ms shift, then sit still until the
// next one. Meanwhile the eyelids blink
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
  lookAndHoldAnimation,
  quickChompsAnimation,
  bigTiltAnimation,
  neckStretchAnimation,
  sleepyBlinkAnimation,
  shakeAnimation,
  shakeChompAnimation,
};
const uint8_t NUM_IDLE_ANIMATIONS = sizeof(idleAnimations) / sizeof(idleAnimations[0]);

void printIdleAnimationName(uint8_t idx) {
  Serial.print(F("[idle] "));
  switch (idx) {
    case 0: Serial.println(F("tilt")); break;
    case 1: Serial.println(F("yawn")); break;
    case 2: Serial.println(F("look around")); break;
    case 3: Serial.println(F("look hold")); break;
    case 4: Serial.println(F("chomp")); break;
    case 5: Serial.println(F("big tilt")); break;
    case 6: Serial.println(F("look up")); break;
    case 7: Serial.println(F("sleepy")); break;
    case 8: Serial.println(F("shake")); break;
    case 9: Serial.println(F("shake chomp")); break;
    default: Serial.println(F("?")); break;
  }
}

void idleAnimation() {
  Serial.println(F("Entering idle mode -- type anything to stop."));

  const char* neckNames[] = { "neck1", "neck2", "neck3" };
  const int neckHome[] = { 90, 90, 90 };

  idleSwayActive = true;
  idleSwayStartMillis = millis();
  idleSwayScale = 1.0;
  randomizeSwayVariance();

  idleGazeStartAngle = 90;
  idleGazeTargetAngle = 90;
  idleGazeMoveStartMillis = millis();
  long nextGazeShiftAt = random(1500, 4000);

  // 15ms (instead of the original 100ms, then 30ms) gives ~40
  // intermediate steps across the 600ms freeze fade instead of just 6,
  // which was coarse enough to feel like discrete little jumps rather
  // than one smooth motion -- tightened further alongside the easing
  // curve's steepness, since a steeper curve concentrates real
  // movement into an even narrower time window that coarser sampling
  // would just turn back into a pop.
  const int swayStepMs = 15;
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
  bool freezeVarianceRerolled = false; // makes sure the reroll below only fires once per freeze

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
      freezeVarianceRerolled = false;

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

        // This is the only genuinely safe moment to reroll during a
        // freeze -- the fade-up (above) already brings scale back to
        // ~1.0 *before* "freeze ended" is detected, so rerolling at
        // that boundary (an earlier bug) was changing the pattern at
        // full amplitude, not zero, and it showed as a visible jump.
        // Here, scale is actually 0, so it's invisible.
        if (!freezeVarianceRerolled) {
          randomizeSwayVariance();
          freezeVarianceRerolled = true;
        }
      }
    } else {
      idleSwayScale = (sinceResync < fadeMs) ? easeInOutExpo((float)sinceResync / fadeMs) : 1.0;
    }
    applyIdleSway(true, true, true);

    if (sinceResync >= nextGazeShiftAt) {
      idleGazeStartAngle = idleGazeTargetAngle; // continue from wherever the last shift landed
      idleGazeTargetAngle = 90 + random(-25, 26); // a subtle glance, not a full look-left/right turn
      idleGazeMoveStartMillis = millis();
      idleGazeMoveDurationMs = random(300, 700); // some shifts a bit quicker/slower than others
      nextGazeShiftAt = sinceResync + idleGazeMoveDurationMs + random(1500, 4000);
    }
    applyIdleGaze(true, true);

    if (sinceResync >= nextBlinkAt) {
      blinkEyelids(); // keeps swaying underneath via its own applyIdleSway() calls
      nextBlinkAt = sinceResync + random(3000, 6000);
      if (Serial.available()) break;
    }

    if (sinceResync >= nextBigAnimationAt) {
      // Lessen (not stop) the sway -- moveServosTogether() inside
      // whichever animation runs will keep driving any neck axis it
      // isn't using itself, at this reduced amplitude. Ease down into
      // that reduced amplitude instead of snapping straight to it --
      // the sway's sine wave has a real position at every instant, so
      // stepping idleSwayScale from ~1.0 to 0.7 in one tick would jump
      // that position immediately (unless the phase happened to be
      // crossing zero right then), the same kind of discontinuity the
      // freeze fade was built to avoid.
      const long swayDampMs = 400;
      float dampStartScale = idleSwayScale;
      unsigned long dampStartMillis = millis();
      while (millis() - dampStartMillis < (unsigned long)swayDampMs) {
        if (Serial.available()) break;
        float dampT = easeInOutExpo((float)(millis() - dampStartMillis) / swayDampMs);
        idleSwayScale = dampStartScale + dampT * (0.7 - dampStartScale);
        applyIdleSway(true, true, true);
        applyIdleGaze(true, true);
        delay(swayStepMs);
      }
      idleSwayScale = 0.7;
      if (Serial.available()) break;

      uint8_t idx = random(0, NUM_IDLE_ANIMATIONS);
      printIdleAnimationName(idx);
      idleAnimations[idx](); // fires from wherever neck2/neck3 currently are

      if (Serial.available()) break;

      // Resync to a clean baseline before the next ambient fade-in.
      moveServosTogether(neckNames, neckHome, 3, 20, 10);
      randomizeSwayVariance(); // safe here too -- the next fade-in starts from scale 0

      // Whatever animation just ran may or may not have touched the
      // eyes itself. Either way, pick up gaze from wherever they
      // actually are right now rather than a target scheduled before
      // the animation started -- otherwise the very next gaze tick
      // could jump toward a now-stale target.
      int currentEyeAngle = lastAngle[servoIndex("eyeLeft")];
      idleGazeStartAngle = currentEyeAngle;
      idleGazeTargetAngle = currentEyeAngle;
      idleGazeMoveStartMillis = millis();
      nextGazeShiftAt = random(1500, 4000);

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

  // Ease the neck and eyes back to exact home in case idle mode
  // stopped mid-sway or mid-glance, so the next command starts from
  // a known position.
  const char* neckAndEyeNames[] = { "neck1", "neck2", "neck3", "eyeLeft", "eyeRight" };
  const int neckAndEyeHome[] = { 90, 90, 90, 90, 90 };
  moveServosTogether(neckAndEyeNames, neckAndEyeHome, 5, 20, 10);

  Serial.println(F("Idle mode stopped."));
}

// ============================================================
// Shared command table
// Each of these animations used to get triggered by its own
// hand-written block in BOTH handleSerialCommands() ("print a message,
// call it, print another message") and test1Animation() ("print its
// name, call it, pause") -- nearly-identical blocks differing only
// in which string/function they used. Centralizing that here means
// the actual dispatch/smoke-test logic exists once in each function,
// looped over this table, instead of duplicated per command. Strings
// are PROGMEM, same reasoning as F() everywhere else in this sketch --
// a table of plain string literals would otherwise cost real RAM
// just for existing.
// ============================================================
const char cmdName0[]  PROGMEM = "blink";
const char cmdName1[]  PROGMEM = "ror";
const char cmdName2[]  PROGMEM = "ror two";
const char cmdName3[]  PROGMEM = "look right";
const char cmdName4[]  PROGMEM = "look left";
const char cmdName5[]  PROGMEM = "front";
const char cmdName6[]  PROGMEM = "tilt";
const char cmdName7[]  PROGMEM = "yawn";
const char cmdName8[]  PROGMEM = "look around";
const char cmdName9[]  PROGMEM = "look hold";
const char cmdName10[] PROGMEM = "eyes closed";
const char cmdName11[] PROGMEM = "eyes open";
const char cmdName12[] PROGMEM = "clip5";
const char cmdName13[] PROGMEM = "chomp";
const char cmdName14[] PROGMEM = "big tilt";
const char cmdName15[] PROGMEM = "look up";
const char cmdName16[] PROGMEM = "sleepy";
const char cmdName17[] PROGMEM = "shake";
const char cmdName18[] PROGMEM = "shake chomp";

const char cmdStart0[]  PROGMEM = "Blinking...";
const char cmdStart1[]  PROGMEM = "Roaring...";
const char cmdStart2[]  PROGMEM = "Roaring (take two)...";
const char cmdStart3[]  PROGMEM = "Looking right...";
const char cmdStart4[]  PROGMEM = "Looking left...";
const char cmdStart5[]  PROGMEM = "Returning to front...";
const char cmdStart6[]  PROGMEM = "Tilting head...";
const char cmdStart7[]  PROGMEM = "Yawning...";
const char cmdStart8[]  PROGMEM = "Looking around...";
const char cmdStart9[]  PROGMEM = "Looking and holding...";
const char cmdStart10[] PROGMEM = "Closing eyes...";
const char cmdStart11[] PROGMEM = "Opening eyes...";
const char cmdStart12[] PROGMEM = "Playing clip5...";
const char cmdStart13[] PROGMEM = "Chomping...";
const char cmdStart14[] PROGMEM = "Big tilt...";
const char cmdStart15[] PROGMEM = "Looking up...";
const char cmdStart16[] PROGMEM = "Getting sleepy...";
const char cmdStart17[] PROGMEM = "Shaking...";
const char cmdStart18[] PROGMEM = "Shake chomping...";

const char cmdDone0[]  PROGMEM = "Blink done.";
const char cmdDone1[]  PROGMEM = "Roar done.";
const char cmdDone2[]  PROGMEM = "Ror two done.";
const char cmdDone3[]  PROGMEM = "Look right done.";
const char cmdDone4[]  PROGMEM = "Look left done.";
const char cmdDone5[]  PROGMEM = "Front done.";
const char cmdDone6[]  PROGMEM = "Tilt done.";
const char cmdDone7[]  PROGMEM = "Yawn done.";
const char cmdDone8[]  PROGMEM = "Look around done.";
const char cmdDone9[]  PROGMEM = "Look hold done.";
const char cmdDone10[] PROGMEM = "Eyes closed.";
const char cmdDone11[] PROGMEM = "Eyes open.";
const char cmdDone12[] PROGMEM = "Clip5 done.";
const char cmdDone13[] PROGMEM = "Chomp done.";
const char cmdDone14[] PROGMEM = "Big tilt done.";
const char cmdDone15[] PROGMEM = "Look up done.";
const char cmdDone16[] PROGMEM = "Sleepy done.";
const char cmdDone17[] PROGMEM = "Shake done.";
const char cmdDone18[] PROGMEM = "Shake chomp done.";

struct SerialCommand {
  const char* name;     // PROGMEM pointer
  const char* startMsg; // PROGMEM pointer
  const char* doneMsg;  // PROGMEM pointer
  AnimationFunc func;
};

const SerialCommand serialCommands[] = {
  { cmdName0,  cmdStart0,  cmdDone0,  blinkEyelids },
  { cmdName1,  cmdStart1,  cmdDone1,  rorAnimation },
  { cmdName2,  cmdStart2,  cmdDone2,  ror2Animation },
  { cmdName3,  cmdStart3,  cmdDone3,  lookRightAnimation },
  { cmdName4,  cmdStart4,  cmdDone4,  lookLeftAnimation },
  { cmdName5,  cmdStart5,  cmdDone5,  frontAnimation },
  { cmdName6,  cmdStart6,  cmdDone6,  curiousTiltAnimation },
  { cmdName7,  cmdStart7,  cmdDone7,  yawnAnimation },
  { cmdName8,  cmdStart8,  cmdDone8,  lookAroundAnimation },
  { cmdName9,  cmdStart9,  cmdDone9,  lookAndHoldAnimation },
  { cmdName10, cmdStart10, cmdDone10, eyesClosedAnimation },
  { cmdName11, cmdStart11, cmdDone11, eyesOpenAnimation },
  { cmdName12, cmdStart12, cmdDone12, clip5Animation },
  { cmdName13, cmdStart13, cmdDone13, quickChompsAnimation },
  { cmdName14, cmdStart14, cmdDone14, bigTiltAnimation },
  { cmdName15, cmdStart15, cmdDone15, neckStretchAnimation },
  { cmdName16, cmdStart16, cmdDone16, sleepyBlinkAnimation },
  { cmdName17, cmdStart17, cmdDone17, shakeAnimation },
  { cmdName18, cmdStart18, cmdDone18, shakeChompAnimation },
};
const uint8_t NUM_SERIAL_COMMANDS = sizeof(serialCommands) / sizeof(serialCommands[0]);

// ============================================================
// Test 1
// Runs every named animation currently in the system, one after
// another, with about a 3 second pause between each. Handy for
// checking that everything still works after changing pins or
// safe ranges. Type "test1" into the Serial Monitor.
// ============================================================
void test1Animation() {
  char nameBuf[16];
  for (uint8_t i = 0; i < NUM_SERIAL_COMMANDS; i++) {
    strcpy_P(nameBuf, serialCommands[i].name);
    Serial.print(F("[test1] "));
    Serial.println(nameBuf);
    serialCommands[i].func();
    delay(3000);
  }
  Serial.println(F("[test1] complete"));
}

// Strips leading/trailing whitespace in place and returns a pointer
// to the first non-whitespace character. Used instead of Arduino's
// String::trim() so handleSerialCommands() doesn't need the String
// class at all -- String pulls in a dynamic memory allocator
// (malloc/realloc/free) that costs roughly 1KB of flash on its own,
// which a fixed-size buffer and plain C string functions avoid
// entirely for something this simple.
char* trimWhitespace(char* s) {
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
  if (*s == '\0') return s;
  char* end = s + strlen(s) - 1;
  while (end > s && (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')) end--;
  end[1] = '\0';
  return s;
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

  char buf[32];
  size_t len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[len] = '\0';
  char* line = trimWhitespace(buf);

  if (line[0] == '\0') {
    return;
  }

  {
    char nameBuf[16];
    for (uint8_t i = 0; i < NUM_SERIAL_COMMANDS; i++) {
      strcpy_P(nameBuf, serialCommands[i].name);
      if (strcasecmp(line, nameBuf) == 0) {
        char msgBuf[26];
        strcpy_P(msgBuf, serialCommands[i].startMsg);
        Serial.println(msgBuf);
        serialCommands[i].func();
        strcpy_P(msgBuf, serialCommands[i].doneMsg);
        Serial.println(msgBuf);
        return;
      }
    }
  }

  if (strncasecmp(line, "play ", 5) == 0) {
    if (!dfPlayerReady) {
      Serial.println(F("DFPlayer not ready -- check wiring and SD card."));
      return;
    }
    int trackNum = atoi(line + 5);
    if (trackNum <= 0) {
      Serial.println(F("Format: play <trackNumber>   e.g. play 5   (plays mp3/0005.mp3)"));
      return;
    }
    Serial.print(F("Playing track "));
    Serial.println(trackNum);
    dfPlayer.playMp3Folder(trackNum);
    return;
  }

  if (strcasecmp(line, "scan") == 0) {
    Serial.println(F("Scanning channels 0-15..."));
    scanChannels();
    return;
  }

  if (strcasecmp(line, "idle") == 0) {
    idleAnimation();
    return;
  }

  if (strcasecmp(line, "test1") == 0) {
    Serial.println(F("Running test1 (all animations)..."));
    test1Animation();
    return;
  }

  char* space = strchr(line, ' ');
  if (space == NULL) {
    Serial.println(F("Format: <servoName> <angle>   e.g. jaw 60"));
    return;
  }

  *space = '\0';
  char* name = line;
  char* angleStr = trimWhitespace(space + 1);

  if (angleStr[0] == '\0') {
    Serial.println(F("Format: <servoName> <angle>   e.g. jaw 60"));
    return;
  }

  int angle = atoi(angleStr);

  int idx = servoIndex(name);
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

  moveServoSmooth(name, angle);

  Serial.print(name);
  Serial.print(F(" -> "));
  Serial.print(constrain(angle, servoConfigs[idx].minAngle, servoConfigs[idx].maxAngle));
  Serial.println(F(" degrees"));
}
