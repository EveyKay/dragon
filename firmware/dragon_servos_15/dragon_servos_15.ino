#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <DFRobotDFPlayerMini.h>
#include <string.h>
#include <stdlib.h>

// ============================================================
// SERVO DRIVER (PCA9685, via I2C)
// Board's SCL -> ESP32 GPIO22, SDA -> ESP32 GPIO21, VCC -> ESP32 3.3V,
// GND -> ESP32 GND (the PCA9685's logic side is fine on 3.3V; servo
// power (V+ terminal, separate from the logic VCC pin) comes from the
// LM2596/battery rail, not the microcontroller, on either board).
//
// Servos used to be driven directly by the Servo library, but that
// relies on constant timer interrupts to hold position -- which
// intermittently collided with SoftwareSerial's interrupts for the
// DFPlayer, causing random servo glitches (and occasional dropouts
// on whichever servo had the least torque margin) right around
// sound-triggering commands. Routing servos through the PCA9685
// instead removes the microcontroller from pulse generation entirely,
// so there's no timer interrupt left for SoftwareSerial to collide
// with -- and on the ESP32, the DFPlayer now gets a real second
// hardware UART instead of SoftwareSerial at all (see below), so that
// whole class of interrupt collision can't happen in the first place.
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
// DFPlayer TX -> ESP32 GPIO32
// DFPlayer RX -> ESP32 GPIO33
// This is UART2, a real hardware serial port -- the ESP32 has three
// independent UARTs, so unlike the Uno (one hardware UART, already
// spoken for by the Serial Monitor connection) the DFPlayer gets its
// own dedicated hardware port instead of the software-bit-banged
// SoftwareSerial the Uno build had to use. The ESP32's UART pins
// aren't fixed to specific GPIOs (unlike the Uno) -- any two general
// purpose pins work here. GPIO32/33 were picked over the more
// "conventional" default RX2/TX2 pins (16/17) because 16/17 double as
// the PSRAM interface on some ESP32 module variants (WROVER-style),
// which would make them unusable as a UART no matter how correctly
// they're wired externally. 32/33 have no alternate function on any
// ESP32 variant.
// SD card layout: an "mp3" folder in the card's root containing
// 0001.mp3, 0002.mp3, etc. -- playMp3Folder(N) plays 000N.mp3.
// ============================================================
HardwareSerial dfSerial(2); // UART2
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerReady = false;

// ============================================================
// MODE BUTTONS
// Three momentary pushbuttons, each wired between its GPIO and GND.
// INPUT_PULLUP means the pin reads HIGH when the button is untouched
// and LOW the instant it's pressed, so no external resistor is needed.
// GPIO25/26/27 were picked because they're plain digital-capable pins
// with no boot-strapping role and no overlap with the I2C (21/22) or
// DFPlayer UART (32/33) pins used elsewhere.
// ============================================================
const uint8_t BUTTON_IDLE_PIN = 25;
const uint8_t BUTTON_TALK_PIN = 26;
const uint8_t BUTTON_HOME_PIN = 27;

// -1 = no mode change pending. Set the instant a button's press is
// confirmed (see checkModeButtons() below); consumed once, right
// before dispatching, by handleModeButtons() in loop().
enum ModeRequest { MODE_NONE = -1, MODE_IDLE = 0, MODE_TALK = 1, MODE_HOME = 2 };
int8_t pendingMode = MODE_NONE;

struct ModeButton {
  uint8_t pin;
  int8_t mode;         // which ModeRequest this button requests when pressed
  bool lastReading;     // raw digitalRead() from the previous poll, for edge detection
  bool debouncedState;  // the reading once it's held stable past BUTTON_DEBOUNCE_MS
  unsigned long lastChangeMillis;
};

ModeButton modeButtons[] = {
  { BUTTON_IDLE_PIN, MODE_IDLE, HIGH, HIGH, 0 },
  { BUTTON_TALK_PIN, MODE_TALK, HIGH, HIGH, 0 },
  { BUTTON_HOME_PIN, MODE_HOME, HIGH, HIGH, 0 },
};
const uint8_t NUM_MODE_BUTTONS = sizeof(modeButtons) / sizeof(modeButtons[0]);
const unsigned long BUTTON_DEBOUNCE_MS = 30;

// Polls all three buttons and debounces each one independently. The
// instant a button's reading settles LOW (pressed) after being stable
// for BUTTON_DEBOUNCE_MS, latches its mode into pendingMode and returns
// true. Cheap enough (three digitalReads and some comparisons) to call
// from inside any blocking loop -- idle/talk's ambient engine, a
// stress test, a held pose -- the exact same way those loops already
// poll Serial.available() to notice they should stop.
bool checkModeButtons() {
  bool pressed = false;
  for (uint8_t i = 0; i < NUM_MODE_BUTTONS; i++) {
    bool reading = digitalRead(modeButtons[i].pin);
    if (reading != modeButtons[i].lastReading) {
      modeButtons[i].lastChangeMillis = millis();
      modeButtons[i].lastReading = reading;
    }
    if (millis() - modeButtons[i].lastChangeMillis > BUTTON_DEBOUNCE_MS && reading != modeButtons[i].debouncedState) {
      modeButtons[i].debouncedState = reading;
      if (reading == LOW) { // pull-up idles HIGH, so LOW means freshly pressed
        pendingMode = modeButtons[i].mode;
        pressed = true;
      }
    }
  }
  return pressed;
}

// The one thing every blocking loop in this sketch (idle/talk mode,
// the stress test, a held pose) needs to know: has anything happened
// that means it should stop? Either a Serial command came in, or a
// mode button was just pressed -- both are treated as "stop whatever
// is running," with the actual mode switch (if it was a button) then
// carried out by handleModeButtons() once control returns to loop().
bool stopRequested() {
  bool buttonPressed = checkModeButtons(); // always poll, so debounce timing stays accurate even if Serial already has data
  return Serial.available() || buttonPressed;
}

// Called every loop() tick. Polls the buttons itself (in case nothing
// else happened to be running to poll them via stopRequested()), and
// once a press has latched a pendingMode, consumes it and dispatches
// to the matching mode -- idle/talk are the same blocking loops the
// "idle"/"talk" Serial commands trigger, so pressing a different
// button while one is already running interrupts it exactly like
// typing a new command would, via the very same stopRequested() checks
// inside them.
void handleModeButtons() {
  checkModeButtons();
  if (pendingMode == MODE_NONE) return;

  int8_t mode = pendingMode;
  pendingMode = MODE_NONE; // consume before dispatching, so a fresh press mid-mode can latch its own new request
  switch (mode) {
    case MODE_IDLE: idleAnimation(); break;
    case MODE_TALK: talkAnimation(); break;
    case MODE_HOME: homeAnimation(); break;
  }
}

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

// Shared default for easeInOutExpo()'s steepness -- one named constant
// instead of the same literal hardcoded separately in both
// easeInOutExpo() and moveServosTogether(), which could silently drift
// apart if only one ever got tuned. Lowered from 20 to 14: a gentler
// curve spikes less hard through the middle of a move, which both
// looks less abrupt and needs the speed cap to intervene less often in
// the first place (less peak velocity means less chance of exceeding
// MAX_DEGREES_PER_MS), so there's less lag for approachAngle() to ever
// need to correct.
const float DEFAULT_EASE_STEEPNESS = 14.0;

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

// Every step-loop in this sketch was computing its target angle purely
// from an idealized curve (start angle + eased progress), then relying
// on a single uncapped "final step" to force an exact landing no
// matter how far behind the real position had drifted if the speed
// cap throttled earlier steps. That's what turned into a visible snap
// at the end of a move -- all of the accumulated lag landing in one
// uncapped jump instead of easing out.
//
// This spreads that catch-up across the last catchUpSteps of a move
// instead: once stepsRemaining drops into that window, the target for
// this tick is computed from the servo's actual current angle (not
// the idealized curve) divided evenly across however many steps are
// actually left, so a lagging servo eases back onto target over
// several ticks instead of snapping on the last one. Same idea as the
// "recompute from wherever you actually are, not where you assumed
// you'd be" trick from the smoothing technique in James Bruton's
// "How To Make Robots Move Smoothly" video, applied specifically to
// the tail end of our existing eased curves rather than replacing them
// outright (which would lose the deliberate ease-in and the precise,
// sound-synced timing several animations depend on).
int approachAngle(const char* name, int idealAngle, int target, int stepsRemaining, int catchUpSteps) {
  if (stepsRemaining > catchUpSteps) return idealAngle; // not in the catch-up window yet -- use the curve as normal
  int idx = servoIndex(name);
  if (idx == -1) return idealAngle;
  int remainingGap = target - lastAngle[idx];
  return lastAngle[idx] + remainingGap / (stepsRemaining + 1);
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
// duration can't fully compensate for a steeper curve. DEFAULT_EASE_STEEPNESS
// is used everywhere movement is eased through this curve.
//
// This used to go as high as 60, then 20 -- both crammed more of a
// move's travel into a narrow sliver of time near the middle than the
// current, gentler default does. MAX_DEGREES_PER_MS
// below is what actually keeps that (or any curve) from commanding a
// servo faster than it can physically track, so this default is free
// to just be picked for how the motion looks rather than doubling as
// the only thing standing between a steep curve and a strained motor.
float easeInOutExpo(float t, float steepness = DEFAULT_EASE_STEEPNESS) {
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
  // neck1's floor was pulled back up from 2000 to 6000 -- fast enough
  // to still read as ambient drift, but a 2-4 second full up/down cycle
  // was quick enough to look like the dragon repeatedly nodding rather
  // than idly swaying. neck2/neck3 (side-to-side) weren't reported as
  // an issue, so left alone.
  neck1SwayPeriodMs = random(6000, 20000);
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
    if (stopRequested()) return;
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
  const int catchUpSteps = min(12, steps / 3); // spread any speed-cap lag over the last several steps instead of snapping it all onto the final one

  for (int i = 0; i <= steps; i++) {
    float t = easeInOutExpo((float)i / steps);
    int idealAngle = startAngle + t * (clampedTarget - startAngle);
    int angle = approachAngle(name, idealAngle, clampedTarget, steps - i, catchUpSteps);
    // The final step is still guaranteed-exact (skips the cap entirely)
    // as an ultimate safety net, but approachAngle() above should have
    // already closed out nearly all of any lag gently over the last
    // several steps, so this no longer needs to be a big jump.
    moveServo(name, angle, i != steps);
    delay(stepDelayMs);
  }
}

void setup() {
  Serial.begin(9600);

  randomSeed(analogRead(34)); // GPIO34 is unconnected -- floating-pin noise seeds curiousTiltAnimation()'s side pick

  for (uint8_t i = 0; i < NUM_MODE_BUTTONS; i++) {
    pinMode(modeButtons[i].pin, INPUT_PULLUP);
    // Seed both debounce fields from a real initial read instead of the
    // hardcoded HIGH default -- if a button happened to be held down at
    // boot, this treats that as its resting state rather than firing a
    // spurious "just pressed" edge the first time checkModeButtons() runs.
    bool initial = digitalRead(modeButtons[i].pin);
    modeButtons[i].lastReading = initial;
    modeButtons[i].debouncedState = initial;
    modeButtons[i].lastChangeMillis = millis();
  }

  Wire.begin(21, 22); // SDA, SCL
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

  dfSerial.begin(9600, SERIAL_8N1, 32, 33); // RX, TX
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
  handleModeButtons();
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
  const int catchUpSteps = min(12, steps / 3); // spread any speed-cap lag over the last several steps instead of snapping it all onto the final one

  // Closing: open -> closed
  for (int i = 0; i <= steps; i++) {
    float t = easeInOutExpo((float)i / steps); // 0.0 -> 1.0
    int rightIdeal = rightOpen + t * (rightClosed - rightOpen);
    int leftIdeal = leftOpen + t * (leftClosed - leftOpen);
    int rightAngle = approachAngle("eyelidRight", rightIdeal, rightClosed, steps - i, catchUpSteps);
    int leftAngle = approachAngle("eyelidLeft", leftIdeal, leftClosed, steps - i, catchUpSteps);
    moveServo("eyelidRight", rightAngle, i != steps); // last step bypasses the speed cap so a closing blink always actually reaches fully closed
    moveServo("eyelidLeft", leftAngle, i != steps);
    applyIdleSway(true, true, true); // blink never touches the neck or eyeballs, so all stay free
    applyIdleGaze(true, true);
    delay(stepDelayMs);
  }

  // Opening: closed -> open
  for (int i = 0; i <= steps; i++) {
    float t = easeInOutExpo((float)i / steps);
    int rightIdeal = rightClosed + t * (rightOpen - rightClosed);
    int leftIdeal = leftClosed + t * (leftOpen - leftClosed);
    int rightAngle = approachAngle("eyelidRight", rightIdeal, rightOpen, steps - i, catchUpSteps);
    int leftAngle = approachAngle("eyelidLeft", leftIdeal, leftOpen, steps - i, catchUpSteps);
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
// Paired with mp3/0025.mp3 (sounds/ror_burst.mp3) -- just the final
// bark from clip2_10.mp3 (of the larger dragon_sound_clips2 batch),
// extracted with ffmpeg, boosted ~10.5dB (mean -29dB -> -18.5dB, peak
// -13.1dB -> -3dB, leaving a couple dB of headroom before clipping),
// then slowed to 60% speed (asetrate+aresample, which drops the pitch
// along with the tempo) to stretch it from ~0.4s to ~0.6s and give it
// a deeper, more dragon-sized growl instead of a small-dog bark. Still
// short relative to this animation's ~2.2s runtime, so unlike the old
// mp3/0011.mp3 pairing it only fills the first second or so, not the
// whole thing.
// ============================================================
void rorAnimation() {
  if (dfPlayerReady) {
    dfPlayer.playMp3Folder(25);
  }

  const int neckStart = 90, neckEnd = 20;
  const int jawStart = 90, jawEnd = 20;
  const int rightOpen = 58, rightClosed = 40;
  const int leftOpen = 96, leftClosed = 114;

  const int steps = 70;       // resolution of the animation (higher = smoother)
  const int stepDelayMs = 14; // a bit quicker than before
  const int catchUpSteps = min(12, steps / 3); // spread any speed-cap lag over the last several steps instead of snapping it all onto the final one

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps; // 0.0 -> 1.0 across the whole animation
    bool isFinalStep = (i == steps); // bypasses the speed cap so this loop always actually lands exactly on its targets, not just close
    int stepsRemaining = steps - i;

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

    moveServo("neck1", approachAngle("neck1", neckAngle, neckEnd, stepsRemaining, catchUpSteps), !isFinalStep);
    moveServo("neck2", approachAngle("neck2", neck2Angle, 90, stepsRemaining, catchUpSteps), !isFinalStep);
    moveServo("neck3", approachAngle("neck3", neck3Angle, 90, stepsRemaining, catchUpSteps), !isFinalStep);
    moveServo("jaw", approachAngle("jaw", jawAngle, jawEnd, stepsRemaining, catchUpSteps), !isFinalStep);
    moveServo("eyelidRight", approachAngle("eyelidRight", rightAngle, rightOpen, stepsRemaining, catchUpSteps), !isFinalStep);
    moveServo("eyelidLeft", approachAngle("eyelidLeft", leftAngle, leftOpen, stepsRemaining, catchUpSteps), !isFinalStep);

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
  const int jawCatchUpSteps = min(12, wobbleSteps / 3); // jaw settles at the end of every sub-loop, so it catches up within each one
  const int neck1CatchUpSteps = 12; // neck1 only settles at the very end of the whole wobble phase, so it catches up against the remaining sub-steps across all reps

  for (int rep = 0; rep < 3; rep++) {
    // low -> high
    for (int i = 0; i <= wobbleSteps; i++) {
      float t = easeInOutExpo((float)i / wobbleSteps);
      int jawIdeal = wobbleLow + t * (wobbleHigh - wobbleLow);
      int jawAngle = approachAngle("jaw", jawIdeal, wobbleHigh, wobbleSteps - i, jawCatchUpSteps);
      moveServo("jaw", jawAngle, i != wobbleSteps);

      float neckT = easeInOutExpo((float)wobbleSubStep / (totalWobbleSubSteps - 1));
      int neck1Ideal = neckEnd + neckT * (90 - neckEnd);
      int neck1Angle = approachAngle("neck1", neck1Ideal, 90, totalWobbleSubSteps - 1 - wobbleSubStep, neck1CatchUpSteps);
      moveServo("neck1", neck1Angle, wobbleSubStep != totalWobbleSubSteps - 1);
      wobbleSubStep++;

      delay(wobbleStepDelayMs);
    }
    // high -> low
    for (int i = 0; i <= wobbleSteps; i++) {
      float t = easeInOutExpo((float)i / wobbleSteps);
      int jawIdeal = wobbleHigh + t * (wobbleLow - wobbleHigh);
      int jawAngle = approachAngle("jaw", jawIdeal, wobbleLow, wobbleSteps - i, jawCatchUpSteps);
      moveServo("jaw", jawAngle, i != wobbleSteps);

      float neckT = easeInOutExpo((float)wobbleSubStep / (totalWobbleSubSteps - 1));
      int neck1Ideal = neckEnd + neckT * (90 - neckEnd);
      int neck1Angle = approachAngle("neck1", neck1Ideal, 90, totalWobbleSubSteps - 1 - wobbleSubStep, neck1CatchUpSteps);
      moveServo("neck1", neck1Angle, wobbleSubStep != totalWobbleSubSteps - 1);
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
  const int jawReturnCatchUpSteps = min(12, returnSteps / 3);

  for (int i = 0; i <= returnSteps; i++) {
    float t = easeInOutExpo((float)i / returnSteps);
    int jawIdeal = jawFrom + t * (homeAngle - jawFrom);
    // This loop's 4ms-per-step pace is fast enough that the speed cap
    // engages hard through the curve's steep middle -- exactly what was
    // leaving the jaw open after "ror". approachAngle() below eases it
    // back onto target gently over the last several steps instead of
    // leaving it all to the guaranteed-exact final step (same fix as
    // moveServosTogether()), so it always actually reaches fully
    // closed without a visible snap doing it.
    int jawAngle = approachAngle("jaw", jawIdeal, homeAngle, returnSteps - i, jawReturnCatchUpSteps);
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

void moveServosTogether(const char* names[], const int targets[], int count, int steps, int stepDelayMs, float easeSteepness = DEFAULT_EASE_STEEPNESS) {
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

  const int catchUpSteps = min(12, steps / 3); // spread any speed-cap lag over the last several steps instead of snapping it all onto the final one

  for (int i = 0; i <= steps; i++) {
    float t = easeInOutExpo((float)i / steps, easeSteepness);
    bool isFinalStep = (i == steps);
    for (int c = 0; c < count; c++) {
      int idealAngle = startAngles[c] + t * (targets[c] - startAngles[c]);
      // approachAngle() catches a servo back up gently over the last
      // few steps if the cap throttled it earlier in the curve, rather
      // than leaving it all to the final step below. That final step
      // still bypasses the cap as an exact-landing guarantee (this is
      // what originally fixed the jaw staying open after a yawn), but
      // now it's just closing out whatever tiny residual is left,
      // instead of the whole accumulated gap in one uncapped jump.
      int angle = approachAngle(names[c], idealAngle, targets[c], steps - i, catchUpSteps);
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

  idleHold(random(600, 900), true, false, false, false, false); // hold the curious pose -- neck1 is the only axis this animation doesn't use, so it's the only one free to keep swaying

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

  idleHold(random(500, 800), false, true, true, true, true); // hold at the peak -- a yawn never touches neck2/neck3 or the eyes, so all of those stay free

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
  idleHold(random(300, 600), true, false, false, false, false); // brief pause, like taking in what's there -- neck1 is the only free axis here

  const int leftTargets[] = { 40, 40, 55, 55 };
  moveServosTogether(names, leftTargets, 4, 130, 12); // slower sweep across to the other side
  idleHold(random(300, 600), true, false, false, false, false);

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
  const int closeTarget[] = { 90 };

  int repCount = random(2, 4); // 2 or 3 -- a real chomp doesn't always come in the same exact count
  for (int rep = 0; rep < repCount; rep++) {
    int openTarget[] = { (int)random(70, 79) }; // a small, shallow open -- not a full yawn, and never quite the same depth twice
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

  idleHold(random(1200, 1700), true, false, false, false, false); // a longer, more deliberate hold than the curious tilt -- neck1 is the only free axis here

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

  idleHold(random(800, 1300), false, true, true, true, true); // hold at full stretch -- doesn't touch neck2/neck3 or the eyes, so all stay free

  const int homeTarget[] = { 90 };
  moveServosTogether(names, homeTarget, 1, 70, 12);
}

// ============================================================
// Look down
// Neck1 dips down and holds, like sniffing something at ground level
// or examining the floor -- the mirror image of "look up". No sound.
// Type "look down" into the Serial Monitor to trigger it.
// ============================================================
void lookDownAnimation() {
  const char* names[] = { "neck1" };
  const int downTarget[] = { 135 }; // higher angle = head down on this axis
  moveServosTogether(names, downTarget, 1, 70, 14); // slow, deliberate dip downward

  idleHold(random(800, 1300), false, true, true, true, true); // hold at full dip -- doesn't touch neck2/neck3 or the eyes, so all stay free

  const int homeTarget[] = { 90 };
  moveServosTogether(names, homeTarget, 1, 70, 12);
}

// ============================================================
// Sniff
// A few quick, shallow head dips, like sniffing the air rapidly --
// smaller and faster than "look down", and repeated rather than held.
// No sound. Type "sniff" into the Serial Monitor to trigger it.
// ============================================================
void sniffAnimation() {
  const char* names[] = { "neck1" };
  const int homeTarget[] = { 90 };

  int repCount = random(3, 5); // 3 or 4 -- never quite the same count twice
  for (int rep = 0; rep < repCount; rep++) {
    int dipTarget[] = { (int)random(98, 106) }; // a small, shallow dip -- not a full look-down, and never quite the same depth twice
    moveServosTogether(names, dipTarget, 1, 12, 8);  // quick dip down
    moveServosTogether(names, homeTarget, 1, 12, 8); // quick return
  }
}

// ============================================================
// Neck roll
// Neck2 and neck3 sway side to side like the ambient idle sway, but
// with a phase offset between them so the motion reads as a slow,
// rolling stretch through the neck rather than both segments leaning
// together in sync (which is what the ambient sway and curious tilt
// already look like). Settles back to exact center afterward. No
// sound. Type "neck roll" into the Serial Monitor to trigger it.
// ============================================================
void neckRollAnimation() {
  const float amplitude2 = 25, amplitude3 = 15;
  const float periodMs = 2200;
  const int cycles = 2;
  const unsigned long durationMs = (unsigned long)(cycles * periodMs);
  const int stepMs = 20;

  unsigned long startMillis = millis();
  while (millis() - startMillis < durationMs) {
    unsigned long elapsed = millis() - startMillis;
    int neck2Angle = 90 + (int)easedOscillate(elapsed, periodMs, amplitude2);
    int neck3Angle = 90 + (int)easedOscillate(elapsed + periodMs / 4, periodMs, amplitude3); // quarter-period offset so neck3 trails neck2 instead of mirroring it
    moveServo("neck2", neck2Angle);
    moveServo("neck3", neck3Angle);
    applyIdleSway(true, false, false); // neck1 stays free -- neck2/neck3 are driven directly above
    applyIdleGaze(true, true);
    delay(stepMs);
  }

  const char* names[] = { "neck2", "neck3" };
  const int homeTargets[] = { 90, 90 };
  moveServosTogether(names, homeTargets, 2, 30, 10); // land both exactly on center
}

// ============================================================
// Double blink
// Two quick blinks back to back, like a surprised double-take --
// distinct from both a single "blink" and the regular blink already
// scheduled every few seconds in idle mode. No sound. Type "double
// blink" into the Serial Monitor to trigger it.
// ============================================================
void doubleBlinkAnimation() {
  blinkEyelids();
  delay(120); // brief beat between the two, like a quick double-take
  blinkEyelids();
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

  idleHold(random(1300, 1900), false, true, true, true, true); // hold the droop -- doesn't touch neck2/neck3 or the eyes, so all stay free

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
    if (stopRequested()) break;
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
// Paired with mp3/0017.mp3 (sounds/clip_17.mp3) -- unlike the original
// pairing (a single bark artificially duplicated), this one has two
// real, distinct barks on its own, about 510ms apart (measured via
// ffmpeg silencedetect), landing close to this animation's two
// mouth-fully-open instants (t=0.25 and t=0.75 of its 1.4s runtime,
// i.e. 350ms and 1050ms) -- the second lines up almost exactly (within
// 20ms), the first is about 170ms early. No re-editing of the clip was
// needed. mp3/0024.mp3 (the old artificially-duplicated bark) is still
// on the SD card but no longer used by anything.
// ============================================================
void ror2Animation() {
  if (dfPlayerReady) {
    dfPlayer.playMp3Folder(17);
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
  const int catchUpSteps = min(12, steps / 3); // spread any speed-cap lag over the last several steps instead of snapping it all onto the final one

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    bool isFinalStep = (i == steps); // bypasses the speed cap so this loop always actually lands on 90, the same fix rorAnimation() needed
    int stepsRemaining = steps - i;

    float wave = center + amplitude * cos(2 * PI * cycles * t);
    int neckIdeal = (int)(wave + 0.5);
    int jawIdeal = (int)(wave + 0.5);
    int neckAngle = approachAngle("neck1", neckIdeal, 90, stepsRemaining, catchUpSteps);
    int jawAngle = approachAngle("jaw", jawIdeal, 90, stepsRemaining, catchUpSteps);

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
// Clip 1 bark
// Same envelope-driven approach as clip5Animation() -- jaw and neck1
// track the volume envelope of clip_01.mp3, sampled in 40ms slices
// with ffmpeg (RMS per slice, converted to dB, smoothed with a 3-frame
// moving average, normalized against this recording's own min/max)
// and mapped to angles the same way clip5Jaw[]/clip5Neck1[] are.
// Eyelids do one quick blink at the recording's loudest instant
// (~5.16s in). The main difference from clip5: neck2/neck3 sway with
// much bigger amplitude and a faster cycle (2.5 cycles across the
// clip instead of 1.5), reading as a deliberate side-to-side turn
// rather than clip5's subtle background wobble.
// Type "clip1" into the Serial Monitor to trigger it.
//
// This only drives the servos -- it doesn't play the audio itself
// unless a sound module is wired up, in which case starting
// clip_01.mp3 at the same time keeps the two in sync, since both run
// off the same 40ms-per-frame timeline.
// ============================================================
const uint8_t CLIP1_FRAME_MS = 40;
const uint8_t CLIP1_NUM_FRAMES = 157;

// Jaw angle per frame: louder in the recording -> lower angle ->
// mouth more open.
static const uint8_t clip1Jaw[CLIP1_NUM_FRAMES] PROGMEM = {
  86, 87, 78, 69, 61, 61, 69, 77, 85, 86, 86, 87, 81, 79, 78, 79, 79, 79, 67, 47,
  31, 31, 36, 45, 56, 68, 74, 76, 77, 69, 55, 41, 45, 58, 72, 71, 64, 61, 65, 71,
  73, 76, 79, 82, 83, 84, 83, 67, 51, 39, 36, 34, 29, 30, 32, 34, 37, 39, 45, 54,
  62, 65, 68, 73, 77, 69, 62, 63, 73, 76, 75, 68, 56, 43, 41, 49, 61, 62, 55, 44,
  39, 38, 46, 54, 67, 66, 74, 77, 88, 83, 77, 77, 82, 88, 80, 68, 51, 44, 39, 41,
  45, 49, 56, 64, 74, 70, 66, 62, 66, 69, 69, 73, 71, 70, 71, 79, 78, 70, 57, 46,
  39, 41, 44, 43, 36, 30, 28, 23, 20, 20, 21, 23, 23, 27, 33, 45, 57, 66, 70, 76,
  78, 78, 79, 82, 85, 81, 77, 77, 82, 72, 64, 60, 72, 82, 85, 87, 86,
};

// Neck1 angle per frame: same shape as the jaw but smoothed with
// extra lag, so the head follows the mouth instead of moving in
// lockstep with it.
static const uint8_t clip1Neck1[CLIP1_NUM_FRAMES] PROGMEM = {
  85, 85, 84, 81, 78, 75, 75, 76, 78, 80, 81, 82, 82, 81, 81, 81, 80, 80, 78, 73,
  66, 62, 59, 59, 60, 63, 67, 69, 72, 72, 70, 66, 64, 64, 67, 69, 69, 69, 69, 71,
  72, 74, 75, 77, 78, 80, 81, 78, 74, 69, 64, 61, 57, 54, 53, 52, 52, 52, 53, 56,
  59, 62, 64, 67, 70, 71, 70, 70, 72, 73, 74, 74, 71, 67, 64, 63, 64, 65, 65, 63,
  61, 59, 58, 60, 63, 65, 68, 70, 74, 76, 77, 77, 78, 80, 80, 78, 74, 69, 65, 62,
  61, 61, 62, 64, 67, 69, 69, 69, 70, 70, 71, 72, 73, 73, 74, 75, 76, 76, 73, 69,
  65, 62, 61, 60, 58, 55, 52, 50, 47, 46, 44, 44, 43, 44, 45, 48, 52, 57, 61, 65,
  69, 71, 74, 76, 78, 79, 79, 79, 80, 78, 76, 74, 74, 76, 78, 80, 81,
};

void clip1Animation() {
  if (dfPlayerReady) {
    dfPlayer.playMp3Folder(1); // mp3/0001.mp3 -- the recording this animation is synced to
  }

  const int rightOpen = 58, rightClosed = 40;
  const int leftOpen = 96, leftClosed = 114;

  // Blink window: centered on the loudest frame in the recording
  // (frame 129 of 0-156, ~5.16s in) -- same 8-frames-each-way pace as
  // clip5Animation().
  const int blinkStartFrame = 122;
  const int blinkCloseFrame = 130;
  const int blinkEndFrame = 138;

  for (uint8_t i = 0; i < CLIP1_NUM_FRAMES; i++) {
    int jawAngle = pgm_read_byte(&clip1Jaw[i]);
    int neckAngle = pgm_read_byte(&clip1Neck1[i]);

    // neck2/neck3 turn back and forth throughout, independent of
    // loudness, much more prominently than clip5Animation()'s subtle
    // background sway (bigger amplitude, more cycles) -- a deliberate
    // side-to-side turn rather than idle-style background life. Fades
    // out over the last 20 frames so it settles to 90 on its own
    // before the settle-to-home phase below takes over.
    float t = (float)i / (CLIP1_NUM_FRAMES - 1);
    const uint8_t swayFadeStartFrame = CLIP1_NUM_FRAMES - 20;
    float swayFade = 1.0;
    if (i >= swayFadeStartFrame) {
      swayFade = 1.0 - (float)(i - swayFadeStartFrame) / (CLIP1_NUM_FRAMES - 1 - swayFadeStartFrame);
    }
    int neck2Angle = 90 + swayFade * 24 * sin(2 * PI * 2.5 * t);
    int neck3Angle = 90 + swayFade * 15 * sin(2 * PI * 2.5 * t);

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

    delay(CLIP1_FRAME_MS);
  }

  // Settle everything back to home once the envelope runs out, same
  // reasoning as clip5Animation()'s settle phase.
  const char* names1[] = { "jaw", "neck1", "neck2", "neck3", "eyelidRight", "eyelidLeft" };
  const int targets1[] = { 90, 90, 90, 90, 58, 96 };
  moveServosTogether(names1, targets1, 6, 60, 12);
}

// ============================================================
// Clip2_02 bark
// Same envelope-driven approach as clip1Animation()/clip5Animation(),
// generated from clip2_02.mp3 (of the dragon_sound_clips2 batch) --
// a much longer recording (10.2s, 254 frames) with a lot more dynamic
// range in it. neck2/neck3 lean side to side even more than clip1's
// (bigger amplitude, and a slower cycle scaled to this clip's length
// so it reads as a deliberate tilt rather than a fast wobble) since
// this one was specifically asked to have even more head tilting.
// Type "clip2_02" into the Serial Monitor to trigger it.
//
// This only drives the servos -- it doesn't play the audio itself
// unless a sound module is wired up, in which case starting
// clip2_02.mp3 at the same time keeps the two in sync, since both run
// off the same 40ms-per-frame timeline.
// ============================================================
const uint8_t CLIP2_02_FRAME_MS = 40;
const uint16_t CLIP2_02_NUM_FRAMES = 254;

// Jaw angle per frame: louder in the recording -> lower angle ->
// mouth more open.
static const uint8_t clip2_02Jaw[CLIP2_02_NUM_FRAMES] PROGMEM = {
  72, 68, 63, 61, 67, 76, 79, 77, 77, 78, 79, 79, 78, 72, 69, 69, 77, 82, 79, 72,
  60, 61, 46, 38, 22, 24, 24, 23, 23, 24, 25, 34, 50, 57, 56, 57, 67, 73, 74, 76,
  80, 81, 81, 81, 82, 82, 82, 82, 80, 80, 71, 52, 32, 21, 20, 23, 30, 31, 28, 22,
  24, 26, 28, 34, 37, 45, 53, 67, 59, 42, 24, 30, 43, 61, 72, 80, 82, 82, 84, 84,
  85, 86, 84, 81, 78, 76, 76, 75, 62, 46, 29, 27, 34, 33, 31, 23, 23, 23, 24, 25,
  25, 30, 44, 58, 62, 48, 35, 37, 54, 71, 80, 84, 85, 87, 86, 87, 88, 87, 85, 84,
  85, 85, 86, 87, 87, 87, 86, 68, 46, 28, 25, 26, 28, 31, 37, 39, 45, 54, 67, 77,
  80, 83, 84, 86, 85, 84, 82, 82, 81, 80, 72, 72, 70, 76, 70, 66, 56, 47, 39, 38,
  41, 35, 29, 23, 23, 24, 24, 24, 24, 29, 34, 39, 50, 61, 75, 80, 84, 85, 84, 82,
  74, 64, 56, 52, 54, 64, 77, 86, 85, 84, 85, 87, 88, 86, 83, 81, 81, 83, 86, 86,
  87, 86, 86, 86, 88, 71, 50, 26, 22, 23, 27, 30, 30, 29, 36, 49, 61, 68, 72, 78,
  82, 85, 85, 84, 84, 85, 86, 86, 83, 79, 75, 61, 62, 64, 77, 76, 66, 60, 46, 35,
  24, 21, 23, 25, 26, 27, 26, 31, 44, 62, 71, 67, 60, 57,
};

// Neck1 angle per frame: same shape as the jaw but smoothed with
// extra lag, so the head follows the mouth instead of moving in
// lockstep with it.
static const uint8_t clip2_02Neck1[CLIP2_02_NUM_FRAMES] PROGMEM = {
  75, 74, 73, 72, 72, 73, 75, 76, 77, 77, 78, 78, 79, 78, 77, 76, 76, 78, 78, 77,
  75, 73, 69, 65, 59, 55, 52, 50, 48, 46, 45, 47, 50, 54, 57, 59, 62, 65, 68, 71,
  73, 75, 77, 78, 79, 80, 80, 81, 81, 81, 79, 75, 68, 61, 56, 52, 51, 50, 49, 47,
  46, 46, 46, 47, 48, 50, 53, 58, 60, 59, 55, 53, 53, 57, 62, 66, 70, 73, 76, 77,
  79, 80, 81, 81, 81, 80, 79, 79, 76, 71, 65, 60, 57, 55, 53, 50, 48, 47, 46, 45,
  45, 45, 48, 53, 57, 57, 55, 54, 56, 61, 66, 70, 74, 77, 79, 80, 82, 83, 83, 83,
  83, 83, 84, 84, 84, 85, 85, 81, 76, 68, 62, 57, 54, 52, 52, 52, 54, 56, 60, 65,
  69, 72, 75, 77, 79, 80, 80, 81, 81, 81, 79, 78, 77, 77, 77, 75, 72, 69, 65, 62,
  60, 57, 54, 51, 49, 48, 46, 45, 45, 45, 46, 48, 51, 55, 61, 66, 70, 74, 76, 77,
  77, 75, 72, 70, 68, 68, 71, 75, 77, 78, 80, 81, 83, 83, 83, 83, 82, 82, 83, 83,
  84, 84, 84, 84, 85, 82, 77, 69, 62, 57, 54, 52, 51, 49, 50, 52, 56, 60, 64, 68,
  71, 74, 77, 78, 80, 81, 82, 82, 82, 82, 81, 77, 75, 74, 75, 76, 75, 73, 69, 64,
  59, 54, 51, 49, 48, 47, 46, 47, 49, 54, 59, 62, 63, 64,
};

void clip2_02Animation() {
  if (dfPlayerReady) {
    dfPlayer.playMp3Folder(26); // mp3/0026.mp3 -- the recording this animation is synced to
  }

  const int rightOpen = 58, rightClosed = 40;
  const int leftOpen = 96, leftClosed = 114;

  // Blink window: centered on the loudest frame in the recording
  // (frame 54 of 0-253, ~2.16s in) -- same 8-frames-each-way pace as
  // clip5Animation()/clip1Animation().
  const int blinkStartFrame = 47;
  const int blinkCloseFrame = 55;
  const int blinkEndFrame = 63;

  for (uint16_t i = 0; i < CLIP2_02_NUM_FRAMES; i++) {
    int jawAngle = pgm_read_byte(&clip2_02Jaw[i]);
    int neckAngle = pgm_read_byte(&clip2_02Neck1[i]);

    // neck2/neck3 lean side to side throughout, independent of
    // loudness -- bigger amplitude and a slower cycle (scaled to this
    // clip's longer length) than clip1Animation(), reading as a
    // deliberate tilt rather than a quick wobble. Fades out over the
    // last 20 frames so it settles to 90 on its own before the
    // settle-to-home phase below takes over.
    float t = (float)i / (CLIP2_02_NUM_FRAMES - 1);
    const uint16_t swayFadeStartFrame = CLIP2_02_NUM_FRAMES - 20;
    float swayFade = 1.0;
    if (i >= swayFadeStartFrame) {
      swayFade = 1.0 - (float)(i - swayFadeStartFrame) / (CLIP2_02_NUM_FRAMES - 1 - swayFadeStartFrame);
    }
    int neck2Angle = 90 + swayFade * 28 * sin(2 * PI * 3 * t);
    int neck3Angle = 90 + swayFade * 18 * sin(2 * PI * 3 * t);

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

    delay(CLIP2_02_FRAME_MS);
  }

  // Settle everything back to home once the envelope runs out, same
  // reasoning as clip5Animation()'s settle phase.
  const char* names2[] = { "jaw", "neck1", "neck2", "neck3", "eyelidRight", "eyelidLeft" };
  const int targets2[] = { 90, 90, 90, 90, 58, 96 };
  moveServosTogether(names2, targets2, 6, 60, 12);
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
// Twitch stress test (diagnostic)
// All 8 servos twitch +/-12 degrees around their own home angle, in
// sync, back and forth, for as long as this runs -- deliberately the
// worst case for the shared servo power rail, since every servo
// accelerates from a stop at the same instant on every direction
// change instead of the more staggered current draw normal animations
// produce. Meant for reproducing/observing a brownout (e.g. watching
// jaw tension or metering V+ at the PCA9685) rather than for looking
// natural. Runs until Serial input arrives, same as idle mode. Type
// "stress test" into the Serial Monitor to trigger it.
// ============================================================
void twitchStressTest() {
  const char* names[] = { "eyeLeft", "eyeRight", "eyelidLeft", "eyelidRight", "jaw", "neck1", "neck2", "neck3" };
  const int twitchAmplitude = 12;

  int homeTargets[NUM_SERVOS];
  int highTargets[NUM_SERVOS];
  int lowTargets[NUM_SERVOS];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    int idx = servoIndex(names[i]);
    homeTargets[i] = servoConfigs[idx].homeAngle;
    highTargets[i] = servoConfigs[idx].homeAngle + twitchAmplitude;
    lowTargets[i] = servoConfigs[idx].homeAngle - twitchAmplitude;
  }

  Serial.println(F("Twitch stress test running -- type anything or press a mode button to stop."));
  while (!stopRequested()) {
    moveServosTogether(names, highTargets, NUM_SERVOS, 20, 10);
    if (stopRequested()) break;
    moveServosTogether(names, lowTargets, NUM_SERVOS, 20, 10);
  }

  moveServosTogether(names, homeTargets, NUM_SERVOS, 20, 10);
  Serial.println(F("Twitch stress test stopped."));
}

// ============================================================
// Home mode
// The opposite of idle/talk mode: eases every servo to its configured
// home angle and then just holds there -- no ambient sway, no
// blinking, no animations of any kind, nothing. A deliberate "at rest"
// state for when the dragon shouldn't be doing anything on its own.
// Unlike idle/talk mode this isn't a blocking loop -- it moves to home
// once and returns immediately, so there's nothing further to
// interrupt. Type "home" into the Serial Monitor, or press the home
// button, to trigger it.
// ============================================================
void homeAnimation() {
  Serial.println(F("Going home..."));

  const char* names[NUM_SERVOS];
  int targets[NUM_SERVOS];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    names[i] = servoConfigs[i].name;
    targets[i] = servoConfigs[i].homeAngle;
  }
  moveServosTogether(names, targets, NUM_SERVOS, 70, 12);

  Serial.println(F("Home."));
}

// ============================================================
// Idle mode (and its sibling, talk mode)
// Randomly runs "self-returning" animations -- ones that do their
// thing and settle back to home on their own -- with a 10-20 second
// gap between each, to make the dragon look alive when nothing else
// is happening. Deliberately excludes "stateful" animations like
// eyesClosedAnimation() or lookRightAnimation() that move somewhere
// and stay there, since a random pick landing on one of those and
// not revisiting it for a while would look broken/stuck rather than
// alive. Idle mode's pool never triggers sound.
//
// Talk mode (talkAnimation(), "talk" on the Serial Monitor) is the
// exact same ambient engine below, just fed talkAnimations[] instead
// of idleAnimations[] -- a pool that also includes the sound-synced
// animations (ror, ror two, clip5, clip1, clip2_02), so the dragon can
// spontaneously bark/roar on its own instead of only ever doing silent
// gestures. See runIdleLikeMode() below, which both modes call into.
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
  lookDownAnimation,
  sniffAnimation,
  neckRollAnimation,
  doubleBlinkAnimation,
};
const uint8_t NUM_IDLE_ANIMATIONS = sizeof(idleAnimations) / sizeof(idleAnimations[0]);

// Relative pick weights, parallel to idleAnimations[] above -- higher
// means more common. Real animals don't do every gesture equally
// often: small, cheap movements (a glance, a little tilt) happen all
// the time, while the big, dramatic ones (a hard shake, leaning almost
// to the mechanical limit) are rare enough to still read as notable
// when they do happen, instead of just another animation in the same
// rotation. Tune freely -- these are just relative to each other, not
// on any fixed scale.
const uint8_t idleAnimationWeights[] = {
  10, // curiousTiltAnimation (tilt)
  6,  // yawnAnimation (yawn)
  8,  // lookAroundAnimation (look around)
  7,  // lookAndHoldAnimation (look hold)
  8,  // quickChompsAnimation (chomp)
  3,  // bigTiltAnimation (big tilt)
  6,  // neckStretchAnimation (look up)
  4,  // sleepyBlinkAnimation (sleepy)
  3,  // shakeAnimation (shake)
  2,  // shakeChompAnimation (shake chomp)
  6,  // lookDownAnimation (look down)
  7,  // sniffAnimation (sniff)
  5,  // neckRollAnimation (neck roll)
  6,  // doubleBlinkAnimation (double blink)
};

// ============================================================
// Talk mode's pool
// Everything idle mode can do, plus the sound-synced animations --
// so "talk" looks exactly like idle mode the rest of the time, but
// can also spontaneously bark/roar instead of only ever doing silent
// gestures. Weighted at 15 apiece (idle's 14 silent gestures sum to
// 81, so the 5 sound animations summing to 75 land a "big animation"
// slot on sound close to half the time -- the whole point of "talk"
// mode over plain idle) -- easy to retune per-clip below if one should
// come up more/less than the others.
// ============================================================
AnimationFunc talkAnimations[] = {
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
  lookDownAnimation,
  sniffAnimation,
  neckRollAnimation,
  doubleBlinkAnimation,
  rorAnimation,
  ror2Animation,
  clip5Animation,
  clip1Animation,
  clip2_02Animation,
};
const uint8_t NUM_TALK_ANIMATIONS = sizeof(talkAnimations) / sizeof(talkAnimations[0]);

const uint8_t talkAnimationWeights[] = {
  10, // curiousTiltAnimation (tilt)
  6,  // yawnAnimation (yawn)
  8,  // lookAroundAnimation (look around)
  7,  // lookAndHoldAnimation (look hold)
  8,  // quickChompsAnimation (chomp)
  3,  // bigTiltAnimation (big tilt)
  6,  // neckStretchAnimation (look up)
  4,  // sleepyBlinkAnimation (sleepy)
  3,  // shakeAnimation (shake)
  2,  // shakeChompAnimation (shake chomp)
  6,  // lookDownAnimation (look down)
  7,  // sniffAnimation (sniff)
  5,  // neckRollAnimation (neck roll)
  6,  // doubleBlinkAnimation (double blink)
  15, // rorAnimation (ror)
  15, // ror2Animation (ror two)
  15, // clip5Animation (clip5)
  15, // clip1Animation (clip1)
  15, // clip2_02Animation (clip2_02)
};

// Picks a random animation from the given pool, weighted by the
// parallel weights[] array, and never the same one that just played
// (excludeIdx, or -1 to not exclude anything) -- back-to-back repeats
// of the exact same gesture read as glitchy/looping rather than alive,
// even with everything else randomized. Shared by idle mode and talk
// mode, which differ only in which pool/weights they pass in.
uint8_t pickWeightedAnimation(const uint8_t weights[], uint8_t count, int8_t excludeIdx) {
  uint16_t totalWeight = 0;
  for (uint8_t i = 0; i < count; i++) {
    if (i == excludeIdx) continue;
    totalWeight += weights[i];
  }

  long roll = random(0, totalWeight);
  for (uint8_t i = 0; i < count; i++) {
    if (i == excludeIdx) continue;
    if (roll < weights[i]) return i;
    roll -= weights[i];
  }
  return 0; // unreachable -- the loop above always returns before falling off the end
}

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
    case 10: Serial.println(F("look down")); break;
    case 11: Serial.println(F("sniff")); break;
    case 12: Serial.println(F("neck roll")); break;
    case 13: Serial.println(F("double blink")); break;
    default: Serial.println(F("?")); break;
  }
}

// Same idea as printIdleAnimationName(), indexed against talkAnimations[]
// instead -- the two pools are ordered differently (talk's has the sound
// animations appended), so this can't just reuse the idle switch.
void printTalkAnimationName(uint8_t idx) {
  Serial.print(F("[talk] "));
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
    case 10: Serial.println(F("look down")); break;
    case 11: Serial.println(F("sniff")); break;
    case 12: Serial.println(F("neck roll")); break;
    case 13: Serial.println(F("double blink")); break;
    case 14: Serial.println(F("ror")); break;
    case 15: Serial.println(F("ror two")); break;
    case 16: Serial.println(F("clip5")); break;
    case 17: Serial.println(F("clip1")); break;
    case 18: Serial.println(F("clip2_02")); break;
    default: Serial.println(F("?")); break;
  }
}

// The shared engine behind both idle mode and talk mode -- ambient
// neck sway, eye gaze drift, regular blinking, occasional freezes, and
// a randomly-picked "big" animation every 10-20 seconds. The two modes
// are identical here; they only differ in which pool of big animations
// (and matching weights/name-printer) gets passed in, so idle mode
// never makes sound and talk mode sometimes does.
void runIdleLikeMode(const __FlashStringHelper* enterMsg, const __FlashStringHelper* stopMsg,
                      AnimationFunc* pool, const uint8_t* weights, uint8_t poolCount,
                      void (*printName)(uint8_t)) {
  Serial.println(enterMsg);

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
  int8_t lastIdleAnimationIdx = -1; // -1 = nothing's played yet, so the first pick has nothing to exclude

  // "Freeze" -- the dragon just holds still for a few seconds before
  // resuming the ambient sway, like it paused. More often than the
  // big animations, less often than blinking.
  long nextFreezeAt = random(6000, 10000);
  long freezeStartedAt = -1; // -1 = not currently frozen
  long freezeDurationMs = 0;
  bool freezeVarianceRerolled = false; // makes sure the reroll below only fires once per freeze

  while (!stopRequested()) {
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
      if (stopRequested()) break;
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
        if (stopRequested()) break;
        float dampT = easeInOutExpo((float)(millis() - dampStartMillis) / swayDampMs);
        idleSwayScale = dampStartScale + dampT * (0.7 - dampStartScale);
        applyIdleSway(true, true, true);
        applyIdleGaze(true, true);
        delay(swayStepMs);
      }
      idleSwayScale = 0.7;
      if (stopRequested()) break;

      uint8_t idx = pickWeightedAnimation(weights, poolCount, lastIdleAnimationIdx);
      lastIdleAnimationIdx = idx;
      printName(idx);
      pool[idx](); // fires from wherever neck2/neck3 currently are

      if (stopRequested()) break;

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

  Serial.println(stopMsg);
}

// Type "idle" into the Serial Monitor to start it.
void idleAnimation() {
  runIdleLikeMode(F("Entering idle mode -- type anything or press a mode button to stop."), F("Idle mode stopped."),
                   idleAnimations, idleAnimationWeights, NUM_IDLE_ANIMATIONS, printIdleAnimationName);
}

// Same as idle mode, but its pool of big animations also includes the
// sound-synced ones (ror, ror two, clip5, clip1, clip2_02) -- so the
// dragon can spontaneously bark/roar on its own instead of only ever
// doing silent gestures. Type "talk" into the Serial Monitor to start
// it.
void talkAnimation() {
  runIdleLikeMode(F("Entering talk mode -- type anything or press a mode button to stop."), F("Talk mode stopped."),
                   talkAnimations, talkAnimationWeights, NUM_TALK_ANIMATIONS, printTalkAnimationName);
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
const char cmdName19[] PROGMEM = "look down";
const char cmdName20[] PROGMEM = "sniff";
const char cmdName21[] PROGMEM = "neck roll";
const char cmdName22[] PROGMEM = "double blink";
const char cmdName23[] PROGMEM = "clip1";
const char cmdName24[] PROGMEM = "clip2_02";

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
const char cmdStart19[] PROGMEM = "Looking down...";
const char cmdStart20[] PROGMEM = "Sniffing...";
const char cmdStart21[] PROGMEM = "Neck rolling...";
const char cmdStart22[] PROGMEM = "Double blinking...";
const char cmdStart23[] PROGMEM = "Playing clip1...";
const char cmdStart24[] PROGMEM = "Playing clip2_02...";

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
const char cmdDone19[] PROGMEM = "Look down done.";
const char cmdDone20[] PROGMEM = "Sniff done.";
const char cmdDone21[] PROGMEM = "Neck roll done.";
const char cmdDone22[] PROGMEM = "Double blink done.";
const char cmdDone23[] PROGMEM = "Clip1 done.";
const char cmdDone24[] PROGMEM = "Clip2_02 done.";

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
  { cmdName19, cmdStart19, cmdDone19, lookDownAnimation },
  { cmdName20, cmdStart20, cmdDone20, sniffAnimation },
  { cmdName21, cmdStart21, cmdDone21, neckRollAnimation },
  { cmdName22, cmdStart22, cmdDone22, doubleBlinkAnimation },
  { cmdName23, cmdStart23, cmdDone23, clip1Animation },
  { cmdName24, cmdStart24, cmdDone24, clip2_02Animation },
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

  if (strcasecmp(line, "stress test") == 0) {
    twitchStressTest();
    return;
  }

  if (strcasecmp(line, "idle") == 0) {
    idleAnimation();
    return;
  }

  if (strcasecmp(line, "talk") == 0) {
    talkAnimation();
    return;
  }

  if (strcasecmp(line, "home") == 0) {
    homeAnimation();
    return;
  }

  if (strcasecmp(line, "buttons") == 0) {
    Serial.println(F("Reading mode button pins for 15s (1 = released, 0 = pressed) -- press each button..."));
    unsigned long endAt = millis() + 15000;
    while (millis() < endAt) {
      Serial.print(F("idle(25)="));
      Serial.print(digitalRead(BUTTON_IDLE_PIN));
      Serial.print(F("  talk(26)="));
      Serial.print(digitalRead(BUTTON_TALK_PIN));
      Serial.print(F("  home(27)="));
      Serial.println(digitalRead(BUTTON_HOME_PIN));
      delay(250);
    }
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
