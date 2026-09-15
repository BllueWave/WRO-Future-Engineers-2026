// Blue Wave - WRO Future Engineers 2026 - Arduino Uno
//
// RULE: a RED pillar is passed on ITS right, so the car goes RIGHT of it.
//       a GREEN pillar is passed on ITS left,  so the car goes LEFT of it.
// Servo above CENTER_ANGLE steers LEFT, below it steers RIGHT.
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <NewPing.h>
#include <Pixy2.h>
#include <math.h>

// ===== Debug Toggle =====
#define DEBUG_OUTPUT true

// ===== Ultrasonic pins =====
const int TRIG_LEFT = 4;
const int ECHO_LEFT = 5;

const int TRIG_RIGHT = 2;
const int ECHO_RIGHT = 9;

const int TRIG_FRONT = 13;
const int ECHO_FRONT = 7;

const int SERVO_PIN = 10;

// ===== Motor Driver (Cytron MD13S) =====
const int PWM_PIN = 3;
const int DIR_PIN = 8;

// ===== Speed Settings =====   <-- YOURS
const int MOTOR_SPEED = 30;
const int MOTOR_SPEED_AVOID = 25;
const int MOTOR_SPEED_STOP = 0;

const int FRONT_AVOID_CM = 48;

const int DEFAULT_SIDE_CM = 60;

// [FIX 4] STICTION KICK - "it stops and needs a push".  A brief pulse above
// breakaway, then straight back to the tuned speed.  At full MOTOR_SPEED it
// does nothing at all.
const int MOTOR_KICK = 55;
const unsigned long KICK_MS   = 70;
const unsigned long REKICK_MS = 500;

// [FIX 5] U-TURN GUARD.  A corner needs 90 degrees; past this the turn has
// stopped being a corner.  Straightening ALONE is a trap though - nose-on to
// the wall it just pushes - so the guard straightens for a WINDOW then hands
// the turn back with a fresh budget.  Worst case: turn / drive-out / turn.
const float UTURN_LIMIT_DEG = 100.0;
const unsigned long UTURN_STRAIGHT_MS = 400;

// [FIX 6] Below this the front sonar is measuring the pillar itself, so the
// steering commits fully IN THE DIRECTION PIXY ALREADY CHOSE.
const int PIXY_URGENT_CM = 25;

// [FIX 9] SIDE-WALL VETO.  Never CHOOSES a direction - the camera keeps that
// job.  It only refuses to steer further INTO a side already this close, so it
// can only ever move the command toward centre.
const int WALL_VETO_CM = 16;

// [FIX 10] HOW FAR IS THAT PILLAR - this is the circling.  A pillar is a KNOWN
// object: 50 mm wide, 100 mm tall, so its apparent size IS its distance.
// Pixy2 frame 316x208, lens 60 x 40 degrees:
//        d_mm = 13683 / width_px          d_mm = 28574 / height_px
// Checked: a 50 mm pillar at 600 mm measures 22.8 px wide and 47.6 px tall,
// and both formulas return 600.
const long DIST_K_W = 13683;
const long DIST_K_H = 28574;
const int PIXY_ENGAGE_MM = 900;      // farther than this is not now's problem
const int SWITCH_MARGIN_MM = 250;    // other colour must be CLEARLY nearer

// [FIX 11] COMMIT A LITTLE EARLIER - "it dodges on the last second".  The
// ladder was never the reason: ENTER_PIXY_FRAMES = 2 AND MODE_MIN_HOLD_MS =
// 280 means the steering cannot change for at least 280 ms after the pillar is
// first seen, and at close range that is most of the room left.  A pillar this
// near is not a maybe, so it skips the wait.  Anything farther still goes
// through the full debounce.
const int PIXY_FAST_ENTRY_MM = 550;

// [FIX 7] THE FRONT SENSOR.  48 cm is a good early warning and is NOT the
// problem - going straight to the mechanical limit at 48 cm and STAYING there
// is.  Same distance, proportional response.
const int AVOID_FULL_CM = 18;
const float AVOID_MIN_URGENCY = 0.25;

// [FIX 12] SCAN WEAVE - seeing the pillars at the edges.  The Pixy2 lens is 60
// degrees wide; a pillar at the far side of a 1000 mm corridor close to the
// car is OUTSIDE that cone, and no code finds it because the light never
// reaches the sensor.  A tiny steering bias makes the wall PID correct, and
// that correction swings the nose - and the camera - a few degrees each way.
// Off completely whenever it could cost anything.  Set 0 to disable.
const int SCAN_AMPLITUDE_DEG = 5;
const unsigned long SCAN_PERIOD_MS = 1300;
const float SCAN_MAX_ERROR_CM = 6.0;      // only weave when already centred

// ===== PID Settings =====
const float KP = 0.6;
const float KD = 0.05;
const float KI = 0.0;

float lastError = 0;
float eintegral = 0;
const float I_MAX = 80.0;

// ===== Steering Angles =====
const int CENTER_ANGLE = 90;
const int MIN_SERVO_ANGLE = 30;
const int MAX_SERVO_ANGLE = 160;

// ===== BNO055 =====
Adafruit_BNO055 bno = Adafruit_BNO055(55);

float yaw = 0;
float prevYaw = 0;
float totalRotation = 0;
int laps = 0;

// ===== Ultrasonic =====
#define MAX_DISTANCE 400

NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);

// ===== Pixy2 =====
Pixy2 pixy;

// [FIX 1] Point the camera at a RED pillar and watch the serial.  If it prints
// GREEN, swap these two numbers and nothing else.
const int SIG_GREEN = 2;
const int SIG_RED   = 1;

// [FIX 8] THE LADDER IS NOW A LADDER - this is the over-turn.  All three RED
// values were 40: full lock for ANY red anywhere, held until the blob left the
// frame, which takes ~38 degrees of rotation.  RED must finish on the car's
// LEFT, so a high x means we are on the wrong side (hard) and a low x means it
// is already past us (soft).  GREEN is the mirror.
const int GREEN_HARD_LEFT = 140;   // x < 120
const int GREEN_MED_LEFT  = 120;   // x < 170
const int GREEN_SOFT_LEFT = 102;   // else - already passing it

const int RED_HARD_RIGHT = 40;     // x > 200
const int RED_MED_RIGHT  = 60;     // x > 150
const int RED_SOFT_RIGHT = 78;     // else - already passing it

enum ControlMode {
  MODE_PID,
  MODE_PIXY
};

ControlMode mode = MODE_PID;

unsigned long modeChangedAt = 0;

const uint16_t ENTER_PIXY_FRAMES = 2;
const uint16_t EXIT_PIXY_MISS_FRAMES = 3;
const unsigned long MODE_MIN_HOLD_MS = 280;

uint16_t pixySeenFrames = 0;
uint16_t pixyMissFrames = 0;

// [FIX 2] `int` is 16 bits on AVR, so w*h wraps NEGATIVE past 32767.
const long PIXY_MIN_AREA = 200;
const int PIXY_MIN_X = 20;
const int PIXY_MAX_X = 300;

// [FIX 4/5] state
unsigned long slowSinceMs = 0;
bool  wasSlow = false;
float avoidStartYaw = 0;
bool  wasAvoiding = false;
unsigned long uturnHoldUntil = 0;

float servoCurrentAngle = CENTER_ANGLE;
const float SERVO_SLEW_DEG_PER_STEP = 6.0;

// ===== Distance Filter =====
bool lpfInit = false;
float lpfLeft = 0;
float lpfRight = 0;
const float ALPHA = 0.9;

Servo steeringServo;

int signDistance(int w, int h);
float getStableDistance(NewPing &sonar);
void runMotor(int speed, bool forward);
void lapsCount();

// ===========================================================================

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(DIR_PIN, HIGH);

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(CENTER_ANGLE);
  servoCurrentAngle = CENTER_ANGLE;

  Serial.begin(115200);
  Wire.begin();

  if (!bno.begin()) {
    Serial.println(F("BNO055 not detected. Check wiring!"));
    while (1)
      ;
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  pixy.init();

  if (DEBUG_OUTPUT) {
    Serial.println(F("=== FAST PID + Managed Pixy Switching ==="));
    Serial.print(F("SIG_GREEN=")); Serial.print(SIG_GREEN);
    Serial.print(F("  SIG_RED=")); Serial.println(SIG_RED);
    Serial.println(F("L(cm)\tR(cm)\tF(cm)\tErr\tServo\tYaw\tLaps\tMode"));
  }
}

// ===========================================================================

void loop() {
  float leftDist = getStableDistance(sonarLeft);
  float rightDist = getStableDistance(sonarRight);
  float frontDist = getStableDistance(sonarFront);

  // Untouched.  Two attempts at changing this made the car worse, not better.
  if (leftDist < 0 && rightDist >= 0)
    leftDist = rightDist;

  if (rightDist < 0 && leftDist >= 0)
    rightDist = leftDist;

  if (leftDist < 0)
    leftDist = lpfInit ? lpfLeft : DEFAULT_SIDE_CM;

  if (rightDist < 0)
    rightDist = lpfInit ? lpfRight : DEFAULT_SIDE_CM;

  // ===== Low Pass Filter =====

  if (!lpfInit) {
    lpfLeft = leftDist;
    lpfRight = rightDist;
    lpfInit = true;
  } else {
    if (leftDist > 0)
      lpfLeft = ALPHA * leftDist + (1.0 - ALPHA) * lpfLeft;

    if (rightDist > 0)
      lpfRight = ALPHA * rightDist + (1.0 - ALPHA) * lpfRight;
  }

  // ===== BNO055 =====

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  yaw = orientationData.orientation.x;

  lapsCount();

  if (laps >= 3) {
    runMotor(MOTOR_SPEED_STOP, false);

    Serial.println(F("3 laps completed"));

    while (1)
      ;
  }

  // ================= PIXY PROCESSING =====================

  int nBlocks = pixy.ccc.getBlocks();

  bool pixyGood = false;
  int targetPixyAngle = CENTER_ANGLE;

  // [FIX 10] Every block, not just [0], and the NEAREST one wins.
  static int lockSig = 0;
  int useSig = 0, useX = 158, useDist = 9999;

  if (nBlocks > 0) {
    int bestSig = 0, bestX = 158, bestDist = 9999;
    int lockX = 158, lockDist = 9999;

    for (int i = 0; i < nBlocks && i < 8; i++) {
      int sig = pixy.ccc.blocks[i].m_signature;
      if (sig != SIG_GREEN && sig != SIG_RED) continue;

      int x = pixy.ccc.blocks[i].m_x;
      int w = pixy.ccc.blocks[i].m_width;
      int h = pixy.ccc.blocks[i].m_height;

      long area = (long)w * h;      // [FIX 2] long, not int
      if (area < PIXY_MIN_AREA) continue;
      if (x < PIXY_MIN_X || x > PIXY_MAX_X) continue;

      int d = signDistance(w, h);
      if (d > PIXY_ENGAGE_MM) continue;        // not this moment's problem

      if (d < bestDist) { bestDist = d; bestSig = sig; bestX = x; }
      if (sig == lockSig && d < lockDist) { lockDist = d; lockX = x; }
    }

    if (lockSig != 0 && lockDist < 9999) {
      useSig = lockSig; useX = lockX; useDist = lockDist;

      // hand over only if the other colour is CLEARLY nearer - never on a tie
      if (bestSig != lockSig && bestDist + SWITCH_MARGIN_MM < lockDist) {
        useSig = bestSig; useX = bestX; useDist = bestDist; lockSig = bestSig;
      }
    } else if (bestSig != 0) {
      useSig = bestSig; useX = bestX; useDist = bestDist; lockSig = bestSig;
    }
  }

  if (useSig == 0) lockSig = 0;

  if (useSig != 0) {
    pixyGood = true;

    // [FIX 10b] The front sonar refines the estimate when the pillar is
    // roughly ahead - a direct measurement beats an inference from pixel size.
    // Only when the two roughly agree, otherwise the sonar is looking at a
    // wall past the pillar rather than at the pillar.
    if (frontDist > 0 && abs(useX - 158) < 45) {
      int sonar_mm = (int)(frontDist * 10.0);
      if (sonar_mm < 1500 && abs(sonar_mm - useDist) < 400) useDist = sonar_mm;
    }

    if (useSig == SIG_GREEN) {
      if (useX < 120)      targetPixyAngle = GREEN_HARD_LEFT;
      else if (useX < 170) targetPixyAngle = GREEN_MED_LEFT;
      else                 targetPixyAngle = GREEN_SOFT_LEFT;
    } else {
      if (useX > 200)      targetPixyAngle = RED_HARD_RIGHT;
      else if (useX > 150) targetPixyAngle = RED_MED_RIGHT;
      else                 targetPixyAngle = RED_SOFT_RIGHT;
    }

    if (DEBUG_OUTPUT) {
      Serial.print(useSig == SIG_GREEN ? F("GREEN->LEFT  ") : F("RED->RIGHT   "));
      Serial.print(useDist); Serial.print(F("mm  x="));
      Serial.print(useX);    Serial.print(F("  servo="));
      Serial.println(targetPixyAngle);
    }
  }

  // ================= MODE MANAGEMENT =====================

  unsigned long t = millis();

  if (pixyGood) {
    pixySeenFrames++;
    pixyMissFrames = 0;

    // [FIX 11] A pillar this close is not a maybe - take the wheel now.
    bool urgent = (useDist <= PIXY_FAST_ENTRY_MM);

    if (mode == MODE_PID && pixySeenFrames >= ENTER_PIXY_FRAMES
        && (urgent || (t - modeChangedAt) >= MODE_MIN_HOLD_MS)) {
      mode = MODE_PIXY;
      modeChangedAt = t;

      if (DEBUG_OUTPUT)
        Serial.println(urgent ? F("ENTER PIXY MODE (near)") : F("ENTER PIXY MODE"));
    }
  } else {
    pixyMissFrames++;
    pixySeenFrames = 0;

    if (mode == MODE_PIXY && pixyMissFrames >= EXIT_PIXY_MISS_FRAMES && (t - modeChangedAt) >= MODE_MIN_HOLD_MS) {
      mode = MODE_PID;
      modeChangedAt = t;

      if (DEBUG_OUTPUT)
        Serial.println(F("EXIT PIXY MODE"));
    }
  }

  // ================== CONTROL OUTPUT =====================

  int servoAngle = CENTER_ANGLE;
  int motorSpeed = MOTOR_SPEED;
  bool frontAvoiding = false;

  if (mode == MODE_PIXY && pixyGood) {
    servoCurrentAngle += constrain(
      targetPixyAngle - servoCurrentAngle,
      -SERVO_SLEW_DEG_PER_STEP,
      SERVO_SLEW_DEG_PER_STEP);

    // [FIX 6] The ultrasonic now HELPS the camera instead of standing by.
    if (frontDist > 0 && frontDist <= PIXY_URGENT_CM) {
      targetPixyAngle = (targetPixyAngle > CENTER_ANGLE) ? MAX_SERVO_ANGLE
                                                         : MIN_SERVO_ANGLE;
      servoCurrentAngle = targetPixyAngle;      // no slew when it is this late
    }

    // [FIX 9] The side sonars get a veto, never a vote.
    if (targetPixyAngle < CENTER_ANGLE && lpfRight < WALL_VETO_CM) {
      targetPixyAngle = CENTER_ANGLE;           // turning right into a wall
      servoCurrentAngle = CENTER_ANGLE;
    } else if (targetPixyAngle > CENTER_ANGLE && lpfLeft < WALL_VETO_CM) {
      targetPixyAngle = CENTER_ANGLE;           // turning left into a wall
      servoCurrentAngle = CENTER_ANGLE;
    }

    servoAngle = (int)servoCurrentAngle;

    if (frontDist > 0 && frontDist <= FRONT_AVOID_CM)
      motorSpeed = MOTOR_SPEED_AVOID;
  } else if (frontDist > 0 && frontDist <= FRONT_AVOID_CM) {
    frontAvoiding = true;
    motorSpeed = MOTOR_SPEED_AVOID;

    // [FIX 5] how far has the car turned since THIS avoid began?
    if (!wasAvoiding) { avoidStartYaw = yaw; uturnHoldUntil = 0; }
    float turned = yaw - avoidStartYaw;
    if (turned >  180) turned -= 360;
    if (turned < -180) turned += 360;

    // [FIX 7] how hard, from how close the wall is
    float urg = (float)(FRONT_AVOID_CM - frontDist)
              / (float)(FRONT_AVOID_CM - AVOID_FULL_CM);
    urg = constrain(urg, AVOID_MIN_URGENCY, 1.0);

    bool goLeft = (lpfLeft > lpfRight + 3);   // tie still defaults to right

    if (fabs(turned) >= UTURN_LIMIT_DEG && uturnHoldUntil == 0) {
      uturnHoldUntil = t + UTURN_STRAIGHT_MS;
    }

    if (uturnHoldUntil != 0 && t < uturnHoldUntil) {
      servoAngle = CENTER_ANGLE;          // drive out of the over-rotation
    } else if (uturnHoldUntil != 0) {
      uturnHoldUntil = 0;
      avoidStartYaw = yaw;                // fresh budget, never stuck
      servoAngle = goLeft
          ? CENTER_ANGLE + (int)((MAX_SERVO_ANGLE - CENTER_ANGLE) * urg)
          : CENTER_ANGLE - (int)((CENTER_ANGLE - MIN_SERVO_ANGLE) * urg);
    } else if (goLeft) {
      servoAngle = CENTER_ANGLE + (int)((MAX_SERVO_ANGLE - CENTER_ANGLE) * urg);
    } else {
      servoAngle = CENTER_ANGLE - (int)((CENTER_ANGLE - MIN_SERVO_ANGLE) * urg);
    }

    lastError = 0;
    eintegral = 0;
  } else {
    float error = lpfLeft - lpfRight;

    eintegral += error;
    eintegral = constrain(eintegral, -I_MAX, I_MAX);

    float derivative = error - lastError;
    lastError = error;

    float output =
      KP * error + KI * eintegral + KD * derivative;

    servoAngle = CENTER_ANGLE + output;

    // [FIX 12] Sweep the camera, but only when it is free.  No pillar in view,
    // no avoid running, and the car already centred - so this can never fight
    // a correction that matters, only add a slow few degrees of look-around.
    if (SCAN_AMPLITUDE_DEG > 0 && !pixyGood && fabs(error) < SCAN_MAX_ERROR_CM) {
      float ph = (float)(t % SCAN_PERIOD_MS) / (float)SCAN_PERIOD_MS;
      servoAngle += (int)(SCAN_AMPLITUDE_DEG * sin(ph * 2.0 * PI));
    }
  }

  servoAngle = constrain(
    servoAngle,
    MIN_SERVO_ANGLE,
    MAX_SERVO_ANGLE);

  steeringServo.write(servoAngle);
  servoCurrentAngle = servoAngle;

  // [FIX 4] Break static friction whenever the car is asked to crawl.
  {
    unsigned long nowMs = millis();
    bool slow = (motorSpeed > 0 && motorSpeed < MOTOR_SPEED);

    if (slow && !wasSlow) slowSinceMs = nowMs;

    if (slow) {
      unsigned long inSlow = nowMs - slowSinceMs;
      if (inSlow < KICK_MS || (inSlow % REKICK_MS) < KICK_MS)
        motorSpeed = MOTOR_KICK;
    }
    wasSlow = slow;
  }

  runMotor(motorSpeed, true);
  wasAvoiding = frontAvoiding;

  // ================= DEBUG OUTPUT ========================

  if (DEBUG_OUTPUT) {
    Serial.print(lpfLeft);
    Serial.print(F("\t"));

    Serial.print(lpfRight);
    Serial.print(F("\t"));

    Serial.print(frontDist);
    Serial.print(F("\t"));

    Serial.print(lastError);
    Serial.print(F("\t"));

    Serial.print(servoAngle);
    Serial.print(F("\t"));

    Serial.print(yaw);
    Serial.print(F("\t"));

    Serial.print(laps);
    Serial.print(F("\t"));

    if (frontAvoiding)
      Serial.println(F("AVOID"));
    else
      Serial.println(mode == MODE_PID ? F("PID") : F("PIXY"));
  }
}

// ========================= FUNCTIONS =======================================

// [FIX 10] Distance to a signal pillar in mm from its apparent size.
// Takes the SMALLER of the two estimates deliberately: a blob clipped by the
// frame edge measures smaller than the real pillar, which INFLATES the
// distance - and over-estimating distance is the dangerous direction, because
// it makes a near pillar look ignorable.
int signDistance(int w, int h) {
  long dw = (w > 0) ? (DIST_K_W / w) : 9999;
  long dh = (h > 0) ? (DIST_K_H / h) : 9999;
  long d = (dw < dh) ? dw : dh;
  if (d > 4000) d = 4000;
  return (int)d;
}

float getStableDistance(NewPing &sonar) {
  delay(3);

  int d = sonar.ping_cm();

  if (d == 0 || d > MAX_DISTANCE)
    return -1;

  return d;
}

void runMotor(int speed, bool forward) {
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  analogWrite(PWM_PIN, speed);
}

void lapsCount() {
  // [FIX 3] prevYaw starts at 0 but the BNO055 gives an ABSOLUTE heading, so
  // the first delta is whatever way the car was pointing at power-on, injected
  // straight into totalRotation.  Lap 1 then ends after 360 minus that heading
  // - early or late by up to half a lap purely according to which way the mat
  // faces in the room.  That is the start-line overshoot.
  static bool yawSeeded = false;
  if (!yawSeeded) {
    prevYaw = yaw;
    yawSeeded = true;
    return;
  }

  float delta = yaw - prevYaw;

  if (delta > 180)
    delta -= 360;

  if (delta < -180)
    delta += 360;

  totalRotation += delta;
  prevYaw = yaw;

  if (fabs(totalRotation) >= 360.0) {
    laps++;
    totalRotation = 0;
  }
}