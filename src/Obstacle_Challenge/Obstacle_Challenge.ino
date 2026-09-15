// Blue Wave - WRO Future Engineers 2026 - OBSTACLE - Arduino Uno - v20
// ===========================================================================
// WHAT THIS IS
//   The driving law that the real car proved on the mat, unchanged where it
//   worked, plus what it never had:
//     1. a LAP-1 MAP of the pillar colours in each straight, used in laps 2-3 to
//        be on the right side BEFORE the camera sees the pillar, and to run
//        empty straights faster;
//     2. a lap counter that cannot miss the third lap (FIX 26), and a stop at the
//        next corner if the park window is missed (never a lap 4);
//     3. the DOCTOR'S PARK: stop at a front-wall distance in the outer lane,
//        then IMU-closed arcs and measured steps into the lot;
//     4. a start on the A2 switch (rule 9.10), or 3 s after power-up (PRACTICE);
//     5. [DIR] a corner whose two 40-deg sonars read the same front wall turns the
//        way this round turns (lot side, then the corners driven), not always right;
//     6. [STUCK] nose against a wall for 0.7 s: back out 0.45 s, wheels reversed;
//     7. the front sonar may only make a pillar nearer (a missed pillar read the
//        wall behind it and cut the steering just before the pass).
//
//   BASE: theobistcalORangeWITHPARK (09-06 build) = obstacle_kuwait (won the
//   national round) + FIX 15/16/17.  Its driving was "the best so far" and its
//   lot exit "excellent" on the real car.  Every number marked [PROVEN] is a
//   number from that build.  Nothing in the lap law is dead-reckoned: the car
//   steers on what it measures (L-R sonar balance, Pixy x, front range) and
//   ends every manoeuvre on the IMU.
//
// RULE: a RED pillar is passed on ITS right, so the car goes RIGHT of it.
//       a GREEN pillar is passed on ITS left,  so the car goes LEFT of it.
// Servo above CENTER_ANGLE steers LEFT, below it steers RIGHT.
//
// MEASURE ON THE CAR (5 minutes, before trusting the park):
//   R_PARK_MM   the rear-axle turning radius at servo 170 and at 10 (full lock).
//               Drive one slow full-lock circle, halve the diameter.
//   CAR_NOSE_MM rear axle centre -> front bumper.
//   SIDE_A/B_L and _R  for EACH side: car parallel to the wall, rear axle 300 mm and 400 mm
//               from it, read that side's sonar (DEBUG print while waiting: "WAIT L R F"), then
//               A = 100 / (cm400 - cm300),  B = 300 - A * cm300.
//   LOT_RIGHT_MM / LOT_LEFT_MM  tape: far wall -> the downstream limiter's far face.
// ===========================================================================
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <NewPing.h>
#include <Pixy2.h>
#include <math.h>

#define DEBUG_OUTPUT 1                 // [PROVEN] the tested build printed every loop

// ===== Pins [PROVEN] =====
const int TRIG_LEFT = 4,  ECHO_LEFT = 5;
const int TRIG_RIGHT = 2, ECHO_RIGHT = 9;
const int TRIG_FRONT = 6, ECHO_FRONT = 7;
const int SERVO_PIN = 10;
const int PWM_PIN = 3, DIR_PIN = 8;    // Cytron MD13S, DIR HIGH = forward
const int START_PIN = A2;              // start switch to GND, internal pull-up

// ===== Start [RULE 9.10] =====
// The round starts on ANY debounced change of A2 (push button or toggle).
// PRACTICE 1 also starts 3 s after power-up (the team's tested code did this and never
// needed A2).  With a start button wired to A2, set PRACTICE 0 for the official round.
#define PRACTICE 0
const unsigned long AUTO_START_MS = PRACTICE ? 3000 : 0;

// ===== Speeds [PROVEN] =====
const int MOTOR_SPEED = 30;
const int MOTOR_SPEED_AVOID = 25;
const int MOTOR_SPEED_STOP = 0;
const int FRONT_AVOID_CM = 48;
const int DEFAULT_SIDE_CM = 60;
const int MOTOR_KICK = 55;
const unsigned long KICK_MS = 70, REKICK_MS = 500;
const float UTURN_LIMIT_DEG = 100.0;
const unsigned long UTURN_STRAIGHT_MS = 400;
const int PIXY_URGENT_CM = 25;
const int WALL_VETO_CM = 16;
const long DIST_K_W = 13683, DIST_K_H = 28574;
const int PIXY_ENGAGE_MM = 900;
const int SWITCH_MARGIN_MM = 250;
const int PIXY_FAST_ENTRY_MM = 550;
const int AVOID_FULL_CM = 18;
const float AVOID_MIN_URGENCY = 0.25;
const int SCAN_AMPLITUDE_DEG = 5;
const unsigned long SCAN_PERIOD_MS = 1300;
const float SCAN_MAX_ERROR_CM = 6.0;
const float KP = 0.6, KD = 0.05, KI = 0.0;
const float I_MAX = 80.0;
const int CENTER_ANGLE = 90, MIN_SERVO_ANGLE = 30, MAX_SERVO_ANGLE = 160;
#define MAX_DISTANCE 400
const int SIG_GREEN = 2, SIG_RED = 1, SIG_PARK_WALL = 3;
const float PIXY_MIN_COMMIT = 0.35;    // [FIX 15]
const int WALL_LIMIT_CM = 25;          // [FIX 16]
const int GREEN_HARD_LEFT = 140, GREEN_MED_LEFT = 120, GREEN_SOFT_LEFT = 102;
const int RED_HARD_RIGHT = 40, RED_MED_RIGHT = 60, RED_SOFT_RIGHT = 78;
const uint16_t ENTER_PIXY_FRAMES = 2, EXIT_PIXY_MISS_FRAMES = 3;
const unsigned long MODE_MIN_HOLD_MS = 280;
const long PIXY_MIN_AREA = 200;
const int PIXY_MIN_X = 20, PIXY_MAX_X = 300;
const float SERVO_SLEW_DEG_PER_STEP = 6.0;
const float ALPHA = 0.9;
const float WALL_ASPECT = 1.4;
const long WALL_MIN_AREA = 600;
const float DEG2RAD = 0.0174532925f;

// ===== Lot exit [PROVEN "excellent", 09-06] =====
const int PARK_SPEED = 18;
const int PARK_LOCK_L = 170, PARK_LOCK_R = 10;
const uint8_t EXIT_MAX_CYCLES = 20;
const float EXIT_TARGET_DEG = 50.0;
const unsigned long P_REV_MS = 200, P_FWD_MS = 200, P_PAUSE_MS = 220, P_OUT_MS = 800;

// ===== [MAP] lap-1 map, laps 2-3 use =====
const float BIAS_CM = 20.0;            // L-R set-point shift toward the side a mapped pillar needs
const unsigned long BIAS_MS = 1600;    // ...for this long after the corner, or until the camera takes over
const float BIAS_ALIGN_DEG = 12.0;     // ...and only once the corner is finished
const int MOTOR_SPEED_FAST = 36;       // an EMPTY mapped straight, laps 2-3, front far
const int FAST_FRONT_CM = 130;
const float DIR_TRUST_CM = 3.0;        // [DIR] a corner turns to the longer side only when L-R exceeds this

// ===== [STUCK] nose against a wall or pillar =====
const int STUCK_FRONT_CM = 15;          // the last valid front reading is this close...
const unsigned long STUCK_MS = 700;     // ...and the heading has not moved 3 deg for this long
const int STUCK_REV_PWM = 30;
const unsigned long STUCK_REV_MS = 450; // back out this long with the wheels reversed

// ===== [PARK] the doctor's park =====
const float R_PARK_MM = 170.0;         // MEASURE: rear-axle radius at servo 170 / 10
const float CAR_LEN_MM = 200.0, CAR_WID_MM = 125.0;
const float CAR_NOSE_MM = 168.0;       // MEASURE: rear axle -> nose (tail = LEN - NOSE); 168 = the sim fit of the real exit
const float LOT_LEN_MM = 300.0, LOT_DEPTH_MM = 200.0, LIM_T_MM = 20.0;
const int LOT_RIGHT_MM = 980;          // far wall -> downstream limiter's far face, lot on the RIGHT
const int LOT_LEFT_MM = 3000 - 340 - LOT_RIGHT_MM;   // the same lot seen from the other wall
const int FRONT_SETBACK_MM = 0;        // front sonar face behind the nose
// rear-axle -> wall mm = A * wall-side cm + B, car parallel.  The two 40-deg units differ: MEASURE both.
const float SIDE_A_L = 5.16, SIDE_B_L = 202.5;    // left unit  (lot on the left)  [sim F2 fit, sd 2 mm]
const float SIDE_A_R = 13.02, SIDE_B_R = -50.2;   // right unit (lot on the right) [sim F2 fit, sd 2 mm]
const float PARK_LANE_MM = 320.0;      // rear axle -> outer wall while approaching (limiter tips at 200)
const int APPROACH_PWM = 25;
const float APPROACH_KH = 2.2;         // servo deg per deg of heading error
const float APPROACH_KL = 0.10;        // deg of heading per mm of lane error
const float APPROACH_PSI_MAX = 10.0;   // < the 12-deg stop gate, so the gate can always open
const int MARK_MARGIN_MM = -40;        // rear bumper this far past the downstream limiter's far face at
                                       // the stop (-40 = 40 mm before it: arc C clears the tip)
const int CW_NEAR_CM = 95;             // lot on the left: run on to here, then reverse to the mark
const int STOP_LEAD_MM = 60;           // cut the motor this early (coast)
const int MARK_TOL_MM = 30;
const int PARK_ARC_PWM = 18, PARK_NUDGE_PWM = 22, PARK_KICK_PWM = 55;
const unsigned long PARK_KICK_MS = 60, PARK_PAUSE_MS = 220, PARK_ARC_CAP_MS = 2600, PARK_STALL_MS = 150;
const unsigned long STEP_KICK_MS = 25, STEP_SETTLE_MS = 380;
const float PARK_COAST_MS = 150.0;
const float PARK_ENTRY_DEG = 50.0, PARK_DIAG_MM = 160.0;
const float PARK_MARGIN_MM = 12.0, PARK_SLACK_DEG = 2.5, PARK_SHUF_DEG = 6.0, PARK_SQUARE_DEG = 3.0;
const float PARK_DEPTH_BIAS = 12.0, PARK_ALONG_BIAS = 12.0;
const unsigned long APPROACH_CAP_MS = 15000;
const unsigned long AP_PAST_MS = 1000;   // lot on the right: straight driven before the car can be past the mark

// ===========================================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
Pixy2 pixy;
Servo steeringServo;

enum { ST_WAIT, ST_EXIT, ST_LAPS, ST_APPROACH, ST_PARK, ST_DONE };
uint8_t state = ST_WAIT;

float yaw = 0, prevYaw = 0;
float turnAccum = 0;                   // [FIX 26] every degree since the start, never zeroed
int quad = 0, lastQuad = 0;            // corners done = round(|turnAccum| / 90)
float headingSign = 1.0;               // +1: the BNO055 heading grows clockwise (datasheet)

enum ControlMode { MODE_PID, MODE_PIXY };
ControlMode mode = MODE_PID;
unsigned long modeChangedAt = 0;
uint16_t pixySeenFrames = 0, pixyMissFrames = 0;
float lastError = 0, eintegral = 0;
unsigned long slowSinceMs = 0;
bool wasSlow = false;
float avoidStartYaw = 0;
bool wasAvoiding = false;
unsigned long uturnHoldUntil = 0;
float servoCurrentAngle = CENTER_ANGLE;
bool lpfInit = false;
float lpfLeft = 0, lpfRight = 0;
int lockSig = 0;

// exit
bool parkWallLeft = false;
uint8_t startSub = 0, startCycle = 0;
float startYaw0 = 0;
unsigned long startPhaseMs = 0;

// [MAP]
struct SecMap { uint8_t n; uint8_t sig[2]; };
SecMap secMap[4];
bool secSeen[4];                       // any valid sign seen while lined up with this straight
bool exitStraight = false;             // lot on the left: the car has straightened after the exit
int biasSig = 0;
unsigned long biasUntil = 0;
unsigned long quadMs = 0;              // when the current straight's corner counted
unsigned long stuckSinceMs = 0;
float stuckYaw = 0, lastFrontCm = 999;
unsigned long lastFrontMs = 0;
int dirVote = 0;                       // + = this round's corners turn left (lot side seeds it)
bool avoidLeft = false;                // the current front-avoid's direction
bool biasUsed = false;

// approach / park
uint8_t apPhase = 0;
unsigned long apStartMs = 0;
float apWallMm = PARK_LANE_MM;
float apF = -1.0, apV = 0.6;           // tracked far-wall range (mm) and closing speed (mm/ms)
unsigned long apFms = 0;
bool apChecked = false;
unsigned long apAfterCornerMs = 0;     // straight driven before the handover
uint8_t apJumps = 0;
uint8_t apNear = 0, apAgree = 0, apClose = 0;
float apCand = -1.0;
bool servoAtLock = false;
float bx = 0, by = 0, bh = 0;          // park pose: rear axle, lot frame, + nose away from the wall

// ===========================================================================
float frontMeanMm();
float wallMeanMm();

void runMotor(int speed, bool forward) {
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  analogWrite(PWM_PIN, speed);
}

float getStableDistance(NewPing &sonar) {
  delay(3);
  int d = sonar.ping_cm();
  if (d == 0 || d > MAX_DISTANCE) return -1;
  return d;
}

float wrap180(float a);

float readYaw() {
  static float last = 0;
  static bool have = false;
  sensors_event_t ev;
  bno.getEvent(&ev, Adafruit_BNO055::VECTOR_EULER);
  float y = ev.orientation.x;
  // a failed I2C read (or the Wire timeout) returns exactly 0.0: one such sample would count a
  // phantom corner that never counts back.  A real heading cannot jump 20 deg in one read.
  if (have && y == 0.0 && fabs(wrap180(last)) > 20.0) y = last;
  last = y;
  have = true;
  return y;
}

float fmin2(float a, float b) { return a < b ? a : b; }
float sideToWallMm(float cm);
float fmax2(float a, float b) { return a > b ? a : b; }

float wrap180(float a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// degrees the car has turned to the RIGHT since heading h0 (sign-corrected)
float rightTurnSince(float h0) { return headingSign * wrap180(readYaw() - h0); }

int signDistance(int w, int h) {
  long dw = (w > 0) ? (DIST_K_W / w) : 9999;
  long dh = (h > 0) ? (DIST_K_H / h) : 9999;
  long d = (dw < dh) ? dw : dh;
  if (d > 4000) d = 4000;
  return (int)d;
}

bool looksLikeBarrier(int w, int h) {
  return (w > 0 && h > 0 && (long)w * h >= WALL_MIN_AREA && (float)w / (float)h >= WALL_ASPECT);
}

void signalServo(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    steeringServo.write(CENTER_ANGLE + 25); delay(180);
    steeringServo.write(CENTER_ANGLE - 25); delay(180);
  }
  steeringServo.write(CENTER_ANGLE);
}

// ===========================================================================
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(START_PIN, INPUT_PULLUP);
  runMotor(0, true);
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(CENTER_ANGLE);
  Serial.begin(115200);
  Wire.begin();
#ifdef __AVR__
  Wire.setWireTimeout(25000, true);      // a stuck I2C bus must not freeze the car
#endif
  bool imu = false;
  for (int i = 0; i < 3 && !imu; i++) { imu = bno.begin(); if (!imu) delay(300); }
  if (!imu) {
    for (;;) signalServo(2);             // two wiggles for ever = BNO055 not answering
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  pixy.init();
  signalServo(1);                        // one wiggle = ready
  memset(secMap, 0, sizeof(secMap));
  memset(secSeen, 0, sizeof(secSeen));
  state = ST_WAIT;
}

// ===========================================================================
bool waitStart() {
  static bool init = false;
  static int level = 0, cand = 0;
  static unsigned long t0 = 0, candMs = 0;
  unsigned long now = millis();
  int r = digitalRead(START_PIN);
  if (!init) { init = true; level = r; cand = r; t0 = now; candMs = now; return false; }
  if (r != cand) { cand = r; candMs = now; }
  if (cand != level && now - candMs >= 30) return true;
  if (AUTO_START_MS > 0 && now - t0 >= AUTO_START_MS) return true;
#if DEBUG_OUTPUT
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 400) {
    lastPrint = now;
    Serial.print(F("WAIT L ")); Serial.print(getStableDistance(sonarLeft));
    Serial.print(F(" R ")); Serial.print(getStableDistance(sonarRight));
    Serial.print(F(" F ")); Serial.print(getStableDistance(sonarFront));
    Serial.print(F(" yaw ")); Serial.println(readYaw());
  }
#endif
  return false;
}

// [PROVEN] which side is the outer wall: 500 ms average of both side sonars at rest
void parkPickSide() {
  float ls = 0, rs = 0;
  int lc = 0, rc = 0;
  unsigned long t0 = millis();
  runMotor(MOTOR_SPEED_STOP, true);
  steeringServo.write(CENTER_ANGLE);
  while (millis() - t0 < 500) {
    float l = getStableDistance(sonarLeft);
    float r = getStableDistance(sonarRight);
    if (l > 0) { ls += l; lc++; }
    if (r > 0) { rs += r; rc++; }
  }
  float la = (lc > 0) ? (ls / lc) : 9999;
  float ra = (rc > 0) ? (rs / rc) : 9999;
  parkWallLeft = (la < ra);
  dirVote = parkWallLeft ? -2 : 2;       // [DIR] outer wall on the left = clockwise = right turns
  startYaw0 = readYaw();
  startPhaseMs = millis();
#if DEBUG_OUTPUT
  Serial.print(F("outer wall = ")); Serial.print(parkWallLeft ? F("LEFT") : F("RIGHT"));
  Serial.print(F(" L=")); Serial.print(la); Serial.print(F(" R=")); Serial.println(ra);
#endif
}

// [PROVEN] the exit ratchet: reverse at the wall lock, forward at the other lock, PWM 18,
// until the heading is 50 deg out, then 800 ms straight.
bool startTick() {
  unsigned long now = millis();
  unsigned long held = now - startPhaseMs;
  float y = readYaw();
  float turned = wrap180(y - startYaw0);
  int toWall = parkWallLeft ? PARK_LOCK_L : PARK_LOCK_R;
  int offWall = parkWallLeft ? PARK_LOCK_R : PARK_LOCK_L;
  if (startSub < 4 && (fabs(turned) >= EXIT_TARGET_DEG || startCycle >= EXIT_MAX_CYCLES)) {
    startSub = 4;
    startPhaseMs = now;
    // [SIGN] the exit rotates the nose AWAY from the wall: wall left -> the car turns right.
    // If that showed as a heading DEcrease, this BNO055 is mounted the other way up.
    if (fabs(turned) >= 20.0) headingSign = ((turned > 0) == parkWallLeft) ? 1.0 : -1.0;
    return false;
  }
  switch (startSub) {
    case 0:
      steeringServo.write(toWall);
      runMotor(MOTOR_SPEED_STOP, true);
      if (held >= P_PAUSE_MS) { startPhaseMs = now; startSub = 1; }
      break;
    case 1:
      steeringServo.write(toWall);
      runMotor(PARK_SPEED, false);
      if (held >= P_REV_MS) { runMotor(MOTOR_SPEED_STOP, true); startPhaseMs = now; startSub = 2; }
      break;
    case 2:
      steeringServo.write(offWall);
      runMotor(MOTOR_SPEED_STOP, true);
      if (held >= P_PAUSE_MS) { startPhaseMs = now; startSub = 3; }
      break;
    case 3:
      steeringServo.write(offWall);
      runMotor(PARK_SPEED, true);
      if (held >= P_FWD_MS) { runMotor(MOTOR_SPEED_STOP, true); startPhaseMs = now; startSub = 0; startCycle++; }
      break;
    default:
      steeringServo.write(CENTER_ANGLE);
      runMotor(PARK_SPEED, true);
      if (held >= P_OUT_MS) {
        lastError = 0;
        eintegral = 0;
        prevYaw = y;
        turnAccum = turned;              // [FIX 26] the exit's rotation IS part of the count
        return true;
      }
      break;
  }
  return false;
}

// [FIX 26] every heading change is summed and never thrown away
void lapsCount() {
  float delta = wrap180(yaw - prevYaw);
  turnAccum += delta;
  prevYaw = yaw;
  // a corner counts once the car has turned 70 of its 90 degrees; the counter never goes
  // back (the exit's ~50 deg is seeded above and straightened away without counting)
  if (fabs(turnAccum) >= 90.0 * quad + 70.0) quad++;
}

// ===========================================================================
//   THE LAP LAW [PROVEN] + [MAP] + [FIX 16 servo jump]
// ===========================================================================
void lapStep() {
  float leftDist = getStableDistance(sonarLeft);
  float rightDist = getStableDistance(sonarRight);
  float frontDist = getStableDistance(sonarFront);
  if (leftDist < 0 && rightDist >= 0) leftDist = rightDist;
  if (rightDist < 0 && leftDist >= 0) rightDist = leftDist;
  if (leftDist < 0) leftDist = lpfInit ? lpfLeft : DEFAULT_SIDE_CM;
  if (rightDist < 0) rightDist = lpfInit ? lpfRight : DEFAULT_SIDE_CM;
  if (!lpfInit) {
    lpfLeft = leftDist; lpfRight = rightDist; lpfInit = true;
  } else {
    if (leftDist > 0) lpfLeft = ALPHA * leftDist + (1.0 - ALPHA) * lpfLeft;
    if (rightDist > 0) lpfRight = ALPHA * rightDist + (1.0 - ALPHA) * lpfRight;
  }

  yaw = readYaw();
  lapsCount();
  unsigned long t = millis();

  // ---- [MAP] which straight the car is lined up with.  Lot on the RIGHT: the lot is at the
  // END of the start straight, so lap 1 leaves it with that straight's pillars behind the
  // camera - straight 0 is never mapped.  Lot on the LEFT: straight 0 is ahead after the exit.
  float absTurn = fabs(turnAccum);
  if (absTurn < 20.0 || quad > 0) exitStraight = true;
  int sec = (int)((absTurn + 45.0) / 90.0);
  bool aligned = fabs(absTurn - 90.0 * sec) < 30.0;
  if (parkWallLeft && !exitStraight) { sec = 0; aligned = true; }
  bool secKnown = parkWallLeft || (quad % 4) != 0;

  // ---- [MAP] a new straight: in laps 2-3, the mapped first pillar sets a side bias
  if (quad != lastQuad) {
    lastQuad = quad;
    quadMs = t;
    // [DIR] the corner that just counted, by its measured rotation (PID- and Pixy-mode corners
    // never set avoidLeft); headingSign +1 = the heading grows turning right
    if (quad > 0) dirVote += ((turnAccum > 0) == (headingSign > 0)) ? -1 : 1;
    biasSig = 0;
    biasUsed = (mode == MODE_PIXY);      // the camera already has a pillar: no bias toward it
    if (quad >= 4 && secKnown) {
      SecMap &m = secMap[quad % 4];
      if (m.n > 0) { biasSig = m.sig[0]; biasUntil = t + BIAS_MS; }
    }
#if DEBUG_OUTPUT
    Serial.print(F("SECTION ")); Serial.print(quad);
    Serial.print(F(" map n=")); Serial.print(secMap[quad % 4].n);
    Serial.print(F(" first=")); Serial.print(secMap[quad % 4].sig[0]);
    Serial.print(F(" seen=")); Serial.println(secSeen[quad % 4]);
#endif
  }

  // ================= PIXY PROCESSING [PROVEN] =====================
  int nBlocks = pixy.ccc.getBlocks();
  bool pixyGood = false;
  bool sawSign = false;                  // [MAP] a valid sign at ANY distance
  int targetPixyAngle = CENTER_ANGLE;
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
      if (looksLikeBarrier(w, h)) continue;
      long area = (long)w * h;
      if (area < PIXY_MIN_AREA) continue;
      if (x < PIXY_MIN_X || x > PIXY_MAX_X) continue;
      sawSign = true;
      int d = signDistance(w, h);
      if (d > PIXY_ENGAGE_MM) continue;
      if (d < bestDist) { bestDist = d; bestSig = sig; bestX = x; }
      if (sig == lockSig && d < lockDist) { lockDist = d; lockX = x; }
    }
    if (lockSig != 0 && lockDist < 9999) {
      useSig = lockSig; useX = lockX; useDist = lockDist;
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
    if (frontDist > 0 && abs(useX - 158) < 45) {
      // [v20] the sonar may only make the pillar NEARER: a 50 mm pillar 5-9 deg off the axis
      // is often missed and the wall behind it read instead, which made the pillar look far
      // and cut the steering (FIX 15) just before the pass
      int sonar_mm = (int)(frontDist * 10.0);
      if (sonar_mm < 1500 && sonar_mm <= useDist + 60 && useDist - sonar_mm < 400) useDist = sonar_mm;
    }
    if (useSig == SIG_GREEN) {
      if (useX < 120) targetPixyAngle = GREEN_HARD_LEFT;
      else if (useX < 170) targetPixyAngle = GREEN_MED_LEFT;
      else targetPixyAngle = GREEN_SOFT_LEFT;
    } else {
      if (useX > 200) targetPixyAngle = RED_HARD_RIGHT;
      else if (useX > 150) targetPixyAngle = RED_MED_RIGHT;
      else targetPixyAngle = RED_SOFT_RIGHT;
    }
    if (useDist > PIXY_FAST_ENTRY_MM) {                                   // [FIX 15]
      float k = (float)(PIXY_ENGAGE_MM - useDist) / (float)(PIXY_ENGAGE_MM - PIXY_FAST_ENTRY_MM);
      k = constrain(k, PIXY_MIN_COMMIT, 1.0);
      targetPixyAngle = CENTER_ANGLE + (int)((targetPixyAngle - CENTER_ANGLE) * k);
    }
  }

  // ================= MODE MANAGEMENT [PROVEN] =====================
  if (pixyGood) {
    pixySeenFrames++;
    pixyMissFrames = 0;
    bool urgent = (useDist <= PIXY_FAST_ENTRY_MM);
    if (mode == MODE_PID && pixySeenFrames >= ENTER_PIXY_FRAMES && (urgent || (t - modeChangedAt) >= MODE_MIN_HOLD_MS)) {
      mode = MODE_PIXY;
      modeChangedAt = t;
      biasUsed = true;                   // [MAP] the camera has the pillar: the bias has done its job
      // [MAP] lap 1: remember the colour order of the straight the car is lined up with
      if (quad < 4 && aligned && sec < 4 && (sec != 0 || parkWallLeft)) {
        SecMap &m = secMap[sec];
        if (m.n < 2) m.sig[m.n++] = (uint8_t)useSig;
      }
    }
  } else {
    if (pixyMissFrames < 1000) pixyMissFrames++;
    pixySeenFrames = 0;
    if (mode == MODE_PIXY && pixyMissFrames >= EXIT_PIXY_MISS_FRAMES && (t - modeChangedAt) >= MODE_MIN_HOLD_MS) {
      mode = MODE_PID;
      modeChangedAt = t;
    }
  }
  if (quad < 4 && sawSign && aligned && sec < 4) secSeen[sec] = true;

  // ---- three laps done, the car is parallel again in the start section and the camera has
  // been clear for 5 frames (the section's pillars are passed or out of the lane): park
  if (quad >= 12 && fabs(absTurn - 1080.0) < 8.0 && mode == MODE_PID && !wasAvoiding && !pixyGood
      && pixyMissFrames >= 5) {
    state = ST_APPROACH;
    apPhase = 0;
    apStartMs = t;
    apF = -1.0;
    apChecked = false;
    apAfterCornerMs = t - quadMs;
    runMotor(MOTOR_SPEED_STOP, true);
    return;
  }
  // the clean window was missed (pillars / avoiding through the whole straight): never start
  // lap 4 - stop at the next corner, still in the start section
  if (quad >= 13 || (quad >= 12 && absTurn > 1040.0 && t - quadMs > 1200 && frontDist > 0
                     && frontDist <= FRONT_AVOID_CM && mode == MODE_PID && !wasAvoiding)) {
    runMotor(MOTOR_SPEED_STOP, true);
    steeringServo.write(CENTER_ANGLE);
    state = ST_DONE;
    return;
  }

  // ================== CONTROL OUTPUT [PROVEN] =====================
  int servoAngle = CENTER_ANGLE;
  int motorSpeed = MOTOR_SPEED;
  bool frontAvoiding = false;

  if (mode == MODE_PIXY && pixyGood) {
    servoCurrentAngle += constrain(targetPixyAngle - servoCurrentAngle, -SERVO_SLEW_DEG_PER_STEP, SERVO_SLEW_DEG_PER_STEP);
    if (frontDist > 0 && frontDist <= PIXY_URGENT_CM) {                   // [FIX 6]
      targetPixyAngle = (targetPixyAngle > CENTER_ANGLE) ? MAX_SERVO_ANGLE : MIN_SERVO_ANGLE;
      servoCurrentAngle = targetPixyAngle;
    }
    // [FIX 9 + FIX 16, PROVEN authority] near a wall on the side of the turn, the command
    // may not go past 90 + (target-90)*room.  It only CLAMPS: the fed-back servo is never
    // rescaled, so the steady state equals the 09-06 build's.
    if (targetPixyAngle != CENTER_ANGLE) {
      bool turnLeft = (targetPixyAngle > CENTER_ANGLE);
      float sideForTurn = turnLeft ? lpfLeft : lpfRight;
      if (sideForTurn < WALL_LIMIT_CM) {
        float room = (sideForTurn - (float)WALL_VETO_CM) / (float)(WALL_LIMIT_CM - WALL_VETO_CM);
        room = constrain(room, 0.0, 1.0);
        float lim = CENTER_ANGLE + (int)((targetPixyAngle - CENTER_ANGLE) * room);
        if (turnLeft && servoCurrentAngle > lim) servoCurrentAngle = lim;
        if (!turnLeft && servoCurrentAngle < lim) servoCurrentAngle = lim;
      }
    }
    servoAngle = (int)servoCurrentAngle;
    if (frontDist > 0 && frontDist <= FRONT_AVOID_CM) motorSpeed = MOTOR_SPEED_AVOID;
  } else if (frontDist > 0 && frontDist <= FRONT_AVOID_CM) {
    frontAvoiding = true;
    motorSpeed = MOTOR_SPEED_AVOID;
    if (!wasAvoiding) { avoidStartYaw = yaw; uturnHoldUntil = 0; }
    float turned = wrap180(yaw - avoidStartYaw);
    float urg = (float)(FRONT_AVOID_CM - frontDist) / (float)(FRONT_AVOID_CM - AVOID_FULL_CM);
    urg = constrain(urg, AVOID_MIN_URGENCY, 1.0);
    // [PROVEN] the open side is the longer one.  [v20] A tie (a pillar or a lost echo
    // making both sides equal) no longer defaults right: it takes the turn the lot's side
    // says every corner of this round turns (wall on the left = clockwise = right turns).
    // [DIR] at the 48 cm trigger both 40-deg sonars see the same front wall and L-R flips
    // frame to frame.  Only a clear difference picks the side; otherwise the corner turns
    // the way this round turns (the lot side, then the corners already driven).
    float diff = lpfLeft - lpfRight;
    if (fabs(diff) > DIR_TRUST_CM) avoidLeft = (diff > 0);
    else avoidLeft = (dirVote > 0);
    bool goLeft = avoidLeft;
    if (fabs(turned) >= UTURN_LIMIT_DEG && uturnHoldUntil == 0) uturnHoldUntil = t + UTURN_STRAIGHT_MS;
    if (uturnHoldUntil != 0 && t < uturnHoldUntil) {
      servoAngle = CENTER_ANGLE;
    } else {
      if (uturnHoldUntil != 0) { uturnHoldUntil = 0; avoidStartYaw = yaw; }
      servoAngle = goLeft ? CENTER_ANGLE + (int)((MAX_SERVO_ANGLE - CENTER_ANGLE) * urg)
                          : CENTER_ANGLE - (int)((CENTER_ANGLE - MIN_SERVO_ANGLE) * urg);
    }
    lastError = 0;
    eintegral = 0;
  } else {
    float error = lpfLeft - lpfRight;
    // [MAP] laps 2-3: shift the balance point toward the side the mapped pillar needs,
    // until the camera engages it (then the proven ladder decides) or BIAS_MS runs out.
    if (biasSig != 0 && !biasUsed && t < biasUntil && fabs(absTurn - 90.0 * quad) < BIAS_ALIGN_DEG) {
      error -= (biasSig == SIG_RED) ? BIAS_CM : -BIAS_CM;   // RED: ride right (L > R)
    }
    eintegral += error;
    eintegral = constrain(eintegral, -I_MAX, I_MAX);
    float derivative = error - lastError;
    lastError = error;
    float output = KP * error + KI * eintegral + KD * derivative;
    servoAngle = CENTER_ANGLE + output;
    if (SCAN_AMPLITUDE_DEG > 0 && !pixyGood && fabs(error) < SCAN_MAX_ERROR_CM) {
      float ph = (float)(t % SCAN_PERIOD_MS) / (float)SCAN_PERIOD_MS;
      servoAngle += (int)(SCAN_AMPLITUDE_DEG * sin(ph * 2.0 * PI));
    }
    // [MAP] laps 2-3, a straight that had no pillar in lap 1: run it faster while the
    // corner is still far; the proven 48 cm corner starts from the proven speed.
    if (quad >= 4 && secKnown && !secSeen[quad % 4] && !sawSign && frontDist > FAST_FRONT_CM
        && fabs(error) < 20.0)
      motorSpeed = MOTOR_SPEED_FAST;
  }

  servoAngle = constrain(servoAngle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  steeringServo.write(servoAngle);
  servoCurrentAngle = servoAngle;

  {                                                                       // [FIX 4]
    unsigned long nowMs = millis();
    bool slow = (motorSpeed > 0 && motorSpeed < MOTOR_SPEED);
    if (slow && !wasSlow) slowSinceMs = nowMs;
    if (slow) {
      unsigned long inSlow = nowMs - slowSinceMs;
      if (inSlow < KICK_MS || (inSlow % REKICK_MS) < KICK_MS) motorSpeed = MOTOR_KICK;
    }
    wasSlow = slow;
  }
  // [STUCK] the proven law never reverses: a nose-on contact at a corner ended the round
  if (frontDist > 0) { lastFrontCm = frontDist; lastFrontMs = t; }
  if (motorSpeed > 0 && lastFrontCm <= STUCK_FRONT_CM && t - lastFrontMs < 200) {
    if (stuckSinceMs == 0 || fabs(wrap180(yaw - stuckYaw)) > 3.0) { stuckSinceMs = t; stuckYaw = yaw; }
    else if (t - stuckSinceMs > STUCK_MS) {
      bool wantedLeft = (servoAngle > CENTER_ANGLE) || (servoAngle == CENTER_ANGLE && dirVote > 0);
      steeringServo.write(wantedLeft ? MIN_SERVO_ANGLE : MAX_SERVO_ANGLE);   // reversing: opposite lock
      runMotor(MOTOR_KICK, false); delay(KICK_MS);
      runMotor(STUCK_REV_PWM, false); delay(STUCK_REV_MS);
      runMotor(MOTOR_SPEED_STOP, true); delay(120);
      servoCurrentAngle = CENTER_ANGLE;
      stuckSinceMs = 0; lastFrontCm = 999; wasSlow = false; lastError = 0; eintegral = 0;
      return;
    }
  } else {
    stuckSinceMs = 0;
  }
  runMotor(motorSpeed, true);
  wasAvoiding = frontAvoiding;

#if DEBUG_OUTPUT
  Serial.print(lpfLeft); Serial.print(F("\t")); Serial.print(lpfRight); Serial.print(F("\t"));
  Serial.print(frontDist); Serial.print(F("\t")); Serial.print(servoAngle); Serial.print(F("\t"));
  Serial.print(yaw); Serial.print(F("\t")); Serial.print(quad); Serial.print(F("\t"));
  Serial.println(frontAvoiding ? F("AVOID") : (mode == MODE_PID ? F("PID") : F("PIXY")));
#endif
}

// ===========================================================================
//   [PARK] APPROACH: outer lane, parallel, to the front-wall mark
// ===========================================================================
int lotMm() { return parkWallLeft ? LOT_LEFT_MM : LOT_RIGHT_MM; }
// front range (nose -> far wall) at which the rear bumper is MARK_MARGIN past the limiter
int markMm() { return lotMm() - (int)CAR_LEN_MM - MARK_MARGIN_MM; }

// signed right-turn heading the approach should hold to move toward the lane
float laneHeading(float wallMm, bool forward) {
  float e = wallMm - PARK_LANE_MM;                 // + = too far from the wall
  float psi = constrain(APPROACH_KL * e, -APPROACH_PSI_MAX, APPROACH_PSI_MAX);
  // toward a RIGHT wall driving forward = turn right (+); reversing, the nose must
  // point AWAY from the wall to move toward it
  float s = parkWallLeft ? -1.0 : 1.0;
  return forward ? s * psi : -s * psi;
}

void approachStep() {
  unsigned long t = millis();
  if (t - apStartMs > APPROACH_CAP_MS) { state = ST_DONE; return; }
  if (apF < 0 && !apChecked) {
    // the handover waits until the start section's pillars are passed, so the car can already
    // be level with or past the lot: measure where it is (stopped, parallel) before choosing
    apChecked = true;
    float fm0 = frontMeanMm();
    float wm0 = wallMeanMm();
    if (wm0 > 0.0) apWallMm = wm0;
    // lot on the right: a short front reading right after the corner is the upstream limiter's
    // tip in the cone edge, not the far wall - only a car that has driven the straight is past
    int firstStop = parkWallLeft ? CW_NEAR_CM * 10 : markMm();
    bool couldBePast = parkWallLeft || apAfterCornerMs > AP_PAST_MS;
    if (fm0 > 0.0 && fm0 < firstStop + 100 && couldBePast) { apPhase = 1; }
    else if (fm0 >= firstStop + 100) { apF = fm0; apFms = millis(); }   // a far wall beyond the mark
    apNear = 0; apAgree = 0; apClose = 0; apCand = -1.0;
    apStartMs = millis();
    return;
  }
  float front = getStableDistance(sonarFront);
  float wall = getStableDistance(parkWallLeft ? sonarLeft : sonarRight);
  // a limiter tip or a silent ping is not the wall: hold the last lane estimate.  A reading
  // that jumps more than 80 mm from it is a limiter tip, unless five in a row agree.
  if (wall > 12 && wall < 150) {
    float w = sideToWallMm(wall);
    if (fabs(w - apWallMm) < 80.0 || ++apJumps >= 5) { apWallMm = 0.6 * apWallMm + 0.4 * w; apJumps = 0; }
  }
  float rt = rightTurnSince(startYaw0);            // 0 = parallel to the lot
  bool fwd = (apPhase == 0);
  float err = rt - laneHeading(apWallMm, fwd);
  int servo = fwd ? CENTER_ANGLE + (int)(APPROACH_KH * err) : CENTER_ANGLE - (int)(APPROACH_KH * err);
  servo = constrain(servo, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  steeringServo.write(servo);
  int fmm = front > 0 ? (int)(front * 10.0) - FRONT_SETBACK_MM : 0;
  int mark = markMm();
  // track the far wall: a limiter-tip echo reads ~900 mm short of the predicted wall and is
  // rejected, so it can never stop the car early
  bool wallEcho = false;
  if (apPhase == 0 && fmm > 0) {
    // seed on two consecutive readings that agree and lie beyond the mark (a limiter tip in the
    // cone edge reads shorter); later, four agreeing readings the tracker rejected re-seed it
    int seedMin = (parkWallLeft ? CW_NEAR_CM * 10 : markMm()) + 100;
    bool agrees = (apCand > 0 && fabs(fmm - apCand) < 100.0);
    apAgree = agrees ? apAgree + 1 : 0;
    apCand = fmm;
    if (apF < 0) {
      if (apAgree >= 1 && fmm >= seedMin) { apF = fmm; apFms = t; apAgree = 0; }
    } else {
      float dt = (float)(t - apFms);
      float pred = apF - apV * dt;
      if (fabs(fmm - pred) < 150.0 + 0.2 * dt) {
        if (dt > 20.0) apV = constrain(0.8 * apV + 0.2 * (apF - fmm) / dt, 0.2, 1.5);
        apF = fmm; apFms = t; wallEcho = true; apAgree = 0;
      } else if (apAgree >= 3 && fmm > 600) {
        apF = fmm; apFms = t; apAgree = 0;   // lost the wall (a tip is never heard beyond ~500 mm)
      }
    }
  }
  if (apPhase == 0) {
    // lot on the right (CCW): the lot is at the END of this straight, run on to the mark.
    // lot on the left (CW): the lot is right after the corner, run on to CW_NEAR, then reverse.
    int stopAt = parkWallLeft ? CW_NEAR_CM * 10 : mark;
    int pwm = APPROACH_PWM;
    if (t - apStartMs < KICK_MS) pwm = MOTOR_KICK;
    runMotor(pwm, true);
    // hard backstop: at 250 mm a limiter tip is outside the cone, so two readings this close are
    // the wall whatever the tracker says - reverse onto the mark
    apClose = (fmm > 0 && fmm <= 250) ? apClose + 1 : 0;
    if (apClose >= 2) {
      runMotor(MOTOR_SPEED_STOP, true);
      delay(400);
      apPhase = 1; apStartMs = millis(); apNear = 0;
      return;
    }
    // backstop the heading gate cannot block: a wall reading well past the stop
    if (wallEcho && ((fmm <= stopAt + STOP_LEAD_MM && fabs(rt) < 12.0) || fmm <= stopAt - 120)) {
      runMotor(MOTOR_SPEED_STOP, true);
      delay(400);
      if (parkWallLeft) { apPhase = 1; apStartMs = millis(); }
      else state = ST_PARK;
    }
  } else {
    int pwm = APPROACH_PWM;
    if (t - apStartMs < KICK_MS) pwm = MOTOR_KICK;
    runMotor(pwm, false);
    apNear = (fmm > 0 && fmm >= mark - STOP_LEAD_MM) ? apNear + 1 : 0;
    if (apNear >= 2 && (fabs(rt) < 12.0 || fmm >= mark + 120)) {
      runMotor(MOTOR_SPEED_STOP, true);
      delay(400);
      state = ST_PARK;
    }
  }
#if DEBUG_OUTPUT
  Serial.print(F("APPROACH ")); Serial.print(apPhase); Serial.print(F(" F ")); Serial.print(fmm);
  Serial.print(F(" wall ")); Serial.print(apWallMm); Serial.print(F(" rt ")); Serial.println(rt);
#endif
}

// ===========================================================================
//   [PARK] THE MANOEUVRE: IMU-closed arcs and measured steps, each checked
//   against the lot (two limiters + the wall) before it is driven
// ===========================================================================
float frontMeanMm() {
  int sum = 0, n = 0;
  for (int i = 0; i < 5; i++) {
    int v = sonarFront.ping_cm();
    if (v > 0) { sum += v; n++; }
    delay(30);
  }
  return n >= 3 ? (float)sum * 10.0 / n - FRONT_SETBACK_MM : 0.0;
}

float wallMeanMm() {
  int sum = 0, n = 0;
  for (int i = 0; i < 5; i++) {
    int v = (parkWallLeft ? sonarLeft : sonarRight).ping_cm();
    if (v > 12 && v < 150) { sum += v; n++; }
    delay(30);
  }
  return n >= 3 ? sideToWallMm((float)sum / n) : -1.0;
}

float sideToWallMm(float cm) {
  return parkWallLeft ? SIDE_A_L * cm + SIDE_B_L : SIDE_A_R * cm + SIDE_B_R;
}

// nose-away-from-the-wall heading, degrees, relative to the start heading
float parkHeading() { return (parkWallLeft ? 1.0 : -1.0) * rightTurnSince(startYaw0); }

// the lot frame: rear axle x along the drive (0 = car centred in the lot), y from the
// lot's centre line (+ away from the wall), heading h (deg, + nose away from the wall)
float lotFar() { return (CAR_NOSE_MM - (CAR_LEN_MM - CAR_NOSE_MM)) / 2.0 + LOT_LEN_MM / 2.0; }

void arcUpdate(float rotDeg, float R, float icr) {
  float h1 = bh * DEG2RAD, h2 = (bh + rotDeg) * DEG2RAD;
  bx += icr * R * (sin(h2) - sin(h1));
  by -= icr * R * (cos(h2) - cos(h1));
  bh += rotDeg;
}

float segLimClear(float px, float py, float x0, float x1) {
  float dx = (px < x0) ? x0 - px : (px > x1 ? px - x1 : 0.0);
  float dy = (py > LOT_DEPTH_MM / 2.0) ? py - LOT_DEPTH_MM / 2.0 : 0.0;
  return sqrt(dx * dx + dy * dy);
}

// smallest clearance (mm) of the car polygon to both limiters and the wall; <= 0 = touch
float bayClear(float x, float y, float hdeg) {
  float c = cos(hdeg * DEG2RAD), s = sin(hdeg * DEG2RAD);
  float tail = CAR_LEN_MM - CAR_NOSE_MM, hw = CAR_WID_MM / 2.0;
  float cx[4] = {CAR_NOSE_MM, CAR_NOSE_MM, -tail, -tail};
  float cy[4] = {hw, -hw, -hw, hw};
  float far0 = lotFar(), far1 = far0 + LIM_T_MM;
  float near1 = far0 - LOT_LEN_MM, near0 = near1 - LIM_T_MM;
  float best = 1e6;
  for (int i = 0; i < 4; i++) {
    float ax = x + cx[i] * c - cy[i] * s, ay = y + cx[i] * s + cy[i] * c;
    int j = (i + 1) & 3;
    float bxx = x + cx[j] * c - cy[j] * s, byy = y + cx[j] * s + cy[j] * c;
    for (int k2 = 0; k2 < 6; k2++) {
      float px = ax + (bxx - ax) * k2 / 6.0, py = ay + (byy - ay) * k2 / 6.0;
      float wall = py + LOT_DEPTH_MM / 2.0;
      if (wall < best) best = wall;
      float a = segLimClear(px, py, far0, far1), b = segLimClear(px, py, near0, near1);
      if (a < best) best = a;
      if (b < best) best = b;
    }
  }
  return best;
}

// largest rotation (0.5 deg grid) of an arc (R, icr, sign) that keeps >= margin (or does
// not get closer than it already is)
float maxLeg(float R, float icr, float signRot, float margin, float cap) {
  float need = fmin2(margin, bayClear(bx, by, bh) - 1.0);
  float best = 0.0;
  float x0 = bx, y0 = by, h0 = bh;
  for (int i = 1; 0.5 * i <= cap; i++) {
    arcUpdate(signRot * 0.5 * i, R, icr);
    float cl = bayClear(bx, by, bh);
    bx = x0; by = y0; bh = h0;
    if (cl < need) break;
    best = 0.5 * i;
  }
  return best;
}

// an arc at `lock` closed on the park heading at `target`, cut early by the turn rate x
// PARK_COAST_MS; kick only if the car has not started.  Returns the measured rotation.
float parkArcTo(int lock, bool forward, float target) {
  float h0 = parkHeading();
  bool up = target > h0;
  steeringServo.write(lock);
  servoAtLock = true;
  delay(PARK_PAUSE_MS);
  runMotor(PARK_ARC_PWM, forward);
  unsigned long t0 = millis(), tp = t0;
  float hP = h0, rate = 0.0;
  bool kicked = false;
  while (millis() - t0 < PARK_ARC_CAP_MS) {
    float h = parkHeading();
    unsigned long now = millis();
    if (now > tp) { rate = 0.6 * rate + 0.4 * (h - hP) / (float)(now - tp); tp = now; hP = h; }
    if (up ? (h + rate * PARK_COAST_MS >= target) : (h + rate * PARK_COAST_MS <= target)) break;
    if (!kicked && now - t0 > PARK_STALL_MS && fabs(h - h0) < 0.3) {
      runMotor(PARK_KICK_PWM, forward);
      delay(PARK_KICK_MS);
      runMotor(PARK_ARC_PWM, forward);
      kicked = true;
    }
    delay(8);
  }
  runMotor(0, true);
  float hl = parkHeading();
  for (int i = 0; i < 12; i++) {                   // the coast still turns the car: wait for rest
    delay(70);
    float hn = parkHeading();
    if (fabs(hn - hl) < 0.15) break;
    hl = hn;
  }
  delay(PARK_PAUSE_MS);
  return parkHeading() - h0;
}

void parkStep(bool forward) {
  steeringServo.write(CENTER_ANGLE);
  if (servoAtLock) { delay(PARK_PAUSE_MS); servoAtLock = false; }   // the step must be straight
  runMotor(PARK_KICK_PWM, forward);
  delay(STEP_KICK_MS);
  runMotor(PARK_NUDGE_PWM, forward);
  delay(STEP_KICK_MS);
  runMotor(0, true);
  delay(STEP_SETTLE_MS);
}

void parkRun() {
  int toWall = parkWallLeft ? PARK_LOCK_L : PARK_LOCK_R;
  int offWall = parkWallLeft ? PARK_LOCK_R : PARK_LOCK_L;
  int lot = lotMm(), mark = markMm();
  runMotor(0, true);
  steeringServo.write(CENTER_ANGLE);
  delay(PARK_PAUSE_MS);

  // B: onto the mark in steps, then measure the reverse step (4 out, 4 back)
  for (int i = 0; i < 10; i++) {
    float f = frontMeanMm();
    if (f <= 0.0) break;
    if (f > mark + MARK_TOL_MM) parkStep(true);
    else if (f < mark - MARK_TOL_MM) parkStep(false);
    else break;
  }
  float stepMm = 45.0;
  {
    float f0 = frontMeanMm();
    for (int i = 0; i < 4; i++) parkStep(false);
    float mv = (frontMeanMm() - f0) / 4.0;
    if (f0 > 0.0 && mv > 5.0 && mv < 150.0) stepMm = mv;
    for (int i = 0; i < 4; i++) parkStep(true);
  }

  // the pose in the lot frame
  float fm = frontMeanMm();
  bh = parkHeading();
  bx = (fm > 0.0) ? lotFar() + LIM_T_MM - CAR_NOSE_MM + (float)lot - fm
                  : lotFar() + LIM_T_MM - CAR_NOSE_MM + (float)(lot - mark);
  float yw = wallMeanMm();
  by = (yw > 0.0 ? yw : apWallMm) - LOT_DEPTH_MM / 2.0;
#if DEBUG_OUTPUT
  Serial.print(F("PARK pose x ")); Serial.print(bx); Serial.print(F(" y ")); Serial.print(by);
  Serial.print(F(" h ")); Serial.print(bh); Serial.print(F(" step ")); Serial.println(stepMm);
#endif

  // the D-end target: the nominal entry arc + straight from the nominal lane and mark
  float the = PARK_ENTRY_DEG * DEG2RAD;
  float tx = (lotFar() + LIM_T_MM - CAR_NOSE_MM + CAR_LEN_MM + MARK_MARGIN_MM)
             - R_PARK_MM * sin(the) - PARK_DIAG_MM * cos(the) + PARK_ALONG_BIAS;
  float ty = (PARK_LANE_MM - LOT_DEPTH_MM / 2.0) - R_PARK_MM * (1.0 - cos(the)) - PARK_DIAG_MM * sin(the)
             - PARK_DEPTH_BIAS;
  // solve, from where the car IS, for the entry angle whose arc + straight lands there
  float thE = PARK_ENTRY_DEG;
  {
    float h0 = bh * DEG2RAD, best = 1e9;
    for (int td = 36; td <= 64; td++) {
      float tt = td * DEG2RAD;
      float xc = bx - R_PARK_MM * (sin(tt) - sin(h0)), yc = by - R_PARK_MM * (cos(h0) - cos(tt));
      float d = (yc - ty) / sin(tt);
      if (d < 0.0) continue;
      float e = fabs(xc - d * cos(tt) - tx);
      if (e < best) { best = e; thE = (float)td; }
    }
  }

  // C: reverse, wheels to the wall, to the entry angle
  {
    float a = maxLeg(R_PARK_MM, -1.0, 1.0, PARK_MARGIN_MM, thE - bh);
    if (a < thE - bh) a -= PARK_SLACK_DEG;
    if (a > 0.5) arcUpdate(parkArcTo(toWall, false, bh + a), R_PARK_MM, -1.0);
  }

  // D: straight reverse in measured steps, each checked before it is driven
  {
    float hr = bh * DEG2RAD, cs = cos(hr), sn = sin(hr);
    float need = (sn > 0.2) ? (by - ty) / sn : 0.0;
    float moved = 0.0;
    for (int i = 0; i < 10 && moved + 0.5 * stepMm < need; i++) {
      float tt = moved + stepMm;
      if (bayClear(bx - tt * cs, by - tt * sn, bh) < PARK_MARGIN_MM) break;
      parkStep(false);
      moved += stepMm;
    }
    bx -= moved * cs;
    by -= moved * sn;
    bh = parkHeading();
  }

  // E: equal reverse/forward pairs turn the car to parallel almost in place; the leg with
  // more room goes first
  for (int cyc = 0; cyc < 14 && bh > PARK_SQUARE_DEG; cyc++) {
    float want = fmin2(bh, PARK_SHUF_DEG);
    bool fw = maxLeg(R_PARK_MM, -1.0, -1.0, PARK_MARGIN_MM, want) > maxLeg(R_PARK_MM, 1.0, -1.0, PARK_MARGIN_MM, want);
    float turned = 0.0;
    for (int leg = 0; leg < 2 && bh > PARK_SQUARE_DEG; leg++, fw = !fw) {
      float cap = fmin2(bh, leg ? fmax2(turned, 1.0) : want);
      float icr = fw ? -1.0 : 1.0;
      float g = maxLeg(R_PARK_MM, icr, -1.0, PARK_MARGIN_MM, cap);
      if (g < cap) g = (g > 2.0 * PARK_SLACK_DEG) ? g - PARK_SLACK_DEG : 0.0;
      if (g > 0.5) {
        float h0 = bh;
        arcUpdate(parkArcTo(fw ? toWall : offWall, fw, bh - g), R_PARK_MM, icr);
        turned += h0 - bh;
      }
    }
    if (turned < 0.5) break;
  }

  // F: centre along the lot on the downstream limiter ahead
  float trim = (LOT_LEN_MM - CAR_LEN_MM) / 2.0;
  for (int i = 0; i < 4 && fabs(parkHeading()) < 8.0; i++) {
    float f = frontMeanMm();
    if (f <= 0.0 || f > 250.0) break;
    if (f > trim + 25) parkStep(true);
    else if (f < trim - 20) parkStep(false);
    else break;
  }
  runMotor(0, true);
  steeringServo.write(CENTER_ANGLE);
#if DEBUG_OUTPUT
  Serial.println(F("PARKED"));
#endif
}

// ===========================================================================
void loop() {
  switch (state) {
    case ST_WAIT:
      if (waitStart()) { parkPickSide(); state = ST_EXIT; }
      break;
    case ST_EXIT:
      if (startTick()) { state = ST_LAPS; lastQuad = quad = 0; }
      break;
    case ST_LAPS:
      lapStep();
      break;
    case ST_APPROACH:
      approachStep();
      break;
    case ST_PARK:
      parkRun();
      state = ST_DONE;
      break;
    default:
      runMotor(0, true);
      steeringServo.write(CENTER_ANGLE);
      delay(20);
      break;
  }
}
