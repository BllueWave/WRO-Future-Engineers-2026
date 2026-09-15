// Blue Wave - WRO Future Engineers 2026 - OPEN - Arduino Uno - v20
// ===========================================================================
// BASE: open_kuwait (won the national round: 3 laps in ~23 s at PWM 30).
// Its L-R balance law and 48 cm corner are unchanged.  v20 adds only:
//   [MAP]   lap 1 is driven exactly like kuwait and RECORDS, per straight, how calm
//           the steering was.  In laps 2-3 a straight that was calm in lap 1 is run
//           faster while the corner is still far (front > FAST_FRONT_CM); the car is
//           back at the proven PWM 30 well before the proven 48 cm corner.  Nothing
//           is dead-reckoned: no distance, no speed model, no early turn.
//   [DIR]   at the 48 cm trigger both 40-deg sonars see the SAME front wall, so kuwait's
//           per-frame L-R (tie = right) is a coin toss and can flip mid-corner.  The turn is
//           decided once, from the L-R of the last ~10 loops before the trigger (the inner
//           side opens first); a tie takes the way this round's corners have turned.
//   [LAPS]  twelve counted corners, then on to the middle of the start straight.
//   [START] waits for a change on the A2 switch (rule 9.10), or AUTO_START_MS.
// Why open_final hit walls: it used the 40-deg side sonars as 90-deg distances, fired
// corners on side SILENCE, and ran 2.2x the proven speed on a guessed brake model.
// None of that is here.
// ===========================================================================
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <NewPing.h>
#include <Pixy2.h>
#include <math.h>

#define DEBUG_OUTPUT 1

const int TRIG_LEFT = 4,  ECHO_LEFT = 5;
const int TRIG_RIGHT = 2, ECHO_RIGHT = 9;
const int TRIG_FRONT = 6, ECHO_FRONT = 7;
const int SERVO_PIN = 10;
const int PWM_PIN = 3, DIR_PIN = 8;
const int START_PIN = A2;
#define PRACTICE 0                     // 1: also start 3 s after power-up.  0 = A2 switch only
const unsigned long AUTO_START_MS = PRACTICE ? 3000 : 0;

// ===== [PROVEN] open_kuwait =====
const int MOTOR_SPEED = 30;
const int MOTOR_SPEED_AVOID = 25;
const int MOTOR_SPEED_STOP = 0;
const int FRONT_AVOID_CM = 48;
const int DEFAULT_SIDE_CM = 60;
const int MOTOR_KICK = 55;
const unsigned long KICK_MS = 70, REKICK_MS = 500;
const float UTURN_LIMIT_DEG = 100.0;
const int AVOID_FULL_CM = 18;
const float AVOID_MIN_URGENCY = 0.25;
const float KP = 0.6, KD = 0.05, KI = 0.0;
const float I_MAX = 80.0;
const int CENTER_ANGLE = 90, MIN_SERVO_ANGLE = 30, MAX_SERVO_ANGLE = 160;
#define MAX_DISTANCE 400
const float ALPHA = 0.9;

// ===== [MAP] =====
const int MOTOR_SPEED_FAST = 38;       // calm mapped straight, laps 2-3
const int FAST_FRONT_CM = 150;         // only while the corner is this far
const float CALM_ERR_CM = 25.0;        // lap 1: a straight is calm if |L-R| stayed under this mid-straight
const float FAST_ERR_CM = 18.0;        // laps 2-3: and only while centred now
const int FINISH_FRONT_CM = 150;       // stop with the nose mid start-straight (far wall 150 cm)
const unsigned long FINISH_MS = 700;  // backup only: from corner 12's count this stays inside the start section

Adafruit_BNO055 bno = Adafruit_BNO055(55);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
Pixy2 pixy;
Servo steeringServo;

float yaw = 0, prevYaw = 0, turnAccum = 0;
int quad = 0, lastQuad = -1;
float lastError = 0, eintegral = 0;
unsigned long slowSinceMs = 0;
bool wasSlow = false, wasAvoiding = false;
float avoidStartYaw = 0;
bool lpfInit = false;
float lpfLeft = 0, lpfRight = 0;
bool started = false, yawSeeded = false;
int firstTurnDir = 0;                  // majority of the corners so far: +1 left, -1 right, 0 unknown
int firstVote = 0;                     // +1 per counted corner turned left, -1 right
bool avoidLeft = false;                // this avoid's direction, decided at its start
float sideMem = 0;                     // L-R over the last ~10 PID loops (before the trigger)
unsigned long finishMs = 0;
bool finishSawFar = false;
uint8_t finishFar = 0, finishNear = 0;   // consecutive samples: one echo must not latch or stop
float secMaxErr[4] = {0, 0, 0, 0};
bool secSeen[4] = {false, false, false, false};
unsigned long sectionStartMs = 0;

float getStableDistance(NewPing &sonar) {
  delay(3);
  int d = sonar.ping_cm();
  if (d == 0 || d > MAX_DISTANCE) return -1;
  return d;
}

void runMotor(int speed, bool forward) {
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  analogWrite(PWM_PIN, speed);
}

float wrap180(float a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

void signalServo(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    steeringServo.write(CENTER_ANGLE + 25); delay(180);
    steeringServo.write(CENTER_ANGLE - 25); delay(180);
  }
  steeringServo.write(CENTER_ANGLE);
}

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
  Wire.setWireTimeout(25000, true);
#endif
  bool imu = false;
  for (int i = 0; i < 3 && !imu; i++) { imu = bno.begin(); if (!imu) delay(300); }
  if (!imu) {
    for (;;) signalServo(2);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  pixy.init();
  signalServo(1);
}

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
  delay(5);
  return false;
}

void loop() {
  if (!started) {
    if (!waitStart()) return;
    started = true;
    sectionStartMs = millis();
  }

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

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  yaw = orientationData.orientation.x;
  // a failed I2C read returns exactly 0.0: never a real jump of more than 20 deg in one loop
  if (yawSeeded && yaw == 0.0 && fabs(wrap180(prevYaw)) > 20.0) yaw = prevYaw;
  if (!yawSeeded) { prevYaw = yaw; yawSeeded = true; }
  float delta = wrap180(yaw - prevYaw);
  turnAccum += delta;
  prevYaw = yaw;
  if (fabs(turnAccum) >= 90.0 * quad + 70.0) quad++;    // [LAPS] a corner counts at 70 of 90 deg

  // [LAPS] three laps = twelve corners (each counted at 70 of its 90 deg, so heading drift
  // cannot add or drop one).  kuwait's 360-deg closes lost the overshoot and sometimes closed
  // lap 3 only at corner 13.  After corner 12 the car drives on with the same law: once the
  // front has looked down the start straight, stop when the far wall is FINISH_FRONT_CM ahead
  // (nose mid-straight), or after FINISH_MS.
  if (quad >= 12 && finishMs == 0) finishMs = millis();
  if (finishMs != 0) {
    finishFar = (frontDist > FINISH_FRONT_CM + 30) ? finishFar + 1 : 0;
    if (finishFar >= 2) finishSawFar = true;
    finishNear = (finishSawFar && frontDist > 0 && frontDist <= FINISH_FRONT_CM) ? finishNear + 1 : 0;
  }
  if (finishMs != 0 && (finishNear >= 2 || millis() - finishMs > FINISH_MS)) {
    runMotor(MOTOR_SPEED_STOP, true);
    steeringServo.write(CENTER_ANGLE);
    Serial.println(F("3 laps completed"));
    while (1) ;
  }

  unsigned long t = millis();
  if (quad != lastQuad) {
    lastQuad = quad;
    sectionStartMs = t;
    if (quad > 0) firstVote += avoidLeft ? 1 : -1;               // [DIR] the corner just counted
    firstTurnDir = (firstVote > 0) ? 1 : (firstVote < 0 ? -1 : 0);
  }

  pixy.ccc.getBlocks();                  // [PROVEN] same loop timing as the tested build

  int servoAngle = CENTER_ANGLE;
  int motorSpeed = MOTOR_SPEED;
  bool frontAvoiding = false;

  if (frontDist > 0 && frontDist <= FRONT_AVOID_CM) {
    frontAvoiding = true;
    motorSpeed = MOTOR_SPEED_AVOID;
    if (!wasAvoiding) avoidStartYaw = yaw;
    float turned = wrap180(yaw - avoidStartYaw);
    float urg = (float)(FRONT_AVOID_CM - frontDist) / (float)(FRONT_AVOID_CM - AVOID_FULL_CM);
    urg = constrain(urg, AVOID_MIN_URGENCY, 1.0);
    // [DIR] [PROVEN] the longer side, every frame.  A tie (both 40-deg sonars on the same
    // front wall) takes the way this round's corners have turned so far; before the first
    // corner has counted, the side that opened in the last ~10 loops before the trigger.
    float diff = lpfLeft - lpfRight;
    if (fabs(diff) > 3.0) avoidLeft = (diff > 0);
    else if (firstTurnDir != 0) avoidLeft = (firstTurnDir > 0);
    else if (fabs(sideMem) >= 4.0) avoidLeft = (sideMem > 0);
    else avoidLeft = false;                                     // [PROVEN] kuwait's default
    bool goLeft = avoidLeft;
    if (fabs(turned) >= UTURN_LIMIT_DEG) servoAngle = CENTER_ANGLE;
    else if (goLeft) servoAngle = CENTER_ANGLE + (int)((MAX_SERVO_ANGLE - CENTER_ANGLE) * urg);
    else servoAngle = CENTER_ANGLE - (int)((CENTER_ANGLE - MIN_SERVO_ANGLE) * urg);
    lastError = 0;
    eintegral = 0;
  } else {
    float error = lpfLeft - lpfRight;
    sideMem = 0.9 * sideMem + 0.1 * error;
    eintegral += error;
    eintegral = constrain(eintegral, -I_MAX, I_MAX);
    float derivative = error - lastError;
    lastError = error;
    servoAngle = CENTER_ANGLE + KP * error + KI * eintegral + KD * derivative;

    int sec = quad % 4;
    bool midStraight = (t - sectionStartMs > 700) && frontDist > FAST_FRONT_CM;
    if (quad < 4) {
      // [MAP] lap 1: record how calm this straight is in its middle
      if (midStraight) {
        secSeen[sec] = true;
        if (fabs(error) > secMaxErr[sec]) secMaxErr[sec] = fabs(error);
      }
    } else if (quad < 12 && midStraight && secSeen[sec] && secMaxErr[sec] < CALM_ERR_CM && fabs(error) < FAST_ERR_CM) {
      motorSpeed = MOTOR_SPEED_FAST;                              // [MAP] laps 2-3
    }
  }

  servoAngle = constrain(servoAngle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  steeringServo.write(servoAngle);

  {
    unsigned long nowMs = millis();
    bool slow = (motorSpeed > 0 && motorSpeed < MOTOR_SPEED);
    if (slow && !wasSlow) slowSinceMs = nowMs;
    if (slow) {
      unsigned long inSlow = nowMs - slowSinceMs;
      if (inSlow < KICK_MS || (inSlow % REKICK_MS) < KICK_MS) motorSpeed = MOTOR_KICK;
    }
    wasSlow = slow;
  }
  runMotor(motorSpeed, true);
  wasAvoiding = frontAvoiding;

#if DEBUG_OUTPUT
  Serial.print(lpfLeft); Serial.print(F("\t")); Serial.print(lpfRight); Serial.print(F("\t"));
  Serial.print(frontDist); Serial.print(F("\t")); Serial.print(servoAngle); Serial.print(F("\t"));
  Serial.print(yaw); Serial.print(F("\t")); Serial.print(quad); Serial.print(F("\t"));
  Serial.println(frontAvoiding ? F("AVOID") : (motorSpeed == MOTOR_SPEED_FAST ? F("FAST") : F("PID")));
#endif
}
