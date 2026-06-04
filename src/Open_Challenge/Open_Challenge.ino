#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <NewPing.h>
#include <Pixy2.h>
#include <math.h>

// ===== Debug Toggle =====
#define DEBUG_OUTPUT true

// ===========================================================================
// ===== Pins =====

// ===== Ultrasonic =====
const int TRIG_LEFT = 4;
const int ECHO_LEFT = 5;

const int TRIG_RIGHT = 2;
const int ECHO_RIGHT = 9;

const int TRIG_FRONT = 6;  // Front ultrasonic TRIG
const int ECHO_FRONT = 7;  // Front ultrasonic ECHO

const int SERVO_PIN = 10;

// ===== Motor Driver (Cytron MD13S) =====
const int PWM_PIN = 3;
const int DIR_PIN = 8;

// ===========================================================================
// ===== Speed Settings =====

const int MOTOR_SPEED = 30;
const int MOTOR_SPEED_AVOID = 20;      // Keep moving slowly when avoiding a front obstacle
const int MOTOR_SPEED_STOP = 0;

// Do NOT stop when the front sensor sees a wall.
// Below this distance, the robot turns toward the side with more space.
const int FRONT_AVOID_CM = 28;

// If one/both side sensors return no echo, reuse the last filtered value.
// If there is no previous value yet, start from this safe default.
const int DEFAULT_SIDE_CM = 60;

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

const int GREEN_SOFT_LEFT = 140;
const int GREEN_MED_LEFT = 140;
const int GREEN_HARD_LEFT = 140;

const int RED_SOFT_RIGHT = 40;
const int RED_MED_RIGHT = 40;
const int RED_HARD_RIGHT = 40;

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

const int PIXY_MIN_AREA = 200;
const int PIXY_MIN_X = 20;
const int PIXY_MAX_X = 300;

float servoCurrentAngle = CENTER_ANGLE;
const float SERVO_SLEW_DEG_PER_STEP = 6.0;

// ===== Distance Filter =====
bool lpfInit = false;
float lpfLeft = 0;
float lpfRight = 0;
const float ALPHA = 0.9;

Servo steeringServo;

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
    Serial.println("BNO055 not detected. Check wiring!");
    while (1)
      ;
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  pixy.init();

  if (DEBUG_OUTPUT) {
    Serial.println("=== FAST PID + Managed Pixy Switching ===");
    Serial.println("L(cm)\tR(cm)\tF(cm)\tErr\tServo\tYaw\tLaps\tMode");
  }
}

// ===========================================================================

void loop() {
  float leftDist = getStableDistance(sonarLeft);
  float rightDist = getStableDistance(sonarRight);
  float frontDist = getStableDistance(sonarFront);

  // IMPORTANT:
  // Do not stop here when the front sensor sees an obstacle.
  // Front-obstacle handling is done later in the control section,
  // after Pixy processing, so red/green signs still get priority.

  // If one side sensor misses an echo, do NOT stop the robot.
  // Use the other side, the last filtered value, or a safe default.
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

    Serial.println("3 laps completed");

    while (1)
      ;
  }

  // =======================================================
  // ================= PIXY PROCESSING =====================
  // =======================================================

  int blocks = pixy.ccc.getBlocks();

  bool pixyGood = false;
  int targetPixyAngle = CENTER_ANGLE;

  if (blocks > 0) {
    auto &blk = pixy.ccc.blocks[0];

    int sig = blk.m_signature;
    int x = blk.m_x;
    int w = blk.m_width;
    int h = blk.m_height;

    int area = w * h;

    if (DEBUG_OUTPUT) {
      Serial.print("sig=");
      Serial.print(sig);

      Serial.print(" x=");
      Serial.print(x);

      Serial.print(" area=");
      Serial.println(area);
    }

    if (area >= PIXY_MIN_AREA && x >= PIXY_MIN_X && x <= PIXY_MAX_X) {
      pixyGood = true;

      // GREEN
      if (sig == 1) {
        if (x < 120)
          targetPixyAngle = GREEN_HARD_LEFT;
        else if (x < 170)
          targetPixyAngle = GREEN_MED_LEFT;
        else
          targetPixyAngle = GREEN_SOFT_LEFT;
      }

      // RED
      else if (sig == 2) {
        if (x > 200)
          targetPixyAngle = RED_HARD_RIGHT;
        else if (x > 150)
          targetPixyAngle = RED_MED_RIGHT;
        else
          targetPixyAngle = RED_SOFT_RIGHT;
      } else {
        pixyGood = false;
      }
    }
  }

  // =======================================================
  // ================= MODE MANAGEMENT =====================
  // =======================================================

  unsigned long t = millis();

  if (pixyGood) {
    pixySeenFrames++;
    pixyMissFrames = 0;

    if (mode == MODE_PID && pixySeenFrames >= ENTER_PIXY_FRAMES && (t - modeChangedAt) >= MODE_MIN_HOLD_MS) {
      mode = MODE_PIXY;
      modeChangedAt = t;

      if (DEBUG_OUTPUT)
        Serial.println("ENTER PIXY MODE");
    }
  } else {
    pixyMissFrames++;
    pixySeenFrames = 0;

    if (mode == MODE_PIXY && pixyMissFrames >= EXIT_PIXY_MISS_FRAMES && (t - modeChangedAt) >= MODE_MIN_HOLD_MS) {
      mode = MODE_PID;
      modeChangedAt = t;

      if (DEBUG_OUTPUT)
        Serial.println("EXIT PIXY MODE");
    }
  }

  // =======================================================
  // ================== CONTROL OUTPUT =====================
  // =======================================================

  int servoAngle = CENTER_ANGLE;
  int motorSpeed = MOTOR_SPEED;
  bool frontAvoiding = false;

  if (mode == MODE_PIXY && pixyGood) {
    // Red/green object handling has priority over the front ultrasonic.
    servoCurrentAngle += constrain(
      targetPixyAngle - servoCurrentAngle,
      -SERVO_SLEW_DEG_PER_STEP,
      SERVO_SLEW_DEG_PER_STEP);

    servoAngle = (int)servoCurrentAngle;

    // If the object is also close to the front ultrasonic,
    // keep Pixy steering but reduce speed instead of stopping.
    if (frontDist > 0 && frontDist <= FRONT_AVOID_CM)
      motorSpeed = MOTOR_SPEED_AVOID;
  } else if (frontDist > 0 && frontDist <= FRONT_AVOID_CM) {
    // Front wall/obstacle detected:
    // keep moving slowly and turn toward the side with more free space.
    frontAvoiding = true;
    motorSpeed = MOTOR_SPEED_AVOID;

    if (lpfLeft > lpfRight + 3) {
      servoAngle = MAX_SERVO_ANGLE;   // More space on left: steer left
    } else if (lpfRight > lpfLeft + 3) {
      servoAngle = MIN_SERVO_ANGLE;   // More space on right: steer right
    } else {
      servoAngle = MIN_SERVO_ANGLE;   // Tie/default: steer right
    }

    // Reset PID memory while avoiding, otherwise the old error can
    // make the robot oversteer after the obstacle is cleared.
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
  }

  servoAngle = constrain(
    servoAngle,
    MIN_SERVO_ANGLE,
    MAX_SERVO_ANGLE);

  steeringServo.write(servoAngle);
  servoCurrentAngle = servoAngle;

  // Never stop just because a wall/obstacle is detected.
  // The robot only stops when 3 laps are completed.
  runMotor(motorSpeed, true);

  // =======================================================
  // ================= DEBUG OUTPUT ========================
  // =======================================================

  if (DEBUG_OUTPUT) {
    Serial.print(lpfLeft);
    Serial.print("\t");

    Serial.print(lpfRight);
    Serial.print("\t");

    Serial.print(frontDist);
    Serial.print("\t");

    Serial.print(lastError);
    Serial.print("\t");

    Serial.print(servoAngle);
    Serial.print("\t");

    Serial.print(yaw);
    Serial.print("\t");

    Serial.print(laps);
    Serial.print("\t");

    if (frontAvoiding)
      Serial.println("AVOID");
    else
      Serial.println(mode == MODE_PID ? "PID" : "PIXY");
  }
}

// ===========================================================================
// ========================= FUNCTIONS =======================================
// ===========================================================================

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
