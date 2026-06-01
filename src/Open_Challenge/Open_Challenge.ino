#include <Servo.h>

// ===== Debug Toggle =====
#define DEBUG_OUTPUT true

// ===== Pins =====
const int TRIG_LEFT  = 4;
const int ECHO_LEFT  = 5;
const int TRIG_RIGHT = 2;
const int ECHO_RIGHT = 9;
const int SERVO_PIN  = A0;

// ===== Motor Driver (Cytron MD13S) =====
const int PWM_PIN = 3;   // PWM pin for motor speed
const int DIR_PIN = 8;   // Direction pin

// ===== Speed Settings =====
const int MOTOR_SPEED = 65;
const int TURN_SPEED  = 55;   // safety slowdown on sharp turns

// ===== PD Control Settings =====
const float KP = 0.1;
const float KD = 0.09;

// ===== Steering Angles =====
const int CENTER_ANGLE = 90;
const int MIN_SERVO_ANGLE = 40;
const int MAX_SERVO_ANGLE = 140;

// ===== Fine Tuning =====
const int   SERVO_TRIM     = 0;     // adjust if car drifts when going straight
const float ERROR_DEADBAND = 0.8;   // ignore tiny sensor noise
const int   HARD_TURN_DEG  = 28;    // if servo offset from center > this, slow down
const float MAX_VALID_CM   = 75.0;  // ignore corner-spike readings (track lane <= 100cm)
const float DERIV_ALPHA    = 0.45;  // derivative low-pass (lower = smoother)
const float SERVO_ALPHA    = 0.55;  // servo output low-pass (lower = smoother)

// ===== Globals =====
Servo steeringServo;
float lastError = 0;
float smoothedDeriv = 0;
float smoothedServo = CENTER_ANGLE;

void setup() {
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);  // Forward

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(CENTER_ANGLE + SERVO_TRIM);

  if (DEBUG_OUTPUT) {
    Serial.begin(115200);
    Serial.println("=== PD Steering (Anti-Zigzag) ===");
    Serial.println("Left(cm)\tRight(cm)\tError\tServo\tSpd");
  }
}

void loop() {
  float leftDist = getStableDistance(TRIG_LEFT, ECHO_LEFT);
  float rightDist = getStableDistance(TRIG_RIGHT, ECHO_RIGHT);

  // Fallback if one sensor fails
  if (leftDist < 0 && rightDist >= 0) leftDist = rightDist;
  if (rightDist < 0 && leftDist >= 0) rightDist = leftDist;
  if (leftDist < 0 && rightDist < 0) {
    runMotor(0, false);
    lastError = 0;
    smoothedDeriv = 0;
    if (DEBUG_OUTPUT) Serial.println("Both sensors failed — stopping");
    delay(100);
    return;
  }

  // Clamp corner-exit spikes (track lane is max 100cm, half = 50cm + margin)
  if (leftDist  > MAX_VALID_CM) leftDist  = MAX_VALID_CM;
  if (rightDist > MAX_VALID_CM) rightDist = MAX_VALID_CM;

  // PD Controller with smoothed derivative
  float error = leftDist - rightDist;
  if (fabs(error) < ERROR_DEADBAND) error = 0;

  float rawDeriv = error - lastError;
  smoothedDeriv = DERIV_ALPHA * rawDeriv + (1.0 - DERIV_ALPHA) * smoothedDeriv;
  float pdOutput = KP * error + KD * smoothedDeriv;

  // Compute target then smooth the actual servo output
  int targetAngle = constrain(CENTER_ANGLE + pdOutput, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  smoothedServo = SERVO_ALPHA * targetAngle + (1.0 - SERVO_ALPHA) * smoothedServo;
  int servoAngle = (int)smoothedServo;

  steeringServo.write(servoAngle + SERVO_TRIM);
  lastError = error;

  // Run Motor — slow down only on sharp turns
  int steerOff = abs(servoAngle - CENTER_ANGLE);
  int speed = (steerOff > HARD_TURN_DEG) ? TURN_SPEED : MOTOR_SPEED;
  runMotor(speed, false);

  // Debug Output
  if (DEBUG_OUTPUT) {
    Serial.print(leftDist, 1);
    Serial.print("\t\t");
    Serial.print(rightDist, 1);
    Serial.print("\t\t");
    Serial.print(error, 1);
    Serial.print("\t");
    Serial.print(servoAngle);
    Serial.print("\t");
    Serial.println(speed);
  }

  delay(100); // Slightly longer for accurate ultrasonic reading
}

// ===== Stable Distance Function =====
float getStableDistance(int trigPin, int echoPin) {
  float sum = 0;
  int validCount = 0;

  // Take 3 readings and average (helps with stability)
  for (int i = 0; i < 3; i++) {
    float dist = getDistance(trigPin, echoPin);
    if (dist > 0) {
      sum += dist;
      validCount++;
    }
    delay(10);
  }

  if (validCount == 0) return -1;
  return sum / validCount;
}

// ===== Single Reading =====
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 40000); // 40ms timeout
  if (duration == 0) return -1;
  return duration * 0.034 / 2.0;
}

// ===== Motor Run Function =====
void runMotor(int speed, bool reverse) {
  digitalWrite(DIR_PIN, reverse ? LOW : HIGH);
  analogWrite(PWM_PIN, speed);
}
