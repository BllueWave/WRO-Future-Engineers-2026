#include <Servo.h>
#include <Pixy2.h>
#include <Wire.h>
#include "DFRobot_BNO055.h"

// ===== Debug Toggle =====
#define DEBUG_OUTPUT true

// ===========================================================================
// ===== PINS =====
// ===========================================================================
//  HC-SR04 LEFT   TRIG → D4    ECHO → D5
//  HC-SR04 RIGHT  TRIG → D2    ECHO → D9
//  Servo          SIG  → A0
//  Cytron         PWM  → D3    DIR  → D8
//  Pixy2 SPI      CS   → D6    MOSI → D11   MISO → D12   SCK → D13
//  BNO055 I2C     SDA  → A4    SCL  → A5
// ===========================================================================
const int TRIG_LEFT  = 4;
const int ECHO_LEFT  = 5;
const int TRIG_RIGHT = 2;
const int ECHO_RIGHT = 9;
const int SERVO_PIN  = A0;   // D10 conflicts with SPI — moved to A0
const int PWM_PIN    = 3;
const int DIR_PIN    = 8;
// Pixy2 CS = D6  (set via pixy.init(6) below)

// ===========================================================================
// ===== SPEED =====
// ===========================================================================
const int MOTOR_SPEED = 35;

// ===========================================================================
// ===== PD CONTROLLER =====
// ===========================================================================
const float KP = 1.0;
const float KD = 1.0;

// ===========================================================================
// ===== STEERING ANGLES =====
// ===========================================================================
const int CENTER_ANGLE    = 130;
const int MIN_SERVO_ANGLE = 80;
const int MAX_SERVO_ANGLE = 180;

// ===========================================================================
// ===== PIXY2 TARGET ANGLES =====
// sig 1 = RED   pillar
// sig 2 = GREEN pillar
// ===========================================================================
const int RED_SOFT_LEFT    = 110;
const int RED_MED_LEFT     = 130;
const int RED_HARD_LEFT    = 150;
const int GREEN_SOFT_RIGHT = 70;
const int GREEN_MED_RIGHT  = 50;
const int GREEN_HARD_RIGHT = 30;

// ===========================================================================
// ===== IMU =====
// ===========================================================================
typedef DFRobot_BNO055_IIC BNO;
BNO bno(&Wire, 0x28);

// ===========================================================================
// ===== GLOBALS =====
// ===========================================================================
Servo  steeringServo;
Pixy2  pixy;

float lastError    = 0;
bool  pixyActive   = false;

float lastHeading  = 0;
float headingTotal = 0;
int   lapCount     = 0;
float currentHeading = 0;

const int REQUIRED_LAPS = 3;
bool  isRunning = true;

// ===========================================================================
// ===== FORWARD DECLARATIONS =====
// ===========================================================================
float getDistance(int trigPin, int echoPin);
void  runMotor(int speed, bool reverse);

// ===========================================================================
void setup() {
  pinMode(TRIG_LEFT,  OUTPUT);
  pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  runMotor(0, false);

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(CENTER_ANGLE);

  if (DEBUG_OUTPUT) Serial.begin(115200);

  // Pixy2: CS على D6 لتجنب تعارض D10 مع SPI
  pixy.init(6);

  // BNO055
  Wire.begin();
  delay(100);
  bno.reset();
  delay(700);
  while (bno.begin() != BNO::eStatusOK) {
    if (DEBUG_OUTPUT) Serial.println("BNO055 init failed — check wiring! SDA=A4 SCL=A5 VCC=3.3V");
    delay(1000);
  }
  if (DEBUG_OUTPUT) Serial.println("=== Robot2 Ready ===");

  lastHeading = bno.getEul().head;

  delay(2000);   // ثانيتين قبل الحركة
  if (DEBUG_OUTPUT) Serial.println("GO!");
}

// ===========================================================================
void loop() {

  // ── إيقاف بعد 3 لفات ─────────────────────────────────────────────────────
  if (!isRunning) {
    runMotor(0, false);
    if (DEBUG_OUTPUT) Serial.println("3 laps done — stopped.");
    while (true);
  }

  // ── عدّ اللفات (IMU) ──────────────────────────────────────────────────────
  BNO::sEulAnalog_t eul = bno.getEul();
  currentHeading = eul.head;

  float delta = currentHeading - lastHeading;
  if (delta >  180) delta -= 360;
  if (delta < -180) delta += 360;
  headingTotal += delta;
  lastHeading = currentHeading;

  if (fabs(headingTotal) >= 360.0) {
    lapCount++;
    headingTotal = 0;
    if (DEBUG_OUTPUT) {
      Serial.print("Lap: ");
      Serial.println(lapCount);
    }
    if (lapCount >= REQUIRED_LAPS) {
      isRunning = false;
      return;
    }
  }

  // ── قراءة الحساسات ────────────────────────────────────────────────────────
  float leftDist  = getDistance(TRIG_LEFT,  ECHO_LEFT);
  float rightDist = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  if (leftDist  < 0) leftDist  = rightDist;
  if (rightDist < 0) rightDist = leftDist;

  // ── Pixy2 ─────────────────────────────────────────────────────────────────
  int  blocks = pixy.ccc.getBlocks();
  pixyActive  = false;

  if (blocks > 0) {
    int sig = pixy.ccc.blocks[0].m_signature;
    int x   = pixy.ccc.blocks[0].m_x;

    if (sig == 1) {                         // RED pillar
      pixyActive = true;
      if      (x >= 200) steeringServo.write(RED_SOFT_LEFT);
      else if (x >= 120) steeringServo.write(RED_MED_LEFT);
      else               steeringServo.write(RED_HARD_LEFT);
    }
    else if (sig == 2) {                    // GREEN pillar
      pixyActive = true;
      if      (x <= 120) steeringServo.write(GREEN_SOFT_RIGHT);
      else if (x <= 200) steeringServo.write(GREEN_MED_RIGHT);
      else               steeringServo.write(GREEN_HARD_RIGHT);
    }
  }

  // ── PD wall-following ─────────────────────────────────────────────────────
  if (!pixyActive) {
    float error      = leftDist - rightDist;
    float derivative = error - lastError;
    float pdOutput   = KP * error + KD * derivative;
    int servoAngle   = constrain(CENTER_ANGLE + (int)pdOutput,
                                 MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    steeringServo.write(servoAngle);
    lastError = error;
  }

  runMotor(MOTOR_SPEED, false);

  // ── Debug ─────────────────────────────────────────────────────────────────
  if (DEBUG_OUTPUT) {
    Serial.print("Pixy:"); Serial.print(pixyActive ? "YES" : "NO");
    Serial.print(" Lap:");  Serial.print(lapCount);
    Serial.print(" Hdg:");  Serial.print(currentHeading, 1);
    Serial.print(" L:");    Serial.print(leftDist, 1);
    Serial.print(" R:");    Serial.print(rightDist, 1);
    Serial.print(" Srv:");  Serial.println(steeringServo.read());
  }

  delay(50);
}

// ===========================================================================
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 30000);
  if (dur <= 0) return -1;
  return dur * 0.034 / 2.0;
}

void runMotor(int speed, bool reverse) {
  digitalWrite(DIR_PIN, reverse ? LOW : HIGH);
  analogWrite(PWM_PIN, speed);
}
