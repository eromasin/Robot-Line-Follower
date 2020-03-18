#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"
#include "kinematics.h"

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define DIR_FWD LOW
#define DIR_BKD HIGH
#define BAUD_RATE 9600
#define BUZZER_PIN 6
#define LINE_LEFT_PIN   A4
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN  A2
#define heading_kp 50.0
#define heading_ki 0.01
#define heading_kd 0.0
#define line_kp 0.1
#define line_ki 0.0
#define line_kd 0.0
#define wheel_kp 0.08
#define wheel_ki 0.0
#define wheel_kd 0.0

#define STATE_START       0
#define STATE_FORWARDS 1
#define STATE_LINE_MEET     2
#define STATE_LINE_FOLLOW    3
#define STATE_LINE_LOST      4
#define STATE_LINE_FIND    5
#define STATE_FACE_HOME   6
#define STATE_DRIVE_HOME     7
#define STATE_STOP           10

LineSensor lineLeft(LINE_LEFT_PIN);
LineSensor lineCentre(LINE_CENTRE_PIN);
LineSensor lineRight(LINE_RIGHT_PIN);
Kinematics pose;
PID headingPID(heading_kp, heading_ki, heading_kd);
PID pidLine(line_kp, line_ki, line_kd);

float checkTimeState, startTimeState;
bool onLine, lineJoined;
bool buzzerOn, buzzerFinish;
float homeDistance, lastDistance;
bool headingCheck = false;
float leftConfidence, centreConfidence, rightConfidence;
float lineTheshold, confidenceThreshold;
float lineDemand, headingDemand, headingMeasurement;
float speedForward, meanCount;
bool checkLeft, checkRight;
int STATE;

void setup()
{
  funcSetupMotors();
  setupLeftEncoder();
  setupRightEncoder();
  funcSetupTimestamps();
  funcSetupLineSensors();
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOn = false;
  buzzerFinish = false;
  headingCheck = false;
  lineTheshold = 300.0;
  confidenceThreshold = 60.0;
  lineDemand = 0.0;
  headingDemand = 0.0;
  headingMeasurement = 0.0;
  speedForward = 18.0;
  lastDistance = 0.0;
  homeDistance = 0.0;
  meanCount = 0;
  checkLeft = false;
  checkRight = false;
  delay(1000);
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");
  STATE = 0;
}

void loop()
{
  pose.update();
  funcUpdateLineConfidence();
  onLine = funcCheckForLine();
  float timePassed = millis() - checkTimeState;
  switch ( STATE ) {
    case STATE_START:
      funcInitTone();
      break;
    case STATE_FORWARDS:
      funcDriveForwards();
      break;
    case STATE_LINE_FOLLOW:
      if (timePassed > 10) {
        checkTimeState = millis();
        funcFollowLine();
      }
      break;
    case STATE_LINE_LOST:
      funcLostLine();
      break;
    case STATE_LINE_FIND:
      if (timePassed > 10) {
        checkTimeState = millis();
        funcRefindLine();
      }
      break;
    case STATE_FACE_HOME:
      funcTurnHome();
      break;
    case STATE_DRIVE_HOME:
      if (timePassed > 5) {
        checkTimeState = millis();
        funcDriveHome();
      }
      break;
    case STATE_STOP:
      funcUpdateSpeed(0, 0);
      break;
    default:
      Serial.println("System Error, Unknown state!");
      break;
  }

}

void funcDriveHome() {
  Serial.println(homeDistance);
  headingDemand = pose.getHeadingTarget();
  funcUpdateHeadingPID(18.0);
  if (abs(pose.getDistanceTarget()) <= 50.0) {
    if (homeDistance >= lastDistance) {
      STATE = STATE_STOP;
    }
  }
  lastDistance = homeDistance;
}

void funcTurnHome() {
  headingDemand = pose.getHeadingTarget();
  if (headingDemand >= pose.getTheta()) {
    funcUpdateSpeed(20.0, -20.0);
  } else {
    funcUpdateSpeed(-20.0, 20.0);
  }

  float angleDiff = headingDemand - pose.getTheta();
  if (abs(angleDiff) <= 0.01) {
    funcUpdateSpeed(0, 0);
    funcPlayTone(10, 200);
    lastDistance = pose.getDistanceTarget();
    STATE = STATE_DRIVE_HOME;
    return;
  }
}

void funcRefindLine() {
  if (onLine) {
    STATE = STATE_LINE_FOLLOW;
    return;
  }
  if (!checkLeft && !checkRight) {
    headingDemand = headingMeasurement - M_PI / 8;
    funcUpdateSpeed(-16, 16);
    if (pose.getTheta() <= headingDemand) {
      checkLeft = true;
    }
  }
  if (checkLeft && !checkRight) {
    headingDemand = headingMeasurement + M_PI / 8;
    funcUpdateSpeed(16, -16);
    if (pose.getTheta() >= headingDemand) {
      checkRight = true;
    }
  }
  if (checkLeft && checkRight) {
    headingDemand = headingMeasurement;
    funcUpdateSpeed(-16, 16);
    if (pose.getTheta() <= headingDemand) {
      funcUpdateSpeed(0, 0);
      delay(1000);
      STATE = STATE_FACE_HOME;
      return;
    }
  }
}

void funcLostLine() {
  if (lineJoined) {
    headingMeasurement = pose.getTheta();
    STATE = STATE_LINE_FIND;
  }
}

void funcFollowLine() {
  if (onLine) {
    funcUpdateLinePID();
  }
  else {
    STATE = STATE_LINE_LOST;
  }
}

void funcDriveForwards() {
  if (!onLine) {
    funcUpdateHeadingPID(30.0);
  } else {
    lineJoined = true;
    STATE = STATE_LINE_FOLLOW;
  }
}

void funcTurnTarget() {
  if (!headingCheck) {
    headingDemand = pose.getHeadingTarget();
    headingCheck = true;
  }
  funcUpdateHeadingPID(0.0);
  if (headingDemand - pose.getTheta() < 2.0 ) {
    STATE = STATE_STOP;
  }
}

void funcUpdateSpeed(float leftSpeedNew, float rightSpeedNew) {
  if (leftSpeedNew < 0) {
    leftSpeedNew = leftSpeedNew * -1;
    digitalWrite( L_DIR_PIN, DIR_BKD );
  } else {
    digitalWrite( L_DIR_PIN, DIR_FWD );
  }
  if (rightSpeedNew < 0) {
    rightSpeedNew = rightSpeedNew * -1;
    digitalWrite( R_DIR_PIN, DIR_BKD );
  } else {
    digitalWrite( R_DIR_PIN, DIR_FWD );
  }
  analogWrite( L_PWM_PIN, leftSpeedNew );
  analogWrite( R_PWM_PIN, rightSpeedNew );
}

void funcUpdateLineConfidence() {
  if (lineLeft.readCalibrated() > lineTheshold) {
    leftConfidence += 1.0;
  }
  else {
    leftConfidence -= 1.0;
  }
  if (lineCentre.readCalibrated() > lineTheshold) {
    centreConfidence += 1.0;
  }
  else {
    centreConfidence -= 1.0;
  }
  if (lineRight.readCalibrated() > lineTheshold) {
    rightConfidence += 1.0;
  }
  else {
    rightConfidence -= 1.0;
  }
  leftConfidence = constrain(leftConfidence, 0.0, 100.0);
  centreConfidence = constrain(centreConfidence, 0.0, 100.0);
  rightConfidence = constrain(rightConfidence, 0.0, 100.0);
  return;
}

bool funcCheckForLine() {
  bool leftLine = leftConfidence > confidenceThreshold;
  bool centreLine = centreConfidence > confidenceThreshold;
  bool rightLine = rightConfidence > confidenceThreshold;
  if (leftLine || centreLine || rightLine) {
    if (!lineJoined) {
      leftConfidence = 100.0;
      centreConfidence = 100.0;
      rightConfidence = 100.0;
      lineJoined = true;
    }
    return true;
  }

  return false;
}

void funcUpdateLinePID() {
  float centreValue = funcGetLineCentre();
  float PIDOut = pidLine.update(lineDemand, centreValue);
  float speedLeft = speedForward - PIDOut;
  float speedRight = speedForward + PIDOut;
  speedLeft = constrain(speedLeft, -254, 254);
  speedRight = constrain(speedRight, -254, 254);
  funcUpdateSpeed(speedLeft, speedRight);
}

void funcUpdateHeadingPID(float forwardBias) {
  float headingOut = headingPID.update(headingDemand, pose.getTheta());
  float speedLeft = forwardBias + headingOut;
  float speedRight = forwardBias - headingOut;
  speedLeft = constrain(speedLeft, -254, 254);
  speedRight = constrain(speedRight, -254, 254);
  funcUpdateSpeed(speedLeft, speedRight);
}

float funcGetLineCentre() {
  float valueLeft = lineLeft.readCalibrated();
  float valueCentre = lineCentre.readCalibrated();
  float valueRight = lineRight.readCalibrated();
  float total = valueLeft + valueCentre + valueRight;
  float p1 = valueLeft / total;
  float p2 = valueCentre / total;
  float p3 = valueRight / total;
  float lineCentre = (p1 * 1000) + (p2 * 2000) + (p3 * 3000);
  lineCentre = lineCentre - 2000;
  lineCentre = constrain(lineCentre, -2000, 2000);
  return lineCentre;
}

void funcInitTone() {
  for (int i = 0; i < 2; i++) {
    funcPlayTone(5, 100);
    delay(900);
  }

  STATE = STATE_FORWARDS;
}

void funcPlayTone(int volume, int duration) {
  analogWrite(BUZZER_PIN, volume);
  delay(duration);
  analogWrite(BUZZER_PIN, 0);
}

void funcSetupMotors() {
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );
  digitalWrite( L_DIR_PIN, DIR_FWD );
  digitalWrite( R_DIR_PIN, DIR_FWD );
}

void funcSetupTimestamps() {
  checkTimeState = millis();
  startTimeState = millis();
}

void funcSetupLineSensors() {
  lineLeft.calibrate();
  lineCentre.calibrate();
  lineRight.calibrate();
  onLine = false;
  lineJoined = false;
  leftConfidence = 0.0;
  centreConfidence = 0.0;
  rightConfidence = 0.0;
}

float funcRoundFloat(float var) {
  float value = (int)(var * 100 + .5);
  return (float)value / 100;
}
