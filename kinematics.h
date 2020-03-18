#ifndef _Kinematics
#define _Kinematics_h
#include <math.h>

const float WHEEL_DIAMETER = 70;
const float WHEEL_RADIUS = 35;
const float WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;
const float WHEEL_SEPARATION = 143.5;
const float COUNTS_PER_WHEEL_REVOLUTION = 1440;
const float MM_PER_COUNT = WHEEL_CIRCUMFERENCE/COUNTS_PER_WHEEL_REVOLUTION;
const float COUNTS_PER_MM = 1/MM_PER_COUNT;

class Kinematics
{
  public:   
    Kinematics();
    void update();
    float getHeadingTarget();
    float getDistanceTarget();
    float getX();
    float getY();
    float getTheta();
    float getDistanceInCounts(float distance);

  private:
    float x;
    float y;
    float d;
    float theta;
    float last_count_left_e;
    float last_count_right_e;   
};

Kinematics::Kinematics() {
	x = 0.0;
  y = 0.0;
  theta = 0.0;
}

void Kinematics :: update() {
  float current_count_left = count_left_e;
  float current_count_right = count_right_e;
  float count_left_diff = current_count_left - last_count_left_e;
  float count_right_diff = current_count_right - last_count_right_e;
  float left_distance = count_left_diff/COUNTS_PER_MM;
  float right_distance = count_right_diff/COUNTS_PER_MM;
  float mean_diff = (left_distance + right_distance)/2;
  theta = theta + (left_distance - right_distance)/WHEEL_SEPARATION;
  x = x + (mean_diff * cos(theta));
  y = y + (mean_diff * sin(theta));
  last_count_left_e = current_count_left;
  last_count_right_e = current_count_right;
  return;
}

float Kinematics :: getHeadingTarget() {
  return atan2(-y, -x);
}

float Kinematics :: getDistanceTarget() {
  return (float)sqrt(pow(x, 2) + pow(y, 2));
}

float Kinematics :: getDistanceInCounts(float distance) {
  return (distance * COUNTS_PER_MM);
}

float Kinematics :: getX() {
  return x;
}

float Kinematics :: getY() {
  return y;
}

float Kinematics :: getTheta() {
  return theta;
}

#endif
