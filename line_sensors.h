#ifndef _Line_follow_h
#define _Line_follow_h

class LineSensor {
  public:
    LineSensor(int pin);   //Constructor
    void calibrate();         //Calibrate
    int readRaw();            //Return the uncalibrated value from the sensor
    float readCalibrated();   //Return the calibrated value from the sensor
    bool isOverLine();

  private:
    int pin;
    int calibrate_count;
    int threshold;
    float bias_compensation; 
};

LineSensor::LineSensor(int Line_pin) {
  calibrate_count = 50;
  bias_compensation = 0.0;
  threshold = 100;
  pin = Line_pin;
  pinMode(pin, INPUT);
}

int LineSensor::readRaw() {
  return analogRead(pin);
}

void LineSensor::calibrate() {
  int running_total = 0;
  for (int i = 0; i < calibrate_count; i++)
  {
    int value = readRaw();
    running_total += value;
  }
  bias_compensation = (float)running_total/(float)calibrate_count;
}

float LineSensor::readCalibrated() {
  float calibrated_reading = (float)readRaw() - bias_compensation;
  return calibrated_reading;
}

bool LineSensor::isOverLine() {
  if (readCalibrated() > threshold) {
    return true;
  }
  return false;
}

#endif