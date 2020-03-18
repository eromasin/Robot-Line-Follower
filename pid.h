#ifndef _PID_h
#define _PID_h
#include <stdint.h>

class PID {
  public:

    PID(float P, float I, float D);                 // This is the class constructor. It is called whenever we create an instance of the PID class 
    void setGains(float P, float I, float D );      // This function updates the values of the gains
    void reset();                                   // This function resets any stored values used by the integral or derative terms
    float update(float demand, float measurement);  // This function calculates the PID control signal. It should be called in a loop
    void printComponents();                        // This function prints the individual components of the control signal and can be used for debugging
    void setMax(float  newMax);                     // This function sets the maximum output the controller can ask for
    void setDebug(bool state);                      // This function sets the debug flag;
    void printResponse();                          // This function prints the ratio of input to output in a way that is nicely interpreted by the Serial plotter
    void setShowResponse(bool state);             // This functions set the show_response flag

  private:
    float Kp; //Proportional
    float Ki; //Integral
    float Kd; //Derivative
    float max_output; 
    float Kp_output; 
    float Ki_output;
    float Kd_output;
    float output_signal;
    float last_demand;      //For storing the previous input
    float last_measurement; //For storing the last measurement
    float last_error;       //For calculating the derivative term
    float integral_error;   //For storing the integral of the error
    long last_millis;       //To track elapsed_time
    bool debug;             //This flag controls whether we print the contributions of each component when update is called
    bool show_response;     // This flag controls whether we print the response of the controller on each update
    
};

 PID::PID(float P, float I, float D) {
  setGains(P, I, D);
  Kp_output     = 0;
  Ki_output     = 0;
  Kd_output     = 0;
  output_signal = 0;
  max_output        = 255;
  last_demand       = 0;
  last_measurement  = 0;
  last_error        = 0;
  integral_error    = 0;
  debug             = false;
  show_response     = false;
  last_millis       = millis();
}

void PID::printComponents() {
  Serial.print(Kp_output);
  Serial.print(",");
  Serial.print(Kd_output);
  Serial.print(",");
  Serial.print(Ki_output);
  Serial.print(",");
  Serial.print(output_signal);
  Serial.print("\n");
}

void PID::setGains(float P, float I, float D) {
  Kp = P;
  Ki = I;
  Kd = D;
}

float PID::update(float demand, float measurement) {
  long time_now = millis();
  int time_delta = time_now - last_millis;
  last_millis = time_now;
  float error;
  error = demand - measurement;  
  float error_delta;
  error_delta = (error - last_error) / time_delta;
  integral_error += (error * time_delta);
  Kp_output = Kp * error;
  Ki_output = Ki * integral_error;
  Kd_output = Kd * error_delta;
  float total = Kp_output + Ki_output + Kd_output;
  last_error = error;
  last_demand = demand;
  last_measurement = measurement;
  if (total > max_output) {
    total = max_output;
  } 
  if (total < -max_output) {
    total = -max_output;
  }
  if (debug) {
    Serial.print(error);
    Serial.print(",");
    Serial.print(error_delta);
    Serial.print(",");
    Serial.print(integral_error);
    Serial.print(",");
    printComponents();
  }
  if (show_response) {
    printResponse();
  }
  return total;
}

void PID::setMax(float newMax)
{
  if (newMax > 0) {
    max_output = newMax;
  } else {
    Serial.println("Max output must be positive");
  }
}

void PID::setDebug(bool state) {
  debug = state;
}

void PID::reset() {
  last_error = 0;
  integral_error = 0;
  last_millis = millis();  
}

void PID::printResponse() {
  float response = last_measurement / last_demand;
  Serial.println(response);
}

void PID::setShowResponse(bool state) {
  show_response = state;
}

#endif