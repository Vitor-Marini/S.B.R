#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP
#include "config_manager.hpp"

class PIDController {
public:
  void begin();
  float compute(float input);
  void setTunings(float kp, float ki, float kd, float sp);

private:
  float setpoint = 0.0f;
  float Kp = 0.0f;
  float Ki = 0.0f;
  float Kd = 0.0f;
  
  float previousError = 0.0f;
  float integral = 0.0f;
  float filteredDerivative = 0.0f;
  unsigned long lastComputeTime = 0;
};

extern PIDController pid;
#endif
