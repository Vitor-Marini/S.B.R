#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
  void begin();
  float compute(float input);
  void setTunings(double kp, double ki, double kd);
  double viewSetPoint();

private:
  double setpoint = 0;
  double Kp = 0.0;
  double Ki = 0.0;
  double Kd = 0.0;
  
  double previousError = 0.0;
  double integral = 0.0;
  unsigned long lastComputeTime = 0;
};

#endif
