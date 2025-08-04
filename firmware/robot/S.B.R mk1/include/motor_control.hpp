#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

class MotorController {
public:
  void begin();
  void setSpeeds(double leftSpeed, double rightSpeed);
  void generateStepPulses();

private:
  const int dirPin1 = 1;
  const int stepPin1 = 0;
  const int dirPin2 = 2;
  const int stepPin2 = 3;

  unsigned long lastStepTimeL = 0;
  unsigned long lastStepTimeR = 0;
  double stepDelayUsL = 1000;
  double stepDelayUsR = 1000;
  int motorDirL = HIGH;
  int motorDirR = HIGH;

  double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);
};

#endif