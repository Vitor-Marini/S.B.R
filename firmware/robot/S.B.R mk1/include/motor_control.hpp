#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

class MotorController {
public:
  void begin();
  void setSpeeds(double leftSpeed, double rightSpeed);
  // generateStepPulses não será mais necessária no loop principal se usarmos timers, 
  // mas vamos mantê-la para retrocompatibilidade ou migrá-la para o Timer.
  void generateStepPulses(); 

  static void IRAM_ATTR onTimerL();
  static void IRAM_ATTR onTimerR();

private:
  static const int dirPin1 = 1;
  static const int stepPin1 = 0;
  static const int dirPin2 = 2;
  static const int stepPin2 = 3;

  static hw_timer_t *timerL;
  static hw_timer_t *timerR;

  unsigned long lastStepTimeL = 0;
  unsigned long lastStepTimeR = 0;
  double stepDelayUsL = 1000;
  double stepDelayUsR = 1000;
  int motorDirL = HIGH;
  int motorDirR = HIGH;

  double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);
};

#endif