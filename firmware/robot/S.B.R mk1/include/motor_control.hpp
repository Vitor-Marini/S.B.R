#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

class MotorController {
public:
  void begin();
  void setSpeeds(float leftTarget, float rightTarget);

  static void IRAM_ATTR onTimerL();
  static void IRAM_ATTR onTimerR();

private:
  static const int dirPin1 = 1;
  static const int stepPin1 = 0;
  static const int dirPin2 = 2;
  static const int stepPin2 = 3;

  static hw_timer_t *timerL;
  static hw_timer_t *timerR;

  static volatile bool stepPhaseL;
  static volatile bool stepPhaseR;

  // Rampa de aceleração — rastreia velocidade atual para suavizar transições
  float currentSpeedL = 0.0f;
  float currentSpeedR = 0.0f;

  // Máxima mudança de velocidade (%) por chamada do setSpeeds (~10ms).
  // 8% por tick → 0→100% em ~125ms, -100→+100 em ~250ms.
  // Suficientemente rápido para equilíbrio, suave o bastante para não perder passo.
  static constexpr float MAX_ACCEL_PER_TICK = 8.0f;

  int motorDirL = HIGH;
  int motorDirR = HIGH;

  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
};

#endif