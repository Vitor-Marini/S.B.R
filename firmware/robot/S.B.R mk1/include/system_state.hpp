#ifndef SYSTEM_STATE_HPP
#define SYSTEM_STATE_HPP

#include <Arduino.h>

struct SystemState {
  float angle = 0.0;          // Ângulo atual do robô
  float pidOutput = 0.0;      // Saída do controle PID
  String direction = "Stop";  // Direção atual (pode ser "Fwd", "Bwd", "Left", "Right", "Stop")
  float kp = 0.0;
  float ki = 0.0;
  float kd = 0.0;
  float setPoint = 0;

  void log();

  void updateDirectionFromPID();
};

#endif