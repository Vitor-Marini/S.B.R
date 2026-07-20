#include <Arduino.h>
#include "pid_controller.hpp"

PIDController pid;

void PIDController::begin() {
  lastComputeTime = millis();
}

void PIDController::setTunings(float kp, float ki, float kd, float sp) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  setpoint = sp;
}


float PIDController::compute(float input) {
  const float dt = 0.01f; // 10ms fixo (100Hz)

  float error = setpoint - input;

  // Integral com Anti-Windup (limitamos a acumulação)
  integral += error * dt;
  integral = constrain(integral, -20.0f, 20.0f);

  // Derivativo com filtro passa-baixa para suprimir ruído do sensor
  float rawDerivative = (error - previousError) / dt;
  filteredDerivative = 0.8f * filteredDerivative + 0.2f * rawDerivative;
  previousError = error;

  // Cálculo PID
  float output = Kp * error + Ki * integral + Kd * filteredDerivative;

  // Limita saída (-100 a 100)
  output = constrain(output, -100.0f, 100.0f);

  // Sinal negativo compensa a orientação física sensor/motor.
  // Se o robô cair para o lado errado, inverta este sinal.
  return -output; 
}
