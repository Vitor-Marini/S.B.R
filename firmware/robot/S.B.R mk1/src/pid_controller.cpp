#include <Arduino.h>
#include "pid_controller.hpp"

PIDController pid;

void PIDController::begin() {
  lastComputeTime = millis();
}

void PIDController::setTunings(double kp, double ki, double kd, double sp) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  setpoint = sp;
}


float PIDController::compute(float input) {
  // Usamos um DT fixo de 10ms (100Hz) definido no main.cpp
  const double dt = 0.01; 

  double error = setpoint - input;

  // Deadzone para evitar micro-oscilações no repouso
  if (abs(error) < 0.5) {
    // Não zeramos tudo, apenas a saída se o erro for desprezível
    // integral = 0; // Opcional: manter ou zerar
  }

  // Integral com Anti-Windup (limitamos a acumulação)
  integral += error * dt;
  integral = constrain(integral, -20.0, 20.0); // Limite menor para evitar overshoot

  double derivative = (error - previousError) / dt;
  previousError = error;

  // Cálculo PID
  double output = Kp * error + Ki * integral + Kd * derivative;

  // Converte saída para porcentagem (-100 a 100)
  output = constrain(output, -100.0, 100.0);

  return -output; 
}



