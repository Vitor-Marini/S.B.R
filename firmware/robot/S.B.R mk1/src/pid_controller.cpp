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
  unsigned long currentTime = millis();
  double dt = (currentTime - lastComputeTime) / 1000.0;
  if (dt <= 0.0) dt = 0.001;
  lastComputeTime = currentTime;

  double error = setpoint - input;

  // Deadzone
  if (abs(error) < 1.5) {////////////essa deadzone na web
    integral = 0;
    previousError = 0;
    return 0;
  }

  integral += error * dt;
  integral = constrain(integral, -50, 50);

  double derivative = (error - previousError) / dt;
  previousError = error;

  // Cálculo PID
  double output = Kp * error + Ki * integral + Kd * derivative;

  // Converte saída para porcentagem (-100 a 100)
  output = constrain(output, -100.0, 100.0);

  return -output;  // Retorna % da potência
}



