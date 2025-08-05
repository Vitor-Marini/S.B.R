#include <Arduino.h>
#include "pid_controller.hpp"

void PIDController::begin() {
  lastComputeTime = millis();
}

float PIDController::compute(float input) {
  unsigned long currentTime = millis();
  double dt = (currentTime - lastComputeTime) / 1000.0;
  if (dt <= 0.0) dt = 0.001;
  lastComputeTime = currentTime;

  double error = setpoint - input;

   // Zona morta (deadzone) de ±1.5 graus (ajuste conforme necessário)
  if (abs(error) < 1.5) {
    integral = 0; // opcional: evitar acúmulo indesejado
    previousError = 0;
    return 0;
  }


  integral += error * dt;
  if (integral > 3) integral = 3;
  if (integral < -3) integral = -3;

  double derivative = (error - previousError) / dt;
  double output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  if (output > 255) output = 255;
  if (output < -255) output = -255;

  return -output;
}

void PIDController::setTunings(double kp, double ki, double kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

double PIDController::viewSetPoint(){
  return setpoint;
}