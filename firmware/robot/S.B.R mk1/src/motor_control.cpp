#include <Arduino.h>
#include "motor_control.hpp"

void MotorController::begin() {
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);

  digitalWrite(dirPin1, motorDirL);
  digitalWrite(dirPin2, motorDirR);
}

void MotorController::setSpeeds(double leftSpeed, double rightSpeed) {
  // Define direção do motor esquerdo
  motorDirL = (leftSpeed >= 0) ? HIGH : LOW;
  digitalWrite(dirPin1, motorDirL);

  // Define direção do motor direito
  motorDirR = (rightSpeed >= 0) ? HIGH : LOW;
  digitalWrite(dirPin2, motorDirR);

  // Converte valores negativos em positivos
  leftSpeed = abs(leftSpeed);
  rightSpeed = abs(rightSpeed);

  // Limites e mapeamento para frequência de passos
  const double MaxSpeed = 100.0;
  const double MaxStepsPerSecond = 6000.0;

  if (leftSpeed > MaxSpeed) leftSpeed = MaxSpeed;
  if (rightSpeed > MaxSpeed) rightSpeed = MaxSpeed;

  double stepsL = mapDouble(leftSpeed, 0.0, MaxSpeed, 0.0, MaxStepsPerSecond);
  double stepsR = mapDouble(rightSpeed, 0.0, MaxSpeed, 0.0, MaxStepsPerSecond);

  if (stepsL < 1) stepsL = 1;
  if (stepsR < 1) stepsR = 1;

  stepDelayUsL = 1e6 / stepsL;
  stepDelayUsR = 1e6 / stepsR;
}

void MotorController::generateStepPulses() {
  unsigned long now = micros();

  if (now - lastStepTimeL >= stepDelayUsL) {
    lastStepTimeL = now;
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin1, LOW);
  }

  if (now - lastStepTimeR >= stepDelayUsR) {
    lastStepTimeR = now;
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin2, LOW);
  }
}

double MotorController::mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
