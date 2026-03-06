#include <Arduino.h>
#include "motor_control.hpp"

hw_timer_t *MotorController::timerL = NULL;
hw_timer_t *MotorController::timerR = NULL;

void IRAM_ATTR MotorController::onTimerL() {
  digitalWrite(stepPin1, HIGH);
  delayMicroseconds(2);
  digitalWrite(stepPin1, LOW);
}

void IRAM_ATTR MotorController::onTimerR() {
  digitalWrite(stepPin2, HIGH);
  delayMicroseconds(2);
  digitalWrite(stepPin2, LOW);
}

void MotorController::begin() {
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);

  digitalWrite(dirPin1, motorDirL);
  digitalWrite(dirPin2, motorDirR);

  // Inicializa Timers (ESP32-C3 tem 2 timers de hardware no grupo 0)
  timerL = timerBegin(0, 80, true); // 1MHz base (80MHz / 80)
  timerAttachInterrupt(timerL, &onTimerL, true);
  timerAlarmWrite(timerL, 1000000, true); // Inicialmente desativado (1s)
  timerAlarmEnable(timerL);

  timerR = timerBegin(1, 80, true);
  timerAttachInterrupt(timerR, &onTimerR, true);
  timerAlarmWrite(timerR, 1000000, true);
  timerAlarmEnable(timerR);
}

void MotorController::setSpeeds(double leftSpeed, double rightSpeed) {
  // Define direção do motor esquerdo
  motorDirL = (leftSpeed >= 0) ? HIGH : LOW;
  digitalWrite(dirPin1, motorDirL);

  // Define direção do motor direito
  motorDirR = (rightSpeed >= 0) ? HIGH : LOW;
  digitalWrite(dirPin2, motorDirR);

  leftSpeed = abs(leftSpeed);
  rightSpeed = abs(rightSpeed);

  const double MaxSpeed = 100.0;
  const double MaxStepsPerSecond = 6000.0;

  if (leftSpeed > MaxSpeed) leftSpeed = MaxSpeed;
  if (rightSpeed > MaxSpeed) rightSpeed = MaxSpeed;

  double stepsL = mapDouble(leftSpeed, 0.0, MaxSpeed, 0.0, MaxStepsPerSecond);
  double stepsR = mapDouble(rightSpeed, 0.0, MaxSpeed, 0.0, MaxStepsPerSecond);

  // Se velocidade for muito baixa, para o timer (alarm desativado ou tempo gigante)
  if (stepsL < 10) {
    timerAlarmWrite(timerL, 2000000, true); // 2 segundos
  } else {
    timerAlarmWrite(timerL, 1000000 / stepsL, true);
  }

  if (stepsR < 10) {
    timerAlarmWrite(timerR, 2000000, true);
  } else {
    timerAlarmWrite(timerR, 1000000 / stepsR, true);
  }
}

void MotorController::generateStepPulses() {
  // Vazio pois agora é via interrupção
}

double MotorController::mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
