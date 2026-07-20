#include <Arduino.h>
#include "motor_control.hpp"

hw_timer_t *MotorController::timerL = NULL;
hw_timer_t *MotorController::timerR = NULL;

volatile bool MotorController::stepPhaseL = false;
volatile bool MotorController::stepPhaseR = false;

// ISR por toggle: cada chamada inverte o pino STEP.
// Duas chamadas = um pulso completo (HIGH→LOW) = um passo do motor.
void IRAM_ATTR MotorController::onTimerL() {
  stepPhaseL = !stepPhaseL;
  digitalWrite(stepPin1, stepPhaseL);
}

void IRAM_ATTR MotorController::onTimerR() {
  stepPhaseR = !stepPhaseR;
  digitalWrite(stepPin2, stepPhaseR);
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
  timerAlarmWrite(timerL, 1000000, true); // Inicialmente parado (1s)
  timerAlarmEnable(timerL);

  timerR = timerBegin(1, 80, true);
  timerAttachInterrupt(timerR, &onTimerR, true);
  timerAlarmWrite(timerR, 1000000, true);
  timerAlarmEnable(timerR);
}

void MotorController::setSpeeds(float leftTarget, float rightTarget) {
  // === Rampa de aceleração ===
  // Suaviza mudanças bruscas para evitar que os steppers percam passos.
  // Se target for exatamente 0 para ambos (failsafe), para imediatamente.
  if (leftTarget == 0.0f && rightTarget == 0.0f) {
    currentSpeedL = 0.0f;
    currentSpeedR = 0.0f;
  } else {
    float deltaL = leftTarget - currentSpeedL;
    if (fabsf(deltaL) > MAX_ACCEL_PER_TICK) {
      currentSpeedL += (deltaL > 0.0f) ? MAX_ACCEL_PER_TICK : -MAX_ACCEL_PER_TICK;
    } else {
      currentSpeedL = leftTarget;
    }

    float deltaR = rightTarget - currentSpeedR;
    if (fabsf(deltaR) > MAX_ACCEL_PER_TICK) {
      currentSpeedR += (deltaR > 0.0f) ? MAX_ACCEL_PER_TICK : -MAX_ACCEL_PER_TICK;
    } else {
      currentSpeedR = rightTarget;
    }
  }

  // Usa velocidade rampada (não o target direto)
  float leftSpeed = currentSpeedL;
  float rightSpeed = currentSpeedR;

  // Define direção do motor esquerdo
  motorDirL = (leftSpeed >= 0) ? HIGH : LOW;
  digitalWrite(dirPin1, motorDirL);

  // Define direção do motor direito
  motorDirR = (rightSpeed >= 0) ? HIGH : LOW;
  digitalWrite(dirPin2, motorDirR);

  leftSpeed = fabsf(leftSpeed);
  rightSpeed = fabsf(rightSpeed);

  const float MaxSpeed = 100.0f;
  const float MaxStepsPerSecond = 6000.0f;

  if (leftSpeed > MaxSpeed) leftSpeed = MaxSpeed;
  if (rightSpeed > MaxSpeed) rightSpeed = MaxSpeed;

  float stepsL = mapFloat(leftSpeed, 0.0f, MaxSpeed, 0.0f, MaxStepsPerSecond);
  float stepsR = mapFloat(rightSpeed, 0.0f, MaxSpeed, 0.0f, MaxStepsPerSecond);

  // Timer usa toggle: 2 toggles = 1 step, então período = 500000 / steps
  if (stepsL < 10.0f) {
    timerAlarmWrite(timerL, 2000000, true); // Efetivamente parado
  } else {
    timerAlarmWrite(timerL, (uint64_t)(500000.0f / stepsL), true);
  }

  if (stepsR < 10.0f) {
    timerAlarmWrite(timerR, 2000000, true);
  } else {
    timerAlarmWrite(timerR, (uint64_t)(500000.0f / stepsR), true);
  }
}

float MotorController::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
