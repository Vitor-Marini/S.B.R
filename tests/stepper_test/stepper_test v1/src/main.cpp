#include"Arduino.h"
#define STEP_PIN 12
#define DIR_PIN 14

// Número de passos para 1 rotação completa
const int fullRotationSteps = 3200;  // usando microstepping de 1/16
const int quarterTurnSteps = fullRotationSteps / 4;  // 90 graus

int stepDelay = 500; // menor delay → mais rápido

void moveMotor(bool direction, int steps); // Declaração

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // Andar para frente (simulado com várias rotações)
  Serial.println("Movendo para frente");
  moveMotor(HIGH, fullRotationSteps * 3);  // anda 3 voltas completas
  delay(1000);

  // Girar 90 graus (direção oposta)
  Serial.println("Girando 90 graus");
  moveMotor(LOW, quarterTurnSteps);  // gira 90 graus
  delay(1000);

  // Voltar para trás
  Serial.println("Voltando para trás");
  moveMotor(LOW, fullRotationSteps * 3);  // volta
  delay(1000);

  Serial.println("--- Novo Ciclo ---");
  delay(2000);
}

void moveMotor(bool direction, int steps) {
  digitalWrite(DIR_PIN, direction);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
  }
}
