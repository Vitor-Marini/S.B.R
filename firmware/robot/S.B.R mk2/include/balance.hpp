#ifndef BALANCE_HPP
#define BALANCE_HPP

#include <Arduino.h>

// PD de ângulo como ACELERAÇÃO da roda, integrada em velocidade.
// Stepper impõe velocidade, mas o que levanta o pêndulo é aceleração:
// PD direto em velocidade é instável com qualquer ganho.
//   kp: (m/s²) por grau     kd: (m/s²) por °/s
class BalanceController {
public:
    void begin();
    void compute();

    float getSpeed() { return _wheel; }  // m/s na roda (equilíbrio + feedforward)
    float getAngleError() { return _angleError; }

    void setAngleOffset(float offset) { _angleOffset = offset; }
    void setTurnOffset(float offset) { _turnOffset = offset; }
    void setSpeedFF(float ms) { _ff = ms; }  // velocidade pedida, direto nas rodas

    void resetIntegral();  // zera a velocidade integrada (entrar em IDLE/FAULT/ACTIVE)

private:
    float _angleError = 0.0f;
    float _speed = 0.0f;   // parte do equilíbrio (PD integrado)
    float _ff = 0.0f;      // feedforward de velocidade (m/s)
    float _wheel = 0.0f;   // total aplicado nas rodas
    float _angleOffset = 0.0f;
    float _turnOffset = 0.0f;  // m/s de diferença entre rodas; positivo = vira à esquerda
};

extern BalanceController balance;
#endif
