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

    float getSpeed() { return _speed; }  // m/s na roda
    float getAngleError() { return _angleError; }

    void setAngleOffset(float offset) { _angleOffset = offset; }
    void setTurnOffset(float offset) { _turnOffset = offset; }

    void resetIntegral();  // zera a velocidade integrada (entrar em IDLE/FAULT/ACTIVE)

private:
    float _angleError = 0.0f;
    float _speed = 0.0f;
    float _angleOffset = 0.0f;
    float _turnOffset = 0.0f;  // passos/s diferenciais (Fase 5)
};

extern BalanceController balance;
#endif
