#include "velocity.hpp"
#include "config.hpp"
#include "balance.hpp"
#include <math.h>

VelocityController velCtrl;

void VelocityController::begin() {
}

// Toda fonte de controle (CLI/USB, UDP/WiFi: teclado, DS4 no PC, scripts) passa por aqui e renova o watchdog.
void VelocityController::setTarget(float ms) {
    // metade do vmax: o equilíbrio precisa de folga acima da velocidade de cruzeiro
    float lim = 0.5f * cfg.maxVelocity;
    _targetVelocity = constrain(ms, -lim, lim);
    _lastCmdMs = millis();
}

void VelocityController::setTurnRate(float degS) {
    _targetTurnRate = constrain(degS, -cfg.maxTurnRate, cfg.maxTurnRate);
    _lastCmdMs = millis();
}

void VelocityController::update() {
    const float dt = CONTROL_DT;

    // watchdog: controle sumiu (programa travou, cabo caiu) → para e fica equilibrando
    if (millis() - _lastCmdMs > CMD_TIMEOUT_MS) {
        _targetVelocity = 0.0f;
        _targetTurnRate = 0.0f;
    }

    // stepper não perde a velocidade comandada (sem encoder): ela é a velocidade da roda
    _estimatedVelocity = balance.getSpeed();

    // === Velocity PI → angle offset (°) ===
    // andando p/ frente além do alvo → offset negativo → inclina p/ trás → freia
    // rampa no alvo: degrau de velocidade vira tranco (o robô precisa inclinar antes de andar)
    float maxStep = TARGET_ACCEL * dt;
    _rampedTarget += constrain(_targetVelocity - _rampedTarget, -maxStep, maxStep);
    float velError = _rampedTarget - _estimatedVelocity;
    // integral só parado: corrige deriva; andando, quem leva é o feedforward + P
    // (senão ele "cobra" a distância não percorrida e o robô dá tranco ao parar)
    if (_rampedTarget == 0.0f) _velIntegral += velError * dt;
    _velIntegral = constrain(_velIntegral, -0.5f, 0.5f);
    _angleOffset = cfg.velocityPid.kp * velError + cfg.velocityPid.ki * _velIntegral;
    _angleOffset = constrain(_angleOffset, -5.0f, 5.0f);

    // === Taxa de curva PI → diferença de velocidade entre rodas (m/s) ===
    // positivo = virar à esquerda (anti-horário visto de cima, gyro Z positivo)
    float turnError = _targetTurnRate - cfg.yawRate;
    _turnIntegral += turnError * dt;
    _turnIntegral = constrain(_turnIntegral, -30.0f, 30.0f);
    _turnOffset = cfg.yawPid.kp * turnError + cfg.yawPid.ki * _turnIntegral;
    _turnOffset = constrain(_turnOffset, -0.3f, 0.3f);
}

float VelocityController::getAngleOffset() {
    return _angleOffset;
}

float VelocityController::getTurnOffset() {
    return _turnOffset;
}

void VelocityController::reset() {
    _targetVelocity = 0.0f;
    _rampedTarget = 0.0f;
    _targetTurnRate = 0.0f;
    _angleOffset = 0.0f;
    _turnOffset = 0.0f;
    _velIntegral = 0.0f;
    _turnIntegral = 0.0f;
    _estimatedVelocity = 0.0f;
}
