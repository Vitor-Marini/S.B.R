#include "velocity.hpp"
#include "config.hpp"
#include "balance.hpp"
#include <math.h>

VelocityController velCtrl;

void VelocityController::begin() {
}

void VelocityController::setTarget(float ms) {
    _targetVelocity = constrain(ms, -cfg.maxVelocity, cfg.maxVelocity);
}

void VelocityController::setTurnRate(float degS) {
    _targetTurnRate = constrain(degS, -cfg.maxTurnRate, cfg.maxTurnRate);
}

void VelocityController::update() {
    const float dt = CONTROL_DT;

    // stepper não perde a velocidade comandada (sem encoder): ela é a velocidade da roda
    _estimatedVelocity = balance.getSpeed();

    // === Velocity PI → angle offset (°) ===
    // andando p/ frente além do alvo → offset negativo → inclina p/ trás → freia
    float velError = _targetVelocity - _estimatedVelocity;
    _velIntegral += velError * dt;
    _velIntegral = constrain(_velIntegral, -0.5f, 0.5f);
    _angleOffset = cfg.velocityPid.kp * velError + cfg.velocityPid.ki * _velIntegral;
    _angleOffset = constrain(_angleOffset, -5.0f, 5.0f);

    // === Turn rate P → differential offset ===
    float turnRateActual = cfg.yawRate;
    float turnError = _targetTurnRate - turnRateActual;
    _turnIntegral += turnError * dt;
    _turnIntegral = constrain(_turnIntegral, -10.0f, 10.0f);
    _turnOffset = cfg.yawPid.kp * turnError + cfg.yawPid.ki * _turnIntegral;
    _turnOffset = constrain(_turnOffset, -50.0f, 50.0f);
}

float VelocityController::getAngleOffset() {
    return _angleOffset;
}

float VelocityController::getTurnOffset() {
    return _turnOffset;
}

void VelocityController::reset() {
    _targetVelocity = 0.0f;
    _targetTurnRate = 0.0f;
    _angleOffset = 0.0f;
    _turnOffset = 0.0f;
    _velIntegral = 0.0f;
    _turnIntegral = 0.0f;
    _estimatedVelocity = 0.0f;
}
