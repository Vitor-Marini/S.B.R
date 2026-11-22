#include "velocity.hpp"
#include "config.hpp"
#include "balance.hpp"
#include "net.hpp"
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
    _turnActive = false;  // comando manual manda
}

void VelocityController::setTurnRate(float degS) {
    _targetTurnRate = constrain(degS, -cfg.maxTurnRate, cfg.maxTurnRate);
    _lastCmdMs = millis();
    _turnActive = false;
}

void VelocityController::turnBy(float deg) {
    _targetVelocity = 0.0f;
    _turnGoal = deg;
    _turnDone = 0.0f;
    _turnStartMs = millis();
    _turnActive = true;
}

void VelocityController::update() {
    const float dt = CONTROL_DT;

    // watchdog: controle sumiu (programa travou, cabo caiu) → para e fica equilibrando
    if (millis() - _lastCmdMs > CMD_TIMEOUT_MS) {
        _targetVelocity = 0.0f;
        _targetTurnRate = 0.0f;
    }

    // turn <graus>: taxa pelo que falta, freando a tempo (√(2·a·falta), metade da rampa de folga) sem passar do ponto
    if (_turnActive) {
        _turnDone += cfg.yawRate * dt;
        float left = _turnGoal - _turnDone;
        if (fabsf(left) < 1.0f || millis() - _turnStartMs > TURN_TIMEOUT_MS) {
            term.printf(fabsf(left) < 1.0f ? "giro ok: %.1f°\n" : "giro desistiu (travado/no suporte?): %.1f°\n", _turnDone);
            _turnActive = false;
            _targetTurnRate = 0.0f;
        } else {
            float w = fminf(fminf(sqrtf(TURN_ACCEL * fabsf(left)), TURN_KP * fabsf(left)), TURN_CMD_RATE);
            _targetTurnRate = copysignf(w, left);
            _lastCmdMs = millis();  // renova o watchdog enquanto gira
        }
    }

    // stepper não perde a velocidade comandada (sem encoder): ela é a velocidade da roda
    _estimatedVelocity = balance.getSpeed();

    // === Velocity PI → angle offset (°) ===
    // andando p/ frente além do alvo → offset negativo → inclina p/ trás → freia
    // rampa no alvo: degrau de velocidade vira tranco (o robô precisa inclinar antes de andar)
    float maxStep = TARGET_ACCEL * dt;
    _rampedTarget += constrain(_targetVelocity - _rampedTarget, -maxStep, maxStep);
    bool driving = _rampedTarget != 0.0f;

    // Parado: segura a posição pela odometria das rodas (stepper = posição exata). Ancora quando a roda
    // quase para (fim da frenagem); tira o "volta um pouco" ao soltar e o deslocamento ao girar no lugar.
    // Empurrou: volta pro lugar (a até HOLD_MAX_V). Além de HOLD_SLIP_M (carregado/arrastado) a âncora vem junto.
    float holdVel = 0.0f;
    if (driving) {
        _holdArmed = false;
    } else {
        if (!_holdArmed && fabsf(_estimatedVelocity) < HOLD_ARM_SPEED) { _holdArmed = true; _holdPos = 0.0f; }
        if (_holdArmed) {
            _holdPos = constrain(_holdPos + _estimatedVelocity * dt, -HOLD_SLIP_M, HOLD_SLIP_M);
            holdVel = constrain(-HOLD_KP * _holdPos, -HOLD_MAX_V, HOLD_MAX_V);
        }
    }
    float velError = _rampedTarget + holdVel - _estimatedVelocity;

    // Parado: só o P (+ a posição acima). Andando: + integral, que vence a resistência à rolagem;
    // ao parar, ele se esvazia em ~0,2 s.
    if (driving) _velIntegral += velError * dt;
    else _velIntegral -= _velIntegral * dt / I_DECAY_S;
    if (cfg.velocityPid.ki > 0.0f) {
        float lim = I_MAX_DEG / cfg.velocityPid.ki;
        _velIntegral = constrain(_velIntegral, -lim, lim);
    } else {
        _velIntegral = 0.0f;
    }
    _angleOffset = cfg.velocityPid.kp * velError + cfg.velocityPid.ki * _velIntegral;
    _angleOffset = constrain(_angleOffset, -5.0f, 5.0f);

    // Auto-ajuste do setpoint COM TRAVAS (a versão sem travas aprendia errado em quedas e com o robô
    // segurado): só depois de 2 s calmo — sem comando, devagar, sem girar, perto do setpoint — e
    // limitado a ±1,5° do valor do start. Fica na RAM: salvo num `stop`; numa queda volta ao do start.
    bool calm = !driving && _rampedTurn == 0.0f && _targetVelocity == 0.0f && _targetTurnRate == 0.0f
             && fabsf(_estimatedVelocity) < TRIM_MAX_SPEED && fabsf(cfg.pitchRate) < TRIM_MAX_RATE
             && fabsf(cfg.pitchAngle - cfg.setpoint) < TRIM_MAX_ERR_DEG;
    _calmTicks = calm ? _calmTicks + 1 : 0;
    if (_calmTicks > TRIM_CALM_TICKS) {
        cfg.setpoint = constrain(cfg.setpoint + _angleOffset * dt / TRIM_TAU_S,
                                 _spStart - TRIM_RANGE_DEG, _spStart + TRIM_RANGE_DEG);
        _trimmed = true;
    }


    // === Taxa de curva PI → diferença de velocidade entre rodas (m/s) ===
    // positivo = virar à esquerda (anti-horário visto de cima, gyro Z positivo)
    // rampa no alvo de curva: degrau vira tranco nas rodas e o giro passa do ponto ao parar
    float turnStep = TURN_ACCEL * dt;
    _rampedTurn += constrain(_targetTurnRate - _rampedTurn, -turnStep, turnStep);
    // Sem comando de giro a malha desliga: as rodas recebem a mesma velocidade e, com motor de passo, o
    // robô não gira. Ligada, ela seguiria o gyro Z, e um erro de zero dele (temperatura) vira giro lento parado.
    bool turning = _rampedTurn != 0.0f || _targetTurnRate != 0.0f;
    float turnError = turning ? _rampedTurn - cfg.yawRate : 0.0f;
    if (turning) _turnIntegral += turnError * dt;
    else _turnIntegral -= _turnIntegral * dt / I_DECAY_S;
    _turnIntegral = constrain(_turnIntegral, -30.0f, 30.0f);
    // feedforward: diferença por roda que a geometria pede (stepper não patina); o PI só corrige o resto
    float turnFF = _rampedTurn * DEG_TO_RAD * TRACK_M / 2.0f;
    _turnOffset = turnFF + cfg.yawPid.kp * turnError + cfg.yawPid.ki * _turnIntegral;
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
    _rampedTurn = 0.0f;
    _targetTurnRate = 0.0f;
    _angleOffset = 0.0f;
    _turnOffset = 0.0f;
    _velIntegral = 0.0f;
    _turnIntegral = 0.0f;
    _turnActive = false;
    _holdArmed = false;
    _estimatedVelocity = 0.0f;
    _calmTicks = 0;
    _spStart = cfg.setpoint;
    _trimmed = false;
}
