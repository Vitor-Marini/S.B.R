#include "velocity.hpp"
#include "config.hpp"
#include "balance.hpp"
#include "net.hpp"
#include "imu_manager.hpp"
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
    _moveActive = false;
}

void VelocityController::setTurnRate(float degS) {
    _targetTurnRate = constrain(degS, -cfg.maxTurnRate, cfg.maxTurnRate);
    _lastCmdMs = millis();
    _turnActive = false;
    _moveActive = false;
}

void VelocityController::turnBy(float deg) {
    _targetVelocity = 0.0f;
    _turnGoal = deg;
    _turnDone = 0.0f;
    _turnStartMs = millis();
    _turnActive = true;
    _moveActive = false;
}

void VelocityController::moveBy(float m) {
    _targetTurnRate = 0.0f;
    _turnActive = false;
    _moveGoal = m;
    _moveDone = 0.0f;
    _moveStartMs = millis();
    _moveActive = true;
    _moveBraking = _moveFailed = false;
    _moveRefP = _moveRefV = 0.0f;
    _moveStartX = _x;
    _velIntegral = 0.0f;
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
        uint32_t timeout = TURN_TIMEOUT_MS + (uint32_t)(fabsf(_turnGoal) / TURN_CMD_RATE * 1500.0f);
        if (fabsf(left) < TURN_TOL_DEG || millis() - _turnStartMs > timeout) {
            term.printf(fabsf(left) < TURN_TOL_DEG ? "giro ok: %.1f°\n" : "giro desistiu (travado/no suporte?): %.1f°\n", _turnDone);
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

    if (cfg.useLqr) _updateLqr(dt);
    else _updateCascade(dt);

    // === Taxa de curva PI → diferença de velocidade entre rodas (m/s) ===
    // positivo = virar à esquerda (anti-horário visto de cima, gyro Z positivo)
    // rampa no alvo de curva: degrau vira tranco nas rodas e o giro passa do ponto ao parar
    float turnStep = TURN_ACCEL * dt;
    _rampedTurn += constrain(_targetTurnRate - _rampedTurn, -turnStep, turnStep);
    // Sem comando de giro a malha desliga: as rodas recebem a mesma velocidade e, com motor de passo, o
    // robô não gira. Ligada, ela seguiria o gyro Z, e um erro de zero dele (temperatura) vira giro lento parado.
    bool turning = _rampedTurn != 0.0f || _targetTurnRate != 0.0f;
    // Sem comando de giro as rodas giram igual: o robô não gira (salvo patinar). O que o gyro Z ler é erro de
    // zero (esquenta e muda; medido ~1,7 °/s derivando o yaw em ACTIVE): corrige devagar, como o trackGyroBias.
    if (!turning) imu.trackGyroZ();
    float turnError = turning ? _rampedTurn - cfg.yawRate : 0.0f;
    if (turning) _turnIntegral += turnError * dt;
    else _turnIntegral -= _turnIntegral * dt / I_DECAY_S;
    _turnIntegral = constrain(_turnIntegral, -30.0f, 30.0f);
    // feedforward: diferença por roda que a geometria pede (stepper não patina); o PI só corrige o resto
    float turnFF = _rampedTurn * DEG_TO_RAD * cfg.motor.trackM / 2.0f;
    _turnOffset = turnFF + cfg.yawPid.kp * turnError + cfg.yawPid.ki * _turnIntegral;
    _turnOffset = constrain(_turnOffset, -0.3f, 0.3f);
}

void VelocityController::_updateCascade(float dt) {
    // move <cm>: em vez de mandar andar e "frear no ponto" (o robô freia devagar e passava 20–40 cm), gera uma
    // trajetória de referência — posição e velocidade, trapezoidal a cfg.moveAccel, que ele consegue seguir — e
    // persegue as duas: a velocidade vira o alvo, o erro de posição entra como na malha de posição parada.
    // No fim da trajetória a âncora fica no alvo e a malha de posição faz o resto.
    float moveHold = 0.0f;
    if (_moveActive) {
        _moveDone += _estimatedVelocity * dt;
        float goal = fabsf(_moveGoal), sgn = _moveGoal >= 0.0f ? 1.0f : -1.0f;
        if (!_moveBraking) {
            _moveRefV = fminf(fminf(_moveRefV + cfg.moveAccel * dt, cfg.moveSpeed),
                              sqrtf(2.0f * cfg.moveAccel * fmaxf(goal - _moveRefP, 0.0f)));
            _moveRefP = fminf(_moveRefP + fmaxf(_moveRefV, 0.005f) * dt, goal);  // piso: não fica assintótico
            _targetVelocity = sgn * _moveRefV;
            _lastCmdMs = millis();
            moveHold = constrain(-cfg.holdKp * (_moveDone - sgn * _moveRefP), -HOLD_MAX_V, HOLD_MAX_V);
            bool timeout = millis() - _moveStartMs > MOVE_TIMEOUT_MS;
            if (_moveRefP >= goal || timeout) {
                _moveBraking = true;  // trajetória acabou: daqui pra frente é a malha de posição no alvo
                _moveFailed = timeout;
                _targetVelocity = 0.0f;
                _rampedTarget = 0.0f;
                _holdArmed = true;
                _holdPos = _moveDone - _moveGoal;
            }
        }
    }

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
        if (!_holdArmed && fabsf(_estimatedVelocity) < HOLD_ARM_SPEED) {
            _holdArmed = true;
            _holdPos = 0.0f;
        }
        if (_holdArmed) {
            _holdPos = constrain(_holdPos + _estimatedVelocity * dt, -HOLD_SLIP_M, HOLD_SLIP_M);
            holdVel = constrain(-cfg.holdKp * _holdPos, -HOLD_MAX_V, HOLD_MAX_V);
            // só responde quando assentou no alvo (3 cm = o balanço parado; 2 cm esperava sorte): um turn logo depois, com o robô ainda voltando, vira arco
            bool settled = fabsf(_holdPos) < 0.03f && fabsf(_estimatedVelocity) < HOLD_ARM_SPEED;
            if (_moveActive && _moveBraking && (settled || millis() - _moveStartMs > MOVE_TIMEOUT_MS)) {
                term.printf(_moveFailed || !settled ? "move desistiu: %.1f cm\n" : "move ok: %.1f cm\n",
                            (_moveGoal + _holdPos) * 100.0f);
                _moveActive = false;
            }
        }
    }
    float velError = _rampedTarget + holdVel + moveHold - _estimatedVelocity;

    // Parado: só o P (+ a posição acima). Andando: + integral, que vence a resistência à rolagem;
    // ao parar, ele se esvazia em ~0,2 s.
    // no move o integral fica de fora: ∫(vref − v) já é o erro de posição, que o moveHold trata; os dois juntos
    // somavam (medido: 0,36 m/s com alvo de 0,2 e 40 cm além do alvo; com vki 0, 0,24 m/s e 10 cm)
    if (driving && !_moveActive) _velIntegral += velError * dt;
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
        // A posição só consegue pedir vkp × HOLD_MAX_V (~0,3°) de correção. Setpoint mais errado que isso: a posição
        // satura e o robô vai embora devagar → aí aprende rápido. Perto do lugar, devagar (rápido sempre = oscilava).
        bool holdSaturated = cfg.holdKp > 0.0f && fabsf(_holdPos) > HOLD_MAX_V / cfg.holdKp;
        float tau = holdSaturated ? TRIM_TAU_FAST_S : TRIM_TAU_S;
        cfg.setpoint = constrain(cfg.setpoint + _angleOffset * dt / tau,
                                 _spStart - TRIM_RANGE_DEG, _spStart + TRIM_RANGE_DEG);
        _trimmed = true;
    }
}

// LQR: só gera a referência (posição, velocidade, aceleração) e os termos de posição/velocidade; os de ângulo
// estão no balance. Parado a referência fica no lugar (empurrou → volta); drive move a referência pela rampa;
// move segue a trajetória trapezoidal. θ_ref = a_ref/g: o quanto precisa inclinar pra acelerar.
void VelocityController::_updateLqr(float dt) {
    float v = _estimatedVelocity;
    _x += v * dt;
    float vr = 0.0f, ar = 0.0f;
    if (_moveActive) {
        float goal = fabsf(_moveGoal), sgn = _moveGoal >= 0.0f ? 1.0f : -1.0f;
        bool timeout = millis() - _moveStartMs > MOVE_TIMEOUT_MS;
        if (!_moveBraking) {
            float prev = _moveRefV;
            _moveRefV = fminf(fminf(_moveRefV + cfg.moveAccel * dt, cfg.moveSpeed),
                              sqrtf(2.0f * cfg.moveAccel * fmaxf(goal - _moveRefP, 0.0f)));
            _moveRefP = fminf(_moveRefP + fmaxf(_moveRefV, 0.005f) * dt, goal);
            vr = sgn * _moveRefV;
            ar = sgn * (_moveRefV - prev) / dt;
            _xRef = _moveStartX + sgn * _moveRefP;
            _lastCmdMs = millis();
            if (_moveRefP >= goal || timeout) {
                _moveBraking = true;
                _moveFailed = timeout;
                vr = ar = 0.0f;
            }
        }
        bool settled = _moveBraking && fabsf(_x - (_moveStartX + _moveGoal)) < 0.03f && fabsf(v) < HOLD_ARM_SPEED;
        if (settled || timeout) {
            term.printf(settled && !_moveFailed ? "move ok: %.1f cm\n" : "move desistiu: %.1f cm\n",
                        (_x - _moveStartX) * 100.0f);
            _moveActive = false;
        }
        _rampedTarget = vr;
    } else {
        float prev = _rampedTarget;
        float maxStep = TARGET_ACCEL * dt;
        _rampedTarget += constrain(_targetVelocity - _rampedTarget, -maxStep, maxStep);
        vr = _rampedTarget;
        ar = (_rampedTarget - prev) / dt;
        _xRef += vr * dt;
    }
    // longe demais da referência (carregado, preso): a referência vem junto, sem sair correndo depois
    // O termo de posição também segura erro de setpoint/piso inclinado: aguenta até kx·slip/kt graus (S6: ~3°).
    // Mas kx·slip não pode passar do teto (senão fica cego pra posição): slip = mín(0,3 m, 2,5 m/s² / kx).
    float slip = fminf(LQR_SLIP_M, 2.5f / fmaxf(cfg.lqrKx, 0.1f));
    float ex = _x - _xRef;
    if (fabsf(ex) > slip) {
        _xRef = _x - copysignf(slip, ex);
        ex = copysignf(slip, ex);
    }
    _lqrAccel = constrain(cfg.lqrKx * ex + cfg.lqrKv * (v - vr) + ar, -LQR_MAX_ACCEL, LQR_MAX_ACCEL);
    _angleOffset = degrees(ar / 9.81f);
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
    _moveActive = false;
    _holdArmed = false;
    _x = _xRef = _lqrAccel = 0.0f;
    _estimatedVelocity = 0.0f;
    _calmTicks = 0;
    _spStart = cfg.setpoint;
    _trimmed = false;
}
