#include "balance.hpp"
#include "config.hpp"
#include "imu_manager.hpp"
#include "motor.hpp"
#include <math.h>

BalanceController balance;

static constexpr float LQR_ACCEL_MAX = 4.0f;  // m/s² (sem o Vref medido: conservador)

void BalanceController::begin() {
}

void BalanceController::compute() {
    const float dt = CONTROL_DT;

    float pitch = imu.getPitch();
    float pitchRate = imu.getPitchRate();

    cfg.pitchAngle = pitch;
    cfg.pitchRate = pitchRate;
    cfg.yawAngle = imu.getYaw();
    cfg.yawRate = imu.getYawRate();

    // pitch positivo = caindo p/ frente → acelerar p/ frente (comando positivo)
    _angleError = pitch - (cfg.setpoint + _angleOffset);
    // LQR: mesmos dois termos com os ganhos do LQR + posição/velocidade (vindos do velocity, em _extra)
    float kp = cfg.useLqr ? cfg.lqrKt : cfg.anglePid.kp, kd = cfg.useLqr ? cfg.lqrKw : cfg.anglePid.kd;
    float accel = kp * _angleError + kd * pitchRate + _extra;
    // LQR: teto no que o motor entrega de verdade. Acima disso o stepper perde passo, a roda real fica pra trás da
    // comandada (e da odometria) e o controle perde a referência: os empurrões pediam 11 m/s² e caía. O LQR com
    // teto honesto aguenta empurrões maiores (labs/03-lqr/lqr.py) que o PD.
    if (cfg.useLqr) accel = constrain(accel, -LQR_ACCEL_MAX, LQR_ACCEL_MAX);

    // roda = equilíbrio + velocidade pedida (feedforward). Andar não depende de inclinar:
    // com o centro de massa baixo o contato do pneu segura ~2° de inclinação sem rolar.
    _speed += accel * dt;
    _wheel = constrain(_speed + _ff, -cfg.maxVelocity, cfg.maxVelocity);
    _speed = _wheel - _ff;  // limita o total, sem acumular além do vmax

    float stepsPerMeter = cfg.motor.stepsPerRev / (PI * cfg.motor.wheelDiameterMm * 0.001f);
    // motor "L" (GPIO0) é a roda ESQUERDA (medido 2026-11-21): R mais rápido → vira à esquerda (giro +)
    motors.setSpeeds((_wheel - _turnOffset) * stepsPerMeter, (_wheel + _turnOffset) * stepsPerMeter);

    cfg.pidOutput = accel;
}

void BalanceController::resetIntegral() {
    _speed = 0.0f;
    _ff = 0.0f;
    _wheel = 0.0f;
    _angleError = 0.0f;
}
