#include "balance.hpp"
#include "config.hpp"
#include "imu_manager.hpp"
#include "motor.hpp"
#include <math.h>

BalanceController balance;

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
    float accel = cfg.anglePid.kp * _angleError + cfg.anglePid.kd * pitchRate;

    _speed += accel * dt;
    _speed = constrain(_speed, -cfg.maxVelocity, cfg.maxVelocity);

    float stepsPerMeter = cfg.motor.stepsPerRev / (PI * cfg.motor.wheelDiameterMm * 0.001f);
    float steps = _speed * stepsPerMeter;
    motors.setSpeeds(steps + _turnOffset, steps - _turnOffset);

    cfg.pidOutput = accel;
}

void BalanceController::resetIntegral() {
    _speed = 0.0f;
    _angleError = 0.0f;
}
