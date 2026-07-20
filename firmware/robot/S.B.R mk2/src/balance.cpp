#include "balance.hpp"
#include "config.hpp"
#include "imu_manager.hpp"
#include "motor.hpp"
#include <math.h>

BalanceController balance;

void BalanceController::begin() {
    _lastCompute = millis();
}

void BalanceController::compute() {
    uint32_t now = millis();
    float dt = (now - _lastCompute) / 1000.0f;
    if (dt > 0.1f || dt <= 0.0f) dt = 0.01f;
    _lastCompute = now;

    float pitch = imu.getPitch();
    float pitchRate = imu.getPitchRate();
    float yawRate = imu.getYawRate();

    cfg.pitchAngle = pitch;
    cfg.pitchRate = pitchRate;
    cfg.yawAngle = imu.getYaw();
    cfg.yawRate = yawRate;

    // ===== Angle PID (inner loop) =====
    _angleError = cfg.setpoint - pitch;

    float pTerm = cfg.anglePid.kp * _angleError;
    float dTerm = cfg.anglePid.kd * (-pitchRate);
    float rawDerivative = (_angleError - _prevError) / dt;
    _filteredDerivative = 0.8f * _filteredDerivative + 0.2f * rawDerivative;
    _prevError = _angleError;

    if (cfg.anglePid.ki > 0.0f) {
        _integral += _angleError * dt;
        _integral = constrain(_integral, -20.0f, 20.0f);
    } else {
        _integral = 0.0f;
    }

    _torque = pTerm + cfg.anglePid.ki * _integral + cfg.anglePid.kd * _filteredDerivative;
    _torque = -_torque;
    _torque = constrain(_torque, -100.0f, 100.0f);

    // ===== Yaw PD =====
    float yawError = 0.0f - imu.getYaw();
    if (yawError > 180.0f) yawError -= 360.0f;
    if (yawError < -180.0f) yawError += 360.0f;

    _yawIntegral += yawError * dt;
    _yawIntegral = constrain(_yawIntegral, -5.0f, 5.0f);

    _yawCorrection = cfg.yawPid.kp * yawError
                   + cfg.yawPid.ki * _yawIntegral
                   + cfg.yawPid.kd * (-yawRate);
    _yawCorrection = constrain(_yawCorrection, -30.0f, 30.0f);

    // ===== Motor output =====
    float leftMotor = _torque + _yawCorrection;
    float rightMotor = _torque - _yawCorrection;

    float maxMotor = max(fabsf(leftMotor), fabsf(rightMotor));
    if (maxMotor > 100.0f) {
        leftMotor = leftMotor / maxMotor * 100.0f;
        rightMotor = rightMotor / maxMotor * 100.0f;
    }

    float motorMaxSteps = cfg.motor.stepsPerRev * 3.0f;
    float leftSteps = leftMotor / 100.0f * motorMaxSteps;
    float rightSteps = rightMotor / 100.0f * motorMaxSteps;

    motors.setSpeeds(leftSteps, rightSteps);

    cfg.pidOutput = _torque;
}

void BalanceController::resetIntegral() {
    _integral = 0.0f;
    _prevError = 0.0f;
    _filteredDerivative = 0.0f;
    _yawIntegral = 0.0f;
}
