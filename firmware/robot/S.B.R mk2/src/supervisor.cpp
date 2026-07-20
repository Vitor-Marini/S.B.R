#include "supervisor.hpp"
#include "config.hpp"
#include "imu_manager.hpp"
#include "balance.hpp"
#include "motor.hpp"

Supervisor supervisor;

void Supervisor::begin() {
    _setState(STATE_INIT);
}

void Supervisor::run() {
    cfg.robotState = (int)_state;

    switch (_state) {
        case STATE_INIT:      _runInit(); break;
        case STATE_CALIBRATE: _runCalibrate(); break;
        case STATE_IDLE:      _runIdle(); break;
        case STATE_STARTING:  _runStarting(); break;
        case STATE_ACTIVE:    _runActive(); break;
        case STATE_MOVING:    _runMoving(); break;
        case STATE_FAULT:     _runFault(); break;
    }
}

const char* Supervisor::getStateName() {
    switch (_state) {
        case STATE_INIT:      return "INIT";
        case STATE_CALIBRATE: return "CALIBRATE";
        case STATE_IDLE:      return "IDLE";
        case STATE_STARTING:  return "STARTING";
        case STATE_ACTIVE:    return "ACTIVE";
        case STATE_MOVING:    return "MOVING";
        case STATE_FAULT:     return "FAULT";
        default:              return "UNKNOWN";
    }
}

void Supervisor::_setState(RobotState newState) {
    _state = newState;
    _stateStartMs = millis();
    _onEnterState();
}

void Supervisor::_onEnterState() {
    Serial.print(">> Estado: ");
    Serial.println(getStateName());

    switch (_state) {
        case STATE_ACTIVE:
        case STATE_MOVING:
            break;
        case STATE_FAULT:
            motors.stop();
            break;
        case STATE_IDLE:
            motors.stop();
            balance.resetIntegral();
            break;
        default:
            break;
    }
}

void Supervisor::_runInit() {
    if (imu.begin()) {
        _setState(STATE_CALIBRATE);
    } else {
        Serial.println("Falha no MPU6050. Tentando novamente...");
        delay(1000);
    }
}

void Supervisor::_runCalibrate() {
    imu.calibrateGyro(500);
    _setState(STATE_IDLE);
}

void Supervisor::_runIdle() {
    imu.read();
    cfg.pitchAngle = imu.getPitch();
}

void Supervisor::_runStarting() {
    float elapsed = (millis() - _stateStartMs) / 1000.0f;
    float progress = constrain(elapsed / 2.0f, 0.0f, 1.0f);

    imu.read();

    if (progress >= 1.0f) {
        _setState(STATE_ACTIVE);
        Serial.println("Rampa de inicialização concluída.");
        return;
    }

    float kp = cfg.anglePid.kp * progress;
    float kd = cfg.anglePid.kd * progress;
    float error = cfg.setpoint - imu.getPitch();
    float torque = -(kp * error + kd * (-imu.getPitchRate()));
    torque = constrain(torque, -100.0f, 100.0f);
    float steps = torque / 100.0f * cfg.motor.stepsPerRev * 3.0f;
    motors.setSpeeds(steps, steps);
}

void Supervisor::_runActive() {
    imu.read();

    float angle = imu.getPitch();
    if (fabsf(angle) > cfg.failsafeAngle) {
        Serial.printf("FALHA: Ângulo %.1f° excede limite %.1f°\n", angle, cfg.failsafeAngle);
        _setState(STATE_FAULT);
        return;
    }

    balance.compute();
}

void Supervisor::_runMoving() {
    _runActive();
}

void Supervisor::_runFault() {
    motors.stop();

    float angle = imu.getPitch();
    if (fabsf(angle) < 15.0f) {
        if (_faultStartMs == 0) {
            _faultStartMs = millis();
        } else if (millis() - _faultStartMs > 3000) {
            Serial.println("Recuperação de falha. Voltando para IDLE.");
            _faultStartMs = 0;
            balance.resetIntegral();
            _setState(STATE_IDLE);
        }
    } else {
        _faultStartMs = 0;
    }
}

void Supervisor::requestStart() {
    if (_state == STATE_IDLE) {
        balance.resetIntegral();
        _setState(STATE_STARTING);
    }
}

void Supervisor::requestStop() {
    if (_state == STATE_ACTIVE || _state == STATE_MOVING || _state == STATE_STARTING) {
        _setState(STATE_IDLE);
    }
}

void Supervisor::requestCalibrate() {
    if (_state == STATE_IDLE) {
        _setState(STATE_CALIBRATE);
    }
}
