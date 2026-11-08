#include "supervisor.hpp"
#include "config.hpp"
#include "imu_manager.hpp"
#include "balance.hpp"
#include "motor.hpp"
#include "velocity.hpp"
#include "ble_gamepad.hpp"

Supervisor supervisor;

void requestBalanceStart() { supervisor.requestStart(); }
void requestBalanceStop() { supervisor.requestStop(); }

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
            balance.resetIntegral();  // começa com roda parada
            velCtrl.reset();
            break;
        case STATE_STARTING:
            motors.stop();
            Serial.println("Segure o robô em pé no ponto de equilíbrio...");
            break;
        case STATE_FAULT:
            motors.stop();
            balance.resetIntegral();
            break;
        case STATE_IDLE:
            motors.stop();
            balance.resetIntegral();
            velCtrl.reset();
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
    motors.stop();  // calibrar com motor vibrando estraga o offset
    imu.calibrateGyro(500);
    _setState(STATE_IDLE);
}

void Supervisor::_runIdle() {
    motors.update();  // rampa do comando 'motor' do CLI
    imu.read();
    cfg.pitchAngle = imu.getPitch();
}

// Motores parados até o robô ficar perto do setpoint por 300 ms; aí liga o equilíbrio.
void Supervisor::_runStarting() {
    imu.read();
    cfg.pitchAngle = imu.getPitch();

    if (fabsf(imu.getPitch() - cfg.setpoint) < 3.0f) {
        if (_nearSince == 0) _nearSince = millis();
        else if (millis() - _nearSince > 300) {
            _nearSince = 0;
            _setState(STATE_ACTIVE);
        }
    } else {
        _nearSince = 0;
    }

    if (millis() - _stateStartMs > 10000) {
        Serial.println("Não ficou em pé em 10 s. Voltando para IDLE.");
        _nearSince = 0;
        _setState(STATE_IDLE);
    }
}

void Supervisor::_runActive() {
    imu.read();

    float angle = imu.getPitch();
    if (fabsf(angle) > cfg.failsafeAngle) {
        Serial.printf("FALHA: Ângulo %.1f° excede limite %.1f°\n", angle, cfg.failsafeAngle);
        velCtrl.reset();
        _setState(STATE_FAULT);
        return;
    }

    // Fase 4: malha de velocidade → offset de ângulo. Curva (Fase 5) ainda desligada.
    velCtrl.update();
    balance.setAngleOffset(velCtrl.getAngleOffset());
    balance.compute();
}

void Supervisor::_runMoving() {
    _runActive();
}

void Supervisor::_runFault() {
    motors.stop();
    imu.read();  // sem isso o ângulo congela e o FAULT nunca sai

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
