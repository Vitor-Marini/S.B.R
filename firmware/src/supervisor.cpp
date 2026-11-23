#include "supervisor.hpp"
#include "net.hpp"
#include "config.hpp"
#include "imu_manager.hpp"
#include "balance.hpp"
#include "motor.hpp"
#include "velocity.hpp"

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
    term.print(">> Estado: ");
    term.println(getStateName());

    // queda: descarta o que o auto-ajuste aprendeu nesta sessão (pode ter sido no começo da queda)
    if (_state == STATE_FAULT) cfg.setpoint = velCtrl.spAtStart();

    // motores já parados nestes estados: hora segura de gravar o que foi ajustado em ACTIVE
    if (cfg.dirty && (_state == STATE_IDLE || _state == STATE_FAULT)) {
        motors.stop();
        cfg.save();
    }

    switch (_state) {
        case STATE_ACTIVE:
        case STATE_MOVING:
            balance.resetIntegral();  // começa com roda parada
            velCtrl.reset();
            break;
        case STATE_STARTING:
            motors.stop();
            term.println("Segure o robô em pé no ponto de equilíbrio...");
            break;
        case STATE_FAULT:
            motors.stop();
            balance.resetIntegral();
            balance.setTurnOffset(0.0f);
            velCtrl.reset();
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
        term.println("Falha no MPU6050. Tentando novamente...");
        delay(1000);
    }
}

void Supervisor::_runCalibrate() {
    motors.stop();  // calibrar com motor vibrando estraga o offset
    if (imu.calibrateGyro(500)) _setState(STATE_IDLE);  // senão tenta de novo no próximo tick
}

void Supervisor::_runIdle() {
    motors.update();  // rampa do comando 'motor' do CLI
    imu.read();
    imu.trackGyroBias();
    cfg.pitchAngle = imu.getPitch();
    if (!imu.healthy()) _restartImu();
}

// MPU sem resposta ou reiniciado (queda de energia/mau contato): reconfigura e recalibra
void Supervisor::_restartImu() {
    term.println("IMU sem resposta ou reiniciado: reconfigurando (deixe o robô parado)...");
    motors.stop();
    _setState(STATE_INIT);
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
        term.println("Não ficou em pé em 10 s. Voltando para IDLE.");
        _nearSince = 0;
        _setState(STATE_IDLE);
    }
}

void Supervisor::_runActive() {
    imu.read();

    if (!imu.healthy()) {
        term.println("FALHA: IMU sem resposta (I2C).");
        _setState(STATE_FAULT);
        return;
    }

    float angle = imu.getPitch();
    if (fabsf(angle) > cfg.failsafeAngle) {
        term.printf("FALHA: Ângulo %.1f° excede limite %.1f°\n", angle, cfg.failsafeAngle);
        _setState(STATE_FAULT);
        return;
    }

    // rodas no limite por 1 s = disparou (ex.: levantado com o equilíbrio ligado): desliga em vez de
    // sair correndo quando voltar pro chão
    if (fabsf(balance.getSpeed()) >= 0.95f * cfg.maxVelocity) {
        if (!_satSince) _satSince = millis();
        else if (millis() - _satSince > 1000) {
            term.println("FALHA: rodas no limite de velocidade por 1 s (levantado ou disparou).");
            _satSince = 0;
            _setState(STATE_FAULT);
            return;
        }
    } else {
        _satSince = 0;
    }

    // malha de velocidade → offset de ângulo; malha de curva → diferença entre rodas
    velCtrl.update();
    balance.setAngleOffset(velCtrl.getAngleOffset());
    balance.setTurnOffset(velCtrl.getTurnOffset());
    balance.setSpeedFF(cfg.useLqr ? 0.0f : velCtrl.getRampedTarget());  // no LQR a velocidade entra por kv
    balance.setExtraAccel(cfg.useLqr ? velCtrl.getLqrAccel() : 0.0f);
    balance.compute();
}

void Supervisor::_runMoving() {
    _runActive();
}

void Supervisor::_runFault() {
    motors.stop();
    imu.read();  // sem isso o ângulo congela e o FAULT nunca sai
    if (!imu.healthy()) { _restartImu(); return; }

    float angle = imu.getPitch();
    if (fabsf(angle) < 15.0f) {
        if (_faultStartMs == 0) {
            _faultStartMs = millis();
        } else if (millis() - _faultStartMs > 3000) {
            term.println("Recuperação de falha. Voltando para IDLE.");
            _faultStartMs = 0;
            balance.resetIntegral();
            _setState(STATE_IDLE);
        }
    } else {
        _faultStartMs = 0;
    }
}

void Supervisor::requestStart() {
    // de FAULT também: depois de cair, pegar o robô e dar start sem esperar os 3 s da recuperação
    if (_state == STATE_IDLE || _state == STATE_FAULT) {
        balance.resetIntegral();
        _setState(STATE_STARTING);
    }
}

void Supervisor::requestStop() {
    // stop de propósito: guarda o setpoint que o auto-ajuste aprendeu (gravado ao entrar em IDLE)
    if (_state == STATE_ACTIVE && velCtrl.trimmed()) {
        term.printf("Setpoint ajustado de %.2f° para %.2f° (salvo).\n", velCtrl.spAtStart(), cfg.setpoint);
        cfg.dirty = true;
    }
    if (_state == STATE_ACTIVE || _state == STATE_MOVING || _state == STATE_STARTING) {
        _setState(STATE_IDLE);
    }
}

void Supervisor::requestCalibrate() {
    if (_state == STATE_IDLE) {
        _setState(STATE_CALIBRATE);
    }
}
