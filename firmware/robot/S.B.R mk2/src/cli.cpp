#include "cli.hpp"
#include "config.hpp"
#include "supervisor.hpp"
#include "imu_manager.hpp"
#include "balance.hpp"
#include "motor.hpp"
#include "velocity.hpp"
#include "ds4_controller.hpp"
#include <Wire.h>

CLI cli;

void CLI::begin() {
    Serial.println("\nS.B.R mk2 — Terminal Interativo");
    Serial.println("Digite 'help' para comandos disponíveis.");
    _printPrompt();
}

void CLI::update() {
    // formato Teleplot (">nome:valor"), 50 Hz
    if (_streaming && millis() - _lastStream >= 20) {
        _lastStream = millis();
        Serial.printf(">pitch:%.2f\n>acc:%.2f\n>gyroint:%.2f\n>rate:%.2f\n>yaw:%.2f\n>speed:%.3f\n>accel:%.2f\n>err:%.2f\n>voff:%.2f\n>state:%d\n>tick:%d\n>temp:%.1f\n",
                      imu.getPitch(), imu.getAccAngle(), imu.getGyroAngle(),
                      imu.getPitchRate(), imu.getYaw(), balance.getSpeed(), cfg.pidOutput,
                      balance.getAngleError(), velCtrl.getAngleOffset(),
                      cfg.robotState, cfg.tickHz, imu.getTemp());
    }
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (_buffer.length() > 0) {
                _processLine(_buffer);
                _buffer = "";
            }
            _printPrompt();
        } else if (c == '\b' || c == 127) {
            if (_buffer.length() > 0) _buffer.remove(_buffer.length() - 1);
        } else {
            _buffer += c;
        }
    }
}

void CLI::_processLine(const String& line) {
    String cmd = line;
    cmd.trim();

    if (cmd.length() == 0) return;

    int spaceIdx = cmd.indexOf(' ');
    String command = (spaceIdx > 0) ? cmd.substring(0, spaceIdx) : cmd;
    String args = (spaceIdx > 0) ? cmd.substring(spaceIdx + 1) : "";
    command.toLowerCase();
    args.trim();

    if (command == "help") {
        _printHelp();
    } else if (command == "start") {
        supervisor.requestStart();
    } else if (command == "stop") {
        supervisor.requestStop();
        velCtrl.setTarget(0.0f);
        velCtrl.setTurnRate(0.0f);
    } else if (command == "calibrate" || command == "cal") {
        supervisor.requestCalibrate();
    } else if (command == "status" || command == "st") {
        cfg.print();
    } else if (command == "state") {
        Serial.printf("Estado atual: %s\n", supervisor.getStateName());
    } else if (command == "tune") {
        // tune <kp|ki|kd|setpoint> <value>
        int space2 = args.indexOf(' ');
        if (space2 > 0) {
            String param = args.substring(0, space2);
            float value = args.substring(space2 + 1).toFloat();
            param.toLowerCase();

            if (param == "kp")      { cfg.anglePid.kp = value; }
            else if (param == "ki") { cfg.anglePid.ki = value; }
            else if (param == "kd") { cfg.anglePid.kd = value; }
            else if (param == "sp" || param == "setpoint") { cfg.setpoint = value; }
            else if (param == "ykp") { cfg.yawPid.kp = value; }
            else if (param == "yki") { cfg.yawPid.ki = value; }
            else if (param == "ykd") { cfg.yawPid.kd = value; }
            else if (param == "spr" && value > 0) { cfg.motor.stepsPerRev = value; }
            else if (param == "accel" && value > 0) { cfg.motor.maxAccelStepsS2 = value; }
            else if (param == "vmax" && value > 0) { cfg.maxVelocity = value; }
            else if (param == "vkp") { cfg.velocityPid.kp = value; }
            else if (param == "vki") { cfg.velocityPid.ki = value; }
            else if (param == "abx") { cfg.filter.accBiasX = value; }
            else if (param == "abz") { cfg.filter.accBiasZ = value; }
            else if (param == "alpha" && value > 0 && value < 1) { cfg.filter.complementaryAlpha = value; }
            else { Serial.println("Parâmetro inválido. Use: kp, ki, kd, sp, ykp, yki, ykd, spr, accel, vmax, vkp, vki, abx, abz, alpha"); return; }

            cfg.save();
            Serial.printf("%s = %.3f\n", param.c_str(), value);
        } else {
            Serial.println("Use: tune <param> <value>");
        }
    } else if (command == "motor") {
        // motor <left|right> <speed>
        int space2 = args.indexOf(' ');
        if (space2 > 0) {
            String which = args.substring(0, space2);
            float speed = args.substring(space2 + 1).toFloat();
            which.toLowerCase();

            float motorMax = cfg.motor.stepsPerRev * 3.0f;
            float steps = speed; // speed is in steps/sec directly

            if (which == "left") {
                motors.setSpeeds(steps, 0);
                Serial.printf("Motor L: %.0f steps/s\n", steps);
            } else if (which == "right") {
                motors.setSpeeds(0, steps);
                Serial.printf("Motor R: %.0f steps/s\n", steps);
            } else if (which == "both") {
                motors.setSpeeds(steps, steps);
                Serial.printf("Motor L+R: %.0f steps/s\n", steps);
            } else {
                Serial.println("Use: motor <left|right|both> <steps/s>");
            }
        } else {
            Serial.println("Use: motor <left|right|both> <steps/s>");
        }
    } else if (command == "save") {
        cfg.save();
        Serial.println("Config salva.");
    } else if (command == "load") {
        cfg.load();
        Serial.println("Config carregada.");
    } else if (command == "angle") {
        // não chama imu.read(): só o tick de controle lê o IMU (dt fixo)
        Serial.printf("Pitch: %.2f°  Acc: %.2f°  Gyro-int: %.2f°  Rate: %.2f°/s  Yaw: %.2f°\n",
                      imu.getPitch(), imu.getAccAngle(), imu.getGyroAngle(),
                      imu.getPitchRate(), imu.getYaw());
        Serial.printf("Tick: %d Hz  pior run: %lu us\n", cfg.tickHz, (unsigned long)cfg.tickMaxUs);
    } else if (command == "raw") {
        Serial.printf("acc m/s2: %6.2f %6.2f %6.2f | gyro deg/s: %7.2f %7.2f %7.2f\n",
                      imu.getAccX(), imu.getAccY(), imu.getAccZ(),
                      degrees(imu.getGyroX()), degrees(imu.getGyroY()), degrees(imu.getGyroZ()));
        Serial.printf("temp: %.1f C\n", imu.getTemp());
    } else if (command == "stream") {
        _streaming = !_streaming;
        if (!_streaming) Serial.println("stream off");
    } else if (command == "regs") {
        // esperado: 75=68 6B=01 19=00 1A=03 1B=08 1C=10
        for (uint8_t r : {0x75, 0x6B, 0x19, 0x1A, 0x1B, 0x1C}) {
            Wire.beginTransmission(0x68);
            Wire.write(r);
            Wire.endTransmission(false);
            Wire.requestFrom(0x68, 1);
            Serial.printf("reg 0x%02X = 0x%02X\n", r, Wire.available() ? Wire.read() : 0xFF);
        }
    } else if (command == "vel") {
        float val = args.toFloat();
        velCtrl.setTarget(val);
        Serial.printf("Velocidade alvo: %.2f m/s\n", val);
    } else if (command == "turn") {
        float val = args.toFloat();
        velCtrl.setTurnRate(val);
        Serial.printf("Curva alvo: %.1f °/s\n", val);
    } else if (command == "pair") {
        ds4Ctrl.startPairing();
    } else if (command == "ds4") {
        Serial.printf("DS4: %s\n", ds4Ctrl.isConnected() ? "Conectado" : "Desconectado");
        if (ds4Ctrl.isConnected()) {
            auto& s = ds4Ctrl.getState();
            Serial.printf("  Endereco: %s\n", ds4Ctrl.getAddress().c_str());
            Serial.printf("  Stick L: %.2f  Stick R: %.2f\n", s.stickLY, s.stickRX);
            Serial.printf("  Botoes: T=%d C=%d X=%d S=%d | PS=%d\n",
                          s.triangle, s.circle, s.cross, s.square, s.ps);
        }
    } else if (command == "reset") {
        motors.resetOdometry();
        balance.resetIntegral();
        velCtrl.reset();
        Serial.println("Odometria, integrais e velocidade resetados.");
    } else {
        Serial.printf("Comando desconhecido: %s\n", command.c_str());
        Serial.println("Digite 'help' para comandos.");
    }
}

void CLI::_printPrompt() {
    Serial.print("sbr> ");
}

void CLI::_printHelp() {
    Serial.println("\n--- Comandos ---");
    Serial.println("  start              → Iniciar balanceamento");
    Serial.println("  stop               → Parar balanceamento");
    Serial.println("  calibrate (cal)    → Recalibrar giroscópio");
    Serial.println("  status (st)        → Mostrar configuração e telemetria");
    Serial.println("  state              → Mostrar estado atual");
    Serial.println("  tune <kp|ki|kd|sp|ykp|yki|ykd|spr|accel|vmax|vkp|vki|abx|abz|alpha> <val> → Ajustar PID/motor");
    Serial.println("  motor <L|R|both> <steps/s> → Teste manual de motor");
    Serial.println("  vel <m/s>         → Velocidade alvo (ex: 0.5)");
    Serial.println("  turn <°/s>        → Curva alvo (ex: -30)");
    Serial.println("  angle             → Ler ângulo atual");
    Serial.println("  raw               → Acelerômetro, gyro e temperatura");
    Serial.println("  stream            → Liga/desliga gráficos (Teleplot) a 50 Hz");
    Serial.println("  save              → Salvar config na flash");
    Serial.println("  load              → Carregar config da flash");
    Serial.println("  reset             → Resetar odometria e integrais");
    Serial.println("  pair              → Escanear e conectar DS4 via BLE");
    Serial.println("  ds4               → Status do controle DS4");
    Serial.println("  help              → Este menu");
    Serial.println("----------------\n");
}
