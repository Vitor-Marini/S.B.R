#include "cli.hpp"
#include "config.hpp"
#include "supervisor.hpp"
#include "imu_manager.hpp"
#include "balance.hpp"
#include "motor.hpp"

CLI cli;

void CLI::begin() {
    Serial.println("\nS.B.R mk2 — Terminal Interativo");
    Serial.println("Digite 'help' para comandos disponíveis.");
    _printPrompt();
}

void CLI::update() {
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
            else { Serial.println("Parâmetro inválido. Use: kp, ki, kd, sp, ykp, yki, ykd"); return; }

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
        imu.read();
        Serial.printf("Pitch: %.2f°  Rate: %.2f rad/s  Yaw: %.2f°\n",
                      imu.getPitch(), imu.getPitchRate(), imu.getYaw());
    } else if (command == "reset") {
        motors.resetOdometry();
        balance.resetIntegral();
        Serial.println("Odometria e integrais resetados.");
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
    Serial.println("  tune <kp|ki|kd|sp|ykp|yki|ykd> <val> → Ajustar PID");
    Serial.println("  motor <L|R|both> <steps/s> → Teste manual de motor");
    Serial.println("  angle              → Ler ângulo atual");
    Serial.println("  save               → Salvar config na flash");
    Serial.println("  load               → Carregar config da flash");
    Serial.println("  reset              → Resetar odometria e integrais");
    Serial.println("  help               → Este menu");
    Serial.println("----------------\n");
}
