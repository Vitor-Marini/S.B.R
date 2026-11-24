#include "cli.hpp"
#include "net.hpp"
#include "config.hpp"
#include "supervisor.hpp"
#include "imu_manager.hpp"
#include "balance.hpp"
#include "motor.hpp"
#include "velocity.hpp"
#include "rl_policy.h"
#include <Wire.h>

CLI cli;

void CLI::begin() {
    term.println("\nS.B.R — Terminal Interativo");
    term.println("Digite 'help' para comandos disponíveis.");
    _printPrompt();
}

// rumo pelas rodas (°, + = esquerda, sem dar a volta): compara com o yaw do gyro para ver se o giro foi real
static float wheelHeading() {
    float stepsPerMeter = cfg.motor.stepsPerRev / (PI * cfg.motor.wheelDiameterMm * 0.001f);
    return degrees((motors.getRightSteps() - motors.getLeftSteps()) / stepsPerMeter / cfg.motor.trackM);
}

void CLI::update() {
    // formato Teleplot (">nome:valor"), 50 Hz
    if (_streaming && millis() - _lastStream >= 20) {
        _lastStream = millis();
        term.printf(">pitch:%.2f\n>acc:%.2f\n>gyroint:%.2f\n>rate:%.2f\n>yaw:%.2f\n>speed:%.3f\n>accel:%.2f\n>err:%.2f\n>voff:%.2f\n>vtgt:%.3f\n>yrate:%.1f\n>wtgt:%.0f\n>state:%d\n>tick:%d\n>head:%.1f\n>temp:%.1f\n",
                      imu.getPitch(), imu.getAccAngle(), imu.getGyroAngle(),
                      imu.getPitchRate(), imu.getYaw(), balance.getSpeed(), cfg.pidOutput,
                      balance.getAngleError(), velCtrl.getAngleOffset(), velCtrl.getTarget(),
                      cfg.yawRate, velCtrl.getTurnRate(),
                      cfg.robotState, cfg.tickHz, wheelHeading(), imu.getTemp());
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
        term.printf("Estado atual: %s\n", supervisor.getStateName());
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
            else if (param == "hkp" && value >= 0) { cfg.holdKp = value; }
            else if (param == "mva" && value > 0) { cfg.moveAccel = value; }
            else if (param == "mvs" && value > 0) { cfg.moveSpeed = value; }
            else if (param == "lkx") { cfg.lqrKx = value; }
            else if (param == "lkv") { cfg.lqrKv = value; }
            else if (param == "lkt") { cfg.lqrKt = value; }
            else if (param == "lkw") { cfg.lqrKw = value; }
            else if (param == "lrev") { cfg.motor.leftReversed = value != 0; }   // inverte o sentido do motor L
            else if (param == "rrev") { cfg.motor.rightReversed = value != 0; }
            else if (param == "gxz") { cfg.filter.gyroCross = value; }
            else if (param == "gzs" && value > 0.8f && value < 1.2f) { cfg.filter.gyroZScale = value; }
            else if (param == "track" && value > 0.10f && value < 0.20f) { cfg.motor.trackM = value; }
            else if (param == "abx") { cfg.filter.accBiasX = value; }
            else if (param == "abz") { cfg.filter.accBiasZ = value; }
            else if (param == "alpha" && value > 0 && value < 1) { cfg.filter.complementaryAlpha = value; }
            else { term.println("Parâmetro inválido. Use: kp, ki, kd, sp, ykp, yki, ykd, spr, accel, vmax, vkp, vki, lrev, rrev, gxz, abx, abz, alpha"); return; }

            if (supervisor.getState() == STATE_ACTIVE) {
                cfg.dirty = true;  // supervisor grava ao sair do ACTIVE
                term.printf("%s = %.3f (grava ao parar)\n", param.c_str(), value);
            } else {
                cfg.save();
                term.printf("%s = %.3f\n", param.c_str(), value);
            }
        } else {
            term.println("Use: tune <param> <value>");
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
                term.printf("Motor L: %.0f steps/s\n", steps);
            } else if (which == "right") {
                motors.setSpeeds(0, steps);
                term.printf("Motor R: %.0f steps/s\n", steps);
            } else if (which == "both") {
                motors.setSpeeds(steps, steps);
                term.printf("Motor L+R: %.0f steps/s\n", steps);
            } else {
                term.println("Use: motor <left|right|both> <steps/s>");
            }
        } else {
            term.println("Use: motor <left|right|both> <steps/s>");
        }
    } else if (command == "save") {
        cfg.save();
        term.println("Config salva.");
    } else if (command == "load") {
        cfg.load();
        term.println("Config carregada.");
    } else if (command == "angle") {
        // não chama imu.read(): só o tick de controle lê o IMU (dt fixo)
        term.printf("Pitch: %.2f°  Acc: %.2f°  Gyro-int: %.2f°  Rate: %.2f°/s  Yaw: %.2f°\n",
                      imu.getPitch(), imu.getAccAngle(), imu.getGyroAngle(),
                      imu.getPitchRate(), imu.getYaw());
        term.printf("Tick: %d Hz  pior run: %lu us  maior intervalo: %lu us\n", cfg.tickHz,
                    (unsigned long)cfg.tickMaxUs, (unsigned long)cfg.tickMaxGapUs);
        term.printf("Alvo: %.2f m/s  %.0f °/s  |  real: %.2f m/s  %.0f °/s\n",
                      velCtrl.getTarget(), velCtrl.getTurnRate(), balance.getSpeed(), cfg.yawRate);
    } else if (command == "raw") {
        term.printf("acc m/s2: %6.2f %6.2f %6.2f | gyro deg/s: %7.2f %7.2f %7.2f\n",
                      imu.getAccX(), imu.getAccY(), imu.getAccZ(),
                      degrees(imu.getGyroX()), degrees(imu.getGyroY()), degrees(imu.getGyroZ()));
        term.printf("temp: %.1f C  falhas I2C: %lu\n", imu.getTemp(), (unsigned long)imu.getFailCount());
    } else if (command == "stream") {
        // stream on | off (sem argumento alterna); "on" explícito é seguro de repetir
        if (args == "on") _streaming = true;
        else if (args == "off") _streaming = false;
        else _streaming = !_streaming;
        if (!_streaming) term.println("stream off");
    } else if (command == "wifi") {
        // wifi <rede> <senha> → salva e conecta; wifi → status. (rede sem espaços)
        int sp = args.indexOf(' ');
        if (sp > 0) {
            cfg.wifiSsid = args.substring(0, sp);
            cfg.wifiPass = args.substring(sp + 1);
            cfg.save();
            net.begin();
        } else {
            net.printStatus();
        }
    } else if (command == "regs") {
        // esperado: 75=68 6B=01 19=00 1A=03 1B=08 1C=10
        for (uint8_t r : {0x75, 0x6B, 0x19, 0x1A, 0x1B, 0x1C}) {
            Wire.beginTransmission(0x68);
            Wire.write(r);
            Wire.endTransmission(false);
            Wire.requestFrom(0x68, 1);
            term.printf("reg 0x%02X = 0x%02X\n", r, Wire.available() ? Wire.read() : 0xFF);
        }
    } else if (command == "motors") {
        // motors <L> <R> passos/s — teste remoto (gamepad). Só em IDLE; vale 0,5 s: repetir a 10–20 Hz.
        // L/R são os motores do firmware: "L" (GPIO0) é a roda ESQUERDA do robô (olhando pra frente).
        if (supervisor.getState() == STATE_IDLE) {
            int sp = args.indexOf(' ');
            motors.setSpeedsTimed(args.substring(0, sp > 0 ? sp : args.length()).toFloat(),
                                  sp > 0 ? args.substring(sp + 1).toFloat() : 0.0f, 500);
        }
    } else if (command == "drive") {
        // drive <m/s> <°/s>  — API única de movimento (PC, câmera, IA...). Sem resposta, para
        // poder ser enviado a 10–20 Hz; parar de enviar por 0,5 s = parar o robô.
        int sp = args.indexOf(' ');
        velCtrl.setTarget(args.substring(0, sp > 0 ? sp : args.length()).toFloat());
        velCtrl.setTurnRate(sp > 0 ? args.substring(sp + 1).toFloat() : 0.0f);
    } else if (command == "turn") {
        // turn <graus> — gira no lugar (+ = esquerda) e responde "giro ok"; só equilibrando
        if (supervisor.getState() == STATE_ACTIVE) velCtrl.turnBy(args.toFloat());
        else term.println("turn: só com o robô equilibrando (start)");
    } else if (command == "ctl") {
        // ctl [lqr|pid|rl] — troca o controle (só fora do equilíbrio: trocar no meio dá tranco)
        args.toLowerCase();
        RobotState st = supervisor.getState();
        if (args.length() && (st == STATE_ACTIVE || st == STATE_STARTING)) term.println("ctl: dê stop antes de trocar");
        else if (args == "lqr") { cfg.useLqr = true; cfg.useRl = false; }
        else if (args == "pid") { cfg.useLqr = false; cfg.useRl = false; }
        else if (args == "rl") { cfg.useLqr = true; cfg.useRl = true; }  // usa a referência do LQR (drive/move)
        term.printf("Controle: %s\n", cfg.useRl ? "RL (rede; não salva: religou, volta pro LQR)"
                                                : cfg.useLqr ? "LQR (save pra manter)" : "PD + cascata (save pra manter)");
    } else if (command == "rltime") {
        // rltime — quanto a rede leva na ESP (sem FPU: ponto flutuante emulado). O loop inteiro tem 5000 us
        float in[5] = {0.1f, -0.2f, 0.05f, 0.1f, 0.0f}, out = 0.0f;
        uint32_t t0 = micros();
        for (int i = 0; i < 100; i++) { in[4] = out * 0.001f; out = rl_policy(in); }
        term.printf("rede: %.0f us por chamada (saida %.3f)\n", (micros() - t0) / 100.0f, out);
    } else if (command == "move") {
        // move <cm> — anda e para no alvo (+ = frente), responde "move ok"; só equilibrando
        if (supervisor.getState() == STATE_ACTIVE) velCtrl.moveBy(args.toFloat() / 100.0f);
        else term.println("move: só com o robô equilibrando (start)");
    } else if (command == "reset") {
        motors.resetOdometry();
        balance.resetIntegral();
        velCtrl.reset();
        term.println("Odometria, integrais e velocidade resetados.");
    } else {
        term.printf("Comando desconhecido: %s\n", command.c_str());
        term.println("Digite 'help' para comandos.");
    }
}

void CLI::_printPrompt() {
    term.print("sbr> ");
}

void CLI::_printHelp() {
    term.println("\n--- Comandos ---");
    term.println("  start              → Iniciar balanceamento");
    term.println("  stop               → Parar balanceamento");
    term.println("  calibrate (cal)    → Recalibrar giroscópio");
    term.println("  status (st)        → Mostrar configuração e telemetria");
    term.println("  state              → Mostrar estado atual");
    term.println("  tune <kp|ki|kd|sp|ykp|yki|ykd|spr|accel|vmax|vkp|vki|hkp|mva|mvs|lkt|lkw|lkx|lkv|lrev|rrev|gxz|gzs|track|abx|abz|alpha> <val> → Ajustar PID/motor");
    term.println("  motor <L|R|both> <steps/s> → Teste manual de motor");
    term.println("  drive <m/s> <°/s> → Andar/virar (+ = frente / esquerda). Repetir a 10-20 Hz");
    term.println("  turn <graus>      → Girar no lugar (+ = esquerda), ex.: turn 90");
    term.println("  ctl [lqr|pid|rl]  → Controle: LQR, PD + cascata ou a rede do labs/07 (fora do equilíbrio)");
    term.println("  rltime            → Tempo da rede na ESP (us)");
    term.println("  move <cm>         → Andar e parar no alvo (+ = frente), ex.: move 50");
    term.println("  motors <L> <R>    → Teste remoto dos motores (passos/s), só IDLE, vale 0,5 s");
    term.println("  angle             → Ler ângulo atual");
    term.println("  raw               → Acelerômetro, gyro e temperatura");
    term.println("  stream [on|off]   → Gráficos (PlotJuggler/Teleplot) a 50 Hz");
    term.println("  save              → Salvar config na flash");
    term.println("  load              → Carregar config da flash");
    term.println("  reset             → Resetar odometria e integrais");
    term.println("  wifi [<rede> <senha>] → Status / salvar rede de casa (UDP 5555, sbr.local)");
    term.println("  help              → Este menu");
    term.println("----------------\n");
}
