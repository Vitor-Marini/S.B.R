#include <Arduino.h>
#include "net.hpp"
#include "config.hpp"
#include "motor.hpp"
#include "imu_manager.hpp"
#include "balance.hpp"
#include "velocity.hpp"
#include "supervisor.hpp"
#include "cli.hpp"
#include "web.hpp"

void setup() {
    Serial.begin(115200);
    // USB ligada sem ninguém lendo: desiste em ~2 ms em vez de travar o loop 100 ms. NÃO usar 0: no
    // HWCDC o contador de tentativas dá a volta e o write trava pra sempre.
    Serial.setTxTimeoutMs(1);
    delay(500);
    term.println("\n\n=== S.B.R ===");

    cfg.begin();
    net.begin();

    motors.begin();

    supervisor.begin();

    velCtrl.begin();

    cli.begin();
}

void loop() {
    static uint32_t lastTick = micros();
    uint32_t nowUs = micros();
    if (nowUs - lastTick >= CONTROL_PERIOD_US) {
        lastTick += CONTROL_PERIOD_US;
        // atrasou mais de um período (calibração bloqueante etc.): ressincroniza em vez de rodar em rajada
        if (nowUs - lastTick >= CONTROL_PERIOD_US) lastTick = nowUs;
        uint32_t t0 = micros();
        supervisor.run();
        uint32_t took = micros() - t0;

        // maior intervalo entre ciclos: mostra se algo (WiFi, página web) atrasa o controle
        static uint32_t ticks = 0, maxUs = 0, maxGap = 0, prevStart = t0, winStart = nowUs;
        ticks++;
        if (took > maxUs) maxUs = took;
        if (t0 - prevStart > maxGap) maxGap = t0 - prevStart;
        prevStart = t0;
        if (nowUs - winStart >= 1000000) {
            cfg.tickHz = ticks; cfg.tickMaxUs = maxUs; cfg.tickMaxGapUs = maxGap;
            ticks = 0; maxUs = 0; maxGap = 0; winStart = nowUs;
        }
    }


    cli.update();
    net.update();
    web.update();

    delay(1);
}
