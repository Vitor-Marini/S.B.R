#include <Arduino.h>
#include "net.hpp"
#include "config.hpp"
#include "motor.hpp"
#include "imu_manager.hpp"
#include "balance.hpp"
#include "velocity.hpp"
#include "supervisor.hpp"
#include "web_interface.hpp"
#include "cli.hpp"
#include "ble_gamepad.hpp"
#include "ds4_controller.hpp"

// Fase 3: rádios desligados no bring-up (núcleo único; WiFi/BLE atrapalham o loop)
#define ENABLE_RADIOS 0

static DS4State prevDs4;
static uint32_t lastDs4Handle = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    term.println("\n\n=== S.B.R ===");

    cfg.begin();
    net.begin();

    motors.begin();

    supervisor.begin();

    velCtrl.begin();

#if ENABLE_RADIOS
    web.begin();
    bleMgr.begin();
    ds4Ctrl.begin();
#endif

    cli.begin();
}

void loop() {
#if ENABLE_RADIOS
    ds4Ctrl.update();
    bleMgr.update();

    uint32_t now = millis();
    if (now - lastDs4Handle > 20) {
        lastDs4Handle = now;

        if (ds4Ctrl.isConnected()) {
            DS4State cur = ds4Ctrl.getState();

            if (cur.triangle && !prevDs4.triangle) supervisor.requestStart();
            if (cur.circle && !prevDs4.circle)     supervisor.requestStop();
            if (cur.square && !prevDs4.square)     supervisor.requestCalibrate();
            if (cur.cross && !prevDs4.cross) {
                velCtrl.setTarget(0);
                velCtrl.setTurnRate(0);
                supervisor.requestStop();
            }

            if (cur.ps && !prevDs4.ps) {
                if (supervisor.getState() == STATE_IDLE)
                    supervisor.requestStart();
                else
                    supervisor.requestStop();
            }

            velCtrl.setTarget(cur.stickLY * cfg.maxVelocity);
            velCtrl.setTurnRate(cur.stickRX * cfg.maxTurnRate);

            prevDs4 = cur;
        }
    }
#endif

    static uint32_t lastTick = micros();
    uint32_t nowUs = micros();
    if (nowUs - lastTick >= CONTROL_PERIOD_US) {
        lastTick += CONTROL_PERIOD_US;
        // atrasou mais de um período (calibração bloqueante etc.): ressincroniza em vez de rodar em rajada
        if (nowUs - lastTick >= CONTROL_PERIOD_US) lastTick = nowUs;
        uint32_t t0 = micros();
        supervisor.run();
        uint32_t took = micros() - t0;

        static uint32_t ticks = 0, maxUs = 0, winStart = nowUs;
        ticks++;
        if (took > maxUs) maxUs = took;
        if (nowUs - winStart >= 1000000) {
            cfg.tickHz = ticks; cfg.tickMaxUs = maxUs;
            ticks = 0; maxUs = 0; winStart = nowUs;
        }
    }

#if ENABLE_RADIOS
    web.broadcast();
#endif

    cli.update();
    net.update();

    delay(1);
}
