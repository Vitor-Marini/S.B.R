#include <Arduino.h>
#include "config.hpp"
#include "motor.hpp"
#include "imu_manager.hpp"
#include "balance.hpp"
#include "supervisor.hpp"
#include "web_interface.hpp"
#include "cli.hpp"

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=== S.B.R mk2 ===");

    cfg.begin();

    motors.begin();

    supervisor.begin();

    web.begin();

    cli.begin();
}

void loop() {
    supervisor.run();

    web.broadcast();

    cli.update();

    delay(1);
}
