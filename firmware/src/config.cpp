#include "config.hpp"
#include "net.hpp"
#include <LittleFS.h>
#include <ArduinoJson.h>

ConfigManager cfg;

void ConfigManager::begin() {
    if (!LittleFS.begin(true)) {
        term.println("LittleFS mount failed");
    }
    load();
}

bool ConfigManager::load() {
    File file = LittleFS.open("/config.json", "r");
    if (!file) {
        term.println("Config: arquivo não encontrado, usando padrões.");
        save();
        return false;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, file);
    file.close();

    if (err) {
        term.println("Config: erro JSON, usando padrões.");
        return false;
    }

    auto readGains = [&](const char* prefix, PIDGains& g) {
        String kpKey = String(prefix) + "_kp";
        String kiKey = String(prefix) + "_ki";
        String kdKey = String(prefix) + "_kd";
        if (!doc[kpKey].isNull()) g.kp = doc[kpKey].as<float>();
        if (!doc[kiKey].isNull()) g.ki = doc[kiKey].as<float>();
        if (!doc[kdKey].isNull()) g.kd = doc[kdKey].as<float>();
    };

    readGains("bal", anglePid);  // chave nova: ignora ganhos antigos em outra unidade
    readGains("turn", yawPid);  // chave nova: ignora ganhos antigos em passos/s
    readGains("spd", velocityPid);  // chave nova: ignora ganhos antigos

    if (!doc["setpoint"].isNull())       setpoint = doc["setpoint"].as<float>();
    if (!doc["failsafe_angle"].isNull()) failsafeAngle = doc["failsafe_angle"].as<float>();

    if (!doc["steps_per_rev"].isNull())      motor.stepsPerRev = doc["steps_per_rev"].as<float>();
    if (!doc["wheel_diameter_mm"].isNull())  motor.wheelDiameterMm = doc["wheel_diameter_mm"].as<float>();
    if (!doc["max_accel"].isNull())          motor.maxAccelStepsS2 = doc["max_accel"].as<float>();
    if (doc["left_reversed"].is<bool>())     motor.leftReversed = doc["left_reversed"].as<bool>();
    if (doc["right_reversed"].is<bool>())    motor.rightReversed = doc["right_reversed"].as<bool>();

    if (!doc["acc_bias_x"].isNull()) filter.accBiasX = doc["acc_bias_x"].as<float>();
    if (!doc["acc_bias_z"].isNull()) filter.accBiasZ = doc["acc_bias_z"].as<float>();
    if (!doc["comp_alpha"].isNull()) filter.complementaryAlpha = doc["comp_alpha"].as<float>();
    if (!doc["gyro_cross"].isNull()) filter.gyroCross = doc["gyro_cross"].as<float>();

    if (!doc["max_velocity"].isNull()) maxVelocity = doc["max_velocity"].as<float>();
    if (!doc["max_turn_rate"].isNull()) maxTurnRate = doc["max_turn_rate"].as<float>();
    if (!doc["hold_kp"].isNull()) holdKp = doc["hold_kp"].as<float>();
    if (!doc["move_accel"].isNull()) moveAccel = doc["move_accel"].as<float>();
    if (!doc["move_speed"].isNull()) moveSpeed = doc["move_speed"].as<float>();
    if (!doc["wifi_ssid"].isNull()) wifiSsid = doc["wifi_ssid"].as<String>();
    if (!doc["wifi_pass"].isNull()) wifiPass = doc["wifi_pass"].as<String>();

    term.println("Config: carregada.");
    return true;
}

bool ConfigManager::save() {
    // grava em arquivo temporário e renomeia: queda de bateria no meio não corrompe a config
    File file = LittleFS.open("/config.tmp", "w");
    if (!file) {
        term.println("Config: erro ao salvar.");
        return false;
    }

    JsonDocument doc;

    auto writeGains = [&](const char* prefix, const PIDGains& g) {
        doc[String(prefix) + "_kp"] = g.kp;
        doc[String(prefix) + "_ki"] = g.ki;
        doc[String(prefix) + "_kd"] = g.kd;
    };

    writeGains("bal", anglePid);
    writeGains("turn", yawPid);
    writeGains("spd", velocityPid);

    doc["setpoint"] = setpoint;
    doc["failsafe_angle"] = failsafeAngle;

    doc["steps_per_rev"] = motor.stepsPerRev;
    doc["wheel_diameter_mm"] = motor.wheelDiameterMm;
    doc["max_accel"] = motor.maxAccelStepsS2;
    doc["left_reversed"] = motor.leftReversed;
    doc["right_reversed"] = motor.rightReversed;

    doc["acc_bias_x"] = filter.accBiasX;
    doc["acc_bias_z"] = filter.accBiasZ;
    doc["comp_alpha"] = filter.complementaryAlpha;
    doc["gyro_cross"] = filter.gyroCross;

    doc["max_velocity"] = maxVelocity;
    doc["max_turn_rate"] = maxTurnRate;
    doc["hold_kp"] = holdKp;
    doc["move_accel"] = moveAccel;
    doc["move_speed"] = moveSpeed;
    doc["wifi_ssid"] = wifiSsid;
    doc["wifi_pass"] = wifiPass;

    bool ok = serializeJson(doc, file) > 0;
    file.close();
    if (!ok || !LittleFS.rename("/config.tmp", "/config.json")) {
        term.println("Config: erro ao salvar.");
        return false;
    }
    dirty = false;
    return true;
}

void ConfigManager::print() {
    term.println("\n========== CONFIG ==========");
    term.printf("Setpoint:     %.1f\n", setpoint);
    term.printf("Failsafe:     %.1f\n", failsafeAngle);
    term.print ("--- Balance PD (m/s² por ° e por °/s) ---\n");
    term.printf("  Kp: %.2f  Ki: %.3f  Kd: %.3f\n", anglePid.kp, anglePid.ki, anglePid.kd);
    term.print ("--- Yaw PID ---\n");
    term.printf("  Kp: %.4f  Ki: %.3f  Kd: %.3f\n", yawPid.kp, yawPid.ki, yawPid.kd);
    term.print ("--- Motor ---\n");
    term.printf("  Steps/rev:   %.0f\n", motor.stepsPerRev);
    term.printf("  Wheel (mm):  %.1f\n", motor.wheelDiameterMm);
    term.printf("  Max accel:   %.0f\n", motor.maxAccelStepsS2);
    term.printf("  L rev: %d  R rev: %d\n", motor.leftReversed, motor.rightReversed);
    term.print ("--- Velocity ---\n");
    term.printf("  PI: Kp %.2f  Ki %.2f (° por m/s)\n", velocityPid.kp, velocityPid.ki);
    term.printf("  Max V:  %.2f m/s\n", maxVelocity);
    term.printf("  Max T:  %.1f °/s\n", maxTurnRate);
    term.printf("  Posição: hkp %.2f  move: mva %.2f m/s²  mvs %.2f m/s\n", holdKp, moveAccel, moveSpeed);
    term.print ("--- Filter ---\n");
    term.printf("  Acc bias X: %.2f  Z: %.2f\n", filter.accBiasX, filter.accBiasZ);
    term.printf("  Comp alpha: %.3f  Gyro cross (gxz): %.4f\n", filter.complementaryAlpha, filter.gyroCross);
    term.print ("--- Telemetry ---\n");
    term.printf("  State:  %d\n", robotState);
    term.printf("  Pitch:  %.2f\n", pitchAngle);
    term.printf("  Rate:   %.2f\n", pitchRate);
    term.printf("  Yaw:    %.2f\n", yawAngle);
    term.printf("  Output: %.2f\n", pidOutput);
    term.printf("  L spd:  %.1f  R spd: %.1f\n", leftSpeed, rightSpeed);
    term.println("=============================\n");
}
