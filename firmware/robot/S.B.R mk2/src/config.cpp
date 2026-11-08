#include "config.hpp"
#include <LittleFS.h>
#include <ArduinoJson.h>

ConfigManager cfg;

void ConfigManager::begin() {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed");
    }
    load();
}

bool ConfigManager::load() {
    File file = LittleFS.open("/config.json", "r");
    if (!file) {
        Serial.println("Config: arquivo não encontrado, usando padrões.");
        save();
        return false;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, file);
    file.close();

    if (err) {
        Serial.println("Config: erro JSON, usando padrões.");
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
    readGains("yaw", yawPid);
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

    if (!doc["max_velocity"].isNull()) maxVelocity = doc["max_velocity"].as<float>();
    if (!doc["max_turn_rate"].isNull()) maxTurnRate = doc["max_turn_rate"].as<float>();
    if (doc["ble_enabled"].is<bool>()) bleEnabled = doc["ble_enabled"].as<bool>();

    Serial.println("Config: carregada.");
    return true;
}

bool ConfigManager::save() {
    File file = LittleFS.open("/config.json", "w");
    if (!file) {
        Serial.println("Config: erro ao salvar.");
        return false;
    }

    JsonDocument doc;

    auto writeGains = [&](const char* prefix, const PIDGains& g) {
        doc[String(prefix) + "_kp"] = g.kp;
        doc[String(prefix) + "_ki"] = g.ki;
        doc[String(prefix) + "_kd"] = g.kd;
    };

    writeGains("bal", anglePid);
    writeGains("yaw", yawPid);
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

    doc["max_velocity"] = maxVelocity;
    doc["max_turn_rate"] = maxTurnRate;
    doc["ble_enabled"] = bleEnabled;

    serializeJson(doc, file);
    file.close();
    return true;
}

void ConfigManager::print() {
    Serial.println("\n========== CONFIG ==========");
    Serial.printf("Setpoint:     %.1f\n", setpoint);
    Serial.printf("Failsafe:     %.1f\n", failsafeAngle);
    Serial.print ("--- Balance PD (m/s² por ° e por °/s) ---\n");
    Serial.printf("  Kp: %.2f  Ki: %.3f  Kd: %.3f\n", anglePid.kp, anglePid.ki, anglePid.kd);
    Serial.print ("--- Yaw PID ---\n");
    Serial.printf("  Kp: %.2f  Ki: %.3f  Kd: %.3f\n", yawPid.kp, yawPid.ki, yawPid.kd);
    Serial.print ("--- Motor ---\n");
    Serial.printf("  Steps/rev:   %.0f\n", motor.stepsPerRev);
    Serial.printf("  Wheel (mm):  %.1f\n", motor.wheelDiameterMm);
    Serial.printf("  Max accel:   %.0f\n", motor.maxAccelStepsS2);
    Serial.printf("  L rev: %d  R rev: %d\n", motor.leftReversed, motor.rightReversed);
    Serial.print ("--- Velocity ---\n");
    Serial.printf("  PI: Kp %.2f  Ki %.2f (° por m/s)\n", velocityPid.kp, velocityPid.ki);
    Serial.printf("  Max V:  %.2f m/s\n", maxVelocity);
    Serial.printf("  Max T:  %.1f °/s\n", maxTurnRate);
    Serial.printf("  BLE:    %s\n", bleEnabled ? "ON" : "OFF");
    Serial.print ("--- Filter ---\n");
    Serial.printf("  Acc bias X: %.2f  Z: %.2f\n", filter.accBiasX, filter.accBiasZ);
    Serial.printf("  Comp alpha: %.3f\n", filter.complementaryAlpha);
    Serial.print ("--- Telemetry ---\n");
    Serial.printf("  State:  %d\n", robotState);
    Serial.printf("  Pitch:  %.2f\n", pitchAngle);
    Serial.printf("  Rate:   %.2f\n", pitchRate);
    Serial.printf("  Yaw:    %.2f\n", yawAngle);
    Serial.printf("  Output: %.2f\n", pidOutput);
    Serial.printf("  L spd:  %.1f  R spd: %.1f\n", leftSpeed, rightSpeed);
    Serial.println("=============================\n");
}
