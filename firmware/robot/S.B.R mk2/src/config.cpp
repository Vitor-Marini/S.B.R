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

    StaticJsonDocument<1024> doc;
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
        if (doc.containsKey(kpKey)) g.kp = doc[kpKey];
        if (doc.containsKey(kiKey)) g.ki = doc[kiKey];
        if (doc.containsKey(kdKey)) g.kd = doc[kdKey];
    };

    readGains("angle", anglePid);
    readGains("yaw", yawPid);
    readGains("vel", velocityPid);

    if (doc.containsKey("setpoint"))       setpoint = doc["setpoint"];
    if (doc.containsKey("failsafe_angle")) failsafeAngle = doc["failsafe_angle"];

    if (doc.containsKey("steps_per_rev"))      motor.stepsPerRev = doc["steps_per_rev"];
    if (doc.containsKey("wheel_diameter_mm"))  motor.wheelDiameterMm = doc["wheel_diameter_mm"];
    if (doc.containsKey("max_accel"))          motor.maxAccelStepsS2 = doc["max_accel"];
    if (doc.containsKey("left_reversed"))      motor.leftReversed = doc["left_reversed"];
    if (doc.containsKey("right_reversed"))     motor.rightReversed = doc["right_reversed"];

    if (doc.containsKey("mahony_kp")) filter.mahonyKp = doc["mahony_kp"];
    if (doc.containsKey("mahony_ki")) filter.mahonyKi = doc["mahony_ki"];
    if (doc.containsKey("comp_alpha")) filter.complementaryAlpha = doc["comp_alpha"];

    Serial.println("Config: carregada.");
    return true;
}

bool ConfigManager::save() {
    File file = LittleFS.open("/config.json", "w");
    if (!file) {
        Serial.println("Config: erro ao salvar.");
        return false;
    }

    StaticJsonDocument<1024> doc;

    auto writeGains = [&](const char* prefix, const PIDGains& g) {
        doc[String(prefix) + "_kp"] = g.kp;
        doc[String(prefix) + "_ki"] = g.ki;
        doc[String(prefix) + "_kd"] = g.kd;
    };

    writeGains("angle", anglePid);
    writeGains("yaw", yawPid);
    writeGains("vel", velocityPid);

    doc["setpoint"] = setpoint;
    doc["failsafe_angle"] = failsafeAngle;

    doc["steps_per_rev"] = motor.stepsPerRev;
    doc["wheel_diameter_mm"] = motor.wheelDiameterMm;
    doc["max_accel"] = motor.maxAccelStepsS2;
    doc["left_reversed"] = motor.leftReversed;
    doc["right_reversed"] = motor.rightReversed;

    doc["mahony_kp"] = filter.mahonyKp;
    doc["mahony_ki"] = filter.mahonyKi;
    doc["comp_alpha"] = filter.complementaryAlpha;

    serializeJson(doc, file);
    file.close();
    return true;
}

void ConfigManager::print() {
    Serial.println("\n========== CONFIG ==========");
    Serial.printf("Setpoint:     %.1f\n", setpoint);
    Serial.printf("Failsafe:     %.1f\n", failsafeAngle);
    Serial.print ("--- Angle PID ---\n");
    Serial.printf("  Kp: %.2f  Ki: %.3f  Kd: %.3f\n", anglePid.kp, anglePid.ki, anglePid.kd);
    Serial.print ("--- Yaw PID ---\n");
    Serial.printf("  Kp: %.2f  Ki: %.3f  Kd: %.3f\n", yawPid.kp, yawPid.ki, yawPid.kd);
    Serial.print ("--- Motor ---\n");
    Serial.printf("  Steps/rev:   %.0f\n", motor.stepsPerRev);
    Serial.printf("  Wheel (mm):  %.1f\n", motor.wheelDiameterMm);
    Serial.printf("  Max accel:   %.0f\n", motor.maxAccelStepsS2);
    Serial.printf("  L rev: %d  R rev: %d\n", motor.leftReversed, motor.rightReversed);
    Serial.print ("--- Filter ---\n");
    Serial.printf("  Mahony Kp: %.3f  Ki: %.4f\n", filter.mahonyKp, filter.mahonyKi);
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
