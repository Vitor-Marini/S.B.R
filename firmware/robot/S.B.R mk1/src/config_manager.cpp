#include "config_manager.hpp"
#include "Arduino.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

ConfigManager config;

bool ConfigManager::load() {
  File file = LittleFS.open("/config.json", "r");
  if (!file) {
    Serial.println("Config: Arquivo não encontrado, usando padrões.");
    save(); // Cria o arquivo com padrões
    return false;
  }

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.println("Config: Erro ao ler JSON.");
    return false;
  }

  if (doc.containsKey("kp")) kp = doc["kp"];
  if (doc.containsKey("ki")) ki = doc["ki"];
  if (doc.containsKey("kd")) kd = doc["kd"];
  if (doc.containsKey("setpoint")) setpoint = doc["setpoint"];
  
  Serial.println("Config: Carregada com sucesso.");
  return true;
}

bool ConfigManager::save() {
  File file = LittleFS.open("/config.json", "w");
  if (!file) {
    Serial.println("Config: Erro ao abrir p/ escrita.");
    return false;
  }

  StaticJsonDocument<512> doc;
  doc["kp"] = kp;
  doc["ki"] = ki;
  doc["kd"] = kd;
  doc["setpoint"] = setpoint;

  if (serializeJson(doc, file) == 0) {
    Serial.println("Config: Erro ao gravar JSON.");
  }
  file.close();
  return true;
}

void ConfigManager::log() {
// ... existing log code ...
  // ANSI escape code para limpar tela
  Serial.write(27); // ESC
  Serial.print("[2J"); // Clear screen
  Serial.write(27);
  Serial.print("[H"); // Cursor para topo

  Serial.println("\r|===================== CONFIG =====================|");
  Serial.print  ("| Time        : "); Serial.println(millis());
  Serial.print  ("| SetPoint    : "); Serial.println(setpoint);
  Serial.print  ("| Pitch Angle : "); Serial.println(pitchAngle);
  Serial.print  ("| PID Output  : "); Serial.println(pidOutput);
  Serial.print  ("| KP          : "); Serial.println(kp);
  Serial.print  ("| KI          : "); Serial.println(ki);
  Serial.print  ("| KD          : "); Serial.println(kd);
  Serial.println("|==================================================|");
}