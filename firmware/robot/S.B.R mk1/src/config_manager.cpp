#include "config_manager.hpp"
#include"Arduino.h"

ConfigManager config;

void ConfigManager::log() {
  Serial.println("|===================== CONFIG =====================|");
  Serial.print("Time: ");Serial.println(millis());
  Serial.print  ("| SetPoint    : "); Serial.println(setpoint);
  Serial.print  ("| Pitch Angle : "); Serial.println(pitchAngle);
  Serial.print  ("| PID Output  : "); Serial.println(pidOutput);
  Serial.print  ("| KP          : "); Serial.println(kp);
  Serial.print  ("| KI          : "); Serial.println(ki);
  Serial.print  ("| KD          : "); Serial.println(kd);
  Serial.println("|==================================================|");
  Serial.println(); 
}