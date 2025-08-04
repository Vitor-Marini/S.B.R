// === Arquivo: src/system_state.cpp ===
#include "system_state.hpp"

void SystemState::log() {
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" | PID Output: ");
  Serial.print(pidOutput);
  Serial.print(" | Direction: ");
  Serial.println(direction);
}
