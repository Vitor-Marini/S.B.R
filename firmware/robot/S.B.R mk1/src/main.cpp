#include <Arduino.h>
#include "motor_control.hpp"
#include "pid_controller.hpp"
#include "sensors.hpp"
#include "config_manager.hpp"
#include "web_interface.hpp"

MotorController motors;
SensorMPU sensor;
WebSocketInterface webInterface;

unsigned long lastLogTime = 0;

void setup() {
  Serial.begin(115200);
  
  motors.begin();
  
  sensor.begin();

  pid.begin();

  webInterface.begin();
  
  // Carrega configurações persistentes
  if (config.load()) {
    pid.setTunings(config.kp, config.ki, config.kd, config.setpoint);
  } else {
    pid.setTunings(config.kp, config.ki, config.kd, config.setpoint);
  }


  
  Serial.println("Sistema iniciado.");
}

unsigned long lastLoopTime = 0;
const int LOOP_INTERVAL = 10; // 10ms = 100Hz

void loop() {
  unsigned long now = millis();
  
  // Garante frequência de amostragem constante para o PID
  if (now - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = now;

    config.pitchAngle = sensor.getAngle();
    
    // Failsafe: Se o robô inclinar mais de 45 graus, desliga os motores
    if (abs(config.pitchAngle) > 45.0) {
      motors.setSpeeds(0, 0);
      config.pidOutput = 0;
    } else {
      config.pidOutput = pid.compute(config.pitchAngle);
      motors.setSpeeds(config.pidOutput, config.pidOutput);
    }

    // Opcional: Enviar ângulo via WebSocket em taxa menor
    // webInterface.broadcastAngle(config.pitchAngle);
  }

  // O log pode rodar fora do loop crítico de 100Hz
  if (millis() - lastLogTime > 200) {
    config.log();
    lastLogTime = millis();
  }

  // generateStepPulses não é mais necessária aqui (está no Timer)
}
