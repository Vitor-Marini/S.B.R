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

  pid.setTunings(config.kp, config.ki, config.kd, config.setpoint);/////////CONFERIR


  
  Serial.println("Sistema iniciado.");
}

void loop() {
  config.pitchAngle= sensor.getAngle();
  config.pidOutput = pid.compute(config.pitchAngle);

  motors.setSpeeds(config.pidOutput, config.pidOutput);
  motors.generateStepPulses();

  
  if (millis() - lastLogTime > 200) {
    config.log();
    lastLogTime = millis();
  }

  delayMicroseconds(200);
}
