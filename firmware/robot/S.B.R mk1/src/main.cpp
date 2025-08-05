#include <Arduino.h>
#include "motor_control.hpp"
#include "pid_controller.hpp"
#include "sensors.hpp"
#include "system_state.hpp"
#include "web_interface.hpp"

MotorController motors;
PIDController pid;
SensorMPU sensor;
SystemState systemState;
WebSocketInterface webInterface;

void setup() {
  Serial.begin(115200);
  
  motors.begin();
  
  sensor.begin();

  pid.begin();

  webInterface.begin();

  pid.setTunings(10, 180, 1.0);

  systemState.setPoint = pid.viewSetPoint();

  Serial.println("Sistema iniciado.");
}

void loop() {
  systemState.angle = sensor.getAngle();
  systemState.pidOutput = pid.compute(systemState.angle);

  motors.setSpeeds(systemState.pidOutput, systemState.pidOutput);
  motors.generateStepPulses();

  systemState.log();

  delayMicroseconds(200);
}
