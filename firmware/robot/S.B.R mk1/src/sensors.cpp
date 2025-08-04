#include <Wire.h>
#include "sensors.hpp"
#include <MPU6050_light.h>

//FICAR DE OLHO TALVEZ MUDAR A LIB DO MPU, POR CAUSA DA AUTO CALIBRAÇAO

MPU6050 mpu(Wire);

void SensorMPU::begin() {
  Wire.begin();
  mpu.begin();
  calibrate();
}

void SensorMPU::calibrate() {
  Serial.println("Calibrando MPU6050...");
  mpu.calcOffsets(true, true);
  Serial.println("Calibração completa.");
}

float SensorMPU::getAngle() {
  mpu.update();
  return mpu.getAngleY() - angleOffset; ///TA PEGANDO ANGLO ERRADO EU ACHO SE PA COLOCAR PARA PEGAR O Y
}
