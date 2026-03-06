#include "sensors.hpp"
#include <Wire.h>
#include <math.h>

void SensorMPU::begin() {
  Wire.begin();
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}



float SensorMPU::getAngle() {
  const float angleOffset = 3.64; 
  const float ALPHA = 0.98; // Peso do giroscópio (estabilidade curto prazo)

  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);

  uint32_t now = millis();
  float dt = (now - lastTimestamp) / 1000.0;
  lastTimestamp = now;
  if (dt > 0.1 || dt <= 0) dt = 0.01; // Proteção contra saltos no tempo

  // Ângulo pelo acelerômetro
  float accAngle = atan2(acc.acceleration.x, acc.acceleration.z) * 180.0 / PI;
  float adjustedAccAngle = accAngle - angleOffset;

  // Filtro Complementar: Ângulo = ALPHA * (Ângulo + Giro * dt) + (1 - ALPHA) * AccAngle
  // Nota: gyro.gyro.y é o eixo de inclinação (pitch) dependendo da orientação do sensor
  filteredAngle = ALPHA * (filteredAngle + (gyro.gyro.y * 180.0 / PI) * dt) + (1.0 - ALPHA) * adjustedAccAngle;

  return filteredAngle;
}

