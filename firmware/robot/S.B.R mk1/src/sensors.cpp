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

void SensorMPU::calibrate(int samples) {
  Serial.println("Calibrando giroscopio... mantenha o robo parado.");

  float sumGyroY = 0.0f;

  for (int i = 0; i < samples; i++) {
    sensors_event_t acc, gyro, temp;
    mpu.getEvent(&acc, &gyro, &temp);
    sumGyroY += gyro.gyro.y;
    delay(2);
  }

  gyroOffsetY = sumGyroY / samples;

  Serial.print("Gyro Y offset: ");
  Serial.println(gyroOffsetY, 6);
  Serial.println("Calibracao concluida.");

  lastTimestamp = millis();
}

float SensorMPU::getAngle() {
  const float angleOffset = 3.64f; 
  const float ALPHA = 0.98f; // Peso do giroscópio (estabilidade curto prazo)

  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);

  uint32_t now = millis();
  float dt = (now - lastTimestamp) / 1000.0f;
  lastTimestamp = now;
  if (dt > 0.1f || dt <= 0.0f) dt = 0.01f; // Proteção contra saltos no tempo

  // Ângulo pelo acelerômetro
  float accAngle = atan2f(acc.acceleration.x, acc.acceleration.z) * 180.0f / PI;
  float adjustedAccAngle = accAngle - angleOffset;

  // Gyro com compensação de offset da calibração
  float gyroRate = (gyro.gyro.y - gyroOffsetY) * 180.0f / PI;

  // Filtro Complementar: Ângulo = ALPHA * (Ângulo + Giro * dt) + (1 - ALPHA) * AccAngle
  filteredAngle = ALPHA * (filteredAngle + gyroRate * dt) + (1.0f - ALPHA) * adjustedAccAngle;

  return filteredAngle;
}
