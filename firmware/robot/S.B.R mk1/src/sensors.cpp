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
  const float DEADZONE_THRESHOLD = 0.5; 

  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);

  float pitch = atan2(acc.acceleration.x, acc.acceleration.z) * 180.0 / PI;
  float adjustedPitch = pitch - angleOffset;


  if (abs(adjustedPitch) < DEADZONE_THRESHOLD) {
    return 0.0;
  }

  return adjustedPitch;
}

