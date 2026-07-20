#ifndef SENSORS_HPP
#define SENSORS_HPP

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class SensorMPU {
public:
  void begin();
  void calibrate(int samples = 500);
  float getAngle(); 

private:
  Adafruit_MPU6050 mpu;
  uint32_t lastTimestamp = 0;
  float filteredAngle = 0.0f;
  float gyroOffsetY = 0.0f;
};

#endif