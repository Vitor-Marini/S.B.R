#ifndef SENSORS_HPP
#define SENSORS_HPP

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class SensorMPU {
public:
  void begin();
  float getAngle(); 

private:
  Adafruit_MPU6050 mpu;
  uint32_t lastTimestamp = 0;
  float filteredAngle = 0.0;
};

#endif