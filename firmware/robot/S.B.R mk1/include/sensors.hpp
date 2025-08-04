#ifndef SENSORS_HPP
#define SENSORS_HPP

class SensorMPU {
public:
  void begin();
  float getAngle();

private:
  void calibrate();
  float angleOffset = 0.0;
};

#endif