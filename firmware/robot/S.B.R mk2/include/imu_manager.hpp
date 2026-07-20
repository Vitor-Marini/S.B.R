#ifndef IMU_MANAGER_HPP
#define IMU_MANAGER_HPP

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class IMUManager {
public:
    bool begin();
    void calibrateGyro(int samples = 500);
    void read();
    void setUseMahony(bool use);

    float getPitch();
    float getPitchRate();
    float getYaw();
    float getYawRate();

    float getAccX() { return _ax; }
    float getAccY() { return _ay; }
    float getAccZ() { return _az; }

private:
    Adafruit_MPU6050 _mpu;
    uint32_t _lastRead = 0;

    float _ax = 0.0f, _ay = 0.0f, _az = 0.0f;
    float _gx = 0.0f, _gy = 0.0f, _gz = 0.0f;
    float _gyroOffsetX = 0.0f, _gyroOffsetY = 0.0f, _gyroOffsetZ = 0.0f;

    float _pitch = 0.0f;
    float _pitchRate = 0.0f;
    float _yaw = 0.0f;
    float _yawRate = 0.0f;

    bool _useMahony = true;
    bool _calibrated = false;

    class MahonyFilter* _mahony = nullptr;
    class ComplementaryFilter* _complementary = nullptr;
};

extern IMUManager imu;
#endif
