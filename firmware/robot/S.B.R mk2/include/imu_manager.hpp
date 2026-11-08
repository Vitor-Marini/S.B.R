#ifndef IMU_MANAGER_HPP
#define IMU_MANAGER_HPP

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Ângulos em graus, taxas em °/s. Pitch positivo = inclinado para a frente
// (lado para onde as rodas andam com comando positivo).
class IMUManager {
public:
    bool begin();
    void calibrateGyro(int samples = 500);
    void read();  // chamar exatamente a cada CONTROL_DT

    float getPitch() { return _pitch; }
    float getPitchRate() { return _pitchRate; }
    float getYaw() { return _yaw; }
    float getYawRate() { return _yawRate; }
    float getAccAngle() { return _accAngle; }
    float getGyroAngle() { return _gyroAngle; }  // diagnóstico: só gyro integrado, zera no cal
    float getTemp() { return _temp; }  // °C do sensor interno do MPU

    float getAccX() { return _ax; }
    float getAccY() { return _ay; }
    float getAccZ() { return _az; }
    float getGyroX() { return _gx; }  // rad/s, sem offset
    float getGyroY() { return _gy; }
    float getGyroZ() { return _gz; }

private:
    Adafruit_MPU6050 _mpu;

    float _ax = 0.0f, _ay = 0.0f, _az = 0.0f;
    float _gx = 0.0f, _gy = 0.0f, _gz = 0.0f;
    float _gyroOffsetX = 0.0f, _gyroOffsetY = 0.0f, _gyroOffsetZ = 0.0f;

    float _pitch = 0.0f;
    float _pitchRate = 0.0f;
    float _accAngle = 0.0f;
    float _gyroAngle = 0.0f;
    float _yaw = 0.0f;
    float _yawRate = 0.0f;
    float _temp = 0.0f;

    void _sample();
};

extern IMUManager imu;
#endif
