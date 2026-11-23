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
    bool calibrateGyro(int samples = 500);  // false se o robô se mexeu (zero não alterado)
    void read();  // chamar exatamente a cada CONTROL_DT
    void trackGyroBias();  // só com o robô parado (IDLE)
    void trackGyroZ();     // só o Z, equilibrando sem comando de giro (as rodas dizem que não está girando)

    float getPitch() { return _pitch; }
    float getPitchRate() { return _pitchRate; }
    float getYaw() { return _yaw; }
    float getYawRate() { return _yawRate; }
    float getAccAngle() { return _accAngle; }
    float getGyroAngle() { return _gyroAngle; }  // diagnóstico: só gyro integrado, zera no cal
    float getTemp() { return _temp; }
    bool healthy() { return _failStreak < 10; }   // 10 leituras seguidas = 50 ms sem IMU
    uint32_t getFailCount() { return _failCount; }  // °C do sensor interno do MPU

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

    float _accFast = 0.0f, _accSlow = 0.0f, _gRef = 9.81f;
    uint8_t _failStreak = 0;
    uint32_t _failCount = 0;

    bool _readRaw(float out[7]);
    void _sample();
};

extern IMUManager imu;
#endif
