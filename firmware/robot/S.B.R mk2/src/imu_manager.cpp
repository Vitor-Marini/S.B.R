#include "imu_manager.hpp"
#include "config.hpp"
#include <Wire.h>
#include <math.h>

IMUManager imu;

bool IMUManager::begin() {
    Wire.begin(8, 9);
    if (!_mpu.begin()) {
        Serial.println("MPU6050 não encontrado!");
        return false;
    }
    Wire.setClock(400000);

    _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    Serial.println("MPU6050 inicializado com DLPF 44Hz, I2C 400kHz");
    return true;
}

void IMUManager::calibrateGyro(int samples) {
    Serial.println("Calibrando giroscópio... mantenha o robô parado.");

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    sensors_event_t acc, gyro, temp;

    for (int i = 0; i < samples; i++) {
        _mpu.getEvent(&acc, &gyro, &temp);
        sumX += gyro.gyro.x;
        sumY += gyro.gyro.y;
        sumZ += gyro.gyro.z;
        delay(2);
    }

    _gyroOffsetX = sumX / samples;
    _gyroOffsetY = sumY / samples;
    _gyroOffsetZ = sumZ / samples;

    Serial.printf("Gyro offsets: X=%.4f  Y=%.4f  Z=%.4f\n",
                  _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ);
    Serial.println("Calibração concluída.");

    // começa o filtro já no ângulo do acelerômetro, sem transiente
    _sample();
    _pitch = _accAngle;
    _gyroAngle = 0.0f;
    _yaw = 0.0f;
}

void IMUManager::_sample() {
    sensors_event_t acc, gyro, temp;
    _mpu.getEvent(&acc, &gyro, &temp);

    _ax = acc.acceleration.x;
    _ay = acc.acceleration.y;
    _az = acc.acceleration.z;
    _temp = temp.temperature;

    _gx = gyro.gyro.x - _gyroOffsetX;
    _gy = gyro.gyro.y - _gyroOffsetY;
    _gz = gyro.gyro.z - _gyroOffsetZ;

    // eixo de queda medido no robô: inclinar p/ frente → ax sobe, gy fica negativo
    _accAngle = degrees(atan2f(_ax - cfg.filter.accBiasX, _az - cfg.filter.accBiasZ));
    _pitchRate = -degrees(_gy);
    _yawRate = degrees(_gz);
}

void IMUManager::read() {
    const float dt = CONTROL_DT;
    _sample();

    const float a = cfg.filter.complementaryAlpha;
    _pitch = a * (_pitch + _pitchRate * dt) + (1.0f - a) * _accAngle;
    _gyroAngle += _pitchRate * dt;

    _yaw += _yawRate * dt;
    if (_yaw > 180.0f) _yaw -= 360.0f;
    if (_yaw < -180.0f) _yaw += 360.0f;
}
