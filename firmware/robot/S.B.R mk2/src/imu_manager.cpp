#include "imu_manager.hpp"
#include "config.hpp"
#include "filters.hpp"
#include <Wire.h>
#include <math.h>

IMUManager imu;

static MahonyFilter mahonyFilter;
static ComplementaryFilter compFilter;

bool IMUManager::begin() {
    Wire.begin();
    if (!_mpu.begin()) {
        Serial.println("MPU6050 não encontrado!");
        return false;
    }

    _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    _mahony = &mahonyFilter;
    _mahony->begin(cfg.filter.mahonyKp, cfg.filter.mahonyKi);

    _complementary = &compFilter;
    _complementary->begin(cfg.filter.complementaryAlpha);

    Serial.println("MPU6050 inicializado com DLPF 44Hz");
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

    _calibrated = true;
    Serial.printf("Gyro offsets: X=%.4f  Y=%.4f  Z=%.4f\n",
                  _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ);
    Serial.println("Calibração concluída.");

    _lastRead = millis();
}

void IMUManager::read() {
    uint32_t now = millis();
    float dt = (now - _lastRead) / 1000.0f;
    if (dt > 0.1f || dt <= 0.0f) dt = 0.01f;
    _lastRead = now;

    sensors_event_t acc, gyro, temp;
    _mpu.getEvent(&acc, &gyro, &temp);

    _ax = acc.acceleration.x;
    _ay = acc.acceleration.y;
    _az = acc.acceleration.z;

    _gx = gyro.gyro.x - _gyroOffsetX;
    _gy = gyro.gyro.y - _gyroOffsetY;
    _gz = gyro.gyro.z - _gyroOffsetZ;

    if (_useMahony) {
        _mahony->update(_gx, _gy, _gz, _ax, _ay, _az, dt);
        _pitch = _mahony->getPitch();
        _pitchRate = _mahony->getPitchRate();
        _yaw = _mahony->getYaw();
        _yawRate = _mahony->getYawRate();
    } else {
        float accAngle = atan2f(_ax, _az) * 180.0f / PI;
        _complementary->update(accAngle, _gy, dt);
        _pitch = _complementary->getAngle();
        _pitchRate = _gy * 180.0f / PI;
        _yaw += _gz * dt * 180.0f / PI;
        _yawRate = _gz * 180.0f / PI;
    }
}

void IMUManager::setUseMahony(bool use) {
    _useMahony = use;
}

float IMUManager::getPitch() { return _pitch; }
float IMUManager::getPitchRate() { return _pitchRate; }
float IMUManager::getYaw() { return _yaw; }
float IMUManager::getYawRate() { return _yawRate; }
