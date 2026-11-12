#include "imu_manager.hpp"
#include "net.hpp"
#include "config.hpp"
#include <Wire.h>
#include <math.h>

IMUManager imu;

bool IMUManager::begin() {
    Wire.begin(8, 9);
    Wire.setTimeOut(3);  // padrão 50 ms: barramento travado congelaria o loop de 5 ms
    if (!_mpu.begin()) {
        term.println("MPU6050 não encontrado!");
        return false;
    }
    Wire.setClock(400000);

    _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    term.println("MPU6050 inicializado com DLPF 44Hz, I2C 400kHz");
    return true;
}

// Lê os 14 bytes (acel, temp, gyro) direto do MPU. A Adafruit::getEvent não reporta falha de
// I2C e converte um buffer não inicializado — lixo que entraria no PD.
// out: ax ay az (m/s²), temp (°C), gx gy gz (rad/s). Escalas de ±8 g e ±500 °/s (ver begin()).
bool IMUManager::_readRaw(float out[7]) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(0x68, 14) != 14) {
        while (Wire.available()) Wire.read();
        return false;
    }
    int16_t raw[7];
    bool allZero = true;
    for (int i = 0; i < 7; i++) {
        int hi = Wire.read();
        raw[i] = (int16_t)((hi << 8) | Wire.read());
        if (raw[i]) allZero = false;
    }
    // MPU que perdeu energia acorda em sleep e responde só zeros: é falha, não leitura
    if (allZero) return false;
    for (int i = 0; i < 3; i++) out[i] = raw[i] * (9.80665f / 4096.0f);
    out[3] = raw[3] / 340.0f + 36.53f;
    for (int i = 4; i < 7; i++) out[i] = radians(raw[i] / 65.5f);
    return true;
}

void IMUManager::calibrateGyro(int samples) {
    term.println("Calibrando giroscópio... mantenha o robô parado.");

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f, v[7];
    int good = 0;

    for (int i = 0; i < samples; i++) {
        if (_readRaw(v)) {
            sumX += v[4]; sumY += v[5]; sumZ += v[6];
            good++;
        }
        delay(2);
    }

    if (good < samples / 2) {
        term.printf("Calibração falhou: só %d de %d leituras. Offsets mantidos.\n", good, samples);
    } else {
        _gyroOffsetX = sumX / good;
        _gyroOffsetY = sumY / good;
        _gyroOffsetZ = sumZ / good;
        term.printf("Gyro offsets: X=%.4f  Y=%.4f  Z=%.4f\n",
                      _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ);
        term.println("Calibração concluída.");
    }

    // começa o filtro já no ângulo do acelerômetro, sem transiente
    _failStreak = 0;
    _sample();
    _pitch = _accAngle;
    _gyroAngle = 0.0f;
    _yaw = 0.0f;
}

void IMUManager::_sample() {
    float v[7];
    if (!_readRaw(v)) {
        // mantém a amostra anterior; o supervisor entra em FAULT se persistir
        _failCount++;
        if (_failStreak < 255) _failStreak++;
        return;
    }
    _failStreak = 0;

    _ax = v[0];
    _ay = v[1];
    _az = v[2];
    _temp = v[3];

    _gx = v[4] - _gyroOffsetX;
    _gy = v[5] - _gyroOffsetY;
    _gz = v[6] - _gyroOffsetZ;

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
