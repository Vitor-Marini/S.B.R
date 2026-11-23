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

bool IMUManager::calibrateGyro(int samples) {
    term.println("Calibrando giroscópio... mantenha o robô parado.");

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f, v[7];
    float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    int good = 0;

    for (int i = 0; i < samples; i++) {
        if (_readRaw(v)) {
            sumX += v[4]; sumY += v[5]; sumZ += v[6];
            for (int k = 0; k < 3; k++) { lo[k] = min(lo[k], v[4 + k]); hi[k] = max(hi[k], v[4 + k]); }
            good++;
        }
        delay(2);
    }

    // robô se mexendo (na mão, sendo levantado) vira um zero errado: descarta e tenta de novo
    float spread = max(hi[0] - lo[0], max(hi[1] - lo[1], hi[2] - lo[2]));
    if (good < samples / 2) {
        term.printf("Calibração falhou: só %d de %d leituras. Offsets mantidos.\n", good, samples);
        return false;
    } else if (spread > radians(4.0f)) {
        term.printf("Robô se mexeu durante a calibração (%.1f °/s). Deixe parado; tentando de novo...\n", degrees(spread));
        return false;
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
    _accFast = _accSlow = _accAngle;
    _gRef = sqrtf(_ax * _ax + _ay * _ay + _az * _az);
    _gyroAngle = 0.0f;
    _yaw = 0.0f;
    return true;
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
    // placa/chip nunca perfeitamente alinhados: girar no lugar (gz) vaza no gyro de pitch e vira inclinação falsa
    _pitchRate = -degrees(_gy + cfg.filter.gyroCross * _gz);
    _yawRate = degrees(_gz);
}

// Parado em IDLE, o zero do gyro segue o bias (temperatura, calibração ruim). Quem diz "parado" é o
// acelerômetro (ângulo estável e |a| de repouso), não o gyro: assim corrige até erros grandes de zero.
void IMUManager::trackGyroBias() {
    const float k = CONTROL_DT / 1.0f;  // τ = 1 s
    float amag = sqrtf(_ax * _ax + _ay * _ay + _az * _az);
    if (fabsf(_accFast - _accSlow) < 0.5f && fabsf(amag - _gRef) < 0.5f) {
        _gyroOffsetX += _gx * k;
        _gyroOffsetY += _gy * k;
        _gyroOffsetZ += _gz * k;
    }
}

void IMUManager::trackGyroZ() {
    _gyroOffsetZ += _gz * CONTROL_DT / 5.0f;  // τ = 5 s
}

void IMUManager::read() {
    const float dt = CONTROL_DT;
    _sample();

    const float a = cfg.filter.complementaryAlpha;
    _pitch = a * (_pitch + _pitchRate * dt) + (1.0f - a) * _accAngle;
    // Erro de zero do gyro Y (muda ao esquentar) desloca o pitch em b·τ (τ = 1 s: 1 °/s → 1°, medido ~1° em ACTIVE)
    // e o setpoint "certo" muda junto. Na média, pitch tem que bater com o acelerômetro (o robô não acelera
    // pra sempre): a diferença que sobra é o erro de zero → corrige devagar (filtro complementar com integral).
    const float tauF = a * dt / (1.0f - a);
    _gyroOffsetY -= radians((_pitch - _accAngle) / tauF) * dt / 20.0f;  // τ = 20 s
    _gyroAngle += _pitchRate * dt;
    _accFast += (_accAngle - _accFast) * dt / 0.3f;  // para o detector de "parado" do trackGyroBias
    _accSlow += (_accAngle - _accSlow) * dt / 2.0f;

    _yaw += _yawRate * dt;
    if (_yaw > 180.0f) _yaw -= 360.0f;
    if (_yaw < -180.0f) _yaw += 360.0f;
}
