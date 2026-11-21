#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Arduino.h>

struct PIDGains {
    float kp;
    float ki;
    float kd;
};

struct MotorParams {
    float stepsPerRev = 3200.0f;          // NEMA17 200 passos × 1/16 (M2 em nível alto)
    float wheelDiameterMm = 74.0f;
    float maxAccelStepsS2 = 400000.0f;    // só trava de segurança; quem acelera é o controle
    bool leftReversed = true;    // montagem de 2026-11-21: os dois motores invertidos
    bool rightReversed = true;
};

// Loop de controle fixo: IMU, PID e supervisor rodam juntos nesta taxa.
constexpr uint32_t CONTROL_PERIOD_US = 5000;  // 200 Hz
constexpr float CONTROL_DT = CONTROL_PERIOD_US * 1e-6f;

struct FilterParams {
    float complementaryAlpha = 0.995f;  // tau = dt*a/(1-a) ≈ 1 s a 200 Hz
    float accBiasX = 0.0f;             // m/s², medido deitando o robô de frente/costas
    float accBiasZ = 0.0f;
    float gyroCross = 0.013f;          // parte do giro (gz) que vaza no gyro de pitch; medido girando
};

class ConfigManager {
public:
    void begin();

    PIDGains anglePid{0.35f, 0.0f, 0.03f};  // kp (m/s²)/°, kd (m/s²)/(°/s); ki não usado
    PIDGains velocityPid{4.0f, 3.0f, 0.0f};  // ° de offset por m/s de erro de velocidade
    PIDGains yawPid{0.001f, 0.01f, 0.0f};  // curva: (m/s) por °/s de erro; kd não usado

    float maxVelocity = 0.8f;
    float maxTurnRate = 180.0f;

    String wifiSsid;  // rede de casa; salva só na flash (comando `wifi`), nunca no git
    String wifiPass;

    MotorParams motor;
    FilterParams filter;

    float setpoint = -1.3f;  // ponto de equilíbrio medido (chassi rígido, IMU sobre os motores, rodas novas)
    float failsafeAngle = 45.0f;

    float pitchAngle = 0.0f;
    float pitchRate = 0.0f;
    float yawAngle = 0.0f;
    float yawRate = 0.0f;
    float pidOutput = 0.0f;
    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;
    int robotState = 0;
    int tickHz = 0;          // ticks de controle no último segundo (alvo 200)
    uint32_t tickMaxUs = 0;  // pior tempo de supervisor.run() no último segundo

    bool dirty = false;  // mudou em ACTIVE; grava ao sair (escrita na flash trava o loop)

    bool load();
    bool save();
    void print();
};

extern ConfigManager cfg;
#endif
