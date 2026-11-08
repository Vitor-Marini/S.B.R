#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Arduino.h>

struct PIDGains {
    float kp;
    float ki;
    float kd;
};

struct MotorParams {
    float stepsPerRev = 200.0f;
    float wheelDiameterMm = 74.0f;
    float maxAccelStepsS2 = 25000.0f;
    bool leftReversed = false;
    bool rightReversed = false;
};

// Loop de controle fixo: IMU, PID e supervisor rodam juntos nesta taxa.
constexpr uint32_t CONTROL_PERIOD_US = 5000;  // 200 Hz
constexpr float CONTROL_DT = CONTROL_PERIOD_US * 1e-6f;

struct FilterParams {
    float complementaryAlpha = 0.98f;  // tau = dt*a/(1-a) ≈ 0,25 s a 200 Hz
    float accBiasX = 0.0f;             // m/s², medido deitando o robô de frente/costas
    float accBiasZ = 0.0f;
};

class ConfigManager {
public:
    void begin();

    PIDGains anglePid{0.35f, 0.0f, 0.03f};  // kp (m/s²)/°, kd (m/s²)/(°/s); ki não usado
    PIDGains velocityPid{4.0f, 0.0f, 0.0f};  // ° de offset por m/s de erro de velocidade
    PIDGains yawPid{2.0f, 0.02f, 0.5f};

    float maxVelocity = 1.5f;
    float maxTurnRate = 180.0f;

    bool bleEnabled = true;

    MotorParams motor;
    FilterParams filter;

    float setpoint = 0.0f;
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

    bool load();
    bool save();
    void print();
};

extern ConfigManager cfg;
#endif
