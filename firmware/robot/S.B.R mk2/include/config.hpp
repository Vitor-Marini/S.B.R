#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Arduino.h>

struct PIDGains {
    float kp;
    float ki;
    float kd;
};

struct MotorParams {
    float stepsPerRev = 3200.0f;
    float wheelDiameterMm = 80.0f;
    float maxAccelStepsS2 = 25000.0f;
    bool leftReversed = false;
    bool rightReversed = false;
};

struct FilterParams {
    float mahonyKp = 0.5f;
    float mahonyKi = 0.001f;
    float complementaryAlpha = 0.98f;
};

class ConfigManager {
public:
    void begin();

    PIDGains anglePid{25.0f, 0.5f, 0.8f};
    PIDGains velocityPid{0.5f, 0.1f, 0.0f};
    PIDGains yawPid{2.0f, 0.02f, 0.5f};

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

    bool load();
    bool save();
    void print();
};

extern ConfigManager cfg;
#endif
