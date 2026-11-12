#ifndef VELOCITY_HPP
#define VELOCITY_HPP

#include <Arduino.h>

class VelocityController {
public:
    void begin();
    void setTarget(float ms);
    void setTurnRate(float degS);
    float getTarget() { return _targetVelocity; }
    float getRampedTarget() { return _rampedTarget; }
    float getTurnRate() { return _targetTurnRate; }
    float getEstimated() { return _estimatedVelocity; }

    void update();
    float getAngleOffset();
    float getTurnOffset();  // m/s de diferença entre rodas
    void reset();

private:
    float _targetVelocity = 0.0f;
    float _targetTurnRate = 0.0f;
    float _angleOffset = 0.0f;
    float _turnOffset = 0.0f;

    float _velIntegral = 0.0f;
    float _turnIntegral = 0.0f;
    float _estimatedVelocity = 0.0f;
    uint32_t _lastCmdMs = 0;
    float _rampedTarget = 0.0f;
    static constexpr float TARGET_ACCEL = 0.3f;  // m/s²
    static const uint32_t CMD_TIMEOUT_MS = 500;
};

extern VelocityController velCtrl;
#endif
