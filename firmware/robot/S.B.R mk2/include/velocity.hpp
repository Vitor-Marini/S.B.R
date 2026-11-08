#ifndef VELOCITY_HPP
#define VELOCITY_HPP

#include <Arduino.h>

class VelocityController {
public:
    void begin();
    void setTarget(float ms);
    void setTurnRate(float degS);
    float getTarget() { return _targetVelocity; }
    float getTurnRate() { return _targetTurnRate; }
    float getEstimated() { return _estimatedVelocity; }

    void update();
    float getAngleOffset();
    float getTurnOffset();
    void reset();

private:
    float _targetVelocity = 0.0f;
    float _targetTurnRate = 0.0f;
    float _angleOffset = 0.0f;
    float _turnOffset = 0.0f;

    float _velIntegral = 0.0f;
    float _turnIntegral = 0.0f;
    float _estimatedVelocity = 0.0f;
};

extern VelocityController velCtrl;
#endif
