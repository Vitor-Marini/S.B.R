#ifndef BALANCE_HPP
#define BALANCE_HPP

#include <Arduino.h>

class BalanceController {
public:
    void begin();
    void compute();

    float getTorque() { return _torque; }
    float getYawCorrection() { return _yawCorrection; }
    float getAngleError() { return _angleError; }

    void resetIntegral();

private:
    float _angleError = 0.0f;
    float _integral = 0.0f;
    float _prevError = 0.0f;
    float _filteredDerivative = 0.0f;

    float _yawIntegral = 0.0f;

    float _torque = 0.0f;
    float _yawCorrection = 0.0f;

    uint32_t _lastCompute = 0;
};

extern BalanceController balance;
#endif
