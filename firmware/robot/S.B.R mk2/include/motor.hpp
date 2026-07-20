#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

class MotorManager {
public:
    void begin();
    void setSpeeds(float leftSteps, float rightSteps);
    void stop();
    void enable();
    void disable();

    int32_t getLeftSteps() { return _leftSteps; }
    int32_t getRightSteps() { return _rightSteps; }
    void resetOdometry();

private:
    static const int PIN_STEP_L = 0;
    static const int PIN_DIR_L  = 1;
    static const int PIN_DIR_R  = 2;
    static const int PIN_STEP_R = 3;

    static const int LEDC_CH_L = 0;
    static const int LEDC_CH_R = 3;
    static const int LEDC_TIM_L = 0;
    static const int LEDC_TIM_R = 1;
    static const int LEDC_RES = 10;

    float _targetL = 0.0f, _targetR = 0.0f;
    float _currentL = 0.0f, _currentR = 0.0f;
    int32_t _leftSteps = 0, _rightSteps = 0;
    uint32_t _lastUpdate = 0;

    void _setChannelFreq(int timer, int channel, int pin, float stepsPerSec);
    void _stopChannel(int channel, int pin);
    void _rampMotor(float target, float& current, float maxDelta);
};

extern MotorManager motors;
#endif
