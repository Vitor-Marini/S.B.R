#ifndef FILTERS_HPP
#define FILTERS_HPP

#include <Arduino.h>

class MahonyFilter {
public:
    void begin(float kp, float ki);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    float getPitch();
    float getRoll();
    float getYaw();
    float getPitchRate();
    float getYawRate();
    void setGains(float kp, float ki);

private:
    float _kp, _ki;
    float _integralFBx = 0.0f, _integralFBy = 0.0f, _integralFBz = 0.0f;
    float _q0 = 1.0f, _q1 = 0.0f, _q2 = 0.0f, _q3 = 0.0f;
    float _pitchRate = 0.0f, _yawRate = 0.0f;

    float invSqrt(float x);
};

class ComplementaryFilter {
public:
    void begin(float alpha);
    void update(float accAngle, float gyroRate, float dt);
    float getAngle();
    void setAlpha(float alpha);

private:
    float _alpha;
    float _angle = 0.0f;
};

#endif
