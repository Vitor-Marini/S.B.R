#include "filters.hpp"
#include <math.h>

// =========== Mahony Filter ===========

void MahonyFilter::begin(float kp, float ki) {
    _kp = kp;
    _ki = ki;
    _q0 = 1.0f; _q1 = 0.0f; _q2 = 0.0f; _q3 = 0.0f;
    _integralFBx = _integralFBy = _integralFBz = 0.0f;
}

void MahonyFilter::setGains(float kp, float ki) {
    _kp = kp;
    _ki = ki;
}

void MahonyFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    float q0q0 = _q0 * _q0;
    float q0q1 = _q0 * _q1;
    float q0q2 = _q0 * _q2;
    float q1q1 = _q1 * _q1;
    float q1q3 = _q1 * _q3;
    float q2q2 = _q2 * _q2;
    float q2q3 = _q2 * _q3;
    float q3q3 = _q3 * _q3;

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;

    halfex = ay * halfvz - az * halfvy;
    halfey = az * halfvx - ax * halfvz;
    halfez = ax * halfvy - ay * halfvx;

    if (_ki > 0.0f) {
        _integralFBx += _ki * halfex * dt;
        _integralFBy += _ki * halfey * dt;
        _integralFBz += _ki * halfez * dt;
        gx += _integralFBx;
        gy += _integralFBy;
        gz += _integralFBz;
    }

    gx += _kp * halfex;
    gy += _kp * halfey;
    gz += _kp * halfez;

    float dq0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    float dq1 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
    float dq2 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
    float dq3 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

    _q0 += dq0 * dt;
    _q1 += dq1 * dt;
    _q2 += dq2 * dt;
    _q3 += dq3 * dt;

    recipNorm = invSqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 *= recipNorm;
    _q1 *= recipNorm;
    _q2 *= recipNorm;
    _q3 *= recipNorm;

    _pitchRate = gy;
    _yawRate = gz;
}

float MahonyFilter::getPitch() {
    return asinf(-2.0f * (_q1 * _q3 - _q0 * _q2)) * 180.0f / PI;
}

float MahonyFilter::getRoll() {
    return atan2f(2.0f * (_q0 * _q1 + _q2 * _q3),
                  _q0 * _q0 - _q1 * _q1 - _q2 * _q2 + _q3 * _q3) * 180.0f / PI;
}

float MahonyFilter::getYaw() {
    return atan2f(2.0f * (_q1 * _q2 + _q0 * _q3),
                  _q0 * _q0 + _q1 * _q1 - _q2 * _q2 - _q3 * _q3) * 180.0f / PI;
}

float MahonyFilter::getPitchRate() {
    return _pitchRate;
}

float MahonyFilter::getYawRate() {
    return _yawRate;
}

float MahonyFilter::invSqrt(float x) {
    return 1.0f / sqrtf(x);
}

// =========== Complementary Filter ===========

void ComplementaryFilter::begin(float alpha) {
    _alpha = alpha;
    _angle = 0.0f;
}

void ComplementaryFilter::update(float accAngle, float gyroRate, float dt) {
    _angle = _alpha * (_angle + gyroRate * dt) + (1.0f - _alpha) * accAngle;
}

float ComplementaryFilter::getAngle() {
    return _angle;
}

void ComplementaryFilter::setAlpha(float alpha) {
    _alpha = alpha;
}
