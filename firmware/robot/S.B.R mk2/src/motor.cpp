#include "motor.hpp"
#include "config.hpp"
#include <driver/ledc.h>
#include <math.h>

MotorManager motors;

void MotorManager::begin() {
    pinMode(PIN_DIR_L, OUTPUT);
    pinMode(PIN_DIR_R, OUTPUT);
    digitalWrite(PIN_DIR_L, LOW);
    digitalWrite(PIN_DIR_R, LOW);

    ledc_timer_config_t tL = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = (ledc_timer_t)LEDC_TIM_L,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&tL);

    ledc_timer_config_t tR = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = (ledc_timer_t)LEDC_TIM_R,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&tR);

    ledc_channel_config_t chL = {
        .gpio_num = PIN_STEP_L,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)LEDC_CH_L,
        .timer_sel = (ledc_timer_t)LEDC_TIM_L,
        .duty = 512,
        .hpoint = 0
    };
    ledc_channel_config(&chL);

    ledc_channel_config_t chR = {
        .gpio_num = PIN_STEP_R,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)LEDC_CH_R,
        .timer_sel = (ledc_timer_t)LEDC_TIM_R,
        .duty = 512,
        .hpoint = 0
    };
    ledc_channel_config(&chR);

    ledc_set_freq(LEDC_LOW_SPEED_MODE, (ledc_timer_t)LEDC_TIM_L, 1);
    ledc_set_freq(LEDC_LOW_SPEED_MODE, (ledc_timer_t)LEDC_TIM_R, 1);

    _lastUpdate = millis();

    Serial.println("Motores inicializados (LEDC PWM).");
}

void MotorManager::_setChannelFreq(int timer, int channel, int pin, float stepsPerSec) {
    int freq = (int)(stepsPerSec + 0.5f);
    if (freq < 1) freq = 1;
    if (freq > 50000) freq = 50000;

    ledc_set_freq(LEDC_LOW_SPEED_MODE, (ledc_timer_t)timer, freq);
}

void MotorManager::_stopChannel(int channel, int pin) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}

void MotorManager::_rampMotor(float target, float& current, float maxDelta) {
    float delta = target - current;
    if (fabsf(delta) <= maxDelta) {
        current = target;
    } else {
        current += (delta > 0.0f) ? maxDelta : -maxDelta;
    }
}

void MotorManager::setSpeeds(float left, float right) {
    uint32_t now = millis();
    float dt = (now - _lastUpdate) / 1000.0f;
    if (dt > 0.1f || dt <= 0.0f) dt = 0.01f;
    _lastUpdate = now;

    int dirL = (left >= 0) ? HIGH : LOW;
    int dirR = (right >= 0) ? HIGH : LOW;

    if (cfg.motor.leftReversed) dirL = (dirL == HIGH) ? LOW : HIGH;
    if (cfg.motor.rightReversed) dirR = (dirR == HIGH) ? LOW : HIGH;

    digitalWrite(PIN_DIR_L, dirL);
    digitalWrite(PIN_DIR_R, dirR);

    float absL = fabsf(left);
    float absR = fabsf(right);

    float maxDelta = cfg.motor.maxAccelStepsS2 * dt;

    float targetL = absL;
    float targetR = absR;

    if (left == 0.0f && right == 0.0f) {
        _currentL = 0.0f;
        _currentR = 0.0f;
    } else {
        _rampMotor(targetL, _currentL, maxDelta);
        _rampMotor(targetR, _currentR, maxDelta);
    }

    if (_currentL < 10.0f) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_L, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_L);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_L, 512);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_L);
        _setChannelFreq(LEDC_TIM_L, LEDC_CH_L, PIN_STEP_L, _currentL);
    }

    if (_currentR < 10.0f) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_R, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_R);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_R, 512);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_R);
        _setChannelFreq(LEDC_TIM_R, LEDC_CH_R, PIN_STEP_R, _currentR);
    }

    _leftSteps += (int32_t)(_currentL * dt);
    _rightSteps += (int32_t)(_currentR * dt);

    cfg.leftSpeed = (dirL == HIGH) ? _currentL : -_currentL;
    cfg.rightSpeed = (dirR == HIGH) ? _currentR : -_currentR;
}

void MotorManager::stop() {
    _currentL = 0.0f;
    _currentR = 0.0f;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_L, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_L);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_R, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LEDC_CH_R);
}

void MotorManager::enable() {
    stop();
}

void MotorManager::disable() {
    stop();
}

void MotorManager::resetOdometry() {
    _leftSteps = 0;
    _rightSteps = 0;
}
