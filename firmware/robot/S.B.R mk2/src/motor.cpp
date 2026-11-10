#include "motor.hpp"
#include "net.hpp"
#include "config.hpp"
#include <driver/ledc.h>
#include <math.h>

MotorManager motors;

// 12 bits a 80 MHz (APB): ~19.1 Hz até ~19.5 kHz. Fora disso ledc_set_freq falha.
static const ledc_timer_bit_t LEDC_RES = LEDC_TIMER_12_BIT;
static const uint32_t DUTY_50 = 2048;
static const int FREQ_MIN = 20;
static const int FREQ_MAX = 19000;
static const int PARK_FREQ = 2000;

void MotorManager::_setupChannel(Channel& c) {
    pinMode(c.pinDir, OUTPUT);
    digitalWrite(c.pinDir, LOW);
    c.dir = LOW;

    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_RES,
        .timer_num = (ledc_timer_t)c.timer,
        .freq_hz = 1000,
        .clk_cfg = LEDC_USE_APB_CLK
    };
    ledc_timer_config(&t);

    // duty 0: sem pulsos até alguém pedir velocidade
    ledc_channel_config_t ch = {
        .gpio_num = c.pinStep,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)c.channel,
        .timer_sel = (ledc_timer_t)c.timer,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch);
    c.freq = 0;
}

void MotorManager::begin() {
    _setupChannel(_l);
    _setupChannel(_r);
    _lastUpdateUs = micros();
    term.println("Motores inicializados (LEDC 12-bit).");
}

// Escreve DIR/freq/duty no hardware, só quando mudam.
void MotorManager::_apply(Channel& c) {
    int dir = (c.current >= 0.0f) != c.reversed ? HIGH : LOW;
    int freq = (int)(fabsf(c.current) + 0.5f);
    if (freq < FREQ_MIN) freq = 0;
    if (freq > FREQ_MAX) freq = FREQ_MAX;

    if (freq == 0) {
        if (c.freq != 0) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)c.channel, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)c.channel);
            // nova freq/duty só valem no fim do período atual: estaciona o timer rápido
            // (duty 0, sem pulsos) para o religamento na inversão não esperar um período lento
            ledc_set_freq(LEDC_LOW_SPEED_MODE, (ledc_timer_t)c.timer, PARK_FREQ);
            c.freq = 0;
        }
        return;
    }

    // DIR só muda com |v| abaixo do piso (rampa com sinal passa por zero), então sem pulsos no meio
    if (dir != c.dir) {
        digitalWrite(c.pinDir, dir);
        c.dir = dir;
    }
    if (freq != c.freq) {
        if (ledc_set_freq(LEDC_LOW_SPEED_MODE, (ledc_timer_t)c.timer, freq) != ESP_OK) return;
        if (c.freq == 0) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)c.channel, DUTY_50);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)c.channel);
        }
        c.freq = freq;
    }
}

void MotorManager::setSpeeds(float left, float right) {
    _l.target = left;
    _r.target = right;
    update();
}

void MotorManager::update() {
    uint32_t now = micros();
    float dt = (now - _lastUpdateUs) * 1e-6f;
    _lastUpdateUs = now;
    if (dt > 0.02f) dt = 0.02f;  // pausa longa: limita o passo de rampa

    _l.reversed = cfg.motor.leftReversed;
    _r.reversed = cfg.motor.rightReversed;

    float maxDelta = cfg.motor.maxAccelStepsS2 * dt;
    for (Channel* c : {&_l, &_r}) {
        float delta = c->target - c->current;
        c->current += constrain(delta, -maxDelta, maxDelta);
        _apply(*c);
    }

    // odometria = velocidade aplicada de fato (0 abaixo do piso)
    _leftSteps  += copysignf(_l.freq, _l.current) * dt;
    _rightSteps += copysignf(_r.freq, _r.current) * dt;

    cfg.leftSpeed = _l.current;
    cfg.rightSpeed = _r.current;
}

void MotorManager::stop() {
    for (Channel* c : {&_l, &_r}) {
        c->target = 0.0f;
        c->current = 0.0f;
        _apply(*c);
    }
    cfg.leftSpeed = cfg.rightSpeed = 0.0f;
}

void MotorManager::enable() {
    stop();
}

void MotorManager::disable() {
    stop();
}

void MotorManager::resetOdometry() {
    _leftSteps = 0.0f;
    _rightSteps = 0.0f;
}
