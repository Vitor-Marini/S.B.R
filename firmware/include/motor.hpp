#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

class MotorManager {
public:
    void begin();
    void setSpeeds(float leftSteps, float rightSteps);  // alvo em passos/s (com sinal)
    // igual, mas zera sozinho se não for renovado em `ms` (teste remoto: PC/WiFi caiu → para)
    void setSpeedsTimed(float leftSteps, float rightSteps, uint32_t ms);
    void update();                                      // avança a rampa; chamar todo loop
    void stop();
    void enable();
    void disable();

    float getLeftSteps() { return _leftSteps; }
    float getRightSteps() { return _rightSteps; }
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

    struct Channel {
        int pinStep, pinDir, timer, channel;
        bool reversed;
        float target, current;  // passos/s com sinal
        int freq;               // frequência aplicada; 0 = parado
        int dir;                // nível aplicado no DIR; -1 = desconhecido
    };

    Channel _l{PIN_STEP_L, PIN_DIR_L, LEDC_TIM_L, LEDC_CH_L, false, 0, 0, 0, -1};
    Channel _r{PIN_STEP_R, PIN_DIR_R, LEDC_TIM_R, LEDC_CH_R, false, 0, 0, 0, -1};
    float _leftSteps = 0.0f, _rightSteps = 0.0f;
    uint32_t _lastUpdateUs = 0;
    uint32_t _deadlineMs = 0;  // 0 = sem prazo

    void _setupChannel(Channel& c);
    void _apply(Channel& c);
};

extern MotorManager motors;
#endif
