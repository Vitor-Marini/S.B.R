#ifndef CONFIG_MANAGER_HPP
#define CONFIG_MANAGER_HPP
#include <Arduino.h>


class ConfigManager{

  public:

    //View and modifie
    float setpoint = 0.0f;
    float kp = 25.0f;
    float ki = 0.0f;
    float kd = 0.8f;


    //Only View
    float pidOutput = 0.0f;
    float pitchAngle = 0.0f;

    void log();
    bool load();
    bool save();
};

extern ConfigManager config;

#endif