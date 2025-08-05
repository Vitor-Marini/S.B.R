#ifndef CONFIG_MANAGER_HPP
#define CONFIG_MANAGER_HPP
#include <Arduino.h>


class ConfigManager{

  public:

    //View and modifie
    double setpoint = 0;
    double kp = 10;
    double ki = 20;
    double kd = 1.0;


    //Only View
    double pidOutput = 0;
    float pitchAngle = 0;

    void log();
};

extern ConfigManager config;

#endif