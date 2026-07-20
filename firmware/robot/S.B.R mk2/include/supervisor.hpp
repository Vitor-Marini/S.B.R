#ifndef SUPERVISOR_HPP
#define SUPERVISOR_HPP

#include <Arduino.h>

enum RobotState {
    STATE_INIT = 0,
    STATE_CALIBRATE,
    STATE_IDLE,
    STATE_STARTING,
    STATE_ACTIVE,
    STATE_MOVING,
    STATE_FAULT
};

class Supervisor {
public:
    void begin();
    void run();

    RobotState getState() { return _state; }
    const char* getStateName();

    void requestStart();
    void requestStop();
    void requestCalibrate();

private:
    RobotState _state = STATE_INIT;
    uint32_t _stateStartMs = 0;
    uint32_t _faultStartMs = 0;

    void _setState(RobotState newState);
    void _onEnterState();
    void _runInit();
    void _runCalibrate();
    void _runIdle();
    void _runStarting();
    void _runActive();
    void _runMoving();
    void _runFault();
};

extern Supervisor supervisor;
#endif
