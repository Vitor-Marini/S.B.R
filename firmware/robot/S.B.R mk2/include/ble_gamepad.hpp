#ifndef BLE_GAMEPAD_HPP
#define BLE_GAMEPAD_HPP

#include <Arduino.h>

class BLEManager {
public:
    void begin();
    void update();
    bool isConnected();

private:
    bool _connected = false;
};

extern BLEManager bleMgr;
#endif
