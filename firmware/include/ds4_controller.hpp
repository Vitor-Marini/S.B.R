#ifndef DS4_CONTROLLER_HPP
#define DS4_CONTROLLER_HPP

#include <Arduino.h>
#include <NimBLEDevice.h>

struct DS4State {
    uint8_t leftStickX = 128;
    uint8_t leftStickY = 128;
    uint8_t rightStickX = 128;
    uint8_t rightStickY = 128;
    uint8_t l2Analog = 0;
    uint8_t r2Analog = 0;

    uint8_t dpad = 0x8;
    bool square = false;
    bool cross = false;
    bool circle = false;
    bool triangle = false;
    bool l1 = false, r1 = false, l2 = false, r2 = false;
    bool share = false, options = false, l3 = false, r3 = false;
    bool ps = false, touchpad = false;

    float stickLY = 0.0f;
    float stickRX = 0.0f;
};

class DS4Controller {
public:
    void begin();
    void update();
    bool isConnected() { return _connected; }
    const DS4State& getState() { return _state; }
    void startPairing();
    void disconnect();
    String getAddress();
    static DS4Controller* _instance;

private:
    friend class DS4ScanCB;
    friend class DS4ClientCB;
    DS4State _state, _prev;
    bool _connected = false;
    bool _scanning = false;
    bool _paired = false;
    uint32_t _scanStart = 0;
    uint32_t _disconnectTime = 0;
    NimBLEClient* _client = nullptr;
    NimBLERemoteCharacteristic* _reportChar = nullptr;
    NimBLEAddress _pairedAddr;

    static void _scanEndCB(NimBLEScanResults results);

    void _onScanResult(NimBLEAdvertisedDevice* dev);
    void _onScanEnd();
    void _onConnect();
    void _onDisconnect();
    void _startScan();
    void _stopScan();
    void _parseReport(uint8_t* data, size_t len);
    void _parseButtons(uint8_t btn0, uint8_t btn1, uint8_t btn2);
    void _updateNormalized();
    float _normY(uint8_t raw);
    float _normX(uint8_t raw);

    static constexpr uint32_t SCAN_MS = 10000;
    static constexpr uint32_t RECONNECT_DELAY_MS = 5000;
    static constexpr float DEADZONE = 0.12f;
};

extern DS4Controller ds4Ctrl;

#endif
