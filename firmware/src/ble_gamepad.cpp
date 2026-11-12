#include "ble_gamepad.hpp"
#include "net.hpp"
#include "velocity.hpp"
#include <NimBLEDevice.h>

BLEManager bleMgr;

static const char* SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* CHAR_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* CHAR_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pTxChar = nullptr;
static bool deviceConnected = false;

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s) {
        deviceConnected = true;
        term.println("BLE: dispositivo conectado");
    }
    void onDisconnect(NimBLEServer* s) {
        deviceConnected = false;
        term.println("BLE: dispositivo desconectado");
        s->startAdvertising();
    }
};

class CharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c) {
        std::string data = c->getValue();
        if (data.length() > 0) {
            String cmd = String(data.c_str());
            cmd.trim();
            term.printf("BLE cmd: %s\n", cmd.c_str());
            _parseCommand(cmd);
        }
    }

    void _parseCommand(const String& cmd) {
        if (cmd == "start") {
            extern void requestBalanceStart();
            requestBalanceStart();
            return;
        }
        if (cmd == "stop") {
            extern void requestBalanceStop();
            requestBalanceStop();
            return;
        }

        int space = cmd.indexOf(' ');
        if (space < 0) return;
        String param = cmd.substring(0, space);
        float val = cmd.substring(space + 1).toFloat();

        if (param == "f" || param == "v") {
            velCtrl.setTarget(val);
        } else if (param == "t" || param == "turn") {
            velCtrl.setTurnRate(val);
        } else if (param == "s") {
            velCtrl.setTarget(0.0f);
            velCtrl.setTurnRate(0.0f);
        }
    }
};

void BLEManager::begin() {
    BLEDevice::init("SBR_CONTROL");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* svc = pServer->createService(SERVICE_UUID);

    pTxChar = svc->createCharacteristic(CHAR_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY);

    NimBLECharacteristic* rxChar = svc->createCharacteristic(CHAR_RX_UUID,
        NIMBLE_PROPERTY::WRITE_NR);
    rxChar->setCallbacks(new CharCallbacks());

    svc->start();

    NimBLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->setName("SBR_CONTROL");
    adv->start();

    term.println("BLE UART: SBR_CONTROL pronto");
}

void BLEManager::update() {
    _connected = deviceConnected;
}

bool BLEManager::isConnected() {
    return _connected;
}
