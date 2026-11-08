#include "ds4_controller.hpp"

DS4Controller ds4Ctrl;
DS4Controller* DS4Controller::_instance = nullptr;

static const char* DS4_NAME = "Wireless Controller";

// ========== Scan Callbacks ==========

class DS4ScanCB : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* dev) override {
        if (DS4Controller::_instance)
            DS4Controller::_instance->_onScanResult(dev);
    }
};

// ========== Client Callbacks ==========

class DS4ClientCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* c) override {
        if (DS4Controller::_instance)
            DS4Controller::_instance->_onConnect();
    }
    void onDisconnect(NimBLEClient* c) override {
        if (DS4Controller::_instance)
            DS4Controller::_instance->_onDisconnect();
    }
};

static DS4ScanCB scanCB;
static DS4ClientCB clientCB;

// ========== Public API ==========

void DS4Controller::begin() {
    _instance = this;
    Serial.println("DS4: digite 'pair' para escanear e conectar.");
}

void DS4Controller::update() {
    if (_scanning && millis() - _scanStart > SCAN_MS) {
        _stopScan();
    }

    if (!_connected && !_scanning && _paired) {
        if (millis() - _disconnectTime > RECONNECT_DELAY_MS) {
            _disconnectTime = millis();
            _startScan();
        }
    }
}

void DS4Controller::startPairing() {
    if (_scanning) {
        Serial.println("DS4: escaneamento ja em andamento.");
        return;
    }
    if (_connected) {
        Serial.println("DS4: ja conectado.");
        return;
    }
    Serial.println("DS4: escaneando por \"Wireless Controller\"...");
    Serial.println("      Pressione PS + Share por 3s no DS4.");
    _startScan();
}

void DS4Controller::disconnect() {
    if (_client && _client->isConnected()) {
        _client->disconnect();
    }
    _connected = false;
    _paired = false;
    _pairedAddr = NimBLEAddress();
    _client = nullptr;
    Serial.println("DS4: desconectado e esquecido.");
}

String DS4Controller::getAddress() {
    if (_paired) return _pairedAddr.toString().c_str();
    return "none";
}

// ========== BLE Internals ==========

void DS4Controller::_startScan() {
    _scanning = true;
    _scanStart = millis();

    NimBLEScan* scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&scanCB, false);
    scan->setInterval(100);
    scan->setWindow(50);
    scan->setActiveScan(false);
    scan->start(SCAN_MS / 1000, _scanEndCB);
}

void DS4Controller::_stopScan() {
    _scanning = false;
    NimBLEScan* scan = BLEDevice::getScan();
    if (scan) scan->stop();
}

void DS4Controller::_scanEndCB(NimBLEScanResults results) {
    if (_instance) _instance->_onScanEnd();
}

void DS4Controller::_onScanResult(NimBLEAdvertisedDevice* dev) {
    if (!dev->haveName()) return;
    if (dev->getName() != DS4_NAME) return;

    Serial.printf("DS4: encontrado %s [%s]\n",
                  dev->getName().c_str(), dev->getAddress().toString().c_str());

    _stopScan();

    _client = BLEDevice::createClient();
    _client->setClientCallbacks(&clientCB, false);
    _client->setConnectTimeout(5);

    if (!_client->connect(dev)) {
        Serial.println("DS4: falha ao conectar.");
        BLEDevice::deleteClient(_client);
        _client = nullptr;
        return;
    }
    Serial.println("DS4: conectado!");

    if (!_client->discoverAttributes()) {
        Serial.println("DS4: falha ao descobrir servicos.");
        _client->disconnect();
        return;
    }

    NimBLERemoteService* hidSvc = _client->getService(NimBLEUUID((uint16_t)0x1812));
    if (!hidSvc) {
        Serial.println("DS4: servico HID nao encontrado.");
        _client->disconnect();
        return;
    }
    Serial.println("DS4: servico HID encontrado.");

    std::vector<NimBLERemoteCharacteristic*>* charsPtr = hidSvc->getCharacteristics();
    if (!charsPtr) { _client->disconnect(); return; }
    std::vector<NimBLERemoteCharacteristic*> chars = *charsPtr;
    for (auto* c : chars) {
        if (c->canNotify()) {
            _reportChar = c;
            break;
        }
    }

    if (!_reportChar) {
        Serial.println("DS4: caracteristica de notificacao nao encontrada.");
        _client->disconnect();
        return;
    }
    Serial.println("DS4: inscrito em notificacoes de input.");

    if (!_reportChar->subscribe(true, [this](NimBLERemoteCharacteristic* pChar,
                                              uint8_t* data, size_t len, bool isNotify) {
        _parseReport(data, len);
    })) {
        Serial.println("DS4: falha ao subscrever notificacoes.");
        _client->disconnect();
        return;
    }

    _pairedAddr = dev->getAddress();
    _paired = true;
    _disconnectTime = 0;
    _connected = true;
    Serial.println("DS4: pronto! Controles mapeados.");
}

void DS4Controller::_onScanEnd() {
    _scanning = false;
    if (!_connected) {
        Serial.println("DS4: nenhum controle encontrado.");
    }
}

void DS4Controller::_onConnect() {
}

void DS4Controller::_onDisconnect() {
    _connected = false;
    _reportChar = nullptr;
    _disconnectTime = millis();
    if (_client) {
        BLEDevice::deleteClient(_client);
        _client = nullptr;
    }
    Serial.println("DS4: desconectado. Reconectando em 5s...");
}

// ========== Report Parsing ==========

void DS4Controller::_parseReport(uint8_t* data, size_t len) {
    if (len < 8) return;

    uint8_t reportId = data[0];
    uint8_t lx, ly, rx, ry, btn0, btn1, btn2;
    uint8_t l2a = 0, r2a = 0;

    if (reportId == 0x01 && len >= 9) {
        lx   = data[1];
        ly   = data[2];
        rx   = data[3];
        ry   = data[4];
        btn0 = data[5];
        btn1 = data[6];
        btn2 = data[7];
        if (len >= 11) { l2a = data[8]; r2a = data[9]; }
    } else if (reportId == 0x11 && len >= 78) {
        lx   = data[3];
        ly   = data[4];
        rx   = data[5];
        ry   = data[6];
        btn0 = data[7];
        btn1 = data[8];
        btn2 = data[9];
        l2a  = data[11];
        r2a  = data[12];
    } else {
        return;
    }

    _state.leftStickX  = lx;
    _state.leftStickY  = ly;
    _state.rightStickX = rx;
    _state.rightStickY = ry;
    _state.l2Analog    = l2a;
    _state.r2Analog    = r2a;

    _parseButtons(btn0, btn1, btn2);
    _updateNormalized();
}

void DS4Controller::_parseButtons(uint8_t btn0, uint8_t btn1, uint8_t btn2) {
    _state.dpad     = btn0 & 0x0F;
    _state.square   = btn0 & (1 << 4);
    _state.cross    = btn0 & (1 << 5);
    _state.circle   = btn0 & (1 << 6);
    _state.triangle = btn0 & (1 << 7);

    _state.l1      = btn1 & (1 << 0);
    _state.r1      = btn1 & (1 << 1);
    _state.l2      = btn1 & (1 << 2);
    _state.r2      = btn1 & (1 << 3);
    _state.share   = btn1 & (1 << 4);
    _state.options = btn1 & (1 << 5);
    _state.l3      = btn1 & (1 << 6);
    _state.r3      = btn1 & (1 << 7);

    _state.ps       = btn2 & (1 << 0);
    _state.touchpad = btn2 & (1 << 1);
}

void DS4Controller::_updateNormalized() {
    _state.stickLY = _normY(_state.leftStickY);
    _state.stickRX = _normX(_state.rightStickX);
}

float DS4Controller::_normY(uint8_t raw) {
    float v = (128.0f - raw) / 128.0f;
    if (fabsf(v) < DEADZONE) return 0.0f;
    return constrain(v, -1.0f, 1.0f);
}

float DS4Controller::_normX(uint8_t raw) {
    float v = (raw - 128.0f) / 128.0f;
    if (fabsf(v) < DEADZONE) return 0.0f;
    return constrain(v, -1.0f, 1.0f);
}
