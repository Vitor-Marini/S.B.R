#include "net.hpp"
#include "config.hpp"
#include "cli.hpp"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

Tee term;
NetManager net;

static WiFiUDP udp;
static IPAddress remoteIp;
static uint16_t remotePort = 0;

size_t Tee::write(uint8_t c) {
    net.queue(&c, 1);
    return Serial.write(c);
}

size_t Tee::write(const uint8_t* buf, size_t len) {
    net.queue(buf, len);
    return Serial.write(buf, len);
}

void NetManager::begin() {
    if (cfg.wifiSsid.length() == 0) {
        Serial.println("WiFi: sem rede salva (use: wifi <rede> <senha>)");
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // modem-sleep atrasa pacotes em dezenas de ms
    WiFi.setHostname("sbr");
    if (_started) WiFi.disconnect();  // rede trocada pelo comando `wifi`
    else udp.begin(PORT);
    WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str());
    _started = true;
    _announced = false;
    Serial.printf("WiFi: conectando em \"%s\"...\n", cfg.wifiSsid.c_str());
}

void NetManager::queue(const uint8_t* buf, size_t len) {
    if (!remotePort) return;
    while (len) {
        size_t n = min(len, sizeof(_out) - _outLen);
        memcpy(_out + _outLen, buf, n);
        _outLen += n; buf += n; len -= n;
        if (_outLen == sizeof(_out)) _flush();
    }
}

void NetManager::_flush() {
    if (_outLen && remotePort && WiFi.status() == WL_CONNECTED) {
        udp.beginPacket(remoteIp, remotePort);
        udp.write(_out, _outLen);
        udp.endPacket();
    }
    _outLen = 0;
}

void NetManager::update() {
    if (!_started) return;

    if (!_announced && WiFi.status() == WL_CONNECTED) {
        _announced = true;
        MDNS.begin("sbr");
        Serial.printf("WiFi: conectado, IP %s (sbr.local), UDP %u\n",
                      WiFi.localIP().toString().c_str(), PORT);
    }

    // cada pacote = uma ou mais linhas de comando; pacote vazio/"\n" só registra quem ouve
    int n;
    while ((n = udp.parsePacket()) > 0) {
        remoteIp = udp.remoteIP();
        remotePort = udp.remotePort();
        char buf[256];
        int len = udp.read(buf, sizeof(buf) - 1);
        if (len <= 0) continue;
        buf[len] = 0;
        char* save = nullptr;
        for (char* line = strtok_r(buf, "\r\n", &save); line; line = strtok_r(nullptr, "\r\n", &save))
            cli.run(String(line));
    }

    _flush();
}

void NetManager::printStatus() {
    if (!_started) { term.println("WiFi: desligado (sem rede salva)"); return; }
    if (WiFi.status() == WL_CONNECTED)
        term.printf("WiFi: \"%s\" IP %s (sbr.local) RSSI %d dBm, ouvinte %s\n",
                    cfg.wifiSsid.c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI(),
                    remotePort ? remoteIp.toString().c_str() : "nenhum");
    else
        term.printf("WiFi: tentando conectar em \"%s\"...\n", cfg.wifiSsid.c_str());
}
