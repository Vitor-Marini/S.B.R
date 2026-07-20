#include "web_interface.hpp"
#include "config.hpp"
#include "supervisor.hpp"
#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

WebManager web;

void WebManager::begin() {
    WiFi.softAP("SBR_ROBOT", "12345678");
    Serial.print("WiFi AP: ");
    Serial.println(WiFi.softAPIP());

    if (!LittleFS.begin(true)) {
        Serial.println("Erro LittleFS");
        return;
    }

    _ws.onEvent(_onWsEvent);
    _server.addHandler(&_ws);

    _server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    _server.begin();
    Serial.println("Web server iniciado.");
}

void WebManager::broadcast() {
    uint32_t now = millis();
    if (now - _lastBroadcast < 100) return;
    _lastBroadcast = now;

    StaticJsonDocument<256> doc;
    doc["angle"] = cfg.pitchAngle;
    doc["rate"] = cfg.pitchRate;
    doc["yaw"] = cfg.yawAngle;
    doc["output"] = cfg.pidOutput;
    doc["state"] = supervisor.getStateName();
    doc["lspd"] = cfg.leftSpeed;
    doc["rspd"] = cfg.rightSpeed;

    String json;
    serializeJson(doc, json);
    _ws.textAll(json);
}

void WebManager::notifyStateChange() {
    StaticJsonDocument<64> doc;
    doc["state"] = supervisor.getStateName();
    String json;
    serializeJson(doc, json);
    _ws.textAll(json);
}

void WebManager::_sendConfig(AsyncWebSocketClient* client) {
    StaticJsonDocument<256> doc;
    doc["kp"] = cfg.anglePid.kp;
    doc["ki"] = cfg.anglePid.ki;
    doc["kd"] = cfg.anglePid.kd;
    doc["setpoint"] = cfg.setpoint;
    doc["state"] = supervisor.getStateName();

    String json;
    serializeJson(doc, json);
    if (client) client->text(json);
}

void WebManager::_onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                            AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket cliente %u conectado\n", client->id());
        web._sendConfig(client);
    }

    if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket cliente %u desconectado\n", client->id());
    }

    if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->opcode == WS_TEXT) {
            DynamicJsonDocument doc(256);
            DeserializationError err = deserializeJson(doc, data, len);
            if (err) return;

            bool changed = false;
            if (doc.containsKey("kp"))       { cfg.anglePid.kp = doc["kp"]; changed = true; }
            if (doc.containsKey("ki"))       { cfg.anglePid.ki = doc["ki"]; changed = true; }
            if (doc.containsKey("kd"))       { cfg.anglePid.kd = doc["kd"]; changed = true; }
            if (doc.containsKey("setpoint")) { cfg.setpoint = doc["setpoint"]; changed = true; }
            if (doc.containsKey("start"))    { supervisor.requestStart(); }
            if (doc.containsKey("stop"))     { supervisor.requestStop(); }
            if (doc.containsKey("calibrate")){ supervisor.requestCalibrate(); }

            if (changed) {
                cfg.save();
                Serial.println("PID params atualizados via WebSocket");
            }
        }
    }
}
