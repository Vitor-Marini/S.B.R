#include "web_interface.hpp"
#include "net.hpp"
#include "config.hpp"
#include "supervisor.hpp"
#include "velocity.hpp"
#include "ds4_controller.hpp"
#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

WebManager web;

void WebManager::begin() {
    WiFi.softAP("SBR_ROBOT", "12345678");
    term.print("WiFi AP: ");
    term.println(WiFi.softAPIP());

    if (!LittleFS.begin(true)) {
        term.println("Erro LittleFS");
        return;
    }

    _ws.onEvent(_onWsEvent);
    _server.addHandler(&_ws);

    _server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    _server.begin();
    term.println("Web server iniciado.");
}

void WebManager::broadcast() {
    uint32_t now = millis();
    if (now - _lastBroadcast < 100) return;
    _lastBroadcast = now;

    JsonDocument doc;
    doc["angle"] = cfg.pitchAngle;
    doc["rate"] = cfg.pitchRate;
    doc["yaw"] = cfg.yawAngle;
    doc["output"] = cfg.pidOutput;
    doc["state"] = supervisor.getStateName();
    doc["lspd"] = cfg.leftSpeed;
    doc["rspd"] = cfg.rightSpeed;
    doc["vel"] = velCtrl.getEstimated();
    doc["vtarget"] = velCtrl.getTarget();
    doc["ttarget"] = velCtrl.getTurnRate();
    doc["ds4"] = ds4Ctrl.isConnected();

    String json;
    serializeJson(doc, json);
    _ws.textAll(json);
}

void WebManager::notifyStateChange() {
    JsonDocument doc;
    doc["state"] = supervisor.getStateName();
    String json;
    serializeJson(doc, json);
    _ws.textAll(json);
}

void WebManager::_sendConfig(AsyncWebSocketClient* client) {
    JsonDocument doc;
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
        term.printf("WebSocket cliente %u conectado\n", client->id());
        web._sendConfig(client);
    }

    if (type == WS_EVT_DISCONNECT) {
        term.printf("WebSocket cliente %u desconectado\n", client->id());
    }

    if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->opcode == WS_TEXT) {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, data, len);
            if (err) return;

            bool changed = false;
            if (!doc["kp"].isNull())       { cfg.anglePid.kp = doc["kp"].as<float>(); changed = true; }
            if (!doc["ki"].isNull())       { cfg.anglePid.ki = doc["ki"].as<float>(); changed = true; }
            if (!doc["kd"].isNull())       { cfg.anglePid.kd = doc["kd"].as<float>(); changed = true; }
            if (!doc["setpoint"].isNull()) { cfg.setpoint = doc["setpoint"].as<float>(); changed = true; }
            if (!doc["start"].isNull())    { supervisor.requestStart(); }
            if (!doc["stop"].isNull())     { supervisor.requestStop(); }
            if (!doc["calibrate"].isNull()){ supervisor.requestCalibrate(); }

            if (!doc["v"].isNull())    { velCtrl.setTarget(doc["v"].as<float>()); }
            if (!doc["turn"].isNull()) { velCtrl.setTurnRate(doc["turn"].as<float>()); }

            if (changed) {
                cfg.save();
                term.println("PID params atualizados via WebSocket");
            }
        }
    }
}
