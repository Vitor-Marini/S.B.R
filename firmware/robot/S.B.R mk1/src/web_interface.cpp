#include "web_interface.hpp"
#include <WiFi.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "config_manager.hpp"
#include "pid_controller.hpp" 

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


void WebSocketInterface::begin() {
  WiFi.softAP("JECA_ROBOT", "12345678");
  Serial.println("Rede criada:");
  Serial.println(WiFi.softAPIP());

  if (!SPIFFS.begin(true)) {
    Serial.println("Erro ao montar SPIFFS");
    return;
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.begin();
}

void WebSocketInterface::broadcastAngle(float angle) {
  StaticJsonDocument<128> doc;
  doc["angle"] = angle;
  String json;
  serializeJson(doc, json);
  ws.textAll(json);  // envia para todos os clientes conectados
}

void WebSocketInterface::handleTextMessage(uint8_t *data, size_t len) {
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, data, len);
  if (error) {
    Serial.println("Erro ao interpretar JSON");
    return;
  }

  handleIncomingData(String((const char*)data));
}

void WebSocketInterface::handleIncomingData(const String &json) {
  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, json);
  if (err) return;

  bool changed = false;

  if (doc.containsKey("kp"))       { config.kp = doc["kp"]; changed = true; }
  if (doc.containsKey("ki"))       { config.ki = doc["ki"]; changed = true; }
  if (doc.containsKey("kd"))       { config.kd = doc["kd"]; changed = true; }
  if (doc.containsKey("setpoint")) { config.setpoint = doc["setpoint"]; changed = true; }

  if (changed) {
    pid.setTunings(config.kp, config.ki, config.kd, config.setpoint); //tem que criar uma nova instasncia nao quero isso entao acho que vou ter que colocar como extern global
    Serial.println("Parâmetros PID atualizados via WebSocket");
  }
}




void WebSocketInterface::onWsEvent(AsyncWebSocket *server,
                                   AsyncWebSocketClient *client,
                                   AwsEventType type, void *arg,
                                   uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->opcode == WS_TEXT) {
      WebSocketInterface wsInterface;
      wsInterface.handleTextMessage(data, len);
    }
  }

  if (type == WS_EVT_CONNECT) {
  Serial.println("Novo cliente WebSocket conectado");

  StaticJsonDocument<256> doc;
  doc["kp"] = config.kp;
  doc["ki"] = config.ki;
  doc["kd"] = config.kd;
  doc["setpoint"] = config.setpoint;
  String json;
  serializeJson(doc, json);
  client->text(json);  
}
}