#include "web_interface.hpp"
#include <WiFi.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "system_state.hpp"

SystemState systemStateOUT;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void WebSocketInterface::handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    DynamicJsonDocument doc(256);
    if (!deserializeJson(doc, data)) {
      if (doc.containsKey("kp")) systemStateOUT.kp = doc["kp"];
      if (doc.containsKey("ki")) systemStateOUT.ki = doc["ki"];
      if (doc.containsKey("kd")) systemStateOUT.kd = doc["kd"];
      if (doc.containsKey("setpoint")) systemStateOUT.setPoint = doc["setpoint"];
    }
  }
}

void WebSocketInterface::onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                                 void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    WebSocketInterface().handleWebSocketMessage(arg, data, len);
  }
}

void WebSocketInterface::begin() {
  WiFi.softAP("JECA_ROBOT", "12345678");
 
  Serial.print("Rede IP: ");
  Serial.println(WiFi.localIP());
  
  if (!SPIFFS.begin(true)) {
    Serial.println("Erro ao montar SPIFFS");
    return;
  }
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.begin();
}

void WebSocketInterface::broadcastAngle(float angle) {
  StaticJsonDocument<128> doc;
  doc["angle"] = angle;
  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}