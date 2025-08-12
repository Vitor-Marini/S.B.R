#ifndef WEB_INTERFACE_HPP
#define WEB_INTERFACE_HPP

#include <ESPAsyncWebServer.h>

class WebSocketInterface {
public:
  void begin();                         
  void broadcastAngle(float angle);     
  void handleIncomingData(const String &json); 

private:
  static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                        AwsEventType type, void *arg, uint8_t *data, size_t len);
                        
  void handleTextMessage(uint8_t *data, size_t len); // Trata a mensagem recebida
};

#endif