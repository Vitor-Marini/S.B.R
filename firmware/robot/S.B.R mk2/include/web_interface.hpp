#ifndef WEB_INTERFACE_HPP
#define WEB_INTERFACE_HPP

#include <ESPAsyncWebServer.h>

class WebManager {
public:
    void begin();
    void broadcast();
    void notifyStateChange();

private:
    AsyncWebServer _server = AsyncWebServer(80);
    AsyncWebSocket _ws = AsyncWebSocket("/ws");

    uint32_t _lastBroadcast = 0;

    static void _onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                           AwsEventType type, void* arg, uint8_t* data, size_t len);
    void _sendConfig(AsyncWebSocketClient* client);
};

extern WebManager web;
#endif
