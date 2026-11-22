#include "web.hpp"
#include "cli.hpp"
#include "net.hpp"
#include "web_page.h"
#include <ESPAsyncWebServer.h>

WebManager web;

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

// fila de linhas recebidas (escrita pela tarefa do servidor, lida pelo loop)
static const int LINES = 8, LINE_LEN = 96;
static char rxLines[LINES][LINE_LEN];
static volatile int rxHead = 0, rxTail = 0;
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

static void pushLine(const uint8_t* data, size_t len) {
    portENTER_CRITICAL(&rxMux);
    int next = (rxHead + 1) % LINES;
    if (next != rxTail) {  // fila cheia: descarta (o próximo drive chega em 50 ms)
        size_t n = min(len, (size_t)LINE_LEN - 1);
        memcpy(rxLines[rxHead], data, n);
        rxLines[rxHead][n] = 0;
        rxHead = next;
    }
    portEXIT_CRITICAL(&rxMux);
}

static void onWs(AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type, void* arg,
                 uint8_t* data, size_t len) {
    // sem Nagle: senão cada pacote pequeno espera a confirmação do anterior (~5 pacotes/s)
    if (type == WS_EVT_CONNECT) client->client()->setNoDelay(true);
    if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
            pushLine(data, len);
    }
}

void WebManager::begin() {
    if (_started) return;
    ws.onEvent(onWs);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html; charset=utf-8", WEB_PAGE);
    });
    server.begin();
    _started = true;
    term.println("Web: página em http://sbr.local (ou pelo IP)");
}

int WebManager::clients() { return _started ? ws.count() : 0; }

void WebManager::queue(const uint8_t* buf, size_t len) {
    if (!_started || !ws.count()) return;
    if (_outLen + len > sizeof(_out)) { _dropped++; _outLen = 0; }  // conexão parada: recomeça do atual
    if (len > sizeof(_out)) return;
    memcpy(_out + _outLen, buf, len);
    _outLen += len;
}

void WebManager::_flush() {
    // A biblioteca só manda a próxima mensagem quando a anterior é confirmada; então acumula enquanto
    // há mensagem a caminho e manda tudo junto: a vazão se adapta à conexão sem perder dado.
    // shortcut: com 2+ páginas, a mais lenta dita o ritmo de todas; separar buffers se precisar.
    if (!_outLen) return;
    for (AsyncWebSocketClient& c : ws.getClients())
        if (c.status() == WS_CONNECTED && c.queueLen() > 0) return;
    ws.textAll((const char*)_out, _outLen);
    _outLen = 0;
}

void WebManager::printStatus() {
    for (AsyncWebSocketClient& c : ws.getClients())
        term.printf("  página %s: fila %u/%u\n", c.remoteIP().toString().c_str(), (unsigned)c.queueLen(),
                    (unsigned)WS_MAX_QUEUED_MESSAGES);
    term.printf("  saída descartada (página sem confirmar por muito tempo): %lu vezes\n", (unsigned long)_dropped);
}

void WebManager::update() {
    if (!_started) return;
    char line[LINE_LEN];
    for (;;) {
        portENTER_CRITICAL(&rxMux);
        bool has = rxTail != rxHead;
        if (has) {
            memcpy(line, rxLines[rxTail], LINE_LEN);
            rxTail = (rxTail + 1) % LINES;
        }
        portEXIT_CRITICAL(&rxMux);
        if (!has) break;
        cli.run(String(line));
    }
    _flush();
    static uint32_t lastClean = 0;
    if (millis() - lastClean > 1000) { lastClean = millis(); ws.cleanupClients(); }
}
