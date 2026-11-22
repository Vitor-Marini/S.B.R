#ifndef WEB_HPP
#define WEB_HPP

#include <Arduino.h>

// Página de controle servida pelo robô + WebSocket (/ws) com o mesmo protocolo do terminal:
// o navegador manda linhas de comando (drive, start, tune...) e recebe tudo que `term` imprime
// (inclusive o stream). O servidor roda em outra tarefa: comandos recebidos vão pra uma fila e
// são executados em update(), no loop principal, nunca no meio do controle.
class WebManager {
public:
    void begin();                                // chamar com a rede de pé
    void update();                               // executa comandos e envia a saída; todo loop
    void queue(const uint8_t* buf, size_t len);  // saída do terminal (chamado pelo Tee)
    int clients();
    void printStatus();

private:
    bool _started = false;
    uint8_t _out[4096];  // ~15 quadros do stream: ~300 ms sem confirmação antes de descartar
    size_t _outLen = 0;
    uint32_t _dropped = 0;
    void _flush();
};

extern WebManager web;
#endif
