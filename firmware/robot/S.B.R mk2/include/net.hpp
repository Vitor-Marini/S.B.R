#ifndef NET_HPP
#define NET_HPP

#include <Arduino.h>

// Terminal do robô sem fio: UDP na rede de casa, mesmo protocolo da serial.
// Comandos chegam por UDP:5555; tudo que o firmware imprime via `term` vai pra serial
// e, em um pacote por loop, de volta para quem mandou o último pacote.
// UDP de propósito: nunca bloqueia o loop de controle (pacote perdido não importa).
class Tee : public Print {
public:
    size_t write(uint8_t c) override;
    size_t write(const uint8_t* buf, size_t len) override;
};

class NetManager {
public:
    void begin();   // conecta se houver rede salva (`wifi <rede> <senha>`)
    void update();  // lê comandos e envia a saída acumulada; chamar todo loop
    void printStatus();

    void queue(const uint8_t* buf, size_t len);

private:
    static const uint16_t PORT = 5555;
    bool _started = false;
    bool _announced = false;
    uint8_t _out[1400];
    size_t _outLen = 0;
    void _flush();
};

extern Tee term;
extern NetManager net;
#endif
