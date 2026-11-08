#ifndef CLI_HPP
#define CLI_HPP

#include <Arduino.h>

class CLI {
public:
    void begin();
    void update();

private:
    String _buffer;
    bool _streaming = false;
    uint32_t _lastStream = 0;
    void _processLine(const String& line);
    void _printPrompt();
    void _printHelp();
};

extern CLI cli;
#endif
