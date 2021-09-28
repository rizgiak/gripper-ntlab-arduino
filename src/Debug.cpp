#include <Debug.h>

Debug::Debug(HardwareSerial* serial, bool serialDebug) : _s(serial), _serialDebug(serialDebug) {
}

void Debug::print(char const* str) {
    if (_serialDebug) {
        _s->print(str);
        delay(10);
    }
}

void Debug::println(char const* str) {
    if (_serialDebug) {
        _s->println(str);
        delay(10);
    }
}