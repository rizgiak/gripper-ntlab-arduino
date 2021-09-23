#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

class Debug {
   private:
    Stream* _s;
    bool _serialDebug;

   public:
    Debug(HardwareSerial* serial, bool serialDebug);
    void println(char const* str);
    void print(char const* str);
};

#endif