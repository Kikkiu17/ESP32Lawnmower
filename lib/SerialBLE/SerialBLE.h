#ifndef _SERIALBLE_H_
#define _SERIALBLE_H_

#include <Arduino.h>

class SerialBLE
{
    public:
        void begin();
        bool available();
        std::string getString();
        void write(std::string data);
        void write(float data);
        void write(int32_t data);
        void write(char* data);
        void write(const __FlashStringHelper* data);
};

#endif
