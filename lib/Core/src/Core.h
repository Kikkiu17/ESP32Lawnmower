#define _CORE_H_
#ifdef _CORE_H_

class Core
{
    public:
        void begin();
        void loop();
        void printTimestamp();
        void printStartDataPacket();
        void printStopDataPacket();
        void println(const char *TYPE, float VALUE = -25565);
        void print(const char *TYPE, float VALUE = -25565);
        void print(const __FlashStringHelper* TYPE, float VALUE = -25565);
        void println(const __FlashStringHelper* TYPE, float VALUE = -25565);
        void lowBat();
};

#endif
