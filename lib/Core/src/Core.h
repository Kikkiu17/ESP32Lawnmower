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
        void println(char* TYPE, float VALUE = -25565);
};

#endif
