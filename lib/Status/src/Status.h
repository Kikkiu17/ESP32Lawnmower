#define _STATUS_H_
#ifdef _STATUS_H_

#include <Arduino.h>
#include <SETTINGS.h>

class Status
{
    public:
        void begin();
        void update();
        void setError(bool);
        void setWarning(bool);
        void setRunning(bool);
        void setReady(bool);
};

#endif
