#include <Status.h>

bool running_led_state = false;
bool running = false;

uint32_t status_time1 = millis();

void Status::begin()
{
    pinMode(RUNNING_LED, OUTPUT);
    pinMode(ERROR_LED, OUTPUT);
    digitalWrite(RUNNING_LED, HIGH);
    digitalWrite(ERROR_LED, HIGH);
}

void Status::update()
{
    uint16_t diff1 = millis() - status_time1;
    
    if(diff1 > 250)
    {
        if(running)
        {
            running_led_state = !running_led_state;
            digitalWrite(RUNNING_LED, running_led_state);
        }

        status_time1 = millis();
    }
}

void Status::setReady(bool condition)
{
    running = false;
    digitalWrite(RUNNING_LED, condition);
    setError(false);
}

void Status::setRunning(bool condition)
{
    running = condition;
    setError(false);
    setWarning(false);
}

void Status::setError(bool condition)
{
    digitalWrite(ERROR_LED, condition);
    if(condition == true)
    {
        running = false;
        digitalWrite(RUNNING_LED, LOW);
    }
}

void Status::setWarning(bool condition)
{
    //digitalWrite(ERROR_LED, condition);
    if(condition == true)
    {
        running = false;
        setError(false);
    }
}
