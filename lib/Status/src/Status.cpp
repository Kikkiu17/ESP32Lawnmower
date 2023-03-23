#include <Status.h>

bool running_led_state = false;
bool running = false;
bool main_motor_starting = false;
bool inactive = false;
bool inactive_status = false;
uint8_t main_motor_warning = 0;

uint32_t status_time1 = millis();

void Status::begin()
{
    // già su output da main.cpp
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

        if (inactive)
        {
            if (!inactive_status)
            {
                ledcWrite(CHANNEL_MAIN, 7);
                inactive_status = !inactive_status;
            }
            else
            {
                ledcWrite(CHANNEL_MAIN, 0);
                inactive_status = !inactive_status;
            }
        }

        status_time1 = millis();
    }

    if (main_motor_starting)
    {
        if (diff1 > 50)
        {
            if (main_motor_warning < 25)
            {
                running_led_state = !running_led_state;
                digitalWrite(RUNNING_LED, running_led_state);
                status_time1 = millis();
                main_motor_warning++;
            }
            else
            {
                main_motor_warning = 0;
                main_motor_starting = false;
                running_led_state = 1;
                digitalWrite(RUNNING_LED, running_led_state);
            }
        }
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

void Status::mainMotorStarting()
{
    main_motor_starting = true;
}

void Status::setInactive(bool state)
{
    inactive = state;
    if (state)
    {   
        inactive_status = false;
        ledcSetup(CHANNEL_MAIN, 496, 8);
        setReady(false);
        setRunning(false);
        main_motor_warning = 0;
        main_motor_starting = false;
    }
    else
    {
        inactive_status = false;
        ledcSetup(CHANNEL_MAIN, 100, 8);
        setReady(true);
    }
}
