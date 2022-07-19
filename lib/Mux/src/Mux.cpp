#include <MUX.h>
#include <BluetoothSerial.h>
#ifdef ENABLE_WEBSERIAL
#include <WebSerial.h>
#endif
#include <SETTINGS.h>
#include <NewPing.h>
#include <Sensors.h>

BluetoothSerial MUXBT;
Sensors MUXSensor;

TaskHandle_t PulseIn;

bool execute_once = false;
bool signal_low = false;
bool rising_edge = false;
bool got_reading = false;
bool stop = false;
uint64_t start_US_time = 0;
uint64_t stop_US_time = 0;

uint8_t channels[16][4] =
{
    {0, 0, 0, 0},
    {0, 0, 0, 1},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 1, 0, 0},
    {0, 1, 0, 1},
    {0, 1, 1, 0},
    {0, 1, 1, 1},
    {1, 0, 0, 0},
    {1, 0, 0, 1},
    {1, 0, 1, 0},
    {1, 0, 1, 1},
    {1, 1, 0, 0},
    {1, 1, 0, 1},
    {1, 1, 1, 0},
    {1, 1, 1, 1},
};

struct
{
    uint8_t write = 0;
    uint8_t read = 0;
} USData;

void Mux::selectChannel(byte ch, uint8_t mode)
{
    ledcDetachPin(MUX_COM);
    if (mode == READ)
        pinMode(MUX_COM, INPUT);
    else
        pinMode(MUX_COM, OUTPUT);

    digitalWrite(MUX0, channels[ch][3]);
    digitalWrite(MUX1, channels[ch][2]);
    digitalWrite(MUX2, channels[ch][1]);
    digitalWrite(MUX3, channels[ch][0]);
    digitalWrite(MUX_ENABLE, 0);
}

void PulseInFunction(void *param)
{
    uint8_t watchdog_counter = 0;
    Mux taskmux;
    for (;;)
    {
        // eseguito in core0 con priorità 24 (max)
        if (!execute_once)
        {
            execute_once = true;
            ledcDetachPin(MUX_COM);
            pinMode(MUX_COM, OUTPUT); // write
            digitalWrite(MUX0, channels[USData.write][3]);
            digitalWrite(MUX1, channels[USData.write][2]);
            digitalWrite(MUX2, channels[USData.write][1]);
            digitalWrite(MUX3, channels[USData.write][0]);
            digitalWrite(MUX_COM, 1);
            delayMicroseconds(9);
            digitalWrite(MUX_COM, 0);
        }
        else if (!got_reading)
        {
            if (!signal_low)
            {
                taskmux.selectChannel(USData.read, READ);
                if (analogRead(MUX_COM) < 2046)
                {
                    signal_low = true;
                }
            }
            else if (!rising_edge)
            {
                watchdog_counter++;
                taskmux.selectChannel(USData.read, READ);
                if (analogRead(MUX_COM) > 2046)
                {
                    rising_edge = true;
                    start_US_time = esp_timer_get_time();
                }
            }
            else if (!got_reading)
            {
                taskmux.selectChannel(USData.read, READ);
                if (analogRead(MUX_COM) < 2046)
                {
                    stop_US_time = esp_timer_get_time();
                    uint64_t diff = stop_US_time - start_US_time;
                    execute_once = false;
                    signal_low = false;
                    rising_edge = false;
                    got_reading = false;
                    MUXSensor.returnUSDistance(diff * 0.017);
                    vTaskDelete(NULL);
                }
            }
        }

        if (watchdog_counter > 100)
        {
            MUXBT.println("WATCHDOG COUNTER TRIGGERED - CURRENT CHANNELS: ");
            MUXBT.print("WRITE: ");
            MUXBT.println(USData.write);
            MUXBT.print("READ: ");
            MUXBT.println(USData.read);
            MUXBT.println("RESETTING...");
            ESP.restart();
            vTaskDelete(NULL);
        }
    }
}

void Mux::begin()
{
    pinMode(MUX0, OUTPUT);
    pinMode(MUX1, OUTPUT);
    pinMode(MUX2, OUTPUT);
    pinMode(MUX3, OUTPUT);
    pinMode(MUX_ENABLE, OUTPUT);
}

void Mux::loop()
{

}

uint16_t Mux::readAnalog(byte ch)
{
    digitalWrite(MUX_ENABLE, 1);
    ledcDetachPin(MUX_COM);
    pinMode(MUX_COM, INPUT);
    digitalWrite(MUX0, channels[ch][3]);
    digitalWrite(MUX1, channels[ch][2]);
    digitalWrite(MUX2, channels[ch][1]);
    digitalWrite(MUX3, channels[ch][0]);
    delayMicroseconds(5);
    digitalWrite(MUX_ENABLE, 0);
    return analogRead(MUX_COM);
}

uint8_t Mux::readDigital(byte ch)
{
    digitalWrite(MUX_ENABLE, 1);
    ledcDetachPin(MUX_COM);
    pinMode(MUX_COM, INPUT);
    digitalWrite(MUX0, channels[ch][3]);
    digitalWrite(MUX1, channels[ch][2]);
    digitalWrite(MUX2, channels[ch][1]);
    digitalWrite(MUX3, channels[ch][0]);
    delayMicroseconds(5);
    digitalWrite(MUX_ENABLE, 0);
    if (analogRead(MUX_COM) > 2046)
        return 1;
    else
        return 0;
}

void Mux::writeDigital(byte ch, uint8_t value)
{
    ledcDetachPin(MUX_COM);
    pinMode(MUX_COM, OUTPUT);
    digitalWrite(MUX0, channels[ch][3]);
    digitalWrite(MUX1, channels[ch][2]);
    digitalWrite(MUX2, channels[ch][1]);
    digitalWrite(MUX3, channels[ch][0]);
    digitalWrite(MUX_ENABLE, 0);
    digitalWrite(MUX_COM, value);
}

void Mux::writeAnalog(byte ch, uint16_t value, uint32_t frequency, uint8_t resolution)
{
    ledcAttachPin(MUX_COM, 1);
    ledcSetup(1, frequency, resolution);
    digitalWrite(MUX0, channels[ch][3]);
    digitalWrite(MUX1, channels[ch][2]);
    digitalWrite(MUX2, channels[ch][1]);
    digitalWrite(MUX3, channels[ch][0]);
    digitalWrite(MUX_ENABLE, 0);
    ledcWrite(1, value);
}

void Mux::fastWrite(uint8_t value)
{
    digitalWrite(MUX_COM, value);
}

uint16_t Mux::fastRead()
{
    return analogRead(MUX_COM);
}

float Mux::requestUSDistance(uint8_t sens)
{
    // non c'è bisogno del ledcDetachPin, perché vengono usate solo funzioni di questa libreria,
    // che incorporano tutte ledcDetachPin

    if (sens == FRONT)
    {
        USData.read = 7;
        USData.write = 4;
    }
    else if (sens == RIGHT)
    {
        USData.read = 6;
        USData.write = 5;
    }
    else
    {
        USData.read = 9;
        USData.write = 10;
    }

    xTaskCreatePinnedToCore(PulseInFunction, "PulseIn", 2000, NULL, 24, &PulseIn, 0);

    return 15;
}

