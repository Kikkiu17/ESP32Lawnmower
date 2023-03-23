#include <MUX.h>
#include <SETTINGS.h>
#include <Sensors.h>
#include <Core.h>

#define READ_ANALOG 1
#define READ_DIGITAL 2
#define SENS_PACKET 3
#define SENS_PACKET_ONESHOT 4
#define STOP_SENS_PACKET 5

// vedere SETTINGS.h in region "Pin diretti ESP" per informazioni sul pin 17 - ENABLE

Core muxcore;
TaskHandle_t PulseInHandle;
TaskHandle_t ReadDigitalHandle;
TaskHandle_t ReadAnalogHandle;
TaskHandle_t TaskManagerHandle;
QueueHandle_t TaskManagerQueue;
QueueHandle_t ReadAnalogQueue;
QueueHandle_t ReadDigitalQueue;
QueueHandle_t PulseInQueue;

static StaticQueue_t TaskManagerStaticQueue;
static StaticQueue_t ReadDigitalStaticQueue;
static StaticQueue_t ReadAnalogStaticQueue;
static StaticQueue_t PulseInStaticQueue;

uint8_t TaskManagerArray[100];
uint8_t ReadDigitalArray[100];
uint8_t ReadAnalogArray[100];
uint8_t PulseInArray[100];

bool pulsein_returned = false;
bool execute_once = false;
bool signal_low = false;
bool rising_edge = false;
bool got_reading = false;
bool stop = false;
int64_t start_US_time = 0;
int64_t stop_US_time = 0;

bool sens_packet_stopped = false;

uint8_t sensor = 0;

// array:
// US_F, US_L, US_R, IR_F, IR_L, BAT, READ_DIGITAL, READ_ANALOG, PACKET_ID
uint64_t mux_data[9] = {};
uint64_t *mux_data_ptr = mux_data;
uint8_t mux_data_idx = 0;
uint8_t mux_ch = 0;

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

void IRAM_ATTR Mux::selectChannel(byte ch, uint8_t mode)
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
}

void IRAM_ATTR PulseInFunction(void *param)
{
    uint8_t watchdog_counter = 0;
    uint32_t buffer = 0;
    uint32_t item_to_queue = 0;
    for (;;)
    {
        if (!execute_once)
        {
            if (xQueueReceive(PulseInQueue, &buffer, 1) == pdTRUE)
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
        }
        else if (!got_reading)
        {
            if (!signal_low)
            {
                pinMode(MUX_COM, INPUT); // read
                digitalWrite(MUX0, channels[USData.read][3]);
                digitalWrite(MUX1, channels[USData.read][2]);
                digitalWrite(MUX2, channels[USData.read][1]);
                digitalWrite(MUX3, channels[USData.read][0]);
                if (analogRead(MUX_COM) < 2046)
                    signal_low = true;
            }
            else if (!rising_edge)
            {
                watchdog_counter++;
                pinMode(MUX_COM, INPUT); // read
                digitalWrite(MUX0, channels[USData.read][3]);
                digitalWrite(MUX1, channels[USData.read][2]);
                digitalWrite(MUX2, channels[USData.read][1]);
                digitalWrite(MUX3, channels[USData.read][0]);
                if (analogRead(MUX_COM) > 2046)
                {
                    rising_edge = true;
                    start_US_time = esp_timer_get_time();
                }
            }
            else if (!got_reading)
            {
                pinMode(MUX_COM, INPUT); // read
                digitalWrite(MUX0, channels[USData.read][3]);
                digitalWrite(MUX1, channels[USData.read][2]);
                digitalWrite(MUX2, channels[USData.read][1]);
                digitalWrite(MUX3, channels[USData.read][0]);
                stop_US_time = esp_timer_get_time();
                if (stop_US_time - start_US_time > US_SENS_DST_TRIG + 50 || analogRead(MUX_COM) < 2046)
                {
                    uint64_t diff = stop_US_time - start_US_time;
                    execute_once = false;
                    signal_low = false;
                    rising_edge = false;
                    got_reading = false;
                    watchdog_counter = 0;
                    mux_data[mux_data_idx] = diff;
                    mux_data_idx++;
                    vTaskPrioritySet(PulseInHandle, 5);
                    vTaskDelay(1);
                    item_to_queue = SENS_PACKET;
                    xQueueSend(TaskManagerQueue, (void *)&item_to_queue, 0);
                }
            }
        }

        if (watchdog_counter > 50)
        {
            execute_once = false;
            signal_low = false;
            rising_edge = false;
            got_reading = false;
            watchdog_counter = 0;
            mux_data[mux_data_idx] = 0;
            mux_data_idx++;
            vTaskPrioritySet(PulseInHandle, 5);
            vTaskDelay(1);
            item_to_queue = SENS_PACKET;
            xQueueSend(TaskManagerQueue, (void *)&item_to_queue, 0);
        }
    }
}

void TaskManagerFunction(void *param)
{
    uint32_t buffer = 0;
    uint32_t item_to_queue = 0;
    for (;;)
    {
        if (xQueueReceive(TaskManagerQueue, &buffer, 1) == pdTRUE)
        {
            if (buffer == READ_ANALOG)
            {
                item_to_queue = READ_ANALOG;
                xQueueSend(ReadAnalogQueue, (void *)&item_to_queue, 0);
            }
            else if (buffer == READ_DIGITAL)
            {
                item_to_queue = READ_DIGITAL;
                xQueueSend(ReadDigitalQueue, (void *)&item_to_queue, 0);
            }
            else if (buffer == SENS_PACKET || buffer == SENS_PACKET_ONESHOT)
            {
                if (!sens_packet_stopped)
                {
                    switch (sensor)
                    {
                        case 0:
                        {
                            sensor++;
                            USData.write = US_TRIG_F;
                            USData.read = US_ECHO_F;
                            vTaskPrioritySet(PulseInHandle, 20);
                            item_to_queue = SENS_PACKET;
                            xQueueSend(PulseInQueue, (void *)&item_to_queue, 0);
                            break;
                        }
                        case 1:
                        {
                            sensor++;
                            USData.write = US_TRIG_L;
                            USData.read = US_ECHO_L;
                            vTaskPrioritySet(PulseInHandle, 20);
                            item_to_queue = SENS_PACKET;
                            xQueueSend(PulseInQueue, (void *)&item_to_queue, 0);
                            break;
                        }
                        case 2:
                        {
                            sensor++;
                            USData.write = US_TRIG_R;
                            USData.read = US_ECHO_R;
                            vTaskPrioritySet(PulseInHandle, 20);
                            item_to_queue = SENS_PACKET;
                            xQueueSend(PulseInQueue, (void *)&item_to_queue, 0);
                            break;
                        }
                        case 3:
                        {
                            sensor++;
                            mux_ch = IR_F;
                            item_to_queue = SENS_PACKET;
                            xQueueSend(ReadDigitalQueue, (void *)&item_to_queue, 0);
                            break;
                        }
                        case 4:
                        {
                            sensor++;
                            mux_ch = IR_L;
                            item_to_queue = SENS_PACKET;
                            xQueueSend(ReadDigitalQueue, (void *)&item_to_queue, 0);
                            break;
                        }
                        case 5:
                        {
                            sensor++;
                            mux_ch = BAT;
                            item_to_queue = SENS_PACKET;
                            xQueueSend(ReadAnalogQueue, (void *)&item_to_queue, 0);
                            break;
                        }
                        case 6:
                        {
                            sensor = 0;
                            mux_data_idx = 0;
                            mux_data[8]++;
                            vTaskDelay(pdMS_TO_TICKS(6));
                            item_to_queue = SENS_PACKET;
                            xQueueSend(TaskManagerQueue, (void *)&item_to_queue, 0);
                            break;
                        }
                    }
                }
                else
                {
                    sensor = 0;
                    mux_data_idx = 0;
                }
            }
        }
    }
}

void ReadDigitalFunction(void *param)
{
    uint32_t buffer = 0;
    uint32_t item_to_queue = 0;
    for (;;)
    {
        if (xQueueReceive(ReadDigitalQueue, &buffer, 1) == pdTRUE)
        {
            pinMode(MUX_COM, INPUT);
            digitalWrite(MUX0, channels[mux_ch][3]);
            digitalWrite(MUX1, channels[mux_ch][2]);
            digitalWrite(MUX2, channels[mux_ch][1]);
            digitalWrite(MUX3, channels[mux_ch][0]);
            delayMicroseconds(10);

            if (buffer == SENS_PACKET)
            {
                mux_data[mux_data_idx] = digitalRead(MUX_COM);
                mux_data_idx++;
                item_to_queue = SENS_PACKET;
                xQueueSend(TaskManagerQueue, (void *)&item_to_queue, 0);
            }
            else if (buffer == READ_DIGITAL)
            {
                mux_data[8]++;
                mux_data[6] = digitalRead(MUX_COM);
            }
        }
    }
}

void ReadAnalogFunction(void *param)
{
    uint32_t buffer = 0;
    uint32_t item_to_queue = 0;
    for (;;)
    {
        if (xQueueReceive(ReadAnalogQueue, &buffer, 1) == pdTRUE)
        {
            pinMode(MUX_COM, INPUT);
            digitalWrite(MUX0, channels[mux_ch][3]);
            digitalWrite(MUX1, channels[mux_ch][2]);
            digitalWrite(MUX2, channels[mux_ch][1]);
            digitalWrite(MUX3, channels[mux_ch][0]);
            delayMicroseconds(10);

            if (buffer == SENS_PACKET)
            {
                mux_data[mux_data_idx] = analogRead(MUX_COM);
                mux_data_idx++;
                item_to_queue = SENS_PACKET;
                xQueueSend(TaskManagerQueue, (void *)&item_to_queue, 0);
            }
            else if (buffer == READ_ANALOG)
            {
                mux_data[8]++;
                mux_data[7] = analogRead(MUX_COM);
            }
        }
    }
}

uint64_t *Mux::getPacketPointer()
{
    return mux_data_ptr;
}

uint16_t Mux::readAnalog(byte ch)
{
    uint32_t item_to_queue = 0;
    mux_ch = ch;
    item_to_queue = READ_ANALOG;
    xQueueSend(TaskManagerQueue, (void *)&item_to_queue, 0);
    return 0;
}

uint8_t Mux::readDigital(byte ch)
{
    // digitalWrite(MUX_ENABLE, 1);
    ledcDetachPin(MUX_COM);
    pinMode(MUX_COM, INPUT);
    digitalWrite(MUX0, channels[ch][3]);
    digitalWrite(MUX1, channels[ch][2]);
    digitalWrite(MUX2, channels[ch][1]);
    digitalWrite(MUX3, channels[ch][0]);
    delayMicroseconds(5);
    // digitalWrite(MUX_ENABLE, 0);
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
    // digitalWrite(MUX_ENABLE, 0);
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
    // digitalWrite(MUX_ENABLE, 0);
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
    // non c'è bisogno di ledcDetachPin, perché vengono usate solo funzioni di questa libreria,
    // che incorporano tutte ledcDetachPin

    if (sens == FRONT)
    {
        USData.read = US_ECHO_F;
        USData.write = US_TRIG_F;
    }
    else if (sens == RIGHT)
    {
        USData.read = US_ECHO_R;
        USData.write = US_TRIG_R;
    }
    else
    {
        USData.read = US_ECHO_L;
        USData.write = US_TRIG_L;
    }

    return 15;
}

void Mux::sensPacketUpdate(bool polling_mode)
{
    if (polling_mode)
    {
        sens_packet_stopped = false;
        uint32_t item_to_queue = 0;
        item_to_queue = SENS_PACKET;
        xQueueSend(TaskManagerQueue, (void *)&item_to_queue, 0);
    }
    else if (!sens_packet_stopped)
        sens_packet_stopped = true;
}

void Mux::begin()
{
    pinMode(MUX0, OUTPUT);
    pinMode(MUX1, OUTPUT);
    pinMode(MUX2, OUTPUT);
    pinMode(MUX3, OUTPUT);

    /*TaskManagerQueue = xQueueCreate(20, sizeof(uint32_t));
    ReadDigitalQueue = xQueueCreate(20, sizeof(uint32_t));
    ReadAnalogQueue = xQueueCreate(20, sizeof(uint32_t));
    PulseInQueue = xQueueCreate(20, sizeof(uint32_t));*/
    TaskManagerQueue = xQueueCreateStatic(20, sizeof(uint32_t), TaskManagerArray, &TaskManagerStaticQueue);
    ReadDigitalQueue = xQueueCreateStatic(20, sizeof(uint32_t), ReadDigitalArray, &ReadDigitalStaticQueue);
    ReadAnalogQueue = xQueueCreateStatic(20, sizeof(uint32_t), ReadAnalogArray, &ReadAnalogStaticQueue);
    PulseInQueue = xQueueCreateStatic(20, sizeof(uint32_t), PulseInArray, &PulseInStaticQueue);
    xTaskCreatePinnedToCore(PulseInFunction, "PulseIn", 2048, NULL, 5, &PulseInHandle, 0);
    xTaskCreatePinnedToCore(ReadDigitalFunction, "ReadDigital", 2048, NULL, 2, &ReadDigitalHandle, 0);
    xTaskCreatePinnedToCore(ReadAnalogFunction, "ReadAnalog", 2048, NULL, 2, &ReadAnalogHandle, 0);
    xTaskCreatePinnedToCore(TaskManagerFunction, "TaskManager", 2048, NULL, 10, &TaskManagerHandle, 0);

    /*muxcore.println("PulseIn heap", uxTaskGetStackHighWaterMark(PulseInHandle));
    muxcore.println("ReadDigital heap", uxTaskGetStackHighWaterMark(ReadDigitalHandle));
    muxcore.println("ReadAnalog heap", uxTaskGetStackHighWaterMark(ReadAnalogHandle));
    muxcore.println("TaskManager heap", uxTaskGetStackHighWaterMark(TaskManagerHandle));*/
}

void Mux::loop()
{
    if (mux_data[8] == 50 && !sens_packet_stopped)
        sens_packet_stopped = true;
    return;
}
