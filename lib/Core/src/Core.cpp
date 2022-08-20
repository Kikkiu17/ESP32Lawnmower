#include <Arduino.h>
#include <Core.h>
#include <Motors.h>
#include <Sensors.h>
#include <Navigation.h>
#include <Status.h>
#include <SETTINGS.h>
#include <BluetoothSerial.h>
#include <Mux.h>

Motors motors;
Sensors sensors;
Status status;
NAV navigation;
Mux mux;
BluetoothSerial coreSerialBT;

uint32_t ref_time = millis();
byte BTData;
bool lowbat = false;
uint64_t *ptr;

void Core::begin()
{
    status.begin();
    motors.begin();
    sensors.begin();
    status.setReady(true);
    mux.begin();

    status.setReady(true);
    motors.setSpeed(0, MAIN);
    ptr = mux.getPacketPointer();
}

void Core::loop()
{
    if (lowbat)
        return;
    // controllo input utente
    if (coreSerialBT.available())
    {
        BTData = coreSerialBT.read();
        if (BTData == 'w')
        {
            println("BT FORWARD");
            navigation.goForward();
        }
        else if (BTData == 'a')
        {
            println("BT LEFT");
            navigation.rotateForDeg(-90);
        }
        else if (BTData == 's')
        {
            println("BT BACK");
            navigation.goBackwards();
        }
        else if (BTData == 'd')
        {
            println("BT RIGHT");
            navigation.rotateForDeg(90);
        }
        else if (BTData == 'y')
        {
            mux.sensPacketUpdate(false);
        }
        else if (BTData == 'i')
        {
            mux.sensPacketUpdate(true);
        }
        else if (BTData == 'o')
        {
            println("GET HDG", navigation.getHDG());
        }
        else if (BTData == 'c')
        {
            sensors.setZero();
        }
        else if (BTData == 'b')
        {
            coreSerialBT.print("Batteria: ");
            coreSerialBT.print(sensors.getBatADC());
            coreSerialBT.println(" su 2770");
        }
        else if (BTData == 'm')
        {
            motors.toggleMainMotor();
        }
        else if (BTData == 'p')
        {
            ledcWrite(CHANNEL_MAIN, 0);
        }
        else if (BTData == 'u')
        {
            navigation.autoRun();
        }
        else
        {
            println("BT STOP");
            navigation.externalStop();
        }
    }

    status.update();
    sensors.update();
    motors.update();
    if (!ENABLE_AUTO_NAVIGATION)
        return;
    navigation.update();

    /*if (millis() - ref_time > 1000)
    {
        ref_time = millis();
        Serial.println("SENSOR DATA PACKET");
        for (int i = 0; i < 9; i++)
            Serial.println(*(ptr + i));
    }*/

    if (SHOW_MODULE_EXECUTION_TIME)
    {
        if (millis() - ref_time > 250)
        {
            ref_time = millis();
            coreSerialBT.println();
            println("(Core.cpp) TIME IN MICROSECONDS (us)");
            println("TIME TO EXECUTE LOOP OF MOT", motors.getTime());
            println("TIME TO EXECUTE LOOP OF SENS", sensors.getTime());
            println("TIME TO EXECUTE LOOP OF NAV", navigation.getTime());
            println("STATUS OF HEAP FRAGMENTATION");
            println("MAX HEAP", heap_caps_get_free_size(MALLOC_CAP_8BIT));
            println("LARGEST BLOCK AVAILABLE", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        }
    }
}

void Core::printTimestamp()
{
    if (USING_USB_SERIAL)
    {
        Serial.print("AT_");
        Serial.print(millis());
        Serial.print(":");
    }
    coreSerialBT.print("AT_");
    coreSerialBT.print(millis());
    coreSerialBT.print(":");
}

void Core::print(const __FlashStringHelper* type, float data)
{
    printTimestamp();
    if (data != -25565)
    {
        if (USING_USB_SERIAL)
        {
            Serial.print(type);
            Serial.print(": ");
            Serial.print(data);
            Serial.print(";");
        }
        coreSerialBT.print(type);
        coreSerialBT.print(": ");
        coreSerialBT.print(data);
        coreSerialBT.print(";");
    }
    else
    {
        if (USING_USB_SERIAL)
            Serial.print(type);
        coreSerialBT.print(type);
    }
}

void Core::println(const __FlashStringHelper* type, float data)
{
    printTimestamp();
    if (data != -25565)
    {
        if (USING_USB_SERIAL)
        {
            Serial.print(type);
            Serial.print(": ");
            Serial.print(data);
            Serial.print(";");
            Serial.println();
        }
        coreSerialBT.print(type);
        coreSerialBT.print(": ");
        coreSerialBT.print(data);
        coreSerialBT.println(";");
    }
    else
    {
        if (USING_USB_SERIAL)
            Serial.println(type);
        coreSerialBT.println(type);
    }
}

void Core::print(const char *type, float data)
{
    printTimestamp();
    if (data != -25565)
    {
        if (USING_USB_SERIAL)
        {
            Serial.print(type);
            Serial.print(": ");
            Serial.print(data);
            Serial.print(";");
        }
        coreSerialBT.print(type);
        coreSerialBT.print(": ");
        coreSerialBT.print(data);
        coreSerialBT.print(";");
    }
    else
    {
        if (USING_USB_SERIAL)
            Serial.print(type);
        coreSerialBT.print(type);
    }
}

void Core::println(const char *type, float data)
{
    printTimestamp();
    if (data != -25565)
    {
        if (USING_USB_SERIAL)
        {
            Serial.print(type);
            Serial.print(": ");
            Serial.print(data);
            Serial.println(";");
        }
        coreSerialBT.print(type);
        coreSerialBT.print(": ");
        coreSerialBT.print(data);
        coreSerialBT.println(";");
    }
    else
    {
        if (USING_USB_SERIAL)
            Serial.println(type);
        coreSerialBT.println(type);
    }
}

void Core::printStartDataPacket()
{
    Serial.print("DATA_PACKET_START:");
    coreSerialBT.print("DATA_PACKET_START:");
}

void Core::printStopDataPacket()
{
    Serial.print("DATA_PACKET_STOP;");
    coreSerialBT.print("DATA_PACKET_STOP;");
}

void Core::lowBat()
{
    println("BATTERIA SCARICA");
    lowbat = true;
    motors.toggleMainMotor(0, STOP);
    motors.stop();
    status.setError(true);
}
