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

bool lowbat = false;

void Core::begin()
{
    status.begin();
    motors.begin();
    sensors.begin();
    sensors.setZero();
    status.setReady(true);
    mux.begin();
}

void Core::loop()
{
    if (lowbat)
        return;
    status.update();
    sensors.update();
    motors.update();
    navigation.update();
}

void Core::printTimestamp()
{
    Serial.print("AT_");
    Serial.print(millis());
    Serial.print(":");
    coreSerialBT.print("AT_");
    coreSerialBT.print(millis());
    coreSerialBT.print(":");
}

void Core::println(char* type, float data)
{
    printTimestamp();
    if (data != -25565)
    {
        Serial.print(type);
        Serial.print(":");
        Serial.print(data);
        Serial.println(";");
        coreSerialBT.print(type);
        coreSerialBT.print(":");
        coreSerialBT.print(data);
        coreSerialBT.println(";");
    }
    else
    {
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
}
