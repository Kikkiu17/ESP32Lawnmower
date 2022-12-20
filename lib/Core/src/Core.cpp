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
bool rst_confirmation = false;
bool erase_map_confirmation = false;
bool core_bordermode = false;
bool stop_logging = false;

void Core::begin()
{
    status.begin();
    println(F("Status OK"));
    motors.begin();
    println(F("Motors OK"));
    sensors.begin();
    println(F("Sensors OK"));
    navigation.begin();
    println(F("Navigation OK"));
    mux.begin();
    println(F("Multiplexer OK"));

    status.setReady(true);
    motors.setSpeed(0, MAIN);
    println("Free heap", ESP.getFreeHeap());
    println("Max allocable heap block", ESP.getMaxAllocHeap());
    //ptr = mux.getPacketPointer();
}

void Core::loop()
{
    if (lowbat && ENABLE_BAT_VOLTAGE_SENSING)
        return;
    // controllo input utente
    if (coreSerialBT.available())
    {
        BTData = coreSerialBT.read();
        if (BTData == 'w')
        {
            println(F("BT FORWARD"));
            navigation.goForward();
        }
        else if (BTData == 'a')
        {
            println(F("BT LEFT"));
            navigation.rotateForDeg(-90);
        }
        else if (BTData == 's')
        {
            println(F("BT BACK"));
            navigation.goBackwards();
        }
        else if (BTData == 'd')
        {
            println(F("BT RIGHT"));
            navigation.rotateForDeg(90);
        }
        else if (BTData == 'i')
        {
            navigation.scroll();
        }
        else if (BTData == 'r')
        {
            core_bordermode = !core_bordermode;
            navigation.mapBorderMode(core_bordermode);
            println("Bordermode", core_bordermode);
        }
        else if (BTData == 'o')
        {
            
        }
        else if (BTData == 'c')
        {
            sensors.setZero();
        }
        else if (BTData == 'b')
        {
            uint64_t *packetptr = mux.getPacketPointer();
            mux.readAnalog(BAT);
            delay(50);
            coreSerialBT.print(F("Batteria: "));
            coreSerialBT.print(*(packetptr+5));
            coreSerialBT.println(F(" su 2770"));
        }
        else if (BTData == 'm'|| BTData == 'M')
        {
            motors.toggleMainMotor();
        }
        else if (BTData == 'p')
        {
            println(F("ERASE MAP? Y/N"));
            erase_map_confirmation = true;
        }
        else if (BTData == 'u')
        {
            navigation.autoRun();
        }
        else if (BTData == 't')
        {
            println(F("RESET? Y/N"));
            rst_confirmation = true;
        }
        else if (BTData == 'y' || BTData == 'Y')
        {
            if (rst_confirmation)
            {
                println(F("RESETTING"));
                ESP.restart();
            }
            if (erase_map_confirmation)
            {
                navigation.eraseSD("/MAP.txt");
                println(F("MAP ERASED"));
                erase_map_confirmation = false;
            }
        }
        else if (BTData == 'n' || BTData == 'N')
        {
            if (rst_confirmation)
            {
                println(F("RESET CANCELLED"));
                rst_confirmation = false;
            }
            if (erase_map_confirmation)
            {
                println(F("ERASE MAP CANCELLED"));
                erase_map_confirmation = false;
            }
        }
        else if (BTData == 'l' || BTData == 'L')
        {
            if (!stop_logging)
            {
                println("Logging data to map DISABLED");
                navigation.mapLogging(false);
                stop_logging = true;
            }
            else
            {
                println("Logging data to map ENABLED");
                stop_logging = false;
                navigation.mapLogging(true);
            }
        }
        else if (BTData == 'h' || BTData == 'H')
        {
            println("");
            println("LISTA COMANDI");
            println("w - a - s - d: movimento robot");
            println("b: mostra stato batteria");
            println("c: calibrazione accelerometro/giroscopio");
            println("i - o - p: funzioni temporanee");
            println("l(L) - attiva o disattiva logging nel file mappa");
            println("m(M): accende e spegne motore principale");
            println("p: reset mappa");
            println("r: core_bordermode on/off");
            println("t: reset ESP");
            println("u: autorun");
            println("y(Y) / n(N): yes/no nel caso di prompt");
            println("Alla pressione di altri tasti il robot si ferma");
        }
        else
        {
            println(F("BT STOP"));
            navigation.externalStop();
        }
    }

    status.update();
    sensors.update();
    motors.update();
    if (!ENABLE_AUTO_NAVIGATION)
        return;
    navigation.update();

    if (SHOW_MODULE_EXECUTION_TIME)
    {
        if (millis() - ref_time > 250)
        {
            ref_time = millis();
            coreSerialBT.println();
            println(F("(Core.cpp) TIME IN MICROSECONDS (us)"));
            println(F("TIME TO EXECUTE LOOP OF MOT"), motors.getTime());
            println(F("TIME TO EXECUTE LOOP OF SENS"), sensors.getTime());
            println(F("TIME TO EXECUTE LOOP OF NAV"), navigation.getTime());
            println(F("STATUS OF HEAP FRAGMENTATION"));
            println(F("MAX HEAP"), heap_caps_get_free_size(MALLOC_CAP_8BIT));
            println(F("LARGEST BLOCK AVAILABLE"), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
            println(F("HEAP MAX SIZE"), ESP.getHeapSize());
        }
    }
}

void Core::printTimestamp()
{
    if (USING_USB_SERIAL)
    {
        Serial.print("AT_");
        Serial.print(millis());
        Serial.print(": ");
    }
    coreSerialBT.print("AT_");
    coreSerialBT.print(millis());
    coreSerialBT.print(": ");
}

void Core::print(const __FlashStringHelper* type, float data)
{
    if (data != -25565)
    {
        if (USING_USB_SERIAL)
        {
            Serial.print(type);
            Serial.print(F(": "));
            Serial.print(data);
        }
        coreSerialBT.print(type);
        coreSerialBT.print(F(": "));
        coreSerialBT.print(data);
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
    if (data != -25565)
    {
        if (USING_USB_SERIAL)
        {
            Serial.print(type);
            Serial.print(F(": "));
            Serial.print(data);
        }
        coreSerialBT.print(type);
        coreSerialBT.print(F(": "));
        coreSerialBT.print(data);
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
            Serial.print(F(": "));
            Serial.print(data);
            Serial.println(F(";"));
        }
        coreSerialBT.print(type);
        coreSerialBT.print(F(": "));
        coreSerialBT.print(data);
        coreSerialBT.println(F(";"));
    }
    else
    {
        if (USING_USB_SERIAL)
            Serial.println(type);
        coreSerialBT.println(type);
    }
}

void Core::println(const String type, float data)
{
    printTimestamp();
    if (data != -25565)
    {
        if (USING_USB_SERIAL)
        {
            Serial.print(type);
            Serial.print(F(": "));
            Serial.print(data);
            Serial.println(F(";"));
        }
        coreSerialBT.print(type);
        coreSerialBT.print(F(": "));
        coreSerialBT.print(data);
        coreSerialBT.println(F(";"));
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
    Serial.print(F("DATA_PACKET_START:"));
    coreSerialBT.print(F("DATA_PACKET_START:"));
}

void Core::printStopDataPacket()
{
    Serial.print(F("DATA_PACKET_STOP;"));
    coreSerialBT.print(F("DATA_PACKET_STOP;"));
}

void Core::lowBat()
{
    println(F("BATTERIA SCARICA"));
    lowbat = true;
    motors.toggleMainMotor(0, STOP);
    navigation.externalStop();
    status.setError(true);
    mux.sensPacketUpdate(false);
}
