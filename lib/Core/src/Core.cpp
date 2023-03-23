#include <Arduino.h>
#include <Core.h>
#include <Motors.h>
#include <Sensors.h>
#include <Navigation.h>
#include <Status.h>
#include <SETTINGS.h>
#include <Mux.h>
#include <SerialBLE.h>

SerialBLE serialbt;
Motors motors;
Sensors sensors;
Status status;
NAV navigation;
Mux mux;

uint32_t ref_time = millis();
bool lowbat = false;
uint64_t *ptr;
bool rst_confirmation = false;
bool erase_map_confirmation = false;
bool core_bordermode = false;
bool stop_logging = false;

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
        serialbt.write(type);
        serialbt.write(F(": "));
        serialbt.write(data);
    }
    else
    {
        if (USING_USB_SERIAL)
            Serial.print(type);
        serialbt.write(type);
    }
}

void Core::println(const __FlashStringHelper* type, float data)
{
    // printTimestamp();
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
        serialbt.write(type);
        serialbt.write(F(": "));
        serialbt.write(data);
        serialbt.write(F(";"));
    }
    else
    {
        if (USING_USB_SERIAL)
            Serial.println(type);
        serialbt.write(type);
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
        serialbt.write(type);
        serialbt.write(F(": "));
        serialbt.write(data);
    }
    else
    {
        if (USING_USB_SERIAL)
            Serial.print(type);
        serialbt.write(type);
    }
}

void Core::println(const char *type, float data)
{
    // printTimestamp();
    if (data != -25565)
    {
        if (USING_USB_SERIAL)
        {
            Serial.print(type);
            Serial.print(F(": "));
            Serial.print(data);
            Serial.println(F(";"));
        }
        serialbt.write(type);
        serialbt.write(F(": "));
        serialbt.write(data);
        serialbt.write(F(";"));
    }
    else
    {
        if (USING_USB_SERIAL)
            Serial.println(type);
        serialbt.write(type);
    }
}


void Core::printStartDataPacket()
{
    Serial.print(F("DATA_PACKET_START:"));
    serialbt.write(F("DATA_PACKET_START:"));
}

void Core::printStopDataPacket()
{
    Serial.print(F("DATA_PACKET_STOP;"));
    serialbt.write(F("DATA_PACKET_STOP;"));
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

/*void recvMsg(uint8_t *data, size_t len)
{
    Core funcore;
    WebSerial.print("recv: ");
    for(int i=0; i < len; i++)
        WebSerial.print(data[i]);
    WebSerial.println();
}*/

void Core::begin()
{
    serialbt.begin();
    //WebSerial.msgCallback(recvMsg);
    status.begin();
    println(F("Status OK"));
    sensors.begin();
    println(F("Sensors OK"));
    navigation.begin();
    println(F("Navigation OK"));
    mux.begin();
    println(F("Multiplexer OK"));

    status.setReady(true);
    motors.setSpeed(0, MAIN);
    motors.playStartSound();
    println("Free heap", ESP.getFreeHeap());
    println("Max allocable heap block", ESP.getMaxAllocHeap());
    //ptr = mux.getPacketPointer();
}

void Core::loop()
{
    if (lowbat && ENABLE_BAT_VOLTAGE_SENSING)
        return;
    // controllo input utente
    status.update();
    sensors.update();
    motors.update();
    if (!ENABLE_AUTO_NAVIGATION)
        return;
    navigation.update();

    if (serialbt.available())
    {
        Serial.print("recv: ");
        char data = serialbt.getString()[0];
        Serial.println(data);
        if (data == 'w')
        {
            println(F("BT FORWARD"));
            navigation.goForward();
        }
        else if (data == 'a')
        {
            println(F("BT LEFT"));
            navigation.rotateForDeg(-90);
        }
        else if (data == 's')
        {
            println(F("BT BACK"));
            navigation.goBackwards();
        }
        else if (data == 'd')
        {
            println(F("BT RIGHT"));
            navigation.rotateForDeg(90);
        }
        else if (data == 'i')
        {
            navigation.sdspeedtest(256);
        }
        else if (data == 'r')
        {
            core_bordermode = !core_bordermode;
            navigation.mapBorderMode(core_bordermode);
            println("Bordermode", core_bordermode);
        }
        else if (data == 'o')
        {
            navigation.fillXBytes(72900);
        }
        else if (data == 'c')
        {
        }
        else if (data == 'b')
        {
            uint64_t *packetptr = mux.getPacketPointer();
            mux.readAnalog(BAT);
            delay(50);
            print(F("Batteria: "));
            println(F(" su 2770"));
        }
        else if (data == 'm'|| data == 'M')
        {
            motors.toggleMainMotor();
        }
        else if (data == 'p')
        {
            println(F("ERASE MAP? Y/N"));
            erase_map_confirmation = true;
        }
        else if (data == 'u')
        {
            navigation.autoRun();
        }
        else if (data == 't')
        {
            println(F("RESET? Y/N"));
            rst_confirmation = true;
        }
        else if (data == 'y' || data == 'Y')
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
        else if (data == 'n' || data == 'N')
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
        else if (data == 'l' || data == 'L')
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
        else if (data == 'h' || data == 'H')
        {
            println("");
            println("LISTA COMANDI");
            println("w - a - s - d: movimento robot");
            println("b: mostra stato batteria");
            println("c: inutilizzato");
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

    if (SHOW_MODULE_EXECUTION_TIME)
    {
        if (millis() - ref_time > MODULE_EXECUTION_TIME_REFRESH)
        {
            ref_time = millis();
            serialbt.write(F(""));
            print(F("MOT LOOP"), motors.getTime());
            println(F(" us"));
            print(F("SENS LOOP"), sensors.getTime());
            println(F(" us"));
            print(F("NAV LOOP"), navigation.getTime());
            println(F(" us"));
            if (SHOW_HEAP_INFO)
            {
                println(F("STATUS OF HEAP FRAGMENTATION (in bytes)"));
                println(F("FREE HEAP"), heap_caps_get_free_size(MALLOC_CAP_8BIT));
                println(F("LARGEST BLOCK AVAILABLE"), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
                println(F("HEAP MAX SIZE"), ESP.getHeapSize());
            }
        }
    }
}
