#include "SerialBLE.h"

// preso da https://gist.github.com/czechdude/a04fb89a49fdb67dae5eab5af8777e17

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;
std::string rxValue;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    char txString[8]; // make sure this is big enuffz
    dtostrf(txValue, 1, 2, txString); // float_val, min_width, digits_after_decimal, char_buffer
    
      pCharacteristic->setValue(txString);
  }
    void onWrite(BLECharacteristic *pCharacteristic) {
      rxValue = pCharacteristic->getValue();
      }
};

std::string SerialBLE::getString()
{
    std::string to_return = rxValue;
    rxValue.erase();
    return to_return;
}

bool SerialBLE::available()
{
    if (rxValue.length() > 0)
        return true;
    else return false;
}

void SerialBLE::write(std::string data)
{
    pCharacteristic->setValue(data);
    pCharacteristic->notify();
}

void SerialBLE::write(float data)
{
    char str[15];
    snprintf(str, sizeof(str), "%f", data);
    pCharacteristic->setValue(str);
    pCharacteristic->notify();
}

void SerialBLE::write(int32_t data)
{
    char str[15];
    snprintf(str, sizeof(str), "%f", data);
    pCharacteristic->setValue(str);
    pCharacteristic->notify();
}

void SerialBLE::write(char* data)
{
    pCharacteristic->setValue(data);
    pCharacteristic->notify();
}

void SerialBLE::write(const __FlashStringHelper* data)
{
    pCharacteristic->setValue(reinterpret_cast<const char *>(data));
    pCharacteristic->notify();
}

void SerialBLE::begin()
{
    BLEDevice::init("ESP32S3 Robot"); // Give it a name

    // Create the BLE Server
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                    );
                        
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCallbacks());

    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                            CHARACTERISTIC_UUID_RX,
                                            BLECharacteristic::PROPERTY_WRITE
                                        );

    pCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();
    // Start advertising
    pServer->getAdvertising()->start();
}
