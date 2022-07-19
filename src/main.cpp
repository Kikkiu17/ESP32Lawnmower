#include <Arduino.h>
#include <Motors.h>
#include <Sensors.h>
#include <Navigation.h>
#include <Status.h>
#include <Core.h>
#include <SETTINGS.h>
#include <BluetoothSerial.h>
#include <limits.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Mux.h>
#include "wifi_creds.h"
#ifdef ENABLE_WEBSERIAL
#include <WebSerial.h>
#endif

AsyncWebServer server(80);

// IMPORTANTE:
// LA PARTE ANTERIORE DELL'IMU è DOVE CI SONO I DISEGNI (DIREZIONE + CONDENSATORE)
// CTRL+M, CTRL+R PER CREARE UNA REGION

// I PIN 17, 5 SONO TRANSISTOR COLLEGATI AL GND DEL DRIVER MOTORI (PER FARLI GIRARE, USARE PWM O RENDERE HIGH I 2 PIN)
// TODO: SALDARE TRANSISTOR NPN PER PIN 5

BluetoothSerial SerialBT;

WifiCredentials wificreds;

Motors robotmotors;
Sensors robotsensors;
Status robotstatus;
Core core;
NAV robotnav;
Mux robotmux;

uint32_t ref_time = millis();
bool red_led = false;

void setup()
{
  Serial.begin(2000000);
  //WebSerial.begin(&server);
  //WebSerial.msgCallback(recvMsg);

  SerialBT.begin("ESP32Robot");
  core.begin();

  /* #region  WiFi, ElegantOTA */
  WiFi.mode(WIFI_STA);
  WiFi.begin(wificreds.ssid, wificreds.password);
  //WebSerial.println("");
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    SerialBT.print(".");
    Serial.print(".");
    red_led = !red_led;
    digitalWrite(ERROR_LED, red_led);
  }

  robotstatus.setReady(true);

  //WebSerial.println("");
  //WebSerial.print("Connected to ");
  //WebSerial.println(ssid);
  //WebSerial.print("IP address: ");
  //WebSerial.println(WiFi.localIP());

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(wificreds.ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hi! I am ESP32."); });

  AsyncElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();
  //WebSerial.println("HTTP server started");

  //WebSerial.print("Total heap: ");
  //WebSerial.println(ESP.getHeapSize());
  //WebSerial.print("Free heap: ");
  //WebSerial.println(ESP.getFreeHeap());
  //WebSerial.print("Flash chip size: ");
  //WebSerial.println(ESP.getFlashChipSize());
  //WebSerial.print("Flash speed: ");
  //WebSerial.println(ESP.getFlashChipSpeed());
  /* #endregion */

  pinMode(19, OUTPUT); // PIN MOTORE PRINCIPALE
}

int spd = 60;
byte BTData;

void loop()
{
  // controllo robot con bluetooth e porta seriale
  if (SerialBT.available())
  {
    BTData = SerialBT.read();
    if (BTData == 'w')
    {
      core.println((char *)"TYPE_BT_FWD");
      robotnav.goForward();
    }
    else if (BTData == 'a')
    {
      core.println((char *)"TYPE_BT_LEFT");
      robotnav.rotateForDeg(-90);
    }
    else if (BTData == 's')
    {
      core.println((char *)"TYPE_BT_BCK");
      robotnav.goBackwards();
    }
    else if (BTData == 'd')
    {
      core.println((char *)"TYPE_BT_RIGHT");
      robotnav.rotateForDeg(90);
    }
    else if (BTData == 'y')
    {
      // funzione libera
    }
    else if (BTData == 'i')
    {
      // funzione libera
    }
    else if (BTData == 'o')
    {
      // funzione libera
    }
    else if (BTData == 'm')
    {
      // accensione motore principale
      robotmotors.setSpeed(100, MAIN);
    }
    else if (BTData == 'p')
    {
      ledcWrite(CHANNEL_MAIN, 0);
    }
    else if (BTData == 'u')
    {
      robotnav.autoRun();
    }
    else
    {
      core.println((char *)"TYPE_BT_STOP");
      robotnav.externalStop();
      /*SerialBT.print("MAX HEAP: ");
      SerialBT.println(ESP.getMaxAllocHeap());*/
    }
  }

  core.loop();

  /*if (millis() - ref_time > 250)
  {
    ref_time = millis();
    SerialBT.println();
    core.println((char*)"(MAIN) TIME IN MICROSECONDS (us)");
    core.println((char *)"(MAIN) TIME TO EXECUTE LOOP OF MOT", robotmotors.getTime());
    core.println((char *)"(MAIN) TIME TO EXECUTE LOOP OF SENS", robotsensors.getTime());
    core.println((char *)"(MAIN) TIME TO EXECUTE LOOP OF NAV", robotnav.getTime());
    core.println((char *)"(MAIN) STATUS OF HEAP FRAGMENTATION");
    core.println((char *)"(MAIN) MAX HEAP", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    core.println((char *)"(MAIN) LARGEST BLOCK AVAILABLE", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  }*/
}
