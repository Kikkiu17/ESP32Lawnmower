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
BluetoothSerial SerialBT;
WifiCredentials wificreds;
Motors robotmotors;
Sensors robotsensors;
Status robotstatus;
Core core;
NAV robotnav;
Mux robotmux;

uint32_t ref_time = millis();
bool status_led = false;
byte BTData;

void setup()
{
  Serial.begin(2000000);
  SerialBT.begin("ESP32Robot");
  core.begin();

  /* #region  WiFi, ElegantOTA */
  WiFi.mode(WIFI_STA);
  WiFi.begin(wificreds.ssid, wificreds.password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    SerialBT.print(".");
    // fa lampeggiare i led
    status_led = !status_led;
    digitalWrite(ERROR_LED, status_led);
    digitalWrite(RUNNING_LED, status_led);
  }

  robotstatus.setReady(true);
  robotmotors.setSpeed(0, MAIN);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hi! I am ESP32."); });

  AsyncElegantOTA.begin(&server);
  server.begin();

  #ifdef ENABLE_WEBSERIAL
    WebSerial.begin(&server);
    WebSerial.msgCallback(recvMsg);
  #endif
  /* #endregion */
}

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
      // inutilizzato
    }
    else if (BTData == 'i')
    {
      // inutilizzato
    }
    else if (BTData == 'o')
    {
      // inutilizzato
    }
    else if (BTData == 'm')
    {
      robotmotors.toggleMainMotor();
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
    }
  }

  core.loop();

  // mostra ogni 250 ms il tempo di esecuzione per ciascun modulo
  // e lo stato dell'heap
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
