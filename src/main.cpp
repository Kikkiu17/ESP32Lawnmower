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

bool status_led = false;

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32ROBOT");

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

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hi! I am ESP32."); });

  AsyncElegantOTA.begin(&server);
  server.begin();

  #ifdef ENABLE_WEBSERIAL
    WebSerial.begin(&server);
    WebSerial.msgCallback(recvMsg);
  #endif
  /* #endregion */

  core.begin();
}

void loop()
{
  core.loop();
}
