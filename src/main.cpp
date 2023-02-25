#include <Arduino.h>
#include <Motors.h>
#include <Sensors.h>
#include <Navigation.h>
#include <Status.h>
#include <Core.h>
#include <SETTINGS.h>
#include <BluetoothSerial.h>
#include <limits.h>
#include <Mux.h>

#if !USE_SD
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "wifi_creds.h"

AsyncWebServer server(80);
WifiCredentials wificreds;
#endif

BluetoothSerial SerialBT;
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
  #if (!USE_SD)
  WiFi.mode(WIFI_STA);
  WiFi.begin(wificreds.ssid, wificreds.password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    // fa lampeggiare i led
    status_led = !status_led;
    digitalWrite(ERROR_LED, status_led);
    digitalWrite(RUNNING_LED, status_led);
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(wificreds.ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hi! I am ESP32."); });

  AsyncElegantOTA.begin(&server);
  server.begin();
  #endif
  /* #endregion */

  core.begin();
}

void loop()
{
  core.loop();
}