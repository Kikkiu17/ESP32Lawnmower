#include <Arduino.h>
#include <Motors.h>
#include <Sensors.h>
#include <Navigation.h>
#include <Status.h>
#include <Core.h>
#include <SETTINGS.h>
#include <limits.h>
#include <Mux.h>

//#if !USE_SD
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "wifi_creds.h"

AsyncWebServer server(80); 
WifiCredentials wificreds;
//#endif

Motors robotmotors;
Core core;

bool err_led = false;
bool run_led = true;

void setup()
{
  Serial.begin(115200);

  robotmotors.begin();
  robotmotors.playInitSound();

  pinMode(ERROR_LED, OUTPUT);
  pinMode(RUNNING_LED, OUTPUT);

  /* #region  WiFi, ElegantOTA */
  //#if (!USE_SD)
  WiFi.mode(WIFI_STA);
  WiFi.begin(wificreds.ssid, wificreds.password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    // fa lampeggiare i led
    err_led = !err_led;
    run_led = !run_led;
    digitalWrite(ERROR_LED, err_led);
    digitalWrite(RUNNING_LED, run_led);
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(wificreds.ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hi! I am ESP32."); });

  AsyncElegantOTA.begin(&server);
  //WebSerial.begin(&server);
  server.begin();
  //#endif
  /* #endregion */

  core.begin();
}

void loop()
{
  core.loop();
}