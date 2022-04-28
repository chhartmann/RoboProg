#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

#include <ros_interface.h>
#include <web_interface.h>
#include <script_interface.h>
#include <servo_handler.h>

void setupOta() {
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      SPIFFS.end();
      Serial.println("Start updating " + type);      
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}


void setup() {
   Serial.begin(115200);

   if(!SPIFFS.begin()){
      while(1);
   }

  WiFi.mode(WIFI_STA);

  // load config from file
  DynamicJsonDocument configDoc(256);
  File configFile = SPIFFS.open("/config.json", "r");
  DeserializationError jsonError = deserializeJson(configDoc, configFile);
  configFile.close();
  if (jsonError) {
    Serial.println("deserializeJson() failed for config file");
  }
  JsonObject configObj = configDoc.to<JsonObject>();


#if defined(MY_WIFY_SSID) && defined(MY_WIFY_PASS)
    if (configObj["Wifi-SSID"] == "") {
      configObj["Wifi-SSID"] = MY_WIFY_SSID;
      configObj["Wifi-Password"] = MY_WIFY_PASS;
    }
#endif

  if (configObj["Wifi-SSID"] == "") {
    WiFi.softAP("RobotProg");
  } else {
    // WiFi.begin(MY_WIFY_SSID, MY_WIFY_PASS) is implicitly done by ros_setup()
    ros_setup(configObj);
  }

  web_setup();
  setupOta();
  servo_setup(configObj);
  script_setup();
}

void loop() {

    ArduinoOTA.handle();
    ros_loop();
}