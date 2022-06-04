#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <WString.h>
#include <ArduinoOTA.h>

#include <ArduinoJson.h>

#include <ros_interface.h>
#include <web_interface.h>
#include <script_interface.h>
#include <servo_handler.h>
#include <qemu_eth.h>
#include <config.h>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

void setupOta(const char* hostname) {
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

  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.begin();
}


void setup() {
   Serial.begin(115200);

   if(!SPIFFS.begin()){
      while(1);
   }

  // load config from file
  ConfigJsonDoc configDoc;
  File configFile = SPIFFS.open("/config.json", "r");
  DeserializationError jsonError = deserializeJson(configDoc, configFile);
  configFile.close();
  if (jsonError) {
    Serial.printf("deserializeJson() failed (%s) for config file\n", jsonError.c_str());
    serializeJsonPretty(configDoc, Serial);
    while(1);
  }

#if defined(MY_WIFY_SSID) && defined(MY_WIFY_PASS)
    if (configDoc[wifi_ssid_key] == "") {
      configDoc[wifi_ssid_key] = MY_WIFY_SSID;
      configDoc[wifi_pwd_key] = MY_WIFY_PASS;
    }
#endif

#ifdef USE_ETH_NOT_WIFI
  Serial.println("Starting ethernet...");
  eth_start();
#else
  WiFi.setHostname(configDoc[wifi_hostname_key]);

  if (configDoc[wifi_ssid_key] == "") {
    WiFi.softAP("RobotProg");
  } else {
    // Wifi is startet in ros_setup()
    ros_setup(configDoc); // TODO use ros also with eth
  }

 setupOta(configDoc[wifi_hostname_key]);
#endif

  web_setup();
  servo_setup(configDoc);
  script_setup();

  if (configDoc[autostart_script_key] == true) {
    File scriptFile = SPIFFS.open("/script.lua", "r");
    String script = scriptFile.readString();
    scriptFile.close();
    script_run(script.c_str());
  }
}

void loop() {
    static int prev_joint_pos[num_servos] = {0};

    ArduinoOTA.handle();
    ros_loop();

    // if a servo has changed position, send it to the frontend
    bool publish_joint_pos = false;
    for (int i = 0; i < num_servos; i++) {
      int angle = get_joint_angle(i);
      if (angle != prev_joint_pos[i]) {
        prev_joint_pos[i] = angle;
        publish_joint_pos = true;
      }
    }
    if (publish_joint_pos) {
      String joint_pos_json = get_joint_angles_as_json();
      web_send("pos", joint_pos_json);
      Serial.println("Web socket: " + joint_pos_json);
    }

    delay(10); // necessary for watchdog reset
}