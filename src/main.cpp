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
#include <config.h>

#include <esp_eth.h>
#include <esp_eth_mac.h>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

// static const char *TAG = "example_connect";
// static esp_eth_handle_t s_eth_handle = NULL;
// static esp_eth_mac_t *s_mac = NULL;
// static esp_eth_phy_t *s_phy = NULL;
// static esp_eth_netif_glue_handle_t s_eth_glue = NULL;

// static esp_netif_t *eth_start(void) {
//   char *desc;
//   esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
//   // Prefix the interface description with the module TAG
//   // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
//   asprintf(&desc, "%s: %s", TAG, esp_netif_config.if_desc);
//   esp_netif_config.if_desc = desc;
//   esp_netif_config.route_prio = 64;
//   esp_netif_config_t netif_config = {
//       .base = &esp_netif_config,
//       .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
//   };
//   esp_netif_t *netif = esp_netif_new(&netif_config);
//   assert(netif);
//   free(desc);

//   eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
//   eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
//   phy_config.phy_addr = 1;

//   // setup openeth for qemu
//   phy_config.autonego_timeout_ms = 100;
//   s_mac = esp_eth_mac_new_openeth(&mac_config);
//   s_phy = esp_eth_phy_new_dp83848(&phy_config);

// }

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

  // WiFi.setHostname(configDoc[wifi_hostname_key]);

  // if (configDoc[wifi_ssid_key] == "") {
  //   WiFi.softAP("RobotProg");
  // } else {
  //   // Wifi is startet in ros_setup()
  //   ros_setup(configDoc);
  // }

  web_setup();
//  setupOta(configDoc[wifi_hostname_key]);
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
      web_send_event("joint_pos", joint_pos_json);
      Serial.println("Web event: " + joint_pos_json);
    }
}