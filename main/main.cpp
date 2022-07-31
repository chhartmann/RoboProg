#include <string>
#include <fstream>
#include <streambuf>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_partition.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <ArduinoJson.h>

#include <ros_interface.h>
#include <web_interface.h>
#include <script_interface.h>
#include <servo_handler.h>
#include <qemu_eth.h>
#include <wifi.h>
#include <config.h>

const char* TAG="rpg";
void setup();
void loop();

extern "C" {
void app_main() {
  setup();
  loop();
}
}

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

// void setupOta(const char* hostname) {
//   ArduinoOTA
//     .onStart([]() {
//       String type;
//       if (ArduinoOTA.getCommand() == U_FLASH)
//         type = "sketch";
//       else // U_SPIFFS
//         type = "filesystem";
//       SPIFFS.end();
//       Serial.println("Start updating " + type);
//     })
//     .onEnd([]() {
//       Serial.println("\nEnd");
//     })
//     .onProgress([](unsigned int progress, unsigned int total) {
//       Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
//     })
//     .onError([](ota_error_t error) {
//       Serial.printf("Error[%u]: ", error);
//       if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
//       else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
//       else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
//       else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
//       else if (error == OTA_END_ERROR) Serial.println("End Failed");
//     });

//   ArduinoOTA.setHostname(hostname);
//   ArduinoOTA.begin();
// }

void setup_spiffs() {
  ESP_LOGI(TAG, "Initializing SPIFFS");

  esp_partition_type_t type = ESP_PARTITION_TYPE_DATA;
  esp_partition_subtype_t subtype = ESP_PARTITION_SUBTYPE_DATA_SPIFFS;
  const char* name ="storage";
  const esp_partition_t * part  = esp_partition_find_first(type, subtype, name);

  if (part != NULL) {
      ESP_LOGI(TAG, "found partition '%s' at offset 0x%x with size 0x%x", part->label, part->address, part->size);
  } else {
      ESP_LOGE(TAG, "partition not found!");
  }

  ESP_LOGI(TAG, "Formating data partition...");
  esp_err_t format_ret = esp_spiffs_format("storage");
  ESP_LOGI(TAG, "%s", format_ret == ESP_OK ? "success" : "failed");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = "storage",
      .max_files = 5,
      .format_if_mount_failed = false
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
}

void setup() {
  //  Serial.begin(115200);

  setup_spiffs();

  // load config from file
  ConfigJsonDoc configDoc;
  std::ifstream t("/spiffs/config.json");
  std::string configFile((std::istreambuf_iterator<char>(t)),
                  std::istreambuf_iterator<char>());
  DeserializationError jsonError = deserializeJson(configDoc, configFile);

//  ESP_LOGI(TAG, "Config read:\n %s", configFile.c_str());

  if (jsonError) {
    ESP_LOGE(TAG, "deserializeJson() failed (%s) for config file\n", jsonError.c_str());
    while(1);
  }

#if defined(MY_WIFY_SSID) && defined(MY_WIFY_PASS)
    if (configDoc[wifi_ssid_key] == "") {
      configDoc[wifi_ssid_key] = MY_WIFY_SSID;
      configDoc[wifi_pwd_key] = MY_WIFY_PASS;
    }
#endif

#ifdef USE_ETH_NOT_WIFI

  esp_log_level_set("esp_eth*", ESP_LOG_VERBOSE);


  ESP_LOGI(TAG, "Starting ethernet");
  eth_start();
#else
   if (configDoc[wifi_ssid_key] == "") {
//TODO     WiFi.softAP("RobotProg");
   } else {
      start_wifi(configDoc[wifi_ssid_key], configDoc[wifi_pwd_key]);
   }

//     // Wifi is startet in ros_setup()
//     ros_setup(configDoc); // TODO use ros also with eth
//   }

//  setupOta(configDoc[wifi_hostname_key]);
#endif

  ESP_LOGI(TAG, "Starting http server");
  web_setup();

  servo_setup(configDoc);
  script_setup();

  ESP_LOGI(TAG, "Setup finished");

  if (configDoc[autostart_script_key] == true) {
    ESP_LOGI(TAG, "Autostart lua script");
    std::ifstream t("/spiffs/script.lua");
    std::string luaScript((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    script_run(luaScript.c_str());
  }

  while(1) loop();
}

void loop() {
    static int prev_joint_pos[num_servos] = {0};

    // ArduinoOTA.handle();
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
      std::string joint_pos_json = get_joint_angles_as_json();
      web_send("pos", joint_pos_json);
      // Serial.println("Web event: " + joint_pos_json);
    }

    vTaskDelay( 10 / portTICK_PERIOD_MS); // necessary for watchdog reset
}