#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

#include <web_interface.h>
#include <script_interface.h>
#include <servo_handler.h>

AsyncWebServer server(80);
AsyncEventSource events("/events");

const char* robo_script_name = "/script.lua";
const char* config_file_name = "/config.json";

void web_setup() {
    // send html page from file
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/index.html", "text/html");
    });

    server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "OK");
    });

    // REST API
    server.on("/rest/run_script", HTTP_POST, [](AsyncWebServerRequest * request){},
        NULL,
        [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    
        script_run((char*)data);
        request->send(200);
      });

    server.on("/rest/stop_script", HTTP_GET, [](AsyncWebServerRequest *request) {
      script_stop();
      request->send(200);
    });

    server.on("/rest/save_script", HTTP_POST, [](AsyncWebServerRequest * request){},
        NULL,
        [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    
        String script = (char*)data;
        File f = SPIFFS.open(robo_script_name, "w");
        f.print(script);
        f.close();

        request->send(200);
      });

    server.on("/rest/read_script", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, robo_script_name, "text/plain");
    });

    server.on("/rest/read_config", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, config_file_name, "text/plain");
    });

    server.on("/rest/read_schema", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/config-schema.json", "text/plain");
    });

    server.on("/rest/save_config", HTTP_POST, [](AsyncWebServerRequest * request){},
        NULL,
        [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    
        String script = (char*)data;
        File f = SPIFFS.open(config_file_name, "w");
        f.print(script);
        f.close();

        request->send(200);
      });

    server.on("/rest/restart", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200);
      Serial.println("Restarting...");
      ESP.restart();
    });

    server.on("/rest/read_diagnosis", HTTP_GET, [](AsyncWebServerRequest *request) {
      StaticJsonDocument<512> doc;
      doc["CPU Freq MHz"] = ESP.getCpuFreqMHz();
      doc["Chip Model"] = ESP.getChipModel();
      doc["Flash Chip Size"] = ESP.getFlashChipSize();
      doc["Heap Size"] = ESP.getHeapSize();
      doc["Free Heap"] = ESP.getFreeHeap();
      doc["Min Free Heap"] = ESP.getMinFreeHeap();

      // Spiffs
      doc["Spiffs Total Bytes"] = SPIFFS.totalBytes();
      doc["Spiffs Used Bytes"] = SPIFFS.usedBytes();

      // Free Rtos
      // TaskStatus_t pxTaskStatus;
      // vTaskGetInfo(NULL, &pxTaskStatus, pdTRUE, eInvalid);
      // doc["Task Name"] = pxTaskStatus.pcTaskName;
      // doc["Task Stack Size"] = pxTaskStatus.usStackHighWaterMark;

      String response;
      serializeJson(doc, response);
//      Serial.println(response);
      request->send(200, "application/json", response);
    });


    server.on("/rest/get_joint_angles", HTTP_GET, [] (AsyncWebServerRequest *request) {
      request->send(200, "application/json", get_servo_angles_as_json());
    });

    AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler("/rest/set_joint_angles", [](AsyncWebServerRequest *request, JsonVariant &json) {
      StaticJsonDocument<200> data;
      if (json.is<JsonArray>())
      {
        data = json.as<JsonArray>();
        for (int i = 0; i < data.size(); ++i) {
          set_servo_angle(i, data[i]);
        }
      }
      String response;
      serializeJson(data, response);
//      Serial.println(response);
      request->send(200, "application/json", response);
    });
    server.addHandler(handler);

    events.onConnect([](AsyncEventSourceClient *client){
      if(client->lastId()){
        Serial.printf("Client reconnected! Last message ID that it gat is: %u\n", client->lastId());
      }
//      client->send("hello!",NULL,millis(),1000);
    });
    server.addHandler(&events);

    server.begin();
}

void web_send_event(const char* event_name, String& event_data) {
  events.send(event_data.c_str(), event_name);
}

void web_send_event(const char* event_name, const char* event_data) {
  events.send(event_data, event_name);
}