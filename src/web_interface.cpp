#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

#include <web_interface.h>
#include <script_interface.h>
#include <servo_handler.h>

AsyncWebServer server(80);
AsyncEventSource events("/events");

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
  Serial.println(logmessage);
  String file;

  if (!index) {
    logmessage = "Upload Start: " + String(filename);
    Serial.println(logmessage);
    script_stop(); // just be sure - might be running from string buffer
    file = "";
  }

  if (len) {
    logmessage = "Writing file: " + String(filename) + " index=" + String(index) + " len=" + String(len);
    Serial.println(logmessage);
    for (size_t i = 0; i < len; i++) {
      file += (char)data[i];
    }
  }

  if (final) {
    logmessage = "Upload Complete: " + String(filename) + ",size: " + String(index + len);
    Serial.println(logmessage);

    if (filename == "run_script") {
      script_run(file.c_str());
    } else {
      File spiffs = SPIFFS.open("/" + filename, "w");
      spiffs.write((const uint8_t*)file.c_str(), file.length());
      spiffs.close();
    }
//    request->redirect("/");
    request->send(200);
  }
}


void web_setup() {

    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
          request->send(200);
        }, handleUpload);

    server.on("/rest/stop_script", HTTP_GET, [](AsyncWebServerRequest *request) {
      script_stop();
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
      request->send(200, "application/json", get_joint_angles_as_json());
    });

    AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler("/rest/set_joint_angles", [](AsyncWebServerRequest *request, JsonVariant &json) {
      StaticJsonDocument<200> data;
      if (json.is<JsonArray>())
      {
        data = json.as<JsonArray>();
        for (int i = 0; i < data.size(); ++i) {
          set_joint_angle(i, data[i]);
        }
      }
      String response;
      serializeJson(data, response);
      Serial.println("/rest/set_joint_angles: " + data.as<String>());
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