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

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
  Serial.println(logmessage);

  if (!index) {
    logmessage = "Upload Start: " + String(filename);
    // open the file on first call and store the file handle in the request object
    request->_tempFile = SPIFFS.open("/" + filename, "w");
    Serial.println(logmessage);
  }

  if (len) {
    // stream the incoming chunk to the opened file
    request->_tempFile.write(data, len);
    logmessage = "Writing file: " + String(filename) + " index=" + String(index) + " len=" + String(len);
    Serial.println(logmessage);
  }

  if (final) {
    logmessage = "Upload Complete: " + String(filename) + ",size: " + String(index + len);
    // close the file handle as the upload is now done
    request->_tempFile.close();
    Serial.println(logmessage);
    request->redirect("/");
  }
}


void web_setup() {

    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
          request->send(200);
        }, handleUpload);

    server.on(
        "/rest/run_script",
        HTTP_POST,
        [](AsyncWebServerRequest * request){},
        NULL,
        [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
        
        static String string;

        if(!index){
            Serial.printf("BodyStart: %u B\n", total);
            script_stop();
            string = "";
        }
        for(size_t i=0; i<len; i++){
          string += (char)data[i];
        }
        if(index + len == total){
          Serial.printf("BodyEnd: %u B\n", total);
          script_run(string.c_str());
          request->send(200);
        }

      });

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