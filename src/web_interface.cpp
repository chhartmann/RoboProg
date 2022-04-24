#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

#include <web_interface.h>
#include <script_interface.h>
#include <servo_handler.h>

AsyncWebServer server(80);
const char* robo_script_name = "/script.lua";

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
    
        String script = (char*)data;
        String result = lua.Lua_dostring(&script);
        Serial.println("Script finished.");
        request->send(200, "text/plain", result);
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

    server.on("/rest/get_joint_angles", HTTP_GET, [] (AsyncWebServerRequest *request) {
      StaticJsonDocument<JSON_ARRAY_SIZE(num_servos)> doc;
      JsonArray angles = doc.to<JsonArray>();
      for (int i = 0; i < num_servos; ++i) {
        angles.add(servos[i]->read());
      }
      String response;
      serializeJson(doc, response);
//      Serial.println(response);
      request->send(200, "application/json", response);
    });

    AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler("/rest/set_joint_angles", [](AsyncWebServerRequest *request, JsonVariant &json) {
      StaticJsonDocument<200> data;
      if (json.is<JsonArray>())
      {
        data = json.as<JsonArray>();
        for (int i = 0; i < data.size(); ++i) {
          servos[i]->write(data[i]);
        }
      }
      String response;
      serializeJson(data, response);
//      Serial.println(response);
      request->send(200, "application/json", response);
    });
    server.addHandler(handler);


    server.begin();
}
