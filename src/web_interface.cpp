//#include <ESPAsyncWebServer.h>
//#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <esp_http_server.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <web_interface.h>
#include <script_interface.h>
#include <servo_handler.h>


static void set_content_type(httpd_req_t *req, const std::string& filename) {
  std::string ending = filename.substr(filename.find('.'));
//  Serial.println(ending.c_str());
  if (ending == ".json") {
    httpd_resp_set_type(req, "application/json");
  } else if (ending == ".txt" || ending == ".lua") {
    httpd_resp_set_type(req, "text/plain");
  } else if (ending == ".html" || ending == ".htm") {
    httpd_resp_set_type(req, "text/html");
  } else if (ending == ".js") {
    httpd_resp_set_type(req, "text/javascript");
  } else if (ending == ".css") {
    httpd_resp_set_type(req, "text/css");
  }
}

esp_err_t download_get_handler(httpd_req_t *req) {
  std::string uri = req->uri;
  uri = uri.substr(0, uri.find('?'));
  if (uri == "/") {
    uri = "/index.html";
  }
  std::string filename = std::string("/spiffs") + uri;
  FILE* file = fopen(filename.c_str(), "r");
  if (!file) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
    Serial.printf("File %s not found\n", filename.c_str());
    return ESP_FAIL;
  } else {

    set_content_type(req, filename);

    char chunk[1024];
    size_t chunksize;
    do {
      chunksize = fread(chunk, 1, sizeof(chunk), file);
      if (chunksize > 0) {
          (void)httpd_resp_send_chunk(req, chunk, chunksize);
      }
    } while (chunksize != 0);

    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
  }
}

esp_err_t rest_stop_script_handler(httpd_req_t *req) {
  script_stop();
  const char resp[] = "OK";
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t rest_reset_handler(httpd_req_t *req) {
  const char resp[] = "OK";
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
  Serial.println("Restarting...");
  ESP.restart();
  return ESP_OK;
}

esp_err_t rest_get_joint_angles_handler(httpd_req_t *req) {
  String resp = get_joint_angles_as_json();
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, resp.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t rest_read_diagnosis_handler(httpd_req_t *req) {
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
  TaskStatus_t *pxTaskStatusArray;
  volatile UBaseType_t uxArraySize, x;
  uint32_t ulTotalRunTime;
   uxArraySize = uxTaskGetNumberOfTasks();
   pxTaskStatusArray = (TaskStatus_t*)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
   if( pxTaskStatusArray != NULL )
   {
      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );
      for( x = 0; x < uxArraySize; x++ ) {
        std::string task_name = "Task " + std::to_string(pxTaskStatusArray[x].xTaskNumber) + " " + pxTaskStatusArray[x].pcTaskName;
        std::string stack_info = "Free Stack " + std::to_string(pxTaskStatusArray[x].usStackHighWaterMark);
        doc[task_name] = stack_info;
      }
      vPortFree( pxTaskStatusArray );
   }

  String resp;
  serializeJson(doc, resp);
//      Serial.println(response);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, resp.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

httpd_uri_t file_download = {
    .uri       = "/*",
    .method    = HTTP_GET,
    .handler   = download_get_handler,
    .user_ctx  = NULL
};

httpd_uri_t rest_stop_script = {
    .uri       = "/rest/stop_script",
    .method    = HTTP_GET,
    .handler   = rest_stop_script_handler,
    .user_ctx  = NULL
};

httpd_uri_t rest_reset = {
    .uri       = "/rest/restart",
    .method    = HTTP_GET,
    .handler   = rest_reset_handler,
    .user_ctx  = NULL
};

httpd_uri_t rest_get_joint_angles = {
    .uri       = "/rest/get_joint_angles",
    .method    = HTTP_GET,
    .handler   = rest_get_joint_angles_handler,
    .user_ctx  = NULL
};

httpd_uri_t rest_read_diagnosis = {
    .uri       = "/rest/read_diagnosis",
    .method    = HTTP_GET,
    .handler   = rest_read_diagnosis_handler,
    .user_ctx  = NULL
};



void web_setup() {

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  httpd_handle_t server = NULL;
  config.uri_match_fn = httpd_uri_match_wildcard;

  ESP_ERROR_CHECK(httpd_start(&server, &config));
  httpd_register_uri_handler(server, &rest_get_joint_angles);
  httpd_register_uri_handler(server, &rest_read_diagnosis);
  httpd_register_uri_handler(server, &rest_stop_script);
  httpd_register_uri_handler(server, &rest_reset);
  httpd_register_uri_handler(server, &file_download);
}

void web_send_event(const char* event_name, String& event_data){}
void web_send_event(const char* event_name, const char* event_data){}


// AsyncWebServer server(80);
// AsyncEventSource events("/events");

// void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
//   String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
//   Serial.println(logmessage);
//   String file;

//   if (!index) {
//     logmessage = "Upload Start: " + String(filename);
//     Serial.println(logmessage);
//     script_stop(); // just be sure - might be running from string buffer
//     file = "";
//   }

//   if (len) {
//     logmessage = "Writing file: " + String(filename) + " index=" + String(index) + " len=" + String(len);
//     Serial.println(logmessage);
//     for (size_t i = 0; i < len; i++) {
//       file += (char)data[i];
//     }
//   }

//   if (final) {
//     logmessage = "Upload Complete: " + String(filename) + ",size: " + String(index + len);
//     Serial.println(logmessage);

//     if (filename == "run_script") {
//       script_run(file.c_str());
//     } else {
//       File spiffs = SPIFFS.open("/" + filename, "w");
//       spiffs.write((const uint8_t*)file.c_str(), file.length());
//       spiffs.close();
//     }
// //    request->redirect("/");
//     request->send(200);
//   }
// }


// void web_setup() {

//     AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler("/rest/set_joint_angles", [](AsyncWebServerRequest *request, JsonVariant &json) {
//       StaticJsonDocument<200> data;
//       if (json.is<JsonArray>())
//       {
//         data = json.as<JsonArray>();
//         for (int i = 0; i < data.size(); ++i) {
//           set_joint_angle(i, data[i]);
//         }
//       }
//       String response;
//       serializeJson(data, response);
//       Serial.println("/rest/set_joint_angles: " + data.as<String>());
// //      Serial.println(response);
//       request->send(200, "application/json", response);
//     });
//     server.addHandler(handler);

//     events.onConnect([](AsyncEventSourceClient *client){
//       if(client->lastId()){
//         Serial.printf("Client reconnected! Last message ID that it gat is: %u\n", client->lastId());
//       }
// //      client->send("hello!",NULL,millis(),1000);
//     });
//     server.addHandler(&events);

//     server.begin();
// }

// void web_send_event(const char* event_name, String& event_data) {
//   events.send(event_data.c_str(), event_name);
// }

// void web_send_event(const char* event_name, const char* event_data) {
//   events.send(event_data, event_name);
// }