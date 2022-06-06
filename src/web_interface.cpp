//#include <ESPAsyncWebServer.h>
//#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <cstdint>
#include <esp_http_server.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <algorithm>
#include <regex>

#include <web_interface.h>
#include <script_interface.h>
#include <servo_handler.h>

httpd_handle_t server = NULL;
bool ws_connected = false;
int ws_fd;

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

std::string get_content(httpd_req_t *req) {
  char buf[1025];
  int received;

  unsigned int remaining = req->content_len;
  std::string content;
  content.reserve(remaining);

  while (remaining > 0) {
    if ((received = httpd_req_recv(req, buf, std::min(remaining, sizeof(buf) - 1))) <= 0) {
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            /* Retry if timeout occurred */
            continue;
        }

        /* In case of unrecoverable error, close and delete the unfinished file*/
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive file");
    } else {
      buf[received] = '\0';
      content += buf;
      remaining -= received;
    }
  }
  return content;
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
//TODO    Serial.printf("File %s not found\n", filename.c_str());
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

esp_err_t upload_post_handler(httpd_req_t *req)
{
  std::string content = get_content(req);

  std::string uri = req->uri;
  uri = uri.substr(0, uri.find('?'));
  std::string filename = std::regex_replace(uri, std::regex("upload"), "spiffs");

  FILE* file = fopen(filename.c_str(), "w");
  if (!file) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");
        return ESP_FAIL;
  } else {
    (void)fwrite(content.c_str(), 1, content.length(), file);
    fclose(file);
//TODO    Serial.printf("File %s uploaded successfully\n", filename.c_str());
    httpd_resp_set_hdr(req, "Connection", "close");
  httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
}

esp_err_t upload_rest_run_script_handler(httpd_req_t *req)
{
  std::string content = get_content(req);
  script_run(content.c_str());
  httpd_resp_set_hdr(req, "Connection", "close");
  httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t rest_set_joint_angles_handler(httpd_req_t *req)
{
  std::string content = get_content(req);
  StaticJsonDocument<200> doc;
  deserializeJson(doc, content);
  JsonArray angles = doc.as<JsonArray>();
  for (int i = 0; i < angles.size(); ++i) {
    set_joint_angle(i, angles[i]);
  }

  httpd_resp_set_hdr(req, "Connection", "close");
  httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t rest_stop_script_handler(httpd_req_t *req) {
  script_stop();
  httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t rest_reset_handler(httpd_req_t *req) {
  httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
//TODO  Serial.println("Restarting...");
//TODO  ESP.restart();
  return ESP_OK;
}

esp_err_t rest_get_joint_angles_handler(httpd_req_t *req) {
  std::string resp = get_joint_angles_as_json();
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, resp.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t rest_read_diagnosis_handler(httpd_req_t *req) {
  StaticJsonDocument<512> doc;
  // doc["CPU Freq MHz"] = ESP.getCpuFreqMHz();
  // doc["Chip Model"] = ESP.getChipModel();
  // doc["Flash Chip Size"] = ESP.getFlashChipSize();
  // doc["Heap Size"] = ESP.getHeapSize();
  // doc["Free Heap"] = ESP.getFreeHeap();
  // doc["Min Free Heap"] = ESP.getMinFreeHeap();

  // Spiffs
  // doc["Spiffs Total Bytes"] = SPIFFS.totalBytes();
  // doc["Spiffs Used Bytes"] = SPIFFS.usedBytes();

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

  std::string resp;
  serializeJson(doc, resp);
//      Serial.println(response);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, resp.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
//TODO        Serial.println("Websocket connection established");
        ws_fd = httpd_req_to_sockfd(req);
        ws_connected = true;
        return ESP_OK;
    }

//TODO    Serial.println("Websocket unexpected method");
    return ESP_FAIL;
}

void web_setup() {

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.uri_match_fn = httpd_uri_match_wildcard;
  config.max_uri_handlers = 16;
  ESP_ERROR_CHECK(httpd_start(&server, &config));

  httpd_uri_t rest_set_joint_angles = {
      .uri       = "/rest/set_joint_angles",
      .method    = HTTP_POST,
      .handler   = rest_set_joint_angles_handler,
      .user_ctx  = NULL
  };
  httpd_register_uri_handler(server, &rest_set_joint_angles);

  httpd_uri_t rest_get_joint_angles = {
      .uri       = "/rest/get_joint_angles",
      .method    = HTTP_GET,
      .handler   = rest_get_joint_angles_handler,
      .user_ctx  = NULL
  };
  httpd_register_uri_handler(server, &rest_get_joint_angles);

  httpd_uri_t rest_read_diagnosis = {
      .uri       = "/rest/read_diagnosis",
      .method    = HTTP_GET,
      .handler   = rest_read_diagnosis_handler,
      .user_ctx  = NULL
  };
  httpd_register_uri_handler(server, &rest_read_diagnosis);

  httpd_uri_t rest_stop_script = {
      .uri       = "/rest/stop_script",
      .method    = HTTP_GET,
      .handler   = rest_stop_script_handler,
      .user_ctx  = NULL
  };
  httpd_register_uri_handler(server, &rest_stop_script);

  httpd_uri_t rest_run_script = {
    .uri       = "/rest/run_script",
    .method    = HTTP_POST,
    .handler   = upload_rest_run_script_handler,
    .user_ctx  = NULL
  };
  httpd_register_uri_handler(server, &rest_run_script);

  httpd_uri_t rest_reset = {
      .uri       = "/rest/restart",
      .method    = HTTP_GET,
      .handler   = rest_reset_handler,
      .user_ctx  = NULL
  };
  httpd_register_uri_handler(server, &rest_reset);

  httpd_uri_t file_upload = {
      .uri       = "/upload/*",
      .method    = HTTP_POST,
      .handler   = upload_post_handler,
      .user_ctx  = NULL
  };
  httpd_register_uri_handler(server, &file_upload);

  static const httpd_uri_t ws = {
          .uri        = "/ws",
          .method     = HTTP_GET,
          .handler    = ws_handler,
          .user_ctx   = NULL,
          .is_websocket = true
  };
  httpd_register_uri_handler(server, &ws);

  httpd_uri_t file_download = {
      .uri       = "/*",
      .method    = HTTP_GET,
      .handler   = download_get_handler,
      .user_ctx  = NULL
  };
  httpd_register_uri_handler(server, &file_download);
}

static void ws_async_send(void *arg)
{
  std::string *data = (std::string*)arg;
  if (ws_connected) {
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)data->c_str();
    ws_pkt.len = data->length();
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    httpd_ws_send_frame_async(server, ws_fd, &ws_pkt);
  }
  delete data;
}

void web_send(const char* type, std::string data) {
  if (ws_connected) {
    const size_t CAPACITY = JSON_OBJECT_SIZE(2);
    StaticJsonDocument<CAPACITY> doc;
    JsonObject object = doc.to<JsonObject>();
    object["type"] = type;
    object["data"] = data.c_str();
    std::string json;
    serializeJson(doc, json);
    void* arg = new std::string(json);
    (void)httpd_queue_work(server, ws_async_send, arg);
  }
}
