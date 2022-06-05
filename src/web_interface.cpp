//#include <ESPAsyncWebServer.h>
//#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <cstdint>
#include <esp_https_server.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <algorithm>
#include <regex>

#include <ws_keep_alive.h>
#include <web_interface.h>
#include <script_interface.h>
#include <servo_handler.h>

struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
    std::string msg;
};

httpd_handle_t server = NULL;
static const size_t max_clients = 4;
static const char *TAG = "wss";

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
    Serial.printf("File %s uploaded successfully\n", filename.c_str());
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

static esp_err_t ws_handler(httpd_req_t *req)
{
  if (req->method == HTTP_GET) {
      ESP_LOGI(TAG, "Handshake done, the new connection was opened");
      return ESP_OK;
  }
  httpd_ws_frame_t ws_pkt;
  uint8_t *buf = NULL;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

  // First receive the full ws message
  /* Set max_len = 0 to get the frame len */
  esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
      return ret;
  }
  ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
  if (ws_pkt.len) {
      /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
      buf = (uint8_t*)calloc(1, ws_pkt.len + 1);
      if (buf == NULL) {
          ESP_LOGE(TAG, "Failed to calloc memory for buf");
          return ESP_ERR_NO_MEM;
      }
      ws_pkt.payload = buf;
      /* Set max_len = ws_pkt.len to get the frame payload */
      ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
      if (ret != ESP_OK) {
          ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
          free(buf);
          return ret;
      }
  }
  // If it was a PONG, update the keep-alive
  if (ws_pkt.type == HTTPD_WS_TYPE_PONG) {
      ESP_LOGD(TAG, "Received PONG message");
      free(buf);
      return wss_keep_alive_client_is_active((wss_keep_alive_storage*)httpd_get_global_user_ctx(req->handle),
              httpd_req_to_sockfd(req));
  }
  free(buf);
  return ESP_OK;
}

static void send_ping(void *arg)
{
    async_resp_arg *resp_arg = (struct async_resp_arg*)arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = NULL;
    ws_pkt.len = 0;
    ws_pkt.type = HTTPD_WS_TYPE_PING;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    delete resp_arg;
}

bool check_client_alive_cb(wss_keep_alive_t h, int fd)
{
    ESP_LOGD(TAG, "Checking if client (fd=%d) is alive", fd);
    async_resp_arg *resp_arg = new async_resp_arg();
    resp_arg->hd = wss_keep_alive_get_user_ctx(h);
    resp_arg->fd = fd;

    if (httpd_queue_work(resp_arg->hd, send_ping, resp_arg) == ESP_OK) {
        return true;
    }
    return false;
}

bool client_not_alive_cb(wss_keep_alive_t h, int fd)
{
    ESP_LOGE(TAG, "Client not alive, closing fd %d", fd);
    httpd_sess_trigger_close(wss_keep_alive_get_user_ctx(h), fd);
    return true;
}

esp_err_t wss_open_fd(httpd_handle_t hd, int sockfd)
{
    ESP_LOGI(TAG, "New client connected %d", sockfd);
    wss_keep_alive_t h = (wss_keep_alive_t)httpd_get_global_user_ctx(hd);
    return wss_keep_alive_add_client(h, sockfd);
}

void wss_close_fd(httpd_handle_t hd, int sockfd)
{
    ESP_LOGI(TAG, "Client disconnected %d", sockfd);
    wss_keep_alive_t h = (wss_keep_alive_t)httpd_get_global_user_ctx(hd);
    wss_keep_alive_remove_client(h, sockfd);
}

void web_setup() {
  wss_keep_alive_config_t keep_alive_config = KEEP_ALIVE_CONFIG_DEFAULT();
  keep_alive_config.max_clients = max_clients;
  keep_alive_config.client_not_alive_cb = client_not_alive_cb;
  keep_alive_config.check_client_alive_cb = check_client_alive_cb;
  wss_keep_alive_t keep_alive = wss_keep_alive_start(&keep_alive_config);

  httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();

  extern const unsigned char servercert_start[] asm("_binary_servercert_pem_start");
  extern const unsigned char servercert_end[]   asm("_binary_servercert_pem_end");
  conf.cacert_pem = servercert_start;
  conf.cacert_len = servercert_end - servercert_start;

  extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
  extern const unsigned char prvtkey_pem_end[]   asm("_binary_prvtkey_pem_end");
  conf.prvtkey_pem = prvtkey_pem_start;
  conf.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

  conf.httpd.uri_match_fn = httpd_uri_match_wildcard;
  conf.httpd.max_uri_handlers = 16;
  conf.httpd.max_open_sockets = max_clients;
  conf.httpd.global_user_ctx = keep_alive;
  conf.httpd.open_fn = wss_open_fd;
  conf.httpd.close_fn = wss_close_fd;

  ESP_ERROR_CHECK(httpd_ssl_start(&server, &conf));

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
  static char* buffer = "hello world";
  async_resp_arg *resp_arg = (async_resp_arg*)arg;
  httpd_handle_t hd = resp_arg->hd;
  int fd = resp_arg->fd;
  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.payload = (uint8_t*)buffer; //(uint8_t*)resp_arg->msg.c_str();
  ws_pkt.len = 5; //resp_arg->msg.length();
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;

  httpd_ws_send_frame_async(hd, fd, &ws_pkt);
  free(resp_arg);
}

void web_send(const char* type, String data) {
  bool send_messages = true;

  size_t clients = max_clients;
  int    client_fds[max_clients];
  if (httpd_get_client_list(server, &clients, client_fds) == ESP_OK) {
      for (size_t i=0; i < clients; ++i) {
          int sock = client_fds[i];
          if (httpd_ws_get_fd_info(server, sock) == HTTPD_WS_CLIENT_WEBSOCKET) {
              ESP_LOGI(TAG, "Active client (fd=%d) -> sending async message", sock);
              async_resp_arg *resp_arg = new async_resp_arg();
              resp_arg->hd = server;
              resp_arg->fd = sock;
              resp_arg->msg = data.c_str();
              if (httpd_queue_work(resp_arg->hd, ws_async_send, resp_arg) != ESP_OK) {
                  ESP_LOGE(TAG, "httpd_queue_work failed!");
                  send_messages = false;
                  break;
              }
          }
      }
  } else {
      ESP_LOGE(TAG, "httpd_get_client_list failed!");
      return;
  }
}
