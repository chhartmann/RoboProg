#include <Arduino.h>
#include <WiFi.h>
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#if not defined(MY_WIFY_SSID) || not defined(MY_WIFY_PASS)
#error "MY_WIFIY_SSID and MY_WIFIY_PASS must be defined in wifi_secrets.h"
#endif
#else
#error "wifi_secrets.h has to be provided with your WiFi credentials"
#endif
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <LuaWrapper.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>

// ROS
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

#define RCCHECK(fn) { if (!ros_connection_failed) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("ROS failed in line %i with error code %i\n", __LINE__, temp_rc); ros_connection_failed = true;}} }

// Global variables

AsyncWebServer server(80);
LuaWrapper lua;
const char* robo_script_name = "/script.lua";

rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool ros_connection_failed = true; // TODO: set to false to activate ROS

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

static Servo* const servos[] = {&servo1, &servo2, &servo3, &servo4};
static const int num_servos = sizeof(servos) / sizeof(Servo*);
static const int servo_pins[] = {15, 16, 14, 4};

void setupOta() {
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

  ArduinoOTA.begin();
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;

  for (int i = 0; i < msg->data.size; ++i) {
    servos[i]->write(msg->data.data[i]);
  }
}

static int lua_set_joint_angles(lua_State *lua_state) {
  for (int i = 0; i < num_servos; ++i) {
    servos[i]->write(lua_tointeger(lua_state, i + 1));
  }
  return 0;
}

static int lua_wrapper_pinMode(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  int b = luaL_checkinteger(lua_state, 2);
  pinMode(a, b);
  return 0;
}

static int lua_wrapper_digitalWrite(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  int b = luaL_checkinteger(lua_state, 2);
  digitalWrite(a, b);
  return 0;
}

static int lua_wrapper_digitalRead(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  int val = digitalRead(a);
  lua_pushnumber(lua_state, val);
  return 0;
}

static int lua_wrapper_delay(lua_State *lua_state) {
  int a = luaL_checkinteger(lua_state, 1);
  delay(a);
  return 0;
}


void setup() {
   Serial.begin(115200);

    WiFi.mode(WIFI_STA);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
    set_microros_wifi_transports(MY_WIFY_SSID, MY_WIFY_PASS, "192.168.0.162", 8888);
#pragma GCC diagnostic pop
    
//    WiFi.begin(MY_WIFY_SSID, MY_WIFY_PASS);

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
    setupOta();

  // Setup Servos
  for (int i = 0; i < num_servos; i++) {
    servos[i]->attach(servo_pins[i]);
  }

   if(!SPIFFS.begin()){
      while(1);
   }

  lua.Lua_register("setJointAngles", (const lua_CFunction) &lua_set_joint_angles);
  lua.Lua_register("pinMode", (const lua_CFunction) &lua_wrapper_pinMode);
  lua.Lua_register("digitalWrite", (const lua_CFunction) &lua_wrapper_digitalWrite);
  lua.Lua_register("digitalRead", (const lua_CFunction) &lua_wrapper_digitalRead);
  lua.Lua_register("delay", (const lua_CFunction) &lua_wrapper_delay);

    // File file = SPIFFS.open("/main.lua", "r");
    // if (!file) {
    //     Serial.println("Failed to open file for reading");
    // }

    // String script = file.readString();
    // file.close();
//    String script = String("print('Hello World')");
//    Serial.println(lua.Lua_dostring(&script));

    // Initialize the ROS node
      allocator = rcl_get_default_allocator();
      RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

      // create node
      RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

      // create subscriber
      RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "micro_ros_arduino_subscriber"));

      // Initialize message
      static std_msgs__msg__MultiArrayDimension dim;
      static char* dim_name = (char*)"joint_angle";
      dim.label = {dim_name, strlen(dim_name), strlen(dim_name)};
      dim.size = 1;
      dim.stride = 0;
      msg.layout.dim.data = &dim;
      msg.layout.data_offset = 0;
      msg.data.data = (int32_t*)malloc(sizeof(int32_t)*num_servos);
      msg.data.size = 0;
      msg.data.capacity = num_servos;

      // create executor
      RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
      RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));      
}

void loop() {

    ArduinoOTA.handle();
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}