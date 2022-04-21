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
#include <std_msgs/msg/int32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("failed in line %i\n", __LINE__);}}

// Global variables

AsyncWebServer server(3232);
LuaWrapper lua;

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int servo1Pin = 15;
int servo2Pin = 16;
int servo3Pin = 14;
int servo4Pin = 4;

int pos = 0;

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
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  pos = msg->data;
  		servo1.write(pos);

  Serial.println("message received");
}

void setup() {
   Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    set_microros_wifi_transports(MY_WIFY_SSID, MY_WIFY_PASS, "192.168.0.162", 8888);
//    WiFi.begin(MY_WIFY_SSID, MY_WIFY_PASS);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
    });

    server.begin();
    setupOta();

  // Setup Servos
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);

   if(!SPIFFS.begin()){
      while(1);
   }

    File file = SPIFFS.open("/main.lua", "r");
    if (!file) {
        Serial.println("Failed to open file for reading");
        while (1);
    }

    String script = file.readString();
    file.close();
//    String script = String("print('Hello World')");
    Serial.println(lua.Lua_dostring(&script));

    // Initialize the ROS node
      allocator = rcl_get_default_allocator();
      RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

      // create node
      RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

      // create subscriber
      RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "micro_ros_arduino_subscriber"));

      // create executor
      RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
      RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));      
}

void loop() {

    ArduinoOTA.handle();
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}