#include <ArduinoJson.h>
#include <ros_interface.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

#include <servo_handler.h>

#define RCCHECK(fn) { if (!ros_connection_failed) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("ROS failed in line %i with error code %i\n", __LINE__, temp_rc); ros_connection_failed = true;}} }

rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool ros_connection_failed = false;

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;

  for (int i = 0; i < msg->data.size; ++i) {
    set_servo_angle(i, msg->data.data[i]);
  }
}

void ros_setup(JsonObject const config) {
    if (!config.containsKey("ROS-Agent-IP")) {
      Serial.println("ROS-Agent-IP not found in config file");
      return;
    }

    const char* ros_ip = config["ROS-Agent-IP"];
    const char* ssid = config["Wifi-SSID"];
    const char* pwd = config["Wifi-Password"];
    
    set_microros_wifi_transports((char*)ssid, (char*)pwd, (char*)ros_ip, 8888);

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

void ros_loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}