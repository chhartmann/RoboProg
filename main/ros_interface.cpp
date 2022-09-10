#include <ros_interface.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdint.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <servo_handler.h>

#define RCCHECK(fn) { if (!ros_connection_failed) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGI("myros", "ROS failed in line %i with error code %i\n", __LINE__, temp_rc); ros_connection_failed = true;}} }

std_msgs__msg__Int32MultiArray msg;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rcl_node_t node;
rcl_timer_t timer;
bool ros_connection_failed = false;


void subscription_callback(const void * msgin)
{
  ESP_LOGI("tag", "received ros message");
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;

  for (int i = 0; i < msg->data.size; ++i) {
    set_joint_angle(i, msg->data.data[i]);
  }
}

void micro_ros_task(void* arg) {
  while (true) {
//    ESP_LOGI("tag", "ros task");
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    usleep(100000);
  }
}

void ros_setup(std::string agent_ip) {
  rclc_support_t support;
  rcl_allocator_t allocator;

  // Initialize the ROS node
  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));

  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  RCCHECK(rmw_uros_options_set_udp_address(agent_ip.c_str(), "8888", rmw_options));
  //RCCHECK(rmw_uros_discover_agent(rmw_options));

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator))

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_robo_prog_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
  &subscriber,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
  "micro_ros_robo_prog_subscriber"));

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

  xTaskCreate(micro_ros_task, "uros_task", 16000, NULL, 5, NULL);
}
