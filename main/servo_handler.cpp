#include <esp_log.h>
#include <iot_servo.h> // from https://github.com/espressif/esp-iot-solution.git
#include <servo_handler.h>
#include <driver/gpio.h>
#include <web_interface.h>

void set_servo_angle(int servo_num, int angle);

static const char *TAG = "servo";
static const gpio_num_t servo_pins[num_servos] = {GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_14, GPIO_NUM_4};

static int servo_min_angle[num_servos] = {0, 0, 0, 0};
static int servo_max_angle[num_servos] = {180, 180, 180, 180};
static int servo_cur_angle[num_servos] = {0, 0, 0, 0};
static int joint_min_angle[num_servos] = {0, 0, 0, 0};
static int joint_max_angle[num_servos] = {180, 180, 180, 180};
static int joint_offset[num_servos] = {0, 0, 0, 0};
static int joint_direction[num_servos] = {1, 1, 1, 1};

void set_joint_angle(int joint_id, int angle) {
  if (angle < joint_min_angle[joint_id]) {
    web_send("log", "Joint limit " + std::to_string(joint_min_angle[joint_id]) + " reached for joint " + std::to_string(joint_id));
    angle = joint_min_angle[joint_id];
  } else if (angle > joint_max_angle[joint_id]) {
    web_send("log", "Joint limit " + std::to_string(joint_max_angle[joint_id]) + " reached for joint " + std::to_string(joint_id));
    angle = joint_max_angle[joint_id];
  }

  int servo_pos = angle * joint_direction[joint_id] + joint_offset[joint_id];
  set_servo_angle(joint_id, servo_pos);
}

int get_joint_angle(int joint_id) {
  int joint_pos = (servo_cur_angle[joint_id] - joint_offset[joint_id]) * joint_direction[joint_id];
  return joint_pos;
}


void servo_setup(ConfigJsonDoc& config) {
#ifndef BUILD_FOR_QEMU
  servo_config_t servo_cfg = {
      .max_angle = 180,
      .min_width_us = 500,
      .max_width_us = 2500,
      .freq = 50,
      .timer_number = LEDC_TIMER_0,
      .channels = {
          .servo_pin = {
              servo_pins[0],
              servo_pins[1],
              servo_pins[2],
              servo_pins[3],
          },
          .ch = {
              LEDC_CHANNEL_0,
              LEDC_CHANNEL_1,
              LEDC_CHANNEL_2,
              LEDC_CHANNEL_3,
          },
      },
      .channel_number = 4,
  };
  iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);

#endif

  for (int i = 0; i < num_servos; i++) {
    joint_min_angle[i] = config["limits"][i]["min"];
    joint_max_angle[i] = config["limits"][i]["max"];
    joint_offset[i] = config["offset"][i];
    joint_direction[i] = config["direction"][i];
  }
}

int get_servo_angle(int servo_num) {
  return servo_cur_angle[servo_num];
}

void set_servo_angle(int servo_num, int angle) {
  if (angle < servo_min_angle[servo_num]) {
    web_send("log", "Servo limit " + std::to_string(servo_min_angle[servo_num]) + " reached for joint " + std::to_string(servo_num));
    angle = servo_min_angle[servo_num];
  } else if (angle > servo_max_angle[servo_num]) {
    web_send("log", "Servo limit " + std::to_string(servo_max_angle[servo_num]) + " reached for joint " + std::to_string(servo_num));
    angle = servo_max_angle[servo_num];
  }

  iot_servo_write_angle(LEDC_LOW_SPEED_MODE, servo_num, angle);
  servo_cur_angle[servo_num] = angle;

  ESP_LOGI(TAG, "Servo angle / Joint Angle %i %i / %i", servo_num, angle, get_joint_angle(servo_num));
}

std::string get_joint_angles_as_json() {
    StaticJsonDocument<JSON_ARRAY_SIZE(num_servos)> doc;
    JsonArray angles = doc.to<JsonArray>();
    for (int i = 0; i < num_servos; ++i) {
      angles.add(get_joint_angle(i));
    }
    std::string response;
    serializeJson(doc, response);
    return response;
}
