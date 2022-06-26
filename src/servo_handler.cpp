#include <servo_handler.h>
#include <web_interface.h>

void set_servo_angle(int servo_num, int angle);

// Servo servo1;
// Servo servo2;
// Servo servo3;
// Servo servo4;

// Servo* const servos[num_servos] = {&servo1, &servo2, &servo3, &servo4};
static const int servo_pins[num_servos] = {15, 16, 14, 4};

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
  for (int i = 0; i < num_servos; i++) {
#ifndef USE_ETH_NOT_WIFI
    // this crashes qemu
//    servos[i]->attach(servo_pins[i]);
#endif
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

//  servos[servo_num]->write(angle);
  servo_cur_angle[servo_num] = angle;
//  Serial.println("Servo angle / Joint Angle " + String(servo_num) + " : " + String(angle) + " / " + String(get_joint_angle(servo_num)));
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
