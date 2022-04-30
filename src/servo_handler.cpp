#include <servo_handler.h>
#include <ESP32Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

Servo* const servos[num_servos] = {&servo1, &servo2, &servo3, &servo4};
static const int servo_pins[num_servos] = {15, 16, 14, 4};

static int servo_min_angle[num_servos] = {0, 0, 0, 0};
static int servo_max_angle[num_servos] = {180, 180, 180, 180};

void servo_setup(ConfigJsonDoc& config) {
  for (int i = 0; i < num_servos; i++) {
    servos[i]->attach(servo_pins[i]);
    servo_min_angle[i] = config["limits"][i]["min"];
    servo_max_angle[i] = config["limits"][i]["max"];
  }
}

void set_servo_angle(int servo_num, int angle) {
  if (angle < servo_min_angle[servo_num]) {
    angle = servo_min_angle[servo_num];
  } else if (angle > servo_max_angle[servo_num]) {
    angle = servo_max_angle[servo_num];
  }
  servos[servo_num]->write(angle);
}

int get_servo_angle(int servo_num) {
  return servos[servo_num]->read();
}
