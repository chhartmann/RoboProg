#include <servo_handler.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

Servo* const servos[num_servos] = {&servo1, &servo2, &servo3, &servo4};
static const int servo_pins[num_servos] = {15, 16, 14, 4};

void servo_setup() {
  for (int i = 0; i < num_servos; i++) {
    servos[i]->attach(servo_pins[i]);
  }
}