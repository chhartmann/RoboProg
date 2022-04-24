#ifndef __SERVO_HANDLER_H__
#define __SERVO_HANDLER_H__

#include <ESP32Servo.h>

static const int num_servos = 4;
extern Servo* const servos[4];

void servo_setup();

#endif // __SERVO_HANDLER_H__