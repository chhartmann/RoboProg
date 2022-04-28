#ifndef __SERVO_HANDLER_H__
#define __SERVO_HANDLER_H__

#include <ArduinoJson.h>

static const int num_servos = 4;

void servo_setup(JsonObject const config);
void set_servo_angle(int servo_num, int angle);
int get_servo_angle(int servo_num);

#endif // __SERVO_HANDLER_H__