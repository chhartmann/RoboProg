#ifndef __SERVO_HANDLER_H__
#define __SERVO_HANDLER_H__

#include <config.h>

static const int num_servos = 4;

void servo_setup(ConfigJsonDoc& config);
void set_servo_angle(int servo_num, int angle);
int get_servo_angle(int servo_num);
String get_servo_angles_as_json();

#endif // __SERVO_HANDLER_H__