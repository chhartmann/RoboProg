#ifndef __SERVO_HANDLER_H__
#define __SERVO_HANDLER_H__

#include <config.h>

static const int num_servos = 4;

void servo_setup(ConfigJsonDoc& config);
void set_joint_angle(int joint_id, int angle);
int get_joint_angle(int joint_id);
String get_joint_angles_as_json();

#endif // __SERVO_HANDLER_H__