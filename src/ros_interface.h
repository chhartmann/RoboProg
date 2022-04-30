#ifndef __ROS_INTERFACE_H__
#define __ROS_INTERFACE_H__

#include <config.h>

void ros_setup(ConfigJsonDoc& config);
void ros_loop();

#endif // __ROS_INTERFACE_H__