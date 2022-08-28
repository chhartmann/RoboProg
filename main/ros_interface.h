#ifndef __ROS_INTERFACE_H__
#define __ROS_INTERFACE_H__

#include <string>

void ros_setup(std::string agent_ip);
void ros_loop();

#endif // __ROS_INTERFACE_H__