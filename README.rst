Getting Started
***************

A micro platform for controlling a robot arm via
* Web UI for jogging
* Programmable via Lua script
* REST interface
* ROS2 interface

Status
******
* Angles are not limited to physical limits (neither in UI nor in controller)
* Web UI under construction
* Configuration for wifi settings and ros agent are hard coded
* Programmable via Lua script not yet implemented
* Touchup position and append to position file to be used in Lua not implemented
* ROS interface needs Agent (at startup) - not based on embeddedRTPS yet (see https://github.com/micro-ROS/micro_ros_espidf_component)


ROS2 commands
**************
* Start micro-ROS agent: docker run -it --rm --net=host microros/micro-ros-agent:galactic udp4 --port 8888 -v6
* ros2 node list
* ros2 topic list -t
* ros2 interface show std_msgs/msg/Int32MultiArray
* ros2 topic pub --once /micro_ros_arduino_subscriber std_msgs/msg/Int32MultiArray "{'data': [10, 20, 30, 40]}"
