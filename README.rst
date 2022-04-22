Getting Started
***************

A micro platform for controlling a robot arm.
* Web UI
* Programmable via Lua script
* ROS interface

Status
******
* Web UI not yet implemented
* Configuration for wifi settings and ros agent are hard coded
* Programmable via Lua script not yet implemented
* ROS interface needs Agent - not based on embeddedRTPS yet (see https://github.com/micro-ROS/micro_ros_espidf_component)

How to use
**********
* Start micro-ROS agent: docker run -it --rm --net=host microros/micro-ros-agent:galactic udp4 --port 8888 -v6

ROS2 commands
**************
* ros2 node list
* ros2 topic list -t
* ros2 interface show std_msgs/msg/Int32MultiArray
* ros2 topic pub --once /micro_ros_arduino_subscriber std_msgs/msg/Int32MultiArray "{'data': [1, 2]}"


