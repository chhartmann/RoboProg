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

