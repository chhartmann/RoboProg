Getting Started
***************

A micro platform for controlling a robot arm via
* Web UI for jogging
* Programmable via Lua script
* REST interface
* ROS2 interface

Robot Arm
*********

http://www.eezyrobots.it/eba_mk1.html

Status
******

* Angles are not limited to physical limits (neither in UI nor in controller)
* Lua error code is logged in web console / no info when script is finished
* Manual control: press and hold not yet implemented
* Configuration for wifi settings and ros agent are hard coded
* Touchup position and append to position file to be used in Lua not implemented
* ROS interface needs Agent (at startup) - not based on embeddedRTPS yet (see https://github.com/micro-ROS/micro_ros_espidf_component)
* Not tested with robot yet

REST API
*********

* /rest/get_joint_angles / HTTP_GET / application/json
* /rest/set_joint_angles / HTTP_POST / application/json


Lua functions
**************

* setJointAngles(int, int, int, int)
* pinMode(int)
* digitalWrite(int)
* digitalRead(int)
* delay(int)

ROS2 commands
**************

* Start micro-ROS agent: docker run -it --rm --net=host microros/micro-ros-agent:galactic udp4 --port 8888 -v6
* ros2 node list
* ros2 topic list -t
* ros2 interface show std_msgs/msg/Int32MultiArray
* ros2 topic pub --once /micro_ros_arduino_subscriber std_msgs/msg/Int32MultiArray "{'data': [10, 20, 30, 40]}"
