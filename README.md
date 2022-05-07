# Getting Started
A micro platform for controlling a robot arm via
* Web UI for jogging
* Programmable via Lua script
* REST interface
* ROS2 interface

# Robot Arm
http://www.eezyrobots.it/eba_mk1.html

# Status
* Manual control: press and hold not yet implemented
* ROS interface needs Agent (at startup) - not based on embeddedRTPS yet (see https://github.com/micro-ROS/micro_ros_espidf_component)
* Not tested with robot yet

# How to use
* Servos controlled at pins 15, 16, 14, 4
* When SSID is not configured, WIFI access point is started (ROS interface is not working in AP mode)

# How to build
[![Open in Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/#https://github.com/chhartmann/RoboProg)

In gitpod compile firmware with "platformio run -e esp32dev" and build spiffs.bin with "platformio run --target buildfs --environment esp32dev". You can download the .bin files in the IDE under .pio/esp32dev and flash them with ESP32 flash tool (https://www.espressif.com/en/support/download/other-tools).

or 

Use vscode with platform.io plugin and use the defined platform.io project tasks to build and flash everything you need.

# REST API
* /rest/get_joint_angles / HTTP_GET / application/json
* /rest/set_joint_angles / HTTP_POST / application/json
* curl -X POST http://esp-robotprog/rest/set_joint_angles -H 'Content-Type: application/json' -d '[10,20,30,40]'

# Lua functions
* setJointAngles(int, int, int, int)
* getConfig() -- returns a table with the fields of the settings to access e.g. limits and home position
* pinMode(int)
* digitalWrite(int)
* digitalRead(int)
* delay(int)
* logSerial(char*)
* logWeb(char*)

# ROS2 commands
* Start micro-ROS agent: docker run -it --rm --net=host microros/micro-ros-agent:galactic udp4 --port 8888 -v6
* ros2 node list
* ros2 topic list -t
* ros2 interface show std_msgs/msg/Int32MultiArray
* ros2 topic pub --once /micro_ros_arduino_subscriber std_msgs/msg/Int32MultiArray "{'data': [10, 20, 30, 40]}"
