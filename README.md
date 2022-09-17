# Getting Started
A micro platform for controlling a robot arm via
* Web UI for jogging
* Programmable via Lua script
* REST interface
* ROS2 interface

# Robot Arm
http://www.eezyrobots.it/eba_mk1.html

# How to use
* Servos controlled at pins 15, 16, 14, 4
* Configuration of direction, home position, offset between joint and servo position in settings tab
* When SSID is not configured, WIFI access point is started (ROS interface is not working in AP mode)

# How to build
[![Open in Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/#https://github.com/chhartmann/RoboProg)

In gitpod compile firmware with "idf.py build".

# Build and test with qemu
* ./qemu_build.sh
* ./qemu_run.sh

# Run tests with qemu
* ./qemu_build.sh
* docker run -it --rm --net=host microros/micro-ros-agent:galactic udp4 --port 8888 -v4
* robot -d tests tests

# Debug with qemu
Start debugging in VS Code (e.g. with F5) starts building for QEMU, runs QEMU and attaches debugger.

# VS Code tasks
Hit ctrl*p and type "task "
* build qemu image
* build and run qemu image, wait for gdb
* backtrace decoder

# TODO
* fix anything which is not working any longer after switching to plain esp idf
* wifi settings hardcoded
* implement over-the-air update
* automated tests with qemu

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
* logSerial(string)
* logWeb(string)

# ROS2 commands
* Start micro-ROS agent: docker run -it --rm --net=host microros/micro-ros-agent:galactic udp4 --port 8888 -v4
* ros2 node list
* ros2 topic list -t
* ros2 interface show std_msgs/msg/Int32MultiArray
* ros2 topic pub --once /micro_ros_robo_prog_subscriber std_msgs/msg/Int32MultiArray "{'data': [10, 20, 30, 40]}"