#!/bin/bash
source /opt/ros/galactic/local_setup.bash
ros2 topic pub --once /micro_ros_robo_prog_subscriber std_msgs/msg/Int32MultiArray "{'data': $1}"
