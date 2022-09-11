#!/bin/bash
docker-up; ocker run -d --name micro_ros_agent --net=host microros/micro-ros-agent:galactic udp4 --port 8888