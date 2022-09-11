#!/bin/bash
docker-up
docker container stop micro_ros_agent
docker rm micro_ros_agent
