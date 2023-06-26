#!/usr/bin/env bash

# First, kill all the processes using the cameras
kill $(ps aux | grep '[c]amera' | awk '{print $2}')
kill $(ps aux | grep '[i]mage' | awk '{print $2}')

# Then, run the container
docker run --runtime nvidia --rm \
    --network host \
    --device /dev/video0 \
    --device /dev/video1 \
	matvalverde/unitree_cam:latest \
    ros2 run unitree_cameras camera_node 0 \
    ros2 run unitree_cameras camera_node 1