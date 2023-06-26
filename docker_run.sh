#!/usr/bin/env bash

# First, kill all the processes using the cameras
kill $(ps aux | grep '[c]amera' | awk '{print $2}')
kill $(ps aux | grep '[i]mage' | awk '{print $2}')

# Then, run the container
docker run --rm \
    --network host \
    --device /dev/video0 \
    --device /dev/video1 \
	matvalverde/unitree_cam:latest \
    sh -c "ros2 launch unitree_cameras run_cameras.launch"