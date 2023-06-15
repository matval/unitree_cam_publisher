#!/usr/bin/env bash

# run the container
docker run --runtime nvidia -it --rm \
    --network host \
    --device /dev/video0 \
    --device /dev/video1 \
	matvalverde/unitree_cam:latest