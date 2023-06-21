#!/usr/bin/env bash

docker build --no-cache -t matvalverde/unitree_cam:latest .
docker rmi --force $(docker images -f "dangling=true" -q)