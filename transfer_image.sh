#!/usr/bin/env bash

docker save matvalverde/unitree_cam:latest | bzip2 | ssh unitree@192.168.123.14 docker load