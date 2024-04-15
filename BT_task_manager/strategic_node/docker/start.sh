#!/bin/bash

CODE=${1:-`pwd`/..}

xhost +local:root

docker run -itd --rm \
           --ipc host \
           --network host \
           --gpus all \
           -e "NVIDIA_DRIVER_CAPABILITIES=all" \
           -e "DISPLAY" \
           -e "QT_X11_NO_MITSHM=1" \
           -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
           -v $CODE:/home/docker_user/strategic_node:rw \
           --name strategic_node \
           x64_noetic/strategic_node

xhost -local:root
