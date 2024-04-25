#!/bin/bash

docker exec -it --user docker_user strategic_node bash -c "source /opt/ros/noetic/setup.bash; cd strategic_node; source devel/setup.bash; cd src/behaviour_tree/scripts; python3 main.py"
