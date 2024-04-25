#!/bin/bash

docker exec -it --user docker_user strategic_node bash -c "source /opt/ros/noetic/setup.bash; cd strategic_node; source devel/setup.bash; bash"
