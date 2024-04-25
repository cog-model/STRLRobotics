#!/bin/bash

cd ~/krishtopik/husky_tidy_bot_cv_ws/src/husky_tidy_bot_cv/scripts
source ~/Repos/manipulator/geom_ws/devel/setup.bash
source ~/krishtopik/husky_tidy_bot_cv_ws/devel/setup.bash --extend
python3 object_pose_estimation_node.py $@
