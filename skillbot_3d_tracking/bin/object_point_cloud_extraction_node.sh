#!/bin/bash

cd ~/krishtopik/husky_tidy_bot_cv_ws/src/husky_tidy_bot_cv/scripts
source ~/Repos/manipulator/geom_ws/devel/setup.bash
source ~/krishtopik/husky_tidy_bot_cv_ws/devel/setup.bash --extend
source ~/krishtopik/cvbridge_build_ws/devel/setup.bash --extend
DEPTH_TO_POINT_CLOUD_ALLOW_PYTHON_IMPLEMENTATION=1 python3 object_point_cloud_extraction_node.py $@
