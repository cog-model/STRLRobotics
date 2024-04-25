#!/bin/bash

cd ~/krishtopik/husky_tidy_bot_cv_ws/src/husky_tidy_bot_cv/scripts
source ~/krishtopik/husky_tidy_bot_cv_ws/devel/setup.bash
source ~/krishtopik/cvbridge_build_ws/devel/setup.bash --extend
python3 yolov8_node.py $@
