#!/bin/bash

rosrun dynamic_reconfigure dynparam set /realsense_gripper/rgb_camera "{'exposure': 100, 'gain': 80, 'enable_auto_exposure': False}"
rosrun dynamic_reconfigure dynparam set /realsense_gripper/stereo_module "{'visual_preset': 4}"
