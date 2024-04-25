# tx2_action_server
Action server for performing exploration in simulated environments.

Is used with [m-explore](https://github.com/cnndepth/m-explore) exploration node and [ThetaStarROS](https://github.com/haiot4105/ThetaStarROS) path planning node.

## Requirements
* ROS version at least Kinetic
* CMake version at least 2.8.3
* Python 2.7
* Python packages:
  * Actionlib
  * Transformations
  
## Usage

rosrun tx2_action_server tx2_action_server_external_driver.py

### Parameters

* `tolerance` (float): distance (in meters) to consider goal as reached. Default value: 1
* `timeout` (float): time (in seconds) to wait for reaching the goal. If this time is exceeded, navigation will be aborted. Default value: 30
* `rate` (float): frequency (in Hz) of checking robot's pose and path planning. Default value: 10
* `max_path_fails` (int): maximum number of path planning fails. If path planning fails more times, navigation will be aborted. Default value: 5

### Input and output

#### Subscribed topics

* odom (`nav_msgs::Odometry`) - robot's position
* path (`nav_msgs::Path`) - path from robot's position to goal
* tf - transform from map frame to odom_frame

#### Used servers
* move_base (`actionlib.SimpleActionServer`)

#### Published topics

* exploration_goal (`geometry_msgs::PoseStamped`) - position of goal
* task (`std_msgs::Float32MultiArray`) - coordinates for path planner
