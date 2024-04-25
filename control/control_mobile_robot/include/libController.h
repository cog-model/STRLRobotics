#ifndef LIB_CONTROL
#define LIB_CONTROL

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "control_mobile_robot/ControlAction.h"
#include "control_mobile_robot/SimulatecollisionAction.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h> 
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "visualization_msgs/Marker.h"
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/OccupancyGrid.h"
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <random>
#include <stdio.h>
#include <time.h>
#include <fstream>

// node parameters
typedef actionlib::SimpleActionServer<control_mobile_robot::ControlAction> ControlServer;
control_mobile_robot::ControlResult result_msg;
control_mobile_robot::ControlFeedback feedback_msg;
control_mobile_robot::SimulatecollisionActionGoal  statusrobot;

using namespace std;


double t_g_map , t_l_map , t_mpc;
bool StartAction = false;
bool theta_star_is_activated = false;
bool result_diagnostic = true;


// writing file paramter

// declare message parameters
geometry_msgs::Twist        tw_msg;

// declare Publishers and Subscribers
ros::Publisher   cmd_pub;
ros::Publisher   detection_passage_pub;
ros::Subscriber  path_theta_sub;
ros::Subscriber  path_mpc_sub;
ros::Subscriber  odom_sub;
ros::Subscriber  direction_sub;
ros::Subscriber  orient_sub;
ros::Subscriber  grid_map_sub;
ros::Subscriber  grid_local_map_sub;

// declare topics
std::string path_theta_topic;
std::string path_mpc_topic;
std::string globla_local_topic;
std::string direction_topic;
std::string orient_topic;
std::string cmd_topic;
std::string global_frame;
std::string base_frame;
std::string odometry_topic;



// to define subscribers and publishers
void define_sub_pub(ros::NodeHandle n);
// to initialize config paramters
void initialize_parameters(ros::NodeHandle n);
// to get the orientation of yaw for the local frame and the last segment
double get_yaw(geometry_msgs::Quaternion q);
// to set saturation on the linear velocity of the robot
void publish_result(ControlServer* control_server);

void check_fails();

void generate_commands();

#endif
