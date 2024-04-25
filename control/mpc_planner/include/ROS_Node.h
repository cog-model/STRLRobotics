#ifndef ROS_NODE
#define ROS_NODE
#include "lib.h"

// publishers and subscribers
ros::Publisher      pub_marker_init_gues,
                    opt_traj_pub ,
                    pub_marker_opt_traj,
                    pub_marker_filter_path,
                    pub_marker_obst_cells;

ros::Subscriber     global_path_sub,
                    odom_sub,
                    gloab_local_sub,
                    status_control_sub,
                    grid_map_sub,
                    grid_local_map_sub;

// ROS messages
visualization_msgs::Marker      opt_traj_mk , init_gues_mk , filterd_path_mk , obst_cells_mk;

//visualization_msgs::MarkerArray markers;

geometry_msgs::Point            opt_traj_pt , init_gues_pt , filtere_path_pt , obst_cells_pt;

nav_msgs::Path                  path_msg;
geometry_msgs:: PoseStamped     path_msg_poses;
geometry_msgs::Pose             path_msg_pose;


OptTraj opt_traj;
X0 x0;

int rate_loop;
std::string global_frame;
std::string local_frame;
std::string globla_local_topic;
std::string mpc_path_topic;
std::string global_path_topic;
std::string grid_map_topic;
std::string grid_local_map_topic;
std::string control_status_topic;
std::string odom_husy_topic;

bool robot_reached = false , flag_global_local = false;

double get_yaw(geometry_msgs::Quaternion q);
void initialize_markers();
void publish_local_path(OptTraj traj);
void define_pub_sub(ros::NodeHandle n);
void initialize_parameters(ros::NodeHandle n);
void setting_mpc_conditions();

#endif
