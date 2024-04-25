#ifndef ControllerClass
#define ControllerClass

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
class Controller
{
public:
    // controller class parameters
    // controller parameters
    double dist_error = 0;
    double angle_error = 0;
    double dist_limit = 0.35;
    double dist_stop = 0;
    double max_v_local = 0;
    int k_v = 0;
    // velocities paramters
    double v_cmd = 0;
    double w_cmd = 0;
    double v_robot = 0;

    // odometry parametrs
    double odom_x;
    double odom_y;
    double odom_theta;
    double final_orientation = 0;
    // boolean paramters
    bool final_theta_corrected = false;
    bool path_stored = false;
    bool path_changed = false;
    bool robot_reached = false;
    bool mpc_is_activated = false;
    int direction = 0;
    double compensate_angle = 0;

    // rate time parameter
    double dt = 0.1;
    geometry_msgs::Pose2D       qp[100] , err;
    // declare config parameters
    double max_v;
    double min_v;
    double acc_v;
    double max_w;
    bool  flag_direction = true , flag_orientation = false;


    
    bool robot_in_tolerance = false;
    
    bool robot_in_passage = true;

    // path parameters
    uint32_t size_path = 0;

    Controller();
    // declare functions
    // to check stop condition 
    // will stop the robot when it is close to the goal
    void stop_mode();
    // calculate the errors between the robot and the path
    void get_path_errors();
    // to reset the paramters when there is no path or path completed
    void reset_parameters();
    // to rotate the robot at the last point to the desired orientation
    void correction_final_theta();
    // the path following algorithm
    void path_following_controller();

    double sat_linear_velocity(double max , double min ,double accel, double v_ref , double v_real);

};

#endif
