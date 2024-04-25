// PATH FOLLOWING Tried On Lift
#include "ros/ros.h"
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

using namespace std;

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
double w_robot = 0;

// path parameters
uint32_t size_path = 0 , size = 0;
double final_orientation = 0;

double time_grid_map , time_local_map , time_mpc;
// odometry parametrs
double odom_x;
double odom_y;
double odom_theta;

int direction = 0;
double compensate_angle = 0;

// rate time parameter
double dt = 0.1;



// boolean paramters
bool final_theta_corrected = false;
bool path_stored = false;
bool path_changed = false;
bool robot_reached = false;
bool activate_cmd = false;

// writing file paramter

// declare message parameters
tf::Point odom_pos;
visualization_msgs::Marker  robot_mk;
geometry_msgs::Point        robot_pt;
geometry_msgs::Twist        tw_msg;
geometry_msgs::Pose2D       qp[100] , new_qp[100], err;
std_msgs::String            status_msg;

// declare Publishers and Subscribers
ros::Publisher   cmd_pub;
ros::Publisher   robot_mk_pub;
ros::Publisher   status_control_pub;
ros::Subscriber  path_theta_sub;
ros::Subscriber  path_mpc_sub;
ros::Subscriber  gloab_local_sub;
ros::Subscriber  odom_sub;
ros::Subscriber  direction_sub;
ros::Subscriber  orient_sub;
ros::Subscriber  relative_value_sub;
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
std::string status_topic;

// declare config parameters
double max_v;
double min_v;
double acc_v;
double max_w;
double acc_w;

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
// to initialize the markers
void initialize_markers();
// to publish three markers: robot position, desired point and
// the distance from robot to path
void publish_marker_odom();
// the path following algorithm
void path_following_controller();
// to define subscribers and publishers
void define_sub_pub(ros::NodeHandle n);
// to initialize config paramters
void initialize_parameters(ros::NodeHandle n);
// to get the orientation of yaw for the local frame and the last segment
double get_yaw(geometry_msgs::Quaternion q);
// to set saturation on the linear velocity of the robot
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double old_velocity);

void orientation_controller(double angle_error);

int ind_control = 0;

bool  flag_direction = true , flag_orientation = false , flag_global_map;


double t = 0;
double how_long = 0 , theta1 = 0 , theta2;
double angle_1_error = 0, angle_2_error=0;
bool angle_1_done= false , angle_2_done = false , activate_relative_cmd = false;
double v_cmd_relative , w_cmd_relative;
bool path_theta_published = false;

void path_theta_back(const nav_msgs::Path msg) 
{
    //activate_cmd = true;
    path_theta_published = true;
    
    int size_2 = msg.poses.size();
    if(size_2!=0)
    {
        
        final_orientation = get_yaw(msg.poses[size_2-1].pose.orientation);
        ROS_INFO("final orient %f " , final_orientation);
    }
    

       /* if(size!=0)
        {         
                ROS_INFO("New Path, it contains: %i points" , size);
                for (int j=0; j<size ; j++)
                { 
                    qp[j].x=msg.poses[j].pose.position.x;
                    qp[j].y=msg.poses[j].pose.position.y;
                    ROS_INFO("x , y %f , %f" , qp[j].x , qp[j].y );
                    
                    final_orientation = get_yaw(msg.poses[size-1].pose.orientation);
                }
            
                size_path = size;
                path_stored = true;
                robot_reached = false;
                k_v=0;
                get_path_errors();
                stop_mode();
                final_theta_corrected = false;
        }
        else
        {
            path_stored = false;
            v_cmd = 0;
            w_cmd = 0;
        }
*/
}

void path_mpc_back(const nav_msgs::Path msg) 
{
        if(path_theta_published)
            activate_cmd = true;
        else
            activate_cmd = false;
        size = msg.poses.size();

        time_mpc = msg.header.stamp.toSec();
        

        if(size!=0)
        {         
            //  ROS_INFO("Odometry Robot %f , %f" , odom_x , odom_y);
            ROS_INFO("New Path, it contains: %i points" , size);
            //  ROS_INFO("time solving: %f" , msg.poses[0].pose.position.z);
            for (int j=0; j<size ; j++)
            { 
                qp[j].x=msg.poses[j].pose.position.x;
                qp[j].y=msg.poses[j].pose.position.y;
            //    ROS_INFO("x , y %f , %f" , qp[j].x , qp[j].y );
                
            }
        
            size_path = size;
            path_stored = true;
            robot_reached = false;
            k_v=0;
            get_path_errors();
            stop_mode();
            final_theta_corrected = false;
        }
        else
        {
            path_stored = false;
            v_cmd = 0;
            w_cmd = 0;
        }
}

void orient_back(const std_msgs::Bool msg) 
{
   // flag_orientation = msg.data;
    flag_orientation = true;

  //  flag_orientation = false;

}

void direction_back(const std_msgs::Bool msg)
{
    if(flag_direction)
    {
    compensate_angle = 0;
        direction = 1;
        min_v = 0;
        max_v = 0.6;
    }
    else
    {
        compensate_angle = 3.14;
        direction = -1;
        min_v = -0.6;
        max_v = 0;
    }
}

void global_local_back(const std_msgs::Bool msg) 
{
    flag_global_map = msg.data;
    if(flag_global_map)
    {
        max_v = 0.4;
        acc_v = 0.04;
        dist_limit = 0.4;  //0.8
    }
    else
    {
        max_v = 0.2;
        acc_v = 0.02;
        dist_limit = 0.4;
    }
}

void odom_back(const nav_msgs::Odometry msg) {
    v_robot = abs(msg.twist.twist.linear.x);
}

void path_following_controller()
{
    max_v = 0.18;
    acc_v = 0.04;
    //  DECREASING
    if (k_v == size_path-1 && dist_error<1.5 && v_cmd>0.3)// && abs(angle_error)<=0.25 && v_cmd>0.3)  // decreasing
    {
        max_v_local=v_cmd;
        if (max_v_local>(max_v/2))
        {
            max_v_local=max_v_local-0.04;
        }
        v_cmd = sat_linear_velocity(max_v_local,min_v,acc_v ,dist_error, v_cmd);
        ind_control = 1;
    }
    else
    {
        // Motion 
        if(abs(angle_error)<0.1)  // case error angle very small
        {
            v_cmd = sat_linear_velocity(max_v,min_v,acc_v,dist_error , v_cmd);
            w_cmd = angle_error;
            ind_control = 2;
        }
        else if(abs(angle_error)>=0.1 && abs(angle_error)<0.38)  // case error angle small
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/6),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.9*angle_error;
            ind_control = 3;
        }
        else if(abs(angle_error)>=0.38 && abs(angle_error)<0.7)  // case error angle medium
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/3),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.85*angle_error;
            ind_control = 4;
        }
        else if(abs(angle_error)>=0.7 && abs(angle_error)<1)  // case error angle big
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/2),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.8*angle_error;
            ind_control = 5;
        }
        else if(abs(angle_error)>=1 && abs(angle_error)<1.25)  // case angle error very big
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/1.5),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.75*angle_error;
            ind_control = 6;   
        }
        else if(abs(angle_error)>=1.25)
        {
            v_cmd = 0;
            w_cmd = 1*angle_error;
            ind_control = 7;
        }
        
    }

 //   ROS_INFO("vel 1 %f" , v_cmd);
    
    if(abs(angle_error)>=0.1 && v_robot<=0.1 && k_v<=5 && size_path > 1) // v_cmd instead of v_robot for simulation
    {
        v_cmd = 0;
    //    ROS_INFO("STOP vel 2 %f " , v_cmd);
        w_cmd = 1*angle_error;
        ind_control = 8;
    }

    if(abs(angle_error)>=0.1 && v_robot<=0.1 && k_v<=5 && size_path == 1)
    {
        v_cmd = 0;
    //    ROS_INFO("STOP vel 2 %f " , v_cmd);
        w_cmd = 1*angle_error;
        ind_control = 10;
    }
    if(w_cmd>max_w)
    {w_cmd = max_w;}
    if(w_cmd<-max_w)
    {w_cmd = -max_w;}
    if(v_cmd>max_v)
    {v_cmd = max_v;}


    tw_msg.linear.x=v_cmd;
    tw_msg.angular.z=w_cmd;
    
  
}

void get_path_errors()
{

    double angle_path1 = 0;
    if(path_stored)
    {
        //Calculate the erros between the robot and the current segement
        err.x = (qp[k_v].x-odom_x) ; 
        err.y =  (qp[k_v].y-odom_y) ;
        dist_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
        angle_path1=atan2(err.y,err.x);
        err.theta =  angle_path1 - odom_theta - compensate_angle;
        angle_error = atan2(sin(err.theta ),cos(err.theta ));

        //Calculate the erros between the robot and the next segement
        if(k_v<(size_path-1))
        {
            err.x = (qp[k_v+1].x-qp[k_v].x);
            err.y =  (qp[k_v+1].y-qp[k_v].y);
            double next_angle_path=atan2(err.y,err.x);
            err.theta =  next_angle_path - odom_theta;
        }
        // accordingly to these errors move to the next segment      
        while (dist_error<dist_limit && k_v <size_path-1)
        {
            err.x = (qp[k_v].x-odom_x) ; 
            err.y =  (qp[k_v].y-odom_y) ;
            dist_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
            angle_path1=atan2(err.y,err.x);
            err.theta =  angle_path1 - odom_theta - compensate_angle;
            angle_error = atan2(sin(err.theta ),cos(err.theta ));
            k_v++;
        }
    }  
}

void stop_mode()
{
    dist_stop = sqrt(pow((qp[size_path-1].x-odom_x), 2) + pow((qp[size_path-1].y-odom_y), 2));
    if (dist_stop<=0.21)
    {
        path_stored=false;
        if(!flag_orientation)
        {
            robot_reached=true;
            ROS_INFO("Robot Reached to the goal");
            status_msg.data="reached";
            //status_control_pub.publish(status_msg);
        }
    }
    else
    {
        ROS_INFO("Robot Not Reached to the goal");
        status_msg.data="not reached";
        robot_reached = false;
    }
    
}

void reset_parameters()
{
    v_cmd = 0;
    w_cmd = 0;
}

void publish_marker_odom()
{
    robot_pt.x=odom_x;
    robot_pt.y=odom_y;
    robot_mk.points.push_back(robot_pt);
    robot_mk_pub.publish(robot_mk);
}

void correction_final_theta()
{
    err.theta =  final_orientation - odom_theta;
    angle_error = atan2(sin(err.theta ),cos(err.theta ));
 //   ROS_INFO("Angle in correction %f ", err.theta);
 //   ROS_INFO("Final theta corrected: %d", final_theta_corrected);
    if (abs(err.theta)>=0.05 && !final_theta_corrected)
    {
        v_cmd = 0.0;
        w_cmd = 1.2*angle_error;
        if (angle_error > 0)
        {
            w_cmd = 0.25;
        }
        else if(angle_error<0)
        {
            w_cmd = -0.25;
        }
    }
    if (abs(err.theta)<0.06 && !final_theta_corrected)
    {
        robot_reached=true;
        v_cmd = 0;
        w_cmd = 0;
        final_theta_corrected = true;
        status_msg.data="reached";
        ROS_INFO("Robot Reached to the goal");
        status_control_pub.publish(status_msg);
        ROS_INFO("The Final ORIENTATION IS CORRECTED");
    }
   /// ROS_INFO("Final orient error %f" , abs(err.theta));
}

double sat_linear_velocity(double max , double min ,double accel, double v_ref , double v_real)
{
    double v_diff = v_ref - v_real;
    // saturations on accelerations
    if(v_diff>=0)
    {
        v_diff = v_real + direction * accel*dt;
    }
    else if(v_diff<0)
    {
        v_diff = v_real - direction * accel*dt;
    }

    if (v_diff<min)
    {
        return min;
    }
    else if (v_diff>max)
    {
        return max;
    }
    else
    {
        return v_diff;
    }
}

void initialize_parameters(ros::NodeHandle n)
{
    n.getParam("path_theta_topic",   path_theta_topic);
    n.getParam("path_mpcl4c_topic",   path_mpc_topic);
    n.getParam("flag_global_map",         flag_global_map);
    n.getParam("globla_local_topic",   globla_local_topic);   
    n.getParam("direction_topic",   direction_topic); 
    n.getParam("orient_topic",   orient_topic);
    n.getParam("cmd_topic" ,    cmd_topic);
    n.getParam("global_frame" , global_frame);
    n.getParam("base_frame" ,   base_frame);
    n.getParam("odometry_topic" ,  odometry_topic);
    n.getParam("status_topic" , status_topic);
    n.getParam("max_v", max_v);
    n.getParam("min_v", min_v);
    n.getParam("acc_v", acc_v);
    n.getParam("max_w", max_w);
    n.getParam("acc_w", acc_w);
}

void value_back(const std_msgs::Float64MultiArray msg)
{
    ROS_INFO("Command Reading");

    if(msg.data[0]==0)  //move forward
    {
        v_cmd_relative = 0.2;
    }
    else if(msg.data[0]==1)  //move backward
    {
        v_cmd_relative = -0.2;
    }
    t = 0;
    angle_1_done = false;
    angle_2_done = false;
    activate_relative_cmd = true;
    theta1 = msg.data[1];
    how_long = msg.data[2]/abs(v_cmd_relative);
    theta2 = msg.data[3];
}
void theta_1_errors()
{
    double err_theta1;
    err_theta1 =  theta1 - odom_theta;
    angle_1_error = atan2(sin(err_theta1),cos(err_theta1));
    if(abs(angle_1_error)<0.1)
    {
        angle_1_done = true;
    }
}

void theta_2_errors()
{
    double err_theta2;
    err_theta2 =  theta2 - odom_theta;
    angle_2_error = atan2(sin(err_theta2),cos(err_theta2));
    if(abs(angle_2_error)<0.1)
    {
        angle_2_done = true;
        activate_relative_cmd = false;
        status_msg.data="done";
        status_control_pub.publish(status_msg);
        tw_msg.linear.x = 0;
        tw_msg.angular.z = 0;
        
    }
}

void orientation_controller(double angle_error_relative)
{
    // Motion 
    if(abs(angle_error_relative)<0.1)  // case error angle very small
    {
        w_cmd_relative = 1*angle_error_relative;
    }
    else if(abs(angle_error_relative)>=0.1 && abs(angle_error_relative)<0.38)  // case error angle small
    {
        w_cmd_relative = 0.9*angle_error_relative;
    }
    else if(abs(angle_error_relative)>=0.38 && abs(angle_error_relative)<0.7)  // case error angle medium
    {
        w_cmd_relative = 0.85*angle_error_relative;
    }
    else if(abs(angle_error_relative)>=0.7 && abs(angle_error_relative)<1)  // case error angle big
    {
        w_cmd_relative = 0.8*angle_error_relative;
    }
    else if(abs(angle_error_relative)>=1 && abs(angle_error_relative)<1.25)  // case angle error very big
    {
        w_cmd_relative = 0.75*angle_error_relative;
    }
    else if(abs(angle_error_relative)>=1.25)
    {
        w_cmd_relative = 1*angle_error_relative;
    }
    

    if(w_cmd_relative>max_w)
    {w_cmd_relative = max_w;}
    if(w_cmd_relative<-max_w)
    {w_cmd_relative = -max_w;}

    tw_msg.angular.z = w_cmd_relative;  
}

void grid_map_back(const nav_msgs::OccupancyGrid& msg)
{
    time_grid_map = msg.header.stamp.toSec();
}

void grid_local_map_back(const nav_msgs::OccupancyGrid& msg)
{
    time_local_map = msg.header.stamp.toSec();
}

void define_sub_pub(ros::NodeHandle n)
{
    // Define the publishers and sunscribers
    path_theta_sub       = n.subscribe                              (path_theta_topic,10 , path_theta_back);  //path_back2
    path_mpc_sub         = n.subscribe                              (path_mpc_topic,10 , path_mpc_back);  //path_mpc
    gloab_local_sub     = n.subscribe                              (globla_local_topic,10 , global_local_back);
    direction_sub       = n.subscribe                              (direction_topic,10 , direction_back);
    orient_sub          = n.subscribe                              (orient_topic,10 , orient_back);
    odom_sub            = n.subscribe                              ("/husky_velocity_controller/odom", 5, odom_back);
    cmd_pub             = n.advertise<geometry_msgs::Twist>        (cmd_topic, 1);
    robot_mk_pub        = n.advertise<visualization_msgs::Marker>  ("marker_real", 1);
    status_control_pub  = n.advertise<std_msgs::String>            (status_topic, 10);
    relative_value_sub  = n.subscribe  ("/relative_motion", 1, value_back);
    grid_map_sub            = n.subscribe("/occupancy_grid_map/grid_map",       10, grid_map_back);  
    grid_local_map_sub      = n.subscribe("/occupancy_grid_local_map/grid_map", 10, grid_local_map_back);

}

void initialize_markers()
{

    robot_mk.header.frame_id = global_frame;
    robot_mk.header.stamp = ros::Time();
    robot_mk.ns = "points_and_lines";
    robot_mk.action = visualization_msgs::Marker::ADD;
    robot_mk.pose.orientation.w = 1.0;
    robot_mk.type = 4;
    robot_mk.scale.x = 0.2;
    robot_mk.scale.y = 0.2;
    robot_mk.color.a = 1.0;
    robot_mk.color.r = 0.0;
    robot_mk.color.g = 0.0;
    robot_mk.color.b = 1.0;
    robot_mk.points.clear();
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;
    initialize_parameters(n);

    initialize_markers();

    define_sub_pub(n);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    if(flag_global_map)
    {
        ROS_WARN("High speed motion");
    }
    else
    {
        ROS_WARN("Low speed motion");
    }
    bool state_maps = true;


    ros::Rate loop_rate(10); // ros spins 20 frames per second

    while (ros::ok()) 
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {    
            transformStamped = tfBuffer.lookupTransform(global_frame, base_frame,
                               ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        odom_x = transformStamped.transform.translation.x;
        odom_y = transformStamped.transform.translation.y;
        odom_theta = get_yaw(transformStamped.transform.rotation);

        double timestamp_now = ros::Time::now().toSec();

        if((timestamp_now - time_grid_map)>2)
        {
            ROS_ERROR("Occupancy Grid Map is Failed");
            state_maps = false;
        }
        else
        {
            state_maps = true;
        }

        if((timestamp_now - time_mpc)>2 && path_theta_published)
        {
            ROS_ERROR("MPC is Failed");
            state_maps = false;
        }
        else
        {
            state_maps = true;
        }


        if (path_stored) 
        {
            get_path_errors();
            path_following_controller();
            publish_marker_odom();
            stop_mode(); 
        }
        else
        {
         //   ROS_INFO("Correct orientation");
            tw_msg.linear.x=0;
            tw_msg.angular.z=0;
            reset_parameters();
            if (flag_orientation)
            {
                correction_final_theta();
                tw_msg.angular.z=w_cmd;
            }
            
        }

        // to start smoothly in case emergency stop
        if (abs(v_robot)<0.06 && abs(v_cmd-v_robot)>0.4)
        {
            v_cmd = 0;
            tw_msg.linear.x = v_cmd;
            ind_control = 9;
        }

        if(!state_maps)
        {
            tw_msg.linear.x = 0;
            tw_msg.angular.z = 0;
        }

        if (activate_cmd)
        {
            cmd_pub.publish(tw_msg);
            status_control_pub.publish(status_msg);
        }
        

        if (robot_reached)
        {
            activate_cmd = false;
            path_theta_published = false;
            status_msg.data="";
        }

       /*if(activate_relative_cmd)
        {
            if(!angle_1_done)
            {
                theta_1_errors();
                orientation_controller(angle_1_error);
                tw_msg.linear.x = 0;
            }
            else if(t <= how_long && angle_1_done)
            {
                t = t + 0.1;
                tw_msg.linear.x = v_cmd_relative;
                tw_msg.angular.z = 0;
            }
            else if(angle_1_done && t > how_long && !angle_2_done)
            {
                theta_2_errors();
                orientation_controller(angle_2_error);
                tw_msg.linear.x = 0;
            }
            ROS_INFO("wcmd vcmd %f %f" , w_cmd_relative , v_cmd_relative);
            cmd_pub.publish(tw_msg);
            
        }*/
        

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

double get_yaw(geometry_msgs::Quaternion q)
{
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
