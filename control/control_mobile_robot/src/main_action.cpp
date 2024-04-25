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
typedef actionlib::SimpleActionServer<control_mobile_robot::ControlAction> ControlServer;
control_mobile_robot::ControlResult result_msg;
control_mobile_robot::ControlFeedback feedback_msg;
control_mobile_robot::SimulatecollisionActionGoal  simulatecollision;

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

// path parameters
uint32_t size_path = 0;
double final_orientation = 0;

double t_g_map , t_l_map , t_mpc;
// odometry parametrs
double odom_x;
double odom_y;
double odom_theta;

int direction = 1;
double compensate_angle = 0;

// rate time parameter
double dt = 0.1;



// boolean paramters
bool final_theta_corrected = false;
bool path_stored = false;
bool robot_reached = false;
bool emergancy_stop = false;
// writing file paramter

// declare message parameters
geometry_msgs::Twist        tw_msg;
geometry_msgs::Pose2D       qp[100] , err;


// declare Publishers and Subscribers
ros::Publisher   cmd_pub;
ros::Publisher   detection_passage_pub;
ros::Publisher   reached_status_pub;
ros::Subscriber  path_theta_sub;
ros::Subscriber  path_mpc_sub;
ros::Subscriber  odom_sub;
ros::Subscriber  orient_sub;
ros::Subscriber  status_collision_sub;
ros::Subscriber  backward_collision_sub;
ros::Subscriber  position_collision_sub;
ros::Subscriber  grid_map_sub;
ros::Subscriber  grid_local_map_sub;

// declare topics
std::string path_theta_topic;
std::string path_mpc_topic;
std::string globla_local_topic;
std::string orient_topic;
std::string cmd_topic;
std::string global_frame;
std::string base_frame;
std::string odometry_topic;

std_msgs::String reached_status_msg;

// declare config parameters
double max_v;
double min_v;
double acc_v;
double max_w;

double x_goal , y_goal , x1_path , y1_path;

bool start_simulate_collision = false;

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
// to define subscribers and publishers
void define_sub_pub(ros::NodeHandle n);
// to initialize config paramters
void initialize_parameters(ros::NodeHandle n);
// to get the orientation of yaw for the local frame and the last segment
double get_yaw(geometry_msgs::Quaternion q);
// to set saturation on the linear velocity of the robot
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double old_velocity);

void publish_result(ControlServer* control_server);

bool  flag_orientation = false ;


bool StartAction = false;
bool rotation_collision = true;
bool position_collision = true;
bool backward_collision = true;
bool simulation_node_is_active = false;

void actionCallback(const control_mobile_robot::ControlGoal::ConstPtr& goal_comand, ControlServer* control_server)
{
   // x_goal = goal_comand->x;
   // y_goal = goal_comand->y;
  // ROS_WARN("Motion Action Received, Goal point %f %f", x_goal , y_goal);
    if(goal_comand->goal== "StartMotion")
    {
        robot_reached = false;
        start_simulate_collision = true;
        StartAction = true;
        path_stored = false;
        reached_status_msg.data = "not reached";
        reached_status_pub.publish(reached_status_msg);
        ros::Rate feedback_rate(2);
        publish_result(control_server);        
                
        feedback_rate.sleep();
    }
}
void publish_result(ControlServer* control_server)
{
    ros::Rate feedback_rate(5);

    while (!robot_reached)
    {
        if (control_server->isPreemptRequested())
        {
            ROS_WARN("EMERGENCY STOP");
            v_cmd = 0;
            w_cmd = 0;
            tw_msg.linear.x = v_cmd;
            tw_msg.angular.z = w_cmd;
            cmd_pub.publish(tw_msg);
            StartAction = false;
            break;
        }
        feedback_rate.sleep();
    }
    if(robot_reached)
    {
        v_cmd = 0;
        w_cmd = 0;
        tw_msg.linear.x = v_cmd;
        tw_msg.angular.z = w_cmd;
        cmd_pub.publish(tw_msg);
        StartAction = false;
        ROS_INFO("Robot Reached to the goal %f %f", x_goal , y_goal);
        result_msg.status = "done";
        reached_status_msg.data = "reached";
        control_server->setSucceeded(result_msg);
        reached_status_pub.publish(reached_status_msg);
    }
}

void path_theta_back(const nav_msgs::Path msg) 
{
    int size_2 = msg.poses.size();
    if(size_2!=0)
    {
        final_orientation = get_yaw(msg.poses[size_2-1].pose.orientation);
        x1_path = msg.poses[0].pose.position.x;
        y1_path = msg.poses[0].pose.position.y;
    }
}

void path_mpc_back(const nav_msgs::Path msg) 
{
        size_path = msg.poses.size();

        t_mpc = msg.header.stamp.toSec();
        
        if(size_path!=0)
        {         
            for (int j=0; j<size_path ; j++)
            { 
                qp[j].x=msg.poses[j].pose.position.x;
                qp[j].y=msg.poses[j].pose.position.y;
            }
            x_goal = qp[size_path-1].x;
            y_goal = qp[size_path-1].y;
            path_stored = true;    
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

void flag_last_orientation_back(const std_msgs::Bool msg) 
{
    flag_orientation = msg.data;
}

void collision_rotation_back(const std_msgs::Bool msg) 
{  
    if (msg.data)
    {
        v_cmd = 0;
        rotation_collision = true;
        ROS_INFO("Rotation motion can lead to collide!");
    }
    else 
    {
        rotation_collision = false;
        ROS_INFO("Robot is rotating safety!");
    }
    simulation_node_is_active = true;
}

void collision_position_back(const std_msgs::Bool msg) 
{  
    if (msg.data)
    {
        v_cmd = 0;
        position_collision = true;
        ROS_INFO("Forward motion can lead to collide!");
    }
    else 
    {
        position_collision = false;
        ROS_INFO("Forward motion safety!");
    }
    simulation_node_is_active = true;
}

void collision_backward_back(const std_msgs::Bool msg) 
{
    if(msg.data)
    {
        backward_collision = true;
        v_cmd = 0;
        ROS_WARN("Backward motion can lead to collision! Stop Motion");
    }
    else
    {
        backward_collision = false;
        
    }
    simulation_node_is_active = true;
}

void odom_back(const nav_msgs::Odometry msg) {
    v_robot = abs(msg.twist.twist.linear.x);
}

void path_following_controller()
{
    //  DECREASING
    if (k_v == size_path-1 && dist_error<1.5 && v_cmd>0.3)// && abs(angle_error)<=0.25 && v_cmd>0.3)  // decreasing
    {
        max_v_local=v_cmd;
        if (max_v_local>(max_v/2))
        {
            max_v_local=max_v_local-0.04;
        }
        v_cmd = sat_linear_velocity(max_v_local,min_v,acc_v ,dist_error, v_cmd);
    }
    else
    {
        // Motion 
        if(abs(angle_error)<0.1)  // case error angle very small
        {
            v_cmd = sat_linear_velocity(max_v,min_v,acc_v,dist_error , v_cmd);
            w_cmd = angle_error;
        }
        else if(abs(angle_error)>=0.1 && abs(angle_error)<0.38)  // case error angle small
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/6),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.9*angle_error;
        }
        else if(abs(angle_error)>=0.38 && abs(angle_error)<0.7)  // case error angle medium
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/3),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.85*angle_error;
        }
        else if(abs(angle_error)>=0.7 && abs(angle_error)<1)  // case error angle big
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/2),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.8*angle_error;
        }
        else if(abs(angle_error)>=1 && abs(angle_error)<1.25)  // case angle error very big
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/1.5),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.75*angle_error;
        }
        else if(abs(angle_error)>=1.25)
        {
            v_cmd = 0;
            w_cmd = 1*angle_error;
        }
        
    }

 //   ROS_INFO("vel 1 %f" , v_cmd);
    
    if(abs(angle_error)>=0.1 && v_robot<=0.1 && k_v<=5 && size_path > 1) // v_cmd instead of v_robot for simulation
    {
        v_cmd = 0;
    //    ROS_INFO("STOP vel 2 %f " , v_cmd);
        w_cmd = 1*angle_error;
    }

    if(abs(angle_error)>=0.1 && v_robot<=0.1 && k_v<=5 && size_path == 1)
    {
        v_cmd = 0;
    //    ROS_INFO("STOP vel 2 %f " , v_cmd);
        w_cmd = 1*angle_error;
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
    double angle_segment = 0;
    if(path_stored)
    {
        if(start_simulate_collision)
        {
            simulatecollision.goal.x = x1_path;
            simulatecollision.goal.y = y1_path;
            detection_passage_pub.publish(simulatecollision);
            start_simulate_collision = false;
            simulation_node_is_active = false;
            ROS_INFO("Action simulate motion is sent");
        }

        //Calculate the erros between the robot and the current segement
        err.x = (qp[k_v].x-odom_x) ; 
        err.y =  (qp[k_v].y-odom_y) ;
        dist_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
        angle_segment=atan2(err.y,err.x);
        err.theta =  angle_segment - odom_theta - compensate_angle;
        angle_error = atan2(sin(err.theta),cos(err.theta));

        // To prevent robot vibration at critical angles at first motion



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
            err.x = (qp[k_v].x-odom_x);
            err.y =  (qp[k_v].y-odom_y);
            dist_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
            angle_segment=atan2(err.y,err.x);
            err.theta =  angle_segment - odom_theta - compensate_angle;
            angle_error = atan2(sin(err.theta ),cos(err.theta ));
            if(abs(angle_error)>3.04)
            {
                if(angle_segment>odom_theta)
                    angle_error = abs(angle_error);
                else
                    angle_error = -abs(angle_error);           
            }
            k_v++;
        }
    }  
}

void stop_mode()
{
    dist_stop = sqrt(pow((x_goal-odom_x), 2) + pow((y_goal-odom_y), 2));
    if (dist_stop<=0.21 )
    {
        path_stored=false;
        if(!flag_orientation )
        {
            robot_reached=true;
        }
    }
    
}

void reset_parameters()
{
    v_cmd = 0;
    w_cmd = 0;
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
        final_theta_corrected = true;        
    }
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
    n.getParam("path_mpc_topic",   path_mpc_topic);
    n.getParam("globla_local_topic",   globla_local_topic);   
    n.getParam("orient_topic",   orient_topic);
    n.getParam("cmd_topic" ,    cmd_topic);
    n.getParam("global_frame" , global_frame);
    n.getParam("base_frame" ,   base_frame);
    n.getParam("odometry_topic" ,  odometry_topic);
    n.getParam("max_v", max_v);
    n.getParam("min_v", min_v);
    n.getParam("acc_v", acc_v);
    n.getParam("max_w", max_w);
}

void grid_map_back(const nav_msgs::OccupancyGrid& msg)
{
    t_g_map = msg.header.stamp.toSec();
}

void grid_local_map_back(const nav_msgs::OccupancyGrid& msg)
{
    t_l_map = msg.header.stamp.toSec();
}

void define_sub_pub(ros::NodeHandle n)
{
    // Define the publishers and sunscribers
    path_theta_sub       = n.subscribe                              (path_theta_topic,10 , path_theta_back);  //path_back2
    path_mpc_sub         = n.subscribe                              (path_mpc_topic,10 , path_mpc_back);  //path_mpc
    status_collision_sub       = n.subscribe                              ("/collision_rotation",10 , collision_rotation_back);
    backward_collision_sub       = n.subscribe                              ("/collision_backward",10 , collision_backward_back);
    position_collision_sub       = n.subscribe                              ("/collision_position",10 , collision_position_back);

    orient_sub          = n.subscribe                              (orient_topic,10 , flag_last_orientation_back);
    odom_sub            = n.subscribe                              ("/husky_velocity_controller/odom", 5, odom_back);
    cmd_pub             = n.advertise<geometry_msgs::Twist>        (cmd_topic, 1);
    grid_map_sub            = n.subscribe("/occupancy_grid_map/grid_map",       10, grid_map_back);  
    grid_local_map_sub      = n.subscribe("/occupancy_grid_local_map/grid_map", 10, grid_local_map_back);
    detection_passage_pub     = n.advertise<control_mobile_robot::SimulatecollisionActionGoal>        ("/detection_passage_server/goal", 1);
    reached_status_pub   = n.advertise<std_msgs::String> ("reached_status", 10);

}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;
    initialize_parameters(n);

    define_sub_pub(n);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    
 
    ros::Rate loop_rate(10); // ros spins 20 frames per second

    ControlServer server(n, "control_motion", boost::bind(&actionCallback, _1, &server), false);
    server.start();

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

        double t_now = ros::Time::now().toSec();

        if(((t_now - t_g_map)>2 || (t_now - t_l_map)>2 || (t_now - t_mpc)>2) && StartAction)
        {
            ROS_ERROR("Occupancy Grid Map, Local map or MPC is Failed, Stop Motion");
            emergancy_stop = true;
        }
        else
        {
            emergancy_stop = false;
        }

        if (path_stored) 
        {
            get_path_errors();
            path_following_controller();
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
        }
        /*
        if(rotation_collision && !position_collision)
        {
            v_cmd = 0;
            w_cmd = 0;
            tw_msg.linear.x = 0.1;
            tw_msg.angular.z = 0;
            ROS_INFO("Forward motion to a free-obstacle position");
        }
        else if(rotation_collision && !backward_collision)
        {
            v_cmd = 0;
            w_cmd = 0;
            tw_msg.linear.x = -0.1;
            tw_msg.angular.z = 0;
            ROS_INFO("Backward motion! Searching free-obstacle rotation");
        }
        else if(rotation_collision && backward_collision)
        {
            v_cmd = 0;
            w_cmd = 0;
            tw_msg.linear.x = v_cmd;
            tw_msg.angular.z = w_cmd;
            ROS_INFO("Robot can't rotate saftey, Stop Motion!");
        }
        
        if(emergancy_stop)
        {
            v_cmd = 0;
            w_cmd = 0;
            tw_msg.linear.x = v_cmd;
            tw_msg.angular.z = w_cmd;
            ROS_WARN("EMERGENCY STOP,  Error in RTABMAP or MPC , Fix it and restart the control node");
        }
        */
        if (!robot_reached && StartAction && simulation_node_is_active) 
        {
            cmd_pub.publish(tw_msg);
        }
       
     

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
