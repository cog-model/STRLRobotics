#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

float x_st = -1000;//= std::stof(argv[0]);
float y_st = -1000;// = std::stof(argv[1]);
float x_gl;// = std::stof(argv[2]);
float y_gl;// = std::stof(argv[3]);

ros::Publisher pub;
std_msgs::Float32MultiArray task;

void startCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    ROS_WARN("START POSITION IS RECIEVED!");
    x_st = msg.pose.pose.position.x;
    y_st = msg.pose.pose.position.y;
}

void goalCallback(const geometry_msgs::PoseStamped& msg)
{
    ROS_WARN("GOAL POSITION IS RECIEVED!");
    x_gl = msg.pose.position.x;
    y_gl = msg.pose.position.y;
    if ((x_st == -1000) && (y_st == -1000))
    {
        ROS_WARN("START POSITION IS NOT SET!");
        return;
    }
    task.data = {x_st, y_st, x_gl, y_gl};
    pub.publish(task);
}


int main(int argc, char **argv)
{
    //std::cin >> x_st >> y_st >> x_gl >> y_gl;
    ros::init(argc, argv, "theta_star_planner_test");
    ros::NodeHandle n;
    pub = n.advertise<std_msgs::Float32MultiArray>("task", 1000);
    ROS_INFO("Task preparing!");
    task.layout.data_offset = 0;
    task.layout.dim.push_back(std_msgs::MultiArrayDimension());
    task.layout.dim[0].label = "width";
    task.layout.dim[0].size  = 4;
    task.layout.dim[0].stride  = 4;
    ros::Subscriber start_sub = n.subscribe("initialpose", 1000, startCallback);
    ros::Subscriber goal_sub = n.subscribe("move_base_simple/goal", 1000, goalCallback);
    ros::spin();
    // ros::Rate loop_rate(1);
}