#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Bool.h"
#include "visualization_msgs/Marker.h"

#include "theta_star_planner/line_of_sight.hpp"
#include "theta_star_planner/map.hpp"
#include "theta_star_planner/environment_options.hpp"
#include "theta_star_planner/theta_star.hpp"

#define DEFAULT_REACH_RADIUS 0.5

std::string DEFAULT_PATH_TO_DOORS = "/home/administrator/pointnav_ws/door_coords.txt";

EnvironmentOptions opt(CN_SP_MT_EUCL, CN_SP_BT_GMAX, false, false, 1);
Map map;
ros::Publisher pub, marker_pub, door_pub, door_marker_pub, inflated_map_pub, flag_orient_pub, flag_forward_pub, flag_global_map_pub;
float agent_radius_thin, agent_radius_thick, reach_radius;
std::string path_to_doors, inflated_map_topic;
std::vector<Door> door_coords;
std::vector<Point> door_points;


void pointCallback(const geometry_msgs::PointStamped& msg)
{
    door_points.push_back(Point(msg.point.x, msg.point.y));
}


void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
    ROS_DEBUG("Map received!");
    Point pos = Point(msg.info.origin.position.x, msg.info.origin.position.y);
    Quaternion orient = {msg.info.origin.orientation.x, msg.info.origin.orientation.y, msg.info.origin.orientation.z, msg.info.origin.orientation.w};
    //ROS_INFO("Time before inflation: %f", ros::Time::now().toSec());
    map.Update(msg.info.resolution, msg.info.height, msg.info.width, pos, orient, msg.data, agent_radius_thin, agent_radius_thick);
    //ROS_INFO("Time after inflation: %f", ros::Time::now().toSec());

    if (inflated_map_topic != "")
    {
        nav_msgs::OccupancyGrid inflated_map_msg(msg);
        inflated_map_msg.data.clear();
        inflated_map_msg.data = map.GetGrid();
        inflated_map_pub.publish(inflated_map_msg);
    }
}

float dist(Point p1, Point p2)
{
    return sqrt(pow(p1.X() - p2.X(), 2) + pow(p1.Y() - p2.Y(), 2));
}

void taskCallback(const std_msgs::Float32MultiArray& msg)
{
    ROS_DEBUG("Received task at time %f", ros::Time::now().toSec());
    std::list<Point> path;

    ThetaStar search_zero_radius(map, opt, Point(msg.data[0], msg.data[1]), Point(msg.data[2], msg.data[3]), 0, reach_radius);
    search_zero_radius.CreateGlobalPath();
    path = search_zero_radius.GetCurrPath();
    ROS_DEBUG("Path search finished at time %f", ros::Time::now().toSec());

    // publish path as nav_msgs::Path
    geometry_msgs::PoseStamped pose;
    ROS_DEBUG("Found path with size: %d", path.size());
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "local_map_lidar";
    auto it = path.begin();
    if (path.size() > 1)
    {
         ++it;
    }
    for(; it != path.end(); ++it)
    {
        pose.pose.position.x = it->X();
        pose.pose.position.y = it->Y();
        path_msg.poses.push_back(pose);
    }
    std_msgs::Bool flag_orient_msg, flag_forward_msg, flag_global_map_msg;
    if ((msg.layout.dim[0].size == 5) && (path_msg.poses.size() > 0))
    {
        double theta = msg.data[4];
        tf2::Quaternion orientation(0., 0., theta);
        path_msg.poses[path_msg.poses.size() - 1].pose.orientation.x = orientation.getX();
        path_msg.poses[path_msg.poses.size() - 1].pose.orientation.y = orientation.getY();
        path_msg.poses[path_msg.poses.size() - 1].pose.orientation.z = orientation.getZ();
        path_msg.poses[path_msg.poses.size() - 1].pose.orientation.w = orientation.getW();
        flag_orient_msg.data = true;
    }
    else
    {
        flag_orient_msg.data = false;
    }
    if (path_msg.poses.size() > 0)
    {
        pub.publish(path_msg);
    }
    else
    {
        ROS_INFO("Theta* could not find path! Nothing will be published");
    }
    flag_forward_msg.data = true;
    flag_global_map_msg.data = true;

    if (path.size() == 0)
        return;
    std::vector<int> doors_found;
    search_zero_radius.FindDoors(door_coords, doors_found);
    Point robot_position = *path.begin();
    for (int j = 0; j < door_coords.size(); j++)
        if (dist(robot_position, Point((door_coords[j].x1 + door_coords[j].x2) / 2., (door_coords[j].y1 + door_coords[j].y2) / 2.)) < 1.0)
        {
            flag_global_map_msg.data = false;
            break;
        }
    for (int i = 0; i < doors_found.size(); i++)
    {
        int j = doors_found[i];
        if (dist(robot_position, Point((door_coords[j].x1 + door_coords[j].x2) / 2., (door_coords[j].y1 + door_coords[j].y2) / 2.)) < 3.0)
        {
            flag_global_map_msg.data = false;
            break;
        }
    }

    flag_orient_pub.publish(flag_orient_msg);
    flag_forward_pub.publish(flag_forward_msg);
    flag_global_map_pub.publish(flag_global_map_msg);

    // publish path as visualization_msgs::Marker
    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = "local_map_lidar";
    marker_msg.ns = "points_and_lines";
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.type = 4;
    marker_msg.scale.x = 0.2;
    marker_msg.scale.y = 0.2;
    marker_msg.color.a = 1.0;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    for(auto& pos : path)
    {
        geometry_msgs::Point pt;
        pt.x = pos.X();
        pt.y = pos.Y();
        pt.z = 0;
        marker_msg.points.push_back(pt);
    }
    marker_pub.publish(marker_msg);
    ROS_DEBUG("Publishing finished at time %f", ros::Time::now().toSec());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "theta_star_planner");
    ros::NodeHandle n;
    agent_radius_thin = 0.2;
    agent_radius_thick = 0.6;
    reach_radius = DEFAULT_REACH_RADIUS;
    path_to_doors = DEFAULT_PATH_TO_DOORS;
    inflated_map_topic = "";
    ros::param::get("~agent_radius_thin", agent_radius_thin);
    ros::param::get("~agent_radius_thick", agent_radius_thick);
    ros::param::get("~reach_radius", reach_radius);
    ros::param::get("~path_to_doors", path_to_doors);
    ros::param::get("~inflated_map_topic", inflated_map_topic);

    std::ifstream fin(path_to_doors);
    float x1, y1, x2, y2;
    while (fin >> x1 >> y1 >> x2 >> y2)
    {
        Door d(x1, y1, x2, y2);
        door_coords.push_back(d);
    }
    fin.close();
  //  ROS_INFO("Read %d doors", door_coords.size());

    ros::Subscriber sub1 = n.subscribe("map", 1, mapCallback);
    ros::Subscriber sub2 = n.subscribe("task", 1, taskCallback);
    ros::Subscriber point_sub = n.subscribe("clicked_point", 10, pointCallback);

    pub = n.advertise<nav_msgs::Path>("path", 100);
    marker_pub = n.advertise<visualization_msgs::Marker>("path_marker", 1);
    door_marker_pub = n.advertise<visualization_msgs::Marker>("door_marker", 1);
    door_pub = n.advertise<std_msgs::Float32MultiArray>("door_coord", 10);
    inflated_map_pub = n.advertise<nav_msgs::OccupancyGrid>(inflated_map_topic, 1);
    flag_orient_pub = n.advertise<std_msgs::Bool>("flag_orient", 10);
    flag_forward_pub = n.advertise<std_msgs::Bool>("flag_forward", 10);
    flag_global_map_pub = n.advertise<std_msgs::Bool>("flag_global_map", 10);
    ros::spin();
}
