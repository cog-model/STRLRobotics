#include "lib.h"
#include "ROS_Node.h"
#include "MPC_Planner.h"
#include "MPC_ROS_Utils.h"

MPC_Planner mpc;
MPC_ROS_Utils mpc_utils;
nav_msgs::Path path_theta_star;
void planner_back(const nav_msgs::Path msg) 
{
    ROS_INFO("Global Path Recieved");
    int size_path = msg.poses.size();
    ROS_INFO("Size theta path %i" , size_path);
    if(size_path != 0)
    {   
        mpc.x_path.clear();
        mpc.y_path.clear();
        mpc.x_path.push_back(x0.x);
        mpc.y_path.push_back(x0.y); 
    //    ROS_INFO(" x0 %f , %f" , mpc.x_path[0] , mpc.y_path[0]);             
        for (int j=0; j<size_path ; j++)
        {   
            mpc.x_path.push_back(msg.poses[j].pose.position.x);
            mpc.y_path.push_back(msg.poses[j].pose.position.y);
    //        ROS_INFO(" theta_star path %i %f , %f" , j , mpc.x_path[j+1] , mpc.y_path[j+1]);
        }
        path_theta_star = msg;
        ROS_INFO("path stored");
    }
}

void setting_mpc_conditions()
{
    mpc.start_angle_tol = 1.2;
    mpc.max_length_path = 1.5;
    mpc.stop_solving_length = 0.35;
    mpc.v_path = 0.2;
    mpc.print_local_path = false;
    mpc.print_initial_guess = false;
 //   mpc.print_params = true;

    mpc_utils.scan_tol = 0.4;
    mpc_utils.margin_width_scan = 1;
    mpc_utils.margin_length_scan = 0.8;
}

void solve_mpc_problem()
{
  //  ROS_INFO("STARTING MPC PROBLEM %i" , mpc.x_path.size());
    if(mpc.x_path.size()!=0 && !robot_reached)
    {
        ROS_INFO("STARTING MPC PROBLEM");
        mpc.x0 = x0;
        mpc.filter_path();
        mpc_utils.x_filter = mpc.x_filter;
        mpc_utils.y_filter = mpc.y_filter;
        mpc_utils.theta_filter_segments = mpc.theta_filter_segments;
        mpc_utils.length_filter_segments = mpc.length_filter_segments;
        if(mpc_utils.map_received)
        {
            mpc_utils.generate_obstacles_circles();
        }
    //    ROS_INFO("PARAMETERS ARE OBTAINED in %f ms" , float(mpc_utils.time_circle_obst)/1000);
        obst_cells_mk.points.clear();
        for(int i = 0; i<mpc.NP; i++)
        {
            mpc.solver_params[i] = mpc_utils.solver_params[i];
        }
        for(int i=0 ; i<mpc.NP;i+=2)
        {
            if(mpc_utils.solver_params[i]!=100)
            {
                obst_cells_pt.x = mpc_utils.solver_params[i];
                obst_cells_pt.y = mpc_utils.solver_params[i+1];
                obst_cells_pt.z = 0;
                obst_cells_mk.points.push_back(obst_cells_pt);
            }
        }
        
        if(opt_traj.is_solved && !robot_reached && opt_traj.cost<4)
        {
            mpc.is_solved = true;
        }
        else
        {
            mpc.is_solved = false;
        }

        opt_traj = mpc.get_path();
        if(opt_traj.cost<100)
        {
            publish_local_path(opt_traj);
            pub_marker_obst_cells.publish(obst_cells_mk);
        //    ROS_WARN("Published MPC Path");
        }
        else
        {
            ROS_ERROR("MPC Doesn't Find Saftey Path, Old path is used");
        }
        
        /*if(mpc.length_filter_path > mpc.stop_solving_length)
        {

            
        }
        else
        {
            ROS_WARN("Publish theta* path");
            opt_traj_pub.publish(path_theta_star);
        }*/
        

    }
}

void odom_back(const nav_msgs::Odometry msg) {

    x0.v = abs(msg.twist.twist.linear.x);        
}

void grid_map_back(const nav_msgs::OccupancyGrid& msg)
{
    if(flag_global_local)
    {
     //   ROS_INFO("Grid Map Recieved");
        mpc_utils.step_scan = msg.info.resolution - 0.1 * msg.info.resolution;
        mpc_utils.map_received = true;
        mpc_utils.grid_map = msg;
    }
}

void grid_local_map_back(const nav_msgs::OccupancyGrid& msg)
{
    if(!flag_global_local)
    {
    //    ROS_INFO("Grid Local Map Recieved");
        mpc_utils.step_scan = msg.info.resolution - 0.1 * msg.info.resolution;
        mpc_utils.map_received = true;
        mpc_utils.grid_map = msg;
    }
}

void global_local_back(const std_msgs::Bool& msg)
{
    // flag_global_local = msg.data;
}

void status_control_back(const std_msgs::String& msg)
{   
    if(msg.data =="reached")
    {
        robot_reached = true;
     //   mpc_utils.map_received = false;
        ROS_INFO("ROBOT REACHED");
    }
    else
    {
        robot_reached = false;
        ROS_INFO("New MPC task");
    }
}

void publish_local_path(OptTraj traj)
{
    opt_traj_mk.points.clear();
    path_msg.header.stamp = ros::Time::now();
    path_msg.poses.clear();
    path_msg.header.frame_id = global_frame;

    for(int i=0 ; i <= mpc.N ; i++)
    {
        path_msg_pose.position.x = traj.x[i];
        path_msg_pose.position.y = traj.y[i];
        path_msg_pose.orientation.x = 0;
        path_msg_pose.orientation.y = 0;
        path_msg_pose.orientation.z = 0; 
        path_msg_pose.orientation.w = 0;
        path_msg_poses.pose = path_msg_pose;
        path_msg_poses.header.frame_id = global_frame;
        path_msg_poses.header.stamp = ros::Time::now();
        path_msg.poses.push_back(path_msg_poses);
        
        opt_traj_pt.x = traj.x[i];
        opt_traj_pt.y = traj.y[i];
        opt_traj_pt.z = 0;
        opt_traj_mk.points.push_back(opt_traj_pt);
    }
    pub_marker_opt_traj.publish(opt_traj_mk);
    opt_traj_pub.publish(path_msg);
}

void initialize_markers()
{
    opt_traj_mk.points.clear();
    opt_traj_mk.header.frame_id = global_frame;
    opt_traj_mk.header.stamp = ros::Time::now();
    opt_traj_mk.ns = "opt_traj_mk" ;
    opt_traj_mk.action = visualization_msgs::Marker::ADD;;
    opt_traj_mk.pose.orientation.w = 1.0;
    opt_traj_mk.type = 4;
    opt_traj_mk.scale.x = 0.05;
    opt_traj_mk.scale.y = 0.05;
    opt_traj_mk.color.a = 1.0;
    opt_traj_mk.color.r = 0.0;
    opt_traj_mk.color.g = 0.0;
    opt_traj_mk.color.b = 1.0;

    filterd_path_mk.points.clear();
    filterd_path_mk.header.frame_id = global_frame;
    filterd_path_mk.header.stamp = ros::Time::now();
    filterd_path_mk.ns = "filterd_path_mk" ;
    filterd_path_mk.action = visualization_msgs::Marker::ADD;;
    filterd_path_mk.pose.orientation.w = 1.0;
    filterd_path_mk.type = 8;
    filterd_path_mk.scale.x = 0.1;
    filterd_path_mk.scale.y = 0.1;
    filterd_path_mk.color.a = 1.0;
    filterd_path_mk.color.r = 0.0;
    filterd_path_mk.color.g = 0.0;
    filterd_path_mk.color.b = 1.0;

    init_gues_mk.header.frame_id = global_frame;
    init_gues_mk.header.stamp = ros::Time();
    init_gues_mk.ns = "points_and_lines";
    init_gues_mk.action = visualization_msgs::Marker::ADD;
    init_gues_mk.pose.orientation.w = 1.0;
    init_gues_mk.type = 8;
    init_gues_mk.scale.x = 0.05;
    init_gues_mk.scale.y = 0.05;
    init_gues_mk.color.a = 1.0;
    init_gues_mk.color.r = 1.0;
    init_gues_mk.color.g = 0.0;
    init_gues_mk.color.b = 0.0;
    init_gues_mk.points.clear();

    obst_cells_mk.points.clear();
    obst_cells_mk.header.frame_id = global_frame;
    obst_cells_mk.header.stamp = ros::Time::now();
    obst_cells_mk.ns = "obst_cells_mk" ;
    obst_cells_mk.action = visualization_msgs::Marker::ADD;;
    obst_cells_mk.pose.orientation.w = 1.0;
    obst_cells_mk.type = 8;
    obst_cells_mk.scale.x = 0.1;
    obst_cells_mk.scale.y = 0.1;
    obst_cells_mk.color.a = 1.0;
    obst_cells_mk.color.r = 1.0;
    obst_cells_mk.color.g = 0.0;
    obst_cells_mk.color.b = 1.0;
}

void initialize_parameters(ros::NodeHandle n)
{
    n.getParam("rate_loop",             rate_loop);
    n.getParam("global_frame",          global_frame);
    n.getParam("local_frame",           local_frame);
    n.getParam("globla_local_topic",    globla_local_topic);
    n.getParam("mpc_path_topic",        mpc_path_topic);
    n.getParam("global_path_topic",     global_path_topic);
    n.getParam("control_status_topic",  control_status_topic);
    n.getParam("odom_husy_topic",       odom_husy_topic); 
    n.getParam("grid_map_topic",        grid_map_topic);  
    n.getParam("grid_local_map_topic",  grid_local_map_topic);
}

void define_pub_sub(ros::NodeHandle n)
{
    pub_marker_init_gues    = n.advertise<visualization_msgs::Marker> ("mpc_planner/mk_initial",    10);
    pub_marker_opt_traj     = n.advertise<visualization_msgs::Marker> ("mpc_planner/mk_local_path", 10);
    pub_marker_filter_path  = n.advertise<visualization_msgs::Marker> ("mpc_planner/mk_filter_path",10);
    pub_marker_obst_cells   = n.advertise<visualization_msgs::Marker> ("mpc_planner/mk_obst_cells", 10);
    opt_traj_pub            = n.advertise<nav_msgs::Path>             (mpc_path_topic,        10);

    global_path_sub         = n.subscribe(global_path_topic,    10, planner_back);
    odom_sub                = n.subscribe(odom_husy_topic,      5 , odom_back); 
    grid_map_sub            = n.subscribe(grid_map_topic,       10, grid_map_back);  
    grid_local_map_sub      = n.subscribe(grid_local_map_topic, 10, grid_local_map_back); 
    gloab_local_sub         = n.subscribe(globla_local_topic,   10, global_local_back);
    status_control_sub      = n.subscribe("/reached_status", 10, status_control_back);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_planner");

    ros::NodeHandle n;

    tf2_ros::Buffer tfBuffer;

    tf2_ros::TransformListener tfListener(tfBuffer);

    initialize_parameters(n);

    define_pub_sub(n);

    initialize_markers();

    setting_mpc_conditions();

    ros::Rate loop_rate(rate_loop);
    
    while(ros::ok())
    {
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform echo_transform;
        try{
        
            transformStamped = tfBuffer.lookupTransform(global_frame, local_frame,
                               ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
        }
        x0.x = transformStamped.transform.translation.x;
        x0.y = transformStamped.transform.translation.y; 
        x0.theta = get_yaw(transformStamped.transform.rotation);

        solve_mpc_problem();

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
  //  return std::atan(siny_cosp/ cosy_cosp);

}

