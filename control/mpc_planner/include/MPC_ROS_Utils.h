#ifndef DATA_MPC_UTILS
#define DATA_MPC_UTILS
#include "lib.h"

class MPC_ROS_Utils 
{
   
public:
    vector<double> x_filter , y_filter , theta_filter_segments, length_filter_segments;
    double margin_length_scan , margin_width_scan , starting_width_scan , step_scan , scan_tol;
    nav_msgs::OccupancyGrid  grid_map;
    double solver_params[ROBOT_MODEL_NP];
    int time_circle_obst;
    X0 x0;
    vector<double> x_filter_circle , y_filter_circle , theta_filter_segments_circle, length_filter_segments_circle;
    bool map_received;
    
    MPC_ROS_Utils();
    
    void generate_obstacles_circles();
    
    int index_cell_at_coordintes(double x_s , double y_s);
    
    double distance_point_to_line(double x1, double x2 , double y1 , double y2, double x_point , double y_point);
    
};

#endif
