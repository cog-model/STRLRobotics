#include "MPC_ROS_Utils.h"

MPC_ROS_Utils::MPC_ROS_Utils()
{
}

void MPC_ROS_Utils::generate_obstacles_circles()
{
    auto start = high_resolution_clock::now();
    x_filter[0] = x_filter[0] - margin_length_scan * cos(theta_filter_segments[0]);
    y_filter[0] = y_filter[0] - margin_length_scan * sin(theta_filter_segments[0]);
    length_filter_segments[0] = length_filter_segments[0] + margin_length_scan;

    bool new_segment , ability_to_modify;
    bool last_scan_found = false;
    double last_x_scan , last_y_scan;
    double d_nearest;
    double x_scan , y_scan , x_nearest , y_nearest;
    double x_start = 0 , y_start = 0;
    int index_scan;
    double d0 , d_current_obst_to_path , d_last_obst_to_path;
    vector<double> obst_x , obst_y;
    vector<int> index_taken_obst;

    for(int i = 0 ; i < x_filter.size()-1 ; i++) 
    {
        new_segment = true;
        d_nearest = 1;
        for(double k = 0; k<(length_filter_segments[i]+margin_length_scan); k+=step_scan)
        {
            x_start = x_filter[i]+k*cos(theta_filter_segments[i]);
            y_start = y_filter[i]+k*sin(theta_filter_segments[i]);
            for(double j=starting_width_scan;j<=margin_width_scan;j+=step_scan)
            {
                x_scan = x_start + j*cos(theta_filter_segments[i]-M_PI/2);
                y_scan = y_start + j*sin(theta_filter_segments[i]-M_PI/2);
                index_scan = index_cell_at_coordintes(x_scan , y_scan);
                if (index_scan > 0)
                {
                    if (std::find(index_taken_obst.begin(), index_taken_obst.end(), index_scan) == index_taken_obst.end())
                    {
                        if (new_segment)
                        {
                            index_taken_obst.push_back(index_scan);
                            obst_x.push_back(x_scan);  
                            obst_y.push_back(y_scan);
                            ability_to_modify = true;
                            new_segment = false;
                        }
                        else
                        {
                            d_current_obst_to_path = distance_point_to_line(x_filter[i],x_filter[i+1],y_filter[i],y_filter[i+1],x_scan , y_scan);
                            d_last_obst_to_path = distance_point_to_line(x_filter[i],x_filter[i+1],y_filter[i],y_filter[i+1],obst_x.back() , obst_y.back());
                            d0 = sqrt(pow((x_scan-obst_x.back()),2)+pow((y_scan-obst_y.back()),2));
                            if(d0<scan_tol && ability_to_modify && d_current_obst_to_path <= d_last_obst_to_path)
                            {
                                index_taken_obst.back() = index_scan;
                                obst_x.back() = x_scan;
                                obst_y.back() = y_scan;
                                ability_to_modify = false;
                            }
                            else if(d0>=scan_tol)
                            {
                                index_taken_obst.push_back(index_scan);
                                obst_x.push_back(x_scan);  
                                obst_y.push_back(y_scan);
                                ability_to_modify = true;
                            }     
                            else if(d_current_obst_to_path < d_nearest)
                            {
                                d_nearest = d_current_obst_to_path;
                                x_nearest = x_scan;
                                y_nearest = y_scan;
                            }
                        }
                        last_x_scan = x_scan;
                        last_y_scan = y_scan;
                        last_scan_found = true;                       
                    }
                    break;
                }
                else if(j>=margin_width_scan && last_scan_found && !new_segment)
                {
                    index_taken_obst.push_back(index_scan);
                    obst_x.push_back(last_x_scan);  
                    obst_y.push_back(last_y_scan);
                    ability_to_modify = false;
                    last_scan_found = false;
                }
            }
        }
        index_taken_obst.push_back(index_scan);
        obst_x.push_back(x_nearest);  
        obst_y.push_back(y_nearest);
        ability_to_modify = false;
    }

    for(int i = 0 ; i <= x_filter.size()-2 ; i++)
    {
        new_segment = true;
        d_nearest = 1;
        for(double k = 0; k<(length_filter_segments[i]+margin_length_scan); k+=step_scan)
        {
            x_start = x_filter[i]+k*cos(theta_filter_segments[i]);
            y_start = y_filter[i]+k*sin(theta_filter_segments[i]);
            for(double j=starting_width_scan;j<=margin_width_scan;j+=step_scan)
            {
                x_scan = x_start + j*cos(theta_filter_segments[i]+M_PI/2);
                y_scan = y_start + j*sin(theta_filter_segments[i]+M_PI/2);
                index_scan = index_cell_at_coordintes(x_scan , y_scan);
                if (index_scan > 0)
                {
                    if ( std::find(index_taken_obst.begin(), index_taken_obst.end(), index_scan) == index_taken_obst.end())
                    {
                        if (new_segment)
                        {
                            index_taken_obst.push_back(index_scan);
                            obst_x.push_back(x_scan);  
                            obst_y.push_back(y_scan);
                            ability_to_modify = true;
                            new_segment = false;
                        }
                        else
                        {
                            d_current_obst_to_path = distance_point_to_line(x_filter[i],x_filter[i+1],y_filter[i],y_filter[i+1],x_scan , y_scan);
                            d_last_obst_to_path = distance_point_to_line(x_filter[i],x_filter[i+1],y_filter[i],y_filter[i+1],obst_x.back() , obst_y.back());
                            d0 = sqrt(pow((x_scan-obst_x.back()),2)+pow((y_scan-obst_y.back()),2));
                            if(d0<scan_tol && ability_to_modify && d_current_obst_to_path <= d_last_obst_to_path)
                            {
                                index_taken_obst.back() = index_scan;
                                obst_x.back() = x_scan;
                                obst_y.back() = y_scan;
                                ability_to_modify = false;
                            }
                            else if(d0>=scan_tol)
                            {
                                index_taken_obst.push_back(index_scan);
                                obst_x.push_back(x_scan);  
                                obst_y.push_back(y_scan);
                                ability_to_modify = true;
                            }     
                            else if(d_current_obst_to_path < d_nearest)
                            {
                                d_nearest = d_current_obst_to_path;
                                x_nearest = x_scan;
                                y_nearest = y_scan;
                            }
                        }
                        last_x_scan = x_scan;
                        last_y_scan = y_scan;
                        last_scan_found = true;                       

                    }
                    break;
                }
                else if(j>=margin_width_scan && last_scan_found && !new_segment)
                {
                    index_taken_obst.push_back(index_scan);
                    obst_x.push_back(last_x_scan);  
                    obst_y.push_back(last_y_scan);
                    ability_to_modify = false;
                    last_scan_found = false;
                }
            }
        }
        index_taken_obst.push_back(index_scan);
        obst_x.push_back(x_nearest);  
        obst_y.push_back(y_nearest);
        ability_to_modify = false;
    }
    int k = 0;
    for(int i = 0 ; i<obst_x.size(); i++)
    {
        solver_params[k] = obst_x[i];
        solver_params[k+1] = obst_y[i];
        k = k + 2;
    }
    for(int i = k ; i<ROBOT_MODEL_NP ; i++)
    {
        solver_params[i] = 100;
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    time_circle_obst = int(duration.count());
}
  
int MPC_ROS_Utils::index_cell_at_coordintes(double x_s , double y_s)
{
    int col = round((x_s - grid_map.info.origin.position.x)/grid_map.info.resolution);
    int row = round((y_s - grid_map.info.origin.position.y)/grid_map.info.resolution);
    int index_s = row * grid_map.info.width + col;
    if (grid_map.data[index_s]==100)
        return index_s;
    else
        return 0;
}

double MPC_ROS_Utils::distance_point_to_line(double x1, double x2 , double y1 , double y2, double x_point , double y_point)
{
    double d_inter = abs((x2-x1)*(y1-y_point) - (x1-x_point)*(y2-y1))/sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    return d_inter;
}
