/*!
\file
\brief File contains Map class implementation.
*/
#include <iostream>
#include "ros/ros.h"
#include "theta_star_planner/map.hpp"

Map::Map()
{
    cellSize = 0;
    height = 0;
    width = 0;
    unknownIsObstacle = true;
}


Map::Map(const Map &&obj)
{
    cellSize = obj.cellSize;
    height = obj.height;
    width = obj.width;
    grid = std::move(obj.grid);
    x.resize(grid.size());
    y.resize(grid.size());
    unknownIsObstacle = obj.unknownIsObstacle;
}


bool Map::CellIsObstacle(int i, int j) const
{
    // std::cout << " obst " << (int) grid[(height - 1 - i) * width + j] << "\n";
    if ((i >= height) || (i < 0) || (j >= width) || (j < 0))
    {
        return true;
    }
    if (unknownIsObstacle)
    {
        return (grid[i * width + j] > 1) || (grid[i * width + j] == -1);
    }
    return (grid[i * width + j] > 1);
}


bool Map::CellIsCloseToObstacle(int i, int j) const
{
    return (grid[i * width + j] > 0);
}


bool Map::CellIsUnknown(int i, int j) const
{
    return (grid[i * width + j] < 0);
}


bool Map::CellIsTraversable(int i, int j) const
{
    return (grid[i * width + j] == 0);
}


bool Map::CellOnGrid(int i, int j) const
{
    return (i < height && i >= 0 && j < width && j >= 0);
}

unsigned int Map::GetHeight() const
{
    return height;
}


unsigned int Map::GetWidth() const
{
    return width;
}


float Map::GetCellSize() const
{
    return cellSize;
}


std::vector<signed char> Map::GetGrid()
{
    return grid;
}


Node Map::GetClosestNode(const Point &point) const
{
    Node res;
    res.i = static_cast<int>(((point.Y() - originPosition.Y()) / cellSize));
    res.j = static_cast<int>(((point.X() - originPosition.X()) / cellSize));

    if(res.i < 0)
    {
        res.i = 0;
    }
    if(res.i > height - 1)
    {
        res.i = height - 1;
    }
    if(res.j < 0)
    {
        res.j = 0;
    }
    if(res.j > width - 1)
    {
        res.j = width - 1;
    }

    return res;
}


Point Map::GetPoint(const Node &node) const
{

    auto pos_x = originPosition.X() + (node.j + 0.5f) * cellSize;
    auto pos_y = originPosition.Y() + (node.i + 0.5f) * cellSize;

    return {pos_x, pos_y};
}

Map& Map::operator= (const Map &&obj)
{
    if(this != &obj)
    {
        cellSize = obj.cellSize;
        height = obj.height;
        width = obj.width;
        grid = std::move(obj.grid);
        x.resize(grid.size());
        y.resize(grid.size());
    }
    return *this;
}


void Map::InflateObstacles(float r1, float r2)
{
    cells1.clear();
    cells2.clear();
    int add_x, add_y, num = r2 + 0.5 - CN_EPS;
    for(int x = -num; x <= +num; x++)
        for(int y = -num; y <= +num; y++)
        {
            add_x = x != 0 ? 1 : 0;
            add_y = y != 0 ? 1 : 0;
            if((pow(2*abs(x) - add_x, 2) + pow(2*abs(y) - add_y, 2)) < pow(2*r1, 2))
                cells1.push_back({x, y});
            if ((pow(2*abs(x) - add_x, 2) + pow(2*abs(y) - add_y, 2)) < pow(2*(r2 - r1), 2))
                cells2.push_back({x, y});
        }
    if(cells1.empty())
        cells1.push_back({0,0});
    if(cells2.empty())
        cells2.push_back({0,0});

    std::vector<signed char> grid_inflated1, grid_inflated2;
    grid_inflated1.reserve(grid.size());
    for (int i = 0; i < grid.size(); i++)
        grid_inflated1.push_back(grid[i]);
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
            if (grid[i * width + j] > 0)
            {
                for (int k = 0; k < cells1.size(); k++)
                    if (CellOnGrid(i + cells1[k].first, j + cells1[k].second))
                        grid_inflated1[(i + cells1[k].first) * width + j + cells1[k].second] = 100;
            }
    grid_inflated2.reserve(grid.size());
    for (int i = 0; i < grid.size(); i++)
        grid_inflated2.push_back(grid_inflated1[i]);
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
            if (grid_inflated1[i * width + j] > 0)
            {
                for (int k = 0; k < cells2.size(); k++)
                    if (CellOnGrid(i + cells2[k].first, j + cells2[k].second) && (!CellIsUnknown(i + cells2[k].first, j + cells2[k].second)) && (grid_inflated1[(i + cells2[k].first) * width + j + cells2[k].second] <= 0))
                        grid_inflated2[(i + cells2[k].first) * width + j + cells2[k].second] = 1;
            }
    for (int i = 0; i < grid.size(); i++)
        grid[i] = grid_inflated2[i];
 //   ROS_INFO("Number of obstacles: %d", n_obst);
}


 void Map::Update(float cellSize, size_t height, size_t width, Point originPos, Quaternion originOr, const std::vector<signed char> &grid, float inflation_radius1, float inflation_radius2)
 {
    this->cellSize = cellSize;
    this->height = height;
    this->width = width;
    this->originPosition = originPos;
    this->originOrientation = originOr;
    this->grid = grid;
    this->x.resize(grid.size());
    this->y.resize(grid.size());
 //   ROS_INFO("Inflation radius and cell size: %f %f", inflation_radius, cellSize);
    float r1 = inflation_radius1 / cellSize;
    float r2 = inflation_radius2 / cellSize;
 //   ROS_INFO("Inflation radius in cells: %f", inflation_cells);
    double time_before = ros::Time::now().toSec();
    InflateObstacles(r1, r2);
    double time_after = ros::Time::now().toSec();
    //ROS_WARN("Time to inflate obstacles: %f", time_after - time_before);
 }