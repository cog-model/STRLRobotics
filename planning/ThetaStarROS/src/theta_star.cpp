/*!
\file
\brief File contains ThetaStar class implementation.
*/

#include "theta_star_planner/theta_star.hpp"
#include "ros/ros.h"
#include <iostream>

ThetaStar::ThetaStar(const Map &map, const EnvironmentOptions &options, const Point &start, const Point &goal, const float &radius, const float& reach_radius) : 
           map(&map), options(&options), glStart(start), glGoal(goal), radius(radius), reach_radius(reach_radius)
{
    currPath = std::list<Point>();
    close = std::unordered_map<int, Node>();
    compare = &g_max_comp_pq;
	open_queue = std::priority_queue<Node, std::vector<Node>, bool(*)(const Node&, const Node&)>(compare);
	open_table = std::unordered_map<std::pair<int, int>, Node, hash_pair>();
    open_size = 0;
    glPathCreated = false;
    visChecker = LineOfSight(this->radius / map.GetCellSize());
    reach_radius_in_cells = reach_radius / map.GetCellSize();
}


ThetaStar::ThetaStar(const ThetaStar &obj)  : map(obj.map), options(obj.options), glStart(obj.glStart), glGoal(obj.glGoal), radius(obj.radius), reach_radius(obj.reach_radius)
{
    currPath = obj.currPath;
    close = obj.close;
    open_table = obj.open_table;
    open_queue = obj.open_queue;
    open_size = obj.open_size;
    glPathCreated = obj.glPathCreated;
    visChecker = obj.visChecker;
    compare = obj.compare;

}



bool ThetaStar::SearchPath(const Node &start, const Node &goal)
{
    close.clear();
    open_size = 0;
	open_queue = std::priority_queue<Node, std::vector<Node>, bool(*)(const Node&, const Node&)>(compare);
	open_table.clear();
    //std::cout << start.i << " " << start.j << std::endl;
    //std::cout << map->CellIsObstacle(start.i, start.j) << std::endl;
    //std::cout << goal.i << " " << goal.j << std::endl;
    //std::cout << map->CellIsObstacle(goal.i, goal.j) << std::endl;
    Node curNode;
    curNode.i = start.i;
    curNode.j = start.j;
    curNode.g = 0;
    curNode.H = ComputeHFromCellToCell(curNode.i, curNode.j, goal.i, goal.j);
    curNode.F = options->hweight * curNode.H;
    curNode.parent = nullptr;
    AddOpen(curNode);
    int closeSize = 0;
    bool pathfound = false;
    while (!StopCriterion())
    {
        curNode = FindMin();
        close.insert({curNode.i * map->GetWidth() + curNode.j, curNode});
        closeSize++;
        if (curNode.H < reach_radius_in_cells)
        {
            pathfound = true;
            break;
        }
        std::list<Node> successors = FindSuccessors(curNode);
        std::list<Node>::iterator it = successors.begin();
        auto parent = &(close.find(curNode.i * map->GetWidth() + curNode.j)->second);
        while (it != successors.end()) {
            it->parent = parent;
            it->H = ComputeHFromCellToCell(it->i, it->j, goal.i, goal.j);
            *it = ResetParent(*it, *it->parent);
            it->F = it->g + options->hweight * it->H;
            AddOpen(*it);
            it++;
        }
    }

    if (pathfound)
    {
        //std::cout << "Path found. Close size is " << close.size() << "\n";
        MakePrimaryPath(curNode);
    }
    return pathfound;
}


bool ThetaStar::StopCriterion() const
{
    if (!open_size)
    {
        return true;
    }
    return false;
}


void ThetaStar::AddOpen(Node newNode)
{
    auto ij = std::pair<int, int>(newNode.i, newNode.j);
	auto old_node_iter = open_table.find(ij);
    //std::cout << "Old node iter: " << std::distance(old_node_iter, open_table.begin()) << std::endl;
	if(old_node_iter == open_table.end())
	{
		open_table[ij] = newNode;
		open_queue.push(newNode);
		open_size++;
	}
    else if(old_node_iter->second.g > newNode.g)
	{
		open_table[ij] = newNode;
		open_queue.push(newNode);
	}
}


Node ThetaStar::FindMin()
{
    Node min = open_queue.top();
    open_queue.pop();

	auto ij = std::pair<int, int>(min.i, min.j);
    while (open_table.find(ij) == open_table.end())
	{
    	min = open_queue.top();
		open_queue.pop();
		ij = std::pair<int, int>(min.i, min.j);
	}
    open_table.erase(ij);
    open_size--;

    return min;
}


float ThetaStar::ComputeHFromCellToCell(int i1, int j1, int i2, int j2) const
{
    switch (options->metrictype)
    {
        case CN_SP_MT_EUCL:
            return static_cast<float>(sqrt((i2 - i1)*(i2 - i1)+(j2 - j1)*(j2 - j1)));
        case CN_SP_MT_DIAG:
            return static_cast<float>(abs(abs(i2 - i1) - abs(j2 - j1)) + sqrt(2) * (std::min(abs(i2 - i1),abs(j2 - j1))));
        case CN_SP_MT_MANH:
            return (abs(i2 - i1) + abs(j2 - j1));
        case CN_SP_MT_CHEB:
            return std::max(abs(i2 - i1),abs(j2 - j1));
        default:
            return 0;
    }
}


Node ThetaStar::ResetParent(Node current, Node parent)
{

    if (parent.parent == nullptr)
        return current;
    if(current == *parent.parent)
        return current;

    if (visChecker.checkLine(parent.parent->i, parent.parent->j, current.i, current.j, *map))
    {
        current.g = parent.parent->g + Distance(parent.parent->i, parent.parent->j, current.i, current.j);
        current.parent = parent.parent;
        return current;
    }
    return current;
}


float ThetaStar::Distance(int i1, int j1, int i2, int j2) //const
{
    float result = static_cast<float>(sqrt(pow(i1 - i2, 2.0f) + pow(j1 - j2, 2.0f)));
    /*if (map->CellIsUnknown(i1, j1) || map->CellIsUnknown(i2, j2))
    {
        result *= 10;
    }*/
    return result;
}


void ThetaStar::MakePrimaryPath(Node curNode)
{


    Node *current = &curNode;
    while(current != nullptr)
    {
        currPath.push_front(map->GetPoint(*current));
        current = current->parent;
    }
}


std::list<Node> ThetaStar::FindSuccessors(Node curNode)
{
    Node newNode;
    std::list<Node> successors;
    for (int i = -1; i <= +1; i++)
        for (int j = -1; j <= +1; j++)
            if ((i != 0 || j != 0) && map->CellOnGrid(curNode.i + i, curNode.j + j) &&
                (visChecker.checkTraversability(curNode.i + i, curNode.j + j, *map)))
            {
                if (i != 0 && j != 0)
                {
                    if (!options->cutcorners)
                    {
                        if (map->CellIsObstacle(curNode.i, curNode.j + j) ||
                            map->CellIsObstacle(curNode.i + i, curNode.j))
                            continue;
                    }
                    else if (!options->allowsqueeze)
                    {
                        if (map->CellIsObstacle(curNode.i, curNode.j + j) &&
                            map->CellIsObstacle(curNode.i + i, curNode.j))
                            continue;
                    }
                }
                if (close.find((curNode.i + i) * map->GetWidth() + curNode.j + j) == close.end())
                {
                    newNode.i = curNode.i + i;
                    newNode.j = curNode.j + j;
                    float cost = 1;
                    if (map->CellIsUnknown(curNode.i + i, curNode.j + j))
                        cost = 20;
                    if (map->CellIsCloseToObstacle(curNode.i + i, curNode.j + j))
                        cost = 10;
                    if(i == 0 || j == 0)
                        newNode.g = curNode.g + cost;
                    else
                        newNode.g = curNode.g + cost * sqrt(2);
                    successors.push_front(newNode);
                }
            }
    return successors;
}


bool ThetaStar::CrossTwoSegments(Point p1, Point p2, Point p3, Point p4)
{
    //std::cout << "First segment: " << p1.X() << ' ' << p1.Y() << ' ' << p2.X() << ' ' << p2.Y() << std::endl;
    //std::cout << "Second segment: " << p3.X() << ' ' << p3.Y() << ' ' << p4.X() << ' ' << p4.Y() << std::endl;
    float eps = 1e-5;
    float a1 = p1.Y() - p2.Y();
    float b1 = p2.X() - p1.X();
    float c1 = -(a1 * p1.X() + b1 * p1.Y());
    //std::cout << "a1 b1 c1: " << a1 << ' ' << b1 << ' ' << c1 << std::endl;
    float a2 = p3.Y() - p4.Y();
    float b2 = p4.X() - p3.X();
    float c2 = -(a2 * p3.X() + b2 * p3.Y());
    //std::cout << "a2 b2 c2: " << a2 << ' ' << b2 << ' ' << c2 << std::endl;
    if (abs(a1 * b2 - b1 * a2) < eps)
    {
        return false;
    }
    float x_cross = (c1 * b2 - b1 * c2) / (b1 * a2 - b2 * a1);
    float y_cross = (c2 * a1 - c1 * a2) / (b1 * a2 - b2 * a1);
    //std::cout << "x_cross y_cross: " << x_cross << ' ' << y_cross << std::endl;
    if ((x_cross >= std::min(p1.X(), p2.X()) - eps) && (x_cross <= std::max(p1.X(), p2.X()) + eps) && \
       (y_cross >= std::min(p1.Y(), p2.Y()) - eps) && (y_cross <= std::max(p1.Y(), p2.Y()) + eps) && \
       (x_cross >= std::min(p3.X(), p4.X()) - eps) && (x_cross <= std::max(p3.X(), p4.X()) + eps) && \
       (y_cross >= std::min(p3.Y(), p4.Y()) - eps) && (y_cross <= std::max(p3.Y(), p4.Y()) + eps))
    {
        //std::cout << "TRUE" << std::endl;
        return true;
    }
    //std::cout << "FALSE" << std::endl;
    return false;
}


void ThetaStar::FindDoors(std::vector<Door> door_coords, std::vector<int> &doors_found)
{
    std::vector<Point> path;
    for (auto pt: currPath)
    {
        path.push_back(pt);
    }
    for (int i = 0; i < path.size(); i++)
    {
        Point p1;
        if (i == 0)
        {
            p1 = glStart;
        }
        else
        {
            p1 = path[i - 1];
        }
        Point p2 = path[i];
        for (int j = 0; j < door_coords.size(); j++)
        {
            Point p3(door_coords[j].x1, door_coords[j].y1);
            Point p4(door_coords[j].x2, door_coords[j].y2);
            if (CrossTwoSegments(p1, p2, p3, p4))
            {
                doors_found.push_back(j);
            }
        }
    }
}


bool ThetaStar::CreateGlobalPath()
{
    if(!glPathCreated)
    {
        Node start = map->GetClosestNode(glStart);
        Node goal  = map->GetClosestNode(glGoal);
        glPathCreated = SearchPath(start, goal);
    }
    return  glPathCreated;
}

ThetaStar &ThetaStar::operator = (const ThetaStar &obj)
{
    if(this != &obj)
    {
        map = obj.map; 
        options = obj.options;
        glStart = obj.glStart;
        glGoal = obj.glGoal;
        radius = obj.radius;
        currPath = obj.currPath;
        close = obj.close;
        open_queue = obj.open_queue;
        open_table = obj.open_table;
        open_size = obj.open_size;
        glPathCreated = obj.glPathCreated;
        visChecker = obj.visChecker;
        compare = obj.compare;
    }
    return *this;
}

ThetaStar *ThetaStar::Clone() const
{
    return new ThetaStar(*this);
}
