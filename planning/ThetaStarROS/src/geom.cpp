/*!
\file
\brief File contains Node, Point classes implementations.
*/


#include "theta_star_planner/geom.hpp"


Point::Point()
{
    x = 0.0;
    y = 0.0;
}


Point::Point(float x, float y)
{
    this->x = x;
    this->y = y;
}


float Point::X() const
{
    return x;
}


float Point::Y() const
{
    return y;
}


Point::Point(const Point &obj)
{
    this->x = obj.x;
    this->y = obj.y;
}


std::pair<float, float> Point::GetPair()
{
    return {x, y};
}

std::string Point::ToString() const
{
    return "x: " + std::to_string(x) + "\ty: "
           + std::to_string(y) + "\n";

}

