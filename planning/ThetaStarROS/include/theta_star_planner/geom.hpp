/*!
\file
\brief File contains Node, Point, classes.
*/

#include <cmath>
#include <vector>
#include <string>
#include <cstddef>
#include <tuple>

#include "const.hpp"

#ifndef GEOM_HPP
#define GEOM_HPP

/*!
 * \brief The Node class defines a cell of grid (see \ref Map "Map class")
 * \ingroup ORCAStarLib
 */
class Node
{
    public:
        int     i; ///< The number of row. Part of (i,j) address of cell on grid.
        int     j; ///< The number of column. Part of (i,j) address of cell on grid.
        double  g; ///< g-value of node for A* based algorithms.
        double  H; ///< h-value of node for A* based algorithms.
        double  F;  ///< F = g + h. Value for A* based algorithms.
        Node    *parent; ///< The node from which the transition to this node was made. Value for A* based algorithms.

        /*!
         * \brief Node constructor with parameters.
         * \param i The number of row. Part of (i,j) address of cell on grid.
         * \param j The number of column. Part of (i,j) address of cell on grid.
         * \param p The node from which the transition to this node was made. Optional. Value for A* based algorithms.
         * \param g g-value of node for A* based algorithms. Optional.
         * \param h h-value of node for A* based algorithms. Optional.
         */
        Node(int i = 0, int j = 0, Node *p = nullptr, double g = 0, double h = 0)
            : i(i), j(j), g(g), H(h), F(g+h), parent(p){}

        /*!
         * \brief operator !=
         * \param other Object to compare
         * \return Inequality of nodes addresses
         */
        bool operator != (const Node &other) const ;

        /*!
         * \brief operator ==
         * \param another Object to compare
         * \return Equality of nodes addresses
         */
        bool operator == (const Node &another) const;
};

/*!
 * \brief The Point class defines a position (or euclidean vector from (0,0)) in 2D space.
 * \ingroup ORCAStarLib
 */
class Point
{
    public:
        /*!
         * \brief Point default constructor.
         */
        Point();

        /*!
         * \brief Point constructor with parameters.
         * \param x X-coordinate of the point.
         * \param y Y-coordinate of the point.
         */
        Point(float x, float y);

        /*!
         * \brief Point copy constructor.
         * \param obj An object to copy.
         */
        Point(const Point &obj);

        /*!
         * \brief Default destructor.
         */
        ~Point() = default;

        /*!
         * \brief Returns X-coordinate of the point.
         * \return X-coordinate of the point.
         */
        float X() const;

        /*!
         * \brief Returns Y-coordinate of the point.
         * \return Y-coordinate of the point.
         */
        float Y() const;

        /*!
         * \brief Creates STL pair (x,y) from point.
         * \return STL pair (x,y).
         */
        std::pair<float, float> GetPair();

        /*!
         * \brief Computes scalar product of vectors (this * anoher)
         * \param another Second operand of scalar product
         * \return Result of scalar product of vectors (this * anoher)
         */
        float ScalarProduct(const Point &another) const;

        /*!
         * \brief Computes euclidean norm of vector.
         * \return Euclidean norm of vector.
         */
        float EuclideanNorm() const;

        /*!
         * \brief Computes squared euclidean norm of vector.
         * \return Squared euclidean norm of vector.
         */
        float SquaredEuclideanNorm() const;

        /*!
         * \brief Computes the determinant of matrix.
         * \details
         * <p align="center">
         * <img width="300" src="./images/det.jpeg" alt="Map sheme" >
         * </p>
         * \param another Second column of matrix
         * \return The determinant of matrix.
         */
        float Det(Point another) const;

        /*!
         * \brief Creates STL string, which contains x,y values.
         * \return STL string, which contains x,y values.
         */
        std::string ToString() const;

        /*!
         * \brief operator -
         * \param another Second operand
         * \return (this.x - another.x, this.y - another.y)
         */
        Point operator - (const Point &another) const;

        /*!
         * \brief operator +
         * \param another Second operand
         * \return (this.x + another.x, this.y + another.y)
         */
        Point operator + (const Point &another) const;

        /*!
         * \brief operator ==
         * \param another Object to compare.
         * \return (x, y) == (another.x, another.y)
         */
        bool operator == (const Point &another) const;

        /*!
         * \brief operator *
         * \param k Scalar multiplier.
         * \return (k * x, k * y)
         */
        Point operator * (float k) const;

        /*!
         * \brief operator /
         * \param k Scalar divider.
         * \return ( x / k, y / k)
         */
        Point operator / (float k) const;

        /*!
         * \brief operator -
         * \return (-x, -y)
         */
        Point operator - () const;

        /*!
         * \brief Assignment operator
         * \param obj Object to assign
         * \return Reference to object.
         */
        Point & operator= (const Point &obj);



    private:
        float x; ///< X-coordinate of the point.
        float y; ///< Y-coordinate of the point.
    //TODO float z; Z-coordinate of the point.
};



struct Quaternion
{
    double x;
    double y;
    double z;
    double w;
};


/*********************************************************
 *                Methods Implementations                *
 *********************************************************/


inline bool Node::operator == (const Node &another) const
{
    return i == another.i && j == another.j;
}

inline bool Node::operator != (const Node &other) const
{
    return i != other.i || j != other.j;
}


inline float Point::ScalarProduct(const Point &another) const
{
    return this->x * another.x + this->y * another.y;
}


inline Point Point::operator - (const Point &another) const
{
    return {this->x - another.x, this->y - another.y};
}


inline Point Point::operator + (const Point &another) const
{
    return {this->x + another.x, this->y + another.y};
}


inline Point Point::operator * (float k) const
{
    return {this->x * k, this->y * k};
}


inline Point Point::operator /(float k) const
{
    const float invK = 1.0f / k;
    return {this->x * invK, this->y * invK};
}


inline float Point::SquaredEuclideanNorm() const
{
    return this->ScalarProduct(*this);
}


inline float Point::EuclideanNorm() const
{
    return std::sqrt(this->ScalarProduct(*this));
}


inline float Point::Det(Point another) const
{
    return (this->x * another.y - this->y * another.x);
}


inline bool Point::operator ==(const Point &another) const
{
    return (this->x == another.x) && (this->y == another.y);
}


inline Point Point::operator-() const
{
    return Point(-this->x, -this->y);
}


inline Point& Point::operator = (const Point &obj)
{
    x = obj.x;
    y = obj.y;
    return *this;
}


#endif 