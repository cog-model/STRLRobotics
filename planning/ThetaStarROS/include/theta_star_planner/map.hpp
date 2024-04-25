/*!
\file
\brief File contains Map class.
*/


#include <string>
#include <vector>

#include "geom.hpp"


#ifndef ORCA_MAP_H
#define ORCA_MAP_H


/*!
 * \brief Map class describes static environment.
 * \details This class contains main information about static environment.
 * The map is set in the form of a grid, are also described in the form of polygons.
 * The grid is a rectangular matrix of square cells, each cell can be traversable or not.
 * The size of cell side is set by the value of cellSize.
 * The address of each cell on the grid is specified by a pair (i, j),
 * where i is the row number of the matrix, j is the column number of the matrix
 * The environment description is specified in the orthogonal coordinates.
 * Every obstacle in form of polygon is vector of points in orthogonal coordinates.
 * Vertices of the simple obstacle shold be listed in counterclockwise order, vertices of boundary in clockwise order.
 * An illustration of the environment is presented in the figure below.
 *
 * <p align="center">
 * <img width="500" src="./images/map.png" alt="Map sheme" >
 * </p>
 *
 * \ingroup ORCAStarLib
 */
class Map
{
    public:
        /*!
         * \brief Map default constructor.
         */
        Map();


        /*!
         * \brief Map copy constructor.
         * \param obj Object to copy.
         */
        Map(const Map &&obj);


        /*!
         * \brief Map destructor.
         */
        ~Map() = default;

        /*!
         * \brief Check Cell (i,j) is obstacle
         * \param i Row number in the matrix.
         * \param j Column number in the matrix
         * \return True if obstacle, otherwise false.
         */
        bool CellIsObstacle(int i, int j) const;

        /*!
         * \brief Check Cell (i,j) is inside grid.
         * \param i Row number in the matrix.
         * \param j Column number in the matrix
         * \return True if inside grid, otherwise false.
         */
        bool CellOnGrid(int i, int j) const;

        /*!
         * \brief Check Cell (i,j) is traversable
         * \param i Row number in the matrix.
         * \param j Column number in the matrix
         * \return False if obstacle, otherwise true.
         */
        bool CellIsTraversable(int i, int j) const;

        bool CellIsCloseToObstacle(int i, int j) const;

        bool CellIsUnknown(int i, int j) const;

        /*!
         * \brief Return height of grid.
         * \return Number of rows in grid matrix
         */
        unsigned int GetHeight() const;

        /*!
         * \brief Return width of grid.
         * \return Number of columns in grid matrix
         */
        unsigned int GetWidth() const;

        /*!
         * \brief Return grid cells.
         * \return Vector with all map cells
         */
        std::vector<signed char> GetGrid();

        /*!
         * \brief Return size of cell side.
         * \return Size of cell side.
         */
        float GetCellSize() const;

        /*!
         * \brief Return cell to which point in orthogonal coordinates belong.
         * \param point Point in orthogonal coordinates .
         * \return Cell of grid matrix.
         */
        Node GetClosestNode(const Point &point) const;

        /*!
         * \brief Return center of grid cell in orthogonal coordinates.
         * \param node Grid cell.
         * \return Position of grid cell center in orthogonal coordinates.
         */
        Point GetPoint(const Node &node) const;

        /*!
         * \brief Assignment operator.
         * \param obj Map to assignment.
         * \return Reference to map.
         */
        Map & operator= (const Map &&obj);

        void InflateObstacles(float r1, float r2);

        /*!
         * \brief Update grid and other map parameters.
         * \param cellSize size of one grid cell in global coordinate system.
         * \param height height of grid.
         * \param width width of grid.
         * \param originPos position of origin point in global coordinate system.
         * \param originOr orientation of origin point in global coordinate system.
         * \param grid occupacy grid.
         */
        void Update(float cellSize, size_t height, size_t width, Point originPos, Quaternion originOr, const std::vector<signed char> &grid, float inflation_radius1, float inflation_radius2);

        Map(const Map &obj) = delete;
        Map & operator= (const Map &obj) = delete;
        bool unknownIsObstacle;

    private:
        //!\cond
        float cellSize;
        size_t height;
        size_t width;
        std::vector<std::pair<int, int>> cells1, cells2;
        std::vector<signed char> grid;
        std::vector<int> x;
        std::vector<int> y;
        std::vector<int> door_pair;
        std::vector<std::pair<int, int> > door_corner_indices;
        Point originPosition;
        Quaternion originOrientation;
        //!\endcond
};


#endif //ORCA_MAP_H