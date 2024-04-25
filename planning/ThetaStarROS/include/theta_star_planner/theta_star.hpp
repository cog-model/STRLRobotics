/*!
\file
\brief File contains ThetaStar class.
*/

#include <list>
#include <unordered_map>
#include <iostream>
#include <queue>
#define CN_EPS 0.00001

#include "geom.hpp"
#include "line_of_sight.hpp"
#include "map.hpp"
#include "environment_options.hpp"

#ifndef THETA_STAR_HPP
#define THETA_STAR_HPP

struct hash_pair
{
	template <class T1, class T2>
	size_t operator()(const std::pair<T1, T2>& p) const
	{
		auto hash1 = std::hash<T1>{}(p.first);
		auto hash2 = std::hash<T2>{}(p.second);
		return hash1 ^ hash2;
	}
};

struct Door{
    float x1, y1, x2, y2;

    Door(float x1_, float y1_, float x2_, float y2_): x1(x1_), y1(y1_), x2(x2_), y2(y2_) {};
};


/*!
 * \brief ThetaStar class implements Theta* algorithm, which creates path from start to goal position.
 */
class ThetaStar
{
    public:

        /*!
         * \brief ThetaStar constructor with parametes.
         * \param map Static environment map. It contains information about grid and static obstacles boundaries. More about it see in \ref index "Main page" and \ref Map "Map class".
         * \param options Environment and algorithm options. See \ref EnvironmentOptions "EnvironmentOptions class".
         * \param start Start position of agent.
         * \param goal Goal position of agent.
         * \param radius Radius of agent.
         */
        ThetaStar() = default;


        ThetaStar(const Map &map, const EnvironmentOptions &options, const Point &start, const Point &goal, const float &radius, const float &reach_radius);

        /*!
         * \brief ThetaStar copy constructor.
         * \param obj Object to copy.
         */
        ThetaStar(const ThetaStar &obj);

        /*!
         * \brief ThetaStar destructor.
         */
        ~ThetaStar() {map = nullptr; options = nullptr;}

        /*!
         * \brief Finds path from global start to global goal position using Theta* algorithm.
         * \return Success of pathfinding.
         */
        bool CreateGlobalPath();

        /*!
         * \brief Finds doors.
         */
        void FindDoors(std::vector<Door> door_coords, std::vector<int> &doors_found);

        /*!
         * \brief Method for cloning inheritors objects. Creates copy of object in memmory and return pointer to copy.
         * \return Pointer to copy of object
         */
        ThetaStar* Clone() const;

        /*!
         * \brief Assignment operator.
         * \param obj Object to assign.
         * \return Reference to assigned object.
         */
        ThetaStar & operator = (const ThetaStar &obj);

        std::list<Point> GetCurrPath()
        {
            return currPath;
        }

    private:

        //! \cond
        bool SearchPath(const Node &start, const Node &goal);
        bool StopCriterion() const;
        void AddOpen(Node newNode);
        Node FindMin();
        float ComputeHFromCellToCell(int i1, int j1, int i2, int j2) const;
        float Distance(int i1, int j1, int i2, int j2); //const;
        Node ResetParent(Node current, Node parent);
        void MakePrimaryPath(Node curNode);
        std::list<Node> FindSuccessors(Node curNode);
        bool CrossTwoSegments(Point p1, Point p2, Point p3, Point p4);

        std::list<Point> currPath;

        std::unordered_map<int, Node>   close;
        // std::vector<std::list<Node> >    open;
        // int                             openSize;

		bool (*compare)(const Node&, const Node &);
		std::priority_queue<Node, std::vector<Node>, decltype(compare)>			open_queue;
		std::unordered_map<std::pair<int, int>, Node, hash_pair>	open_table;
        int open_size;


        bool                            glPathCreated;
        LineOfSight                     visChecker;
        const Map *map;
        const EnvironmentOptions *options;
        Point glStart;
        Point glGoal;
        float radius, reach_radius, reach_radius_in_cells;

        //! \endcond



};


static bool g_max_comp(const Node &lhs, const Node &rhs)
{
	if (std::fabs(lhs.F - rhs.F) < CN_EPS)
	{
		if (std::fabs(lhs.g - rhs.g) < CN_EPS)
		{
			return lhs.i < rhs.i || lhs.i == rhs.i && lhs.j < rhs.j;
		}
		else
		{
			return  lhs.g - rhs.g > CN_EPS;
		}
	}
	return lhs.F - rhs.F < CN_EPS;
}

static bool g_min_comp(const Node &lhs, const Node &rhs)
{
	if (std::fabs(lhs.F - rhs.F) < CN_EPS)
	{
		if (std::fabs(lhs.g - rhs.g) < CN_EPS)
		{
			return lhs.i < rhs.i || lhs.i == rhs.i && lhs.j < rhs.j;
		}
		else
		{
			return lhs.g - rhs.g < CN_EPS;
		}
	}
	return lhs.F - rhs.F < CN_EPS;
}


static bool inline g_max_comp_pq(const Node &lhs, const Node &rhs)
{
	return !g_max_comp(lhs, rhs);
}

static bool inline g_min_comp_pq(const Node &lhs, const Node &rhs)
{
	return !g_min_comp(lhs, rhs);
}

#endif //THETA_STAR_HPP