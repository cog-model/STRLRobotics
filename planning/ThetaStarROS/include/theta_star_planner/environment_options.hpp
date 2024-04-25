/*!
\file
\brief File contains EnvironmentOptions class.
*/

#include "const.hpp"

#ifndef ENVIRONMENT_OPTIONS_HPP
#define ENVIRONMENT_OPTIONS_HPP

/*!
* \brief Class EnvironmentOptions contains environment and algoritms parameters
*/
class EnvironmentOptions
{
    public:
        /*!
      * \brief EnvironmentOptions default constructor.
      */
        EnvironmentOptions() = default;
        /*!
         * \brief EnvironmentOptions copy constructor.
         * \param obj Object to copy.
         */
        EnvironmentOptions(const EnvironmentOptions &obj);

        /*!
         * \brief EnvironmentOptions constructor with parametes.
         * \param mt Heurostic type for Theta*. Can be chosen Diagonal (0), Manhattan (1), Euclidean (2), Chebyshev (3) distance.
         * \param bt Option that defines the priority in OPEN list for nodes with equal f-values.
         * \param as Option that allows to move through "bottleneck"
         * \param cc Option that allows to make diagonal moves, when one adjacent cell is untraversable.
         * \param hw Option that defines the weight of the heuristic function.
         */
        EnvironmentOptions(int mt, bool bt, bool as, bool cc, float hw);

        int     metrictype;     ///< Heurostic type for Theta*. Can be chosen Diagonal (0), Manhattan (1), Euclidean (2), Chebyshev (3) distance.
        bool    breakingties;   ///< Option that defines the priority in OPEN list for nodes with equal f-values.
        bool    allowsqueeze;   ///< Option that allows to move through "bottleneck".
        bool    cutcorners;     ///< Option that allows to make diagonal moves, when one adjacent cell is untraversable.
        float   hweight;        ///< Option that defines the weight of the heuristic function.

};

#endif