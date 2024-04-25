/*!
\file
\brief File contains EnvironmentOptions class implementation.
*/


#include "theta_star_planner/environment_options.hpp"

EnvironmentOptions::EnvironmentOptions(const EnvironmentOptions &obj)
{
    this->metrictype = obj.metrictype;
    this->breakingties = obj.breakingties;
    this->allowsqueeze = obj.allowsqueeze;
    this->cutcorners = obj.cutcorners;
    this->hweight = obj.hweight;
}


EnvironmentOptions::EnvironmentOptions(int mt, bool bt, bool as, bool cc, float hw)
        :metrictype(mt), breakingties(bt), allowsqueeze(as), cutcorners(cc), hweight(hw) {}