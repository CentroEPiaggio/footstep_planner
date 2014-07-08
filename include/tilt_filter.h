#ifndef TILT_FILTER_H
#define TILT_FILTER_H

#include <kinematics_utilities.h>
#include "data_types.h"

using namespace planner;

class tilt_filter
{
    tilt_filter(double max_tilt_=0.5); //DEFAULT 30°
    ~tilt_filter();

    void set_max_tilt(double max_tilt_);

    void filter_normals(std::list<polygon_with_normals>& data);
    bool normal_is_in_bounds(pcl::PointXYZRGBNormal& normal);

    double max_tilt;
    double value;
public:

};

#endif //TILT_FILTER_H