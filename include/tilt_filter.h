#ifndef TILT_FILTER_H
#define TILT_FILTER_H

#include <kdl/frames.hpp>
#include "data_types.h"
#include <list>

using namespace planner;

class tilt_filter
{
public:
    tilt_filter(double max_tilt_=0.5); //DEFAULT 30°
    ~tilt_filter();
    void filter_normals(std::list<polygon_with_normals>& data);
    void filter_single_normals(std::list<polygon_with_normals>& data);
    void set_world(KDL::Frame World_Camera_);

private:
    void set_max_tilt(double max_tilt_);
    bool normal_is_in_bounds(pcl::PointXYZRGBNormal& normal);

    double max_tilt;
    double value;
    
    KDL::Frame World_Camera;
    bool world_set;
    KDL::Vector World_point,World_normal;
};

#endif //TILT_FILTER_H