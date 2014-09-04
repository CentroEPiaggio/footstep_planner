#ifndef FOOT_COLLISION_FILTER_H
#define FOOT_COLLISION_FILTER_H

#include <kdl/frames.hpp>
#include "data_types.h"
#include <list>

using namespace planner;

class foot_collision_filter
{
public:
    foot_collision_filter(double h_=0.30, double l_=0.15);
    void filter_points(std::list<polygon_with_normals>& data, bool left);
    void set_stance_foot(KDL::Frame StanceFoot_Camera_);
    
private:
    bool point_is_in_bounds(pcl::PointXYZRGBNormal& point);

    double default_h,default_l,l,h;
    KDL::Frame StanceFoot_Camera;
    bool stance_foot_set;
    KDL::Vector StanceFoot_point;
};

#endif //FOOT_COLLISION_FILTER_H