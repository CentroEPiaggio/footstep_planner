#ifndef COORDINATE_FILTER_H
#define COORDINATE_FILTER_H

#include <kdl/frames.hpp>
#include "data_types.h"
#include <list>

using namespace planner;

class coordinate_filter
{
public:
    coordinate_filter(unsigned int filter_axis, double axis_min, double axis_max);
    ~coordinate_filter();
    void filter_borders(std::list<polygon_with_normals>& data, bool left);
    void filter_points(std::list<polygon_with_normals>& data, bool left);
    void set_stance_foot(KDL::Frame StanceFoot_Camera);
    
private:
    bool border_is_in_bounds(pcl::PointCloud<pcl::PointXYZ>::Ptr border);
    bool point_is_in_bounds(pcl::PointXYZRGBNormal& point);

    double multiplier_default;
    double multiplier_axis;
    double default_axis_max, default_axis_min;
    double axis_min, axis_max;
    unsigned int filter_axis;
    double m_x, m_y, m_z,t;
    bool stance_foot_set;
    double value;
};

#endif //COORDINATE_FILTER_H
