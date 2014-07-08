#ifndef COORDINATE_FILTER_H
#define COORDINATE_FILTER_H

#include <kinematics_utilities.h>
#include "data_types.h"

using namespace planner;

class coordinate_filter
{
    coordinate_filter(unsigned int filter_axis, double axis_min, double axis_max);
    ~coordinate_filter();

    void set_stance_foot(KDL::Frame Camera_StanceFoot);
    void set_bounds(double min, double max);

    void filter_borders(std::list<polygon_with_normals>& data);
    void filter_normals(std::list<polygon_with_normals>& data);
    bool border_is_in_bounds(pcl::PointCloud<pcl::PointXYZ>::Ptr border);
    bool normal_is_in_bounds(pcl::PointXYZRGBNormal& normal);

    double axis_min, axis_max;
    unsigned int filter_axis;
    KDL::Frame Camera_StanceFoot;
    KDL::Vector rotate_lower_limit, rotate_upper_limit;
    double m_x, m_y, m_z;
    bool stance_foot_set;
public:

};

#endif //COORDINATE_FILTER_H