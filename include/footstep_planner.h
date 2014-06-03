#ifndef FOOTSTEP_PLANNER_H
#define FOOTSTEP_PLANNER_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "kinematics_utilities.h"

namespace planner
{

class footstepPlanner
{
    
private:
    kinematics_utilities kinematics;
public:
    bool centroid_is_reachable(KDL::Frame centroid);
    
    bool step_is_stable(KDL::Frame centroid);
    
    bool plane_is_compatible(Eigen::Matrix< double, 4, 1 > centroid);
    
    std::vector< pcl::PointCloud<pcl::PointXYZ> > compute_polygon_grid(pcl::PointCloud<pcl::PointXYZ> polygon);
    
    bool polygon_in_feasibile_area(pcl::PointCloud<pcl::PointXYZ> polygon);
    
    double dist_from_robot(pcl::PointXYZ point);
    
    std::map< int, Eigen::Matrix< double, 4, 1 > > getFeasibleCentroids(std::vector< pcl::PointCloud< pcl::PointXYZ > > polygons, bool left);
    void setParams(double feasible_area_);
    
    
    // robot area for the footstep planner
    double feasible_area_;
};

}
#endif //FOOTSTEP_PLANNER_H