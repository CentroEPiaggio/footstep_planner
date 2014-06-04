#ifndef FOOTSTEP_PLANNER_H
#define FOOTSTEP_PLANNER_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "kinematics_utilities.h"
#include <tf/transform_datatypes.h>

namespace planner
{

class footstepPlanner
{
    
private:
    kinematics_utilities kinematics;
    KDL::Frame fromCloudToWorld;
    KDL::JntArray left_joints,right_joints,leg_joints;
    KDL::Frame current_foot;
    KDL::ChainIkSolverPos_NR_JL* current_ik_solver;
public:
    footstepPlanner();
    
    bool centroid_is_reachable(KDL::Frame centroid);
    
    bool step_is_stable(KDL::Frame centroid);
    
    bool plane_is_compatible(Eigen::Matrix< double, 4, 1 > centroid);
    
    std::vector< pcl::PointCloud<pcl::PointXYZ> > compute_polygon_grid(std::shared_ptr< pcl::PointCloud< pcl::PointXYZ > > polygon);
    
    bool polygon_in_feasibile_area(std::shared_ptr< pcl::PointCloud< pcl::PointXYZ > > polygon);
    
    double dist_from_robot(pcl::PointXYZ point);
    
    std::map< int, Eigen::Matrix< double, 4, 1 > > getFeasibleCentroids(std::vector< std::shared_ptr< pcl::PointCloud< pcl::PointXYZ > > > polygons, bool left);
    void setParams(double feasible_area_);
    void setWorldTransform(KDL::Frame transform);
    
    
    // robot area for the footstep planner
    double feasible_area_;
};

}
#endif //FOOTSTEP_PLANNER_H