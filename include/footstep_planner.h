#ifndef FOOTSTEP_PLANNER_H
#define FOOTSTEP_PLANNER_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "kinematics_utilities.h"
#include "borderextraction.h"
#include <tf/transform_datatypes.h>

namespace planner
{

class footstepPlanner
{
    
private:
    KDL::Frame fromCloudToWorld;
    KDL::JntArray left_joints,right_joints,leg_joints;
    KDL::Frame current_foot;
    KDL::ChainIkSolverPos_NR_JL* current_ik_solver;
    KDL::ChainFkSolverPos_recursive* current_fk_solver;
public:
    footstepPlanner();
    kinematics_utilities kinematics;
    
    bool centroid_is_reachable(KDL::Frame centroid, KDL::JntArray& jnt_pos);
    
    bool step_is_stable(KDL::Frame centroid);
    
    bool plane_is_compatible(Eigen::Matrix< double, 4, 1 > centroid);
    
    std::vector< pcl::PointCloud<pcl::PointXYZ> > compute_polygon_grid(pcl::PointCloud< pcl::PointXYZ >::Ptr polygon);
    
    bool polygon_in_feasibile_area(pcl::PointCloud< pcl::PointXYZ >::Ptr polygon);
    
    double dist_from_robot(pcl::PointXYZ point);
    
    std::map< int, std::tuple< Eigen::Matrix< double, 4, 1 >, KDL::JntArray, KDL::Frame > > getFeasibleCentroids(std::vector< polygon_with_normals > polygons, bool left);
    void setParams(double feasible_area_);
    void setWorldTransform(KDL::Frame transform);
    
    
    // robot area for the footstep planner
    double feasible_area_;
};

}
#endif //FOOTSTEP_PLANNER_H