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

typedef std::tuple<KDL::Frame,KDL::JntArray> foot_with_joints;
    
class footstepPlanner
{
    
private:
    KDL::Frame fromCloudToWorld;
    KDL::JntArray left_joints,right_joints,leg_joints;
    KDL::Frame current_foot;
    KDL::ChainIkSolverPos_NR_JL* current_ik_solver;
    KDL::ChainFkSolverPos_recursive* current_fk_solver;
    std::vector<KDL::Frame> createFramesFromNormal(pcl::PointNormal normal);
    bool centroid_is_reachable(KDL::Frame centroid, KDL::JntArray& jnt_pos);
    
    bool step_is_stable(KDL::Frame centroid);
    
    bool plane_is_compatible(Eigen::Matrix< double, 4, 1 > centroid);
    
    bool polygon_in_feasibile_area(pcl::PointCloud< pcl::PointXYZ >::Ptr polygon);
    
    double dist_from_robot(pcl::PointXYZ point);
public:
    footstepPlanner();
    kinematics_utilities kinematics;
    
    std::map< int, foot_with_joints > getFeasibleCentroids(std::vector< planner::polygon_with_normals > polygons, bool left);
    void setParams(double feasible_area_);
    void setCurrentSupportFoot(KDL::Frame foot_position);
    void setWorldTransform(KDL::Frame transform);
    std::pair<int,foot_with_joints> selectBestCentroid(std::map< int,foot_with_joints > centroids, bool left);
    inline KDL::Frame getWorldTransform(){return fromCloudToWorld;};
    
    
    // robot area for the footstep planner
    double feasible_area_;
};

}
#endif //FOOTSTEP_PLANNER_H