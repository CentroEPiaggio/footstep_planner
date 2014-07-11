#ifndef FOOTSTEP_PLANNER_H
#define FOOTSTEP_PLANNER_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "kinematics_utilities.h"
#include "borderextraction.h"
#include <tf/transform_datatypes.h>
#include "gram_schmidt.h"
#include "kinematic_filter.h"
#include "com_filter.h"
#include "step_quality_evaluator.h"
#include <data_types.h>
#include "coordinate_filter.h"
#include "tilt_filter.h"

namespace planner
{

//typedef std::tuple<KDL::Frame,KDL::JntArray, KDL::Frame> foot_with_joints;
    
class footstepPlanner
{
    
private:
    KDL::Frame World_Camera;
    KDL::JntArray left_joints,right_joints,leg_joints;
    
    //World frame
    KDL::Frame World_StanceFoot;
    
    kinematic_filter kinematicFilter;
    com_filter comFilter;
    step_quality_evaluator stepQualityEvaluator;
    //Camera Link Frame
    KDL::Vector Camera_DesiredDirection;
    bool world_camera_set=false;
    
    //Camera Link frame
    KDL::Frame createFramesFromNormal(pcl::PointXYZRGBNormal normal);
    bool prepareForROSVisualization(std::list<foot_with_joints>& steps);

    //World frame
    bool centroid_is_reachable(KDL::Frame World_MovingFoot, KDL::JntArray& jnt_pos);
            
    void generate_frames_from_normals(std::list< polygon_with_normals >& affordances, std::list< foot_with_joints >& steps);
    
    void geometric_filtering(std::list< polygon_with_normals >& affordances, bool left);
    
    void kinematic_filtering(std::list<foot_with_joints>& steps, bool left);
    
    void dynamic_filtering(std::list<foot_with_joints>& steps, bool left);
    
    tilt_filter* filter_by_tilt;
    std::vector<coordinate_filter*> filter_by_coordinates;
    
    KDL::Vector World_CurrentDirection;
    
public:
    footstepPlanner();
    gram_schmidt gs_utils;
    kinematics_utilities kinematics; //TODO: remove!!

    //Camera link frame
    std::list<foot_with_joints> getFeasibleCentroids(std::list<polygon_with_normals> &polygons, bool left);
    void setParams(double feasible_area_);
    
    //World frame
    void setCurrentSupportFoot(KDL::Frame foot_position);
    
    
    void setWorldTransform(KDL::Frame transform);
    foot_with_joints selectBestCentroid(std::list<foot_with_joints> centroids, bool left);
    inline KDL::Frame getWorldTransform(){return World_Camera;}
    
    //Camera Link Frame
    void setCurrentDirection(KDL::Vector direction);
    
    void setDirectionVector(double x, double y, double z);
    
    // robot area for the footstep planner
    double feasible_area_;
};

}
#endif //FOOTSTEP_PLANNER_H
