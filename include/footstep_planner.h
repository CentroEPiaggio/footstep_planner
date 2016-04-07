/* Copyright [2014] [Mirko Ferrati, Alessandro Settimi, Corrado Pavan, Carlos J Rosales]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

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
#include "lipm_filter.h"
#include "step_quality_evaluator.h"
#include <data_types.h>
#include "coordinate_filter.h"
#include "foot_collision_filter.h"
#include "tilt_filter.h"
#include "ros_publisher.h"
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
namespace planner
{

    
class footstepPlanner
{
    
private:
    KDL::Frame World_Camera;
    KDL::JntArray left_joints,right_joints,leg_joints;
    
    //World frame
    KDL::Frame World_StanceFoot;
    kinematic_filter kinematicFilter;
    com_filter comFilter;
    lipm_filter lipmFilter;
    step_quality_evaluator stepQualityEvaluator;
    //Camera Link Frame
    KDL::Vector Camera_DesiredDirection;
    bool world_camera_set=false;
    
    //Camera Link frame
    KDL::Frame createFramesFromNormal(pcl::PointXYZRGBNormal normal);

    //World frame
    //bool centroid_is_reachable(KDL::Frame World_MovingFoot, KDL::JntArray& jnt_pos);
            
    void generate_frames_from_normals(std::list< polygon_with_normals >const& affordances, std::list< foot_with_joints >& steps);
    
    void geometric_filtering(std::list< polygon_with_normals >& affordances, bool left);
    
    void kinematic_filtering(std::list<foot_with_joints>& steps, bool left);
    
    void dynamic_filtering(std::list<foot_with_joints>& steps, bool left, int dyn_filter_type);
    void dynamic_filtering(std::list<foot_with_com>& steps, bool left, int dyn_filter_type);
    
    tilt_filter* filter_by_tilt;
    std::vector<coordinate_filter*> filter_by_coordinates;
    foot_collision_filter filter_to_avoid_foot;
    
    KDL::Vector World_CurrentDirection;
    std::vector< std::string > last_used_joint_names;
    
    double min_angle,max_angle;
    int angle_step;
    
    ros_publisher* ros_pub;
    int color_filtered;
    chain_and_solvers joint_chain;
    KDL::JntArray left_leg_initial_position,right_leg_initial_position;
    
public:
    footstepPlanner(std::string robot_name_, std::string robot_urdf_file_, ros_publisher* ros_pub_);
    gram_schmidt gs_utils;
    kinematics_utilities kinematics; //TODO: remove!!
    
    //Camera link frame
    std::list<foot_with_joints> getFeasibleCentroids(std::list< polygon_with_normals >& affordances, bool left, int dyn_filter_type);
    void setParams(double feasible_area_);
    
    void setCurrentSupportFoot(KDL::Frame World_StanceFoot, bool left);
    KDL::Frame Waist_LeftFoot, InitialWaist_LeftFoot;
    KDL::Frame Waist_RightFoot, InitialWaist_RightFoot;
    KDL::Frame World_InitialWaist;
    KDL::Frame World_Gravity;
    KDL::Frame InitialWaist_MeanFoot;
    KDL::Frame World_Waist;
    
    void setWorldTransform(KDL::Frame transform);
    foot_with_joints selectBestCentroid(const std::list< foot_with_joints >& centroids, bool left, int loss_function_type = 4);
    inline KDL::Frame getWorldTransform(){return World_Camera;}
    
    //Camera Link Frame
    void setCurrentDirection(KDL::Vector direction);
    
    void setInitialPosition(const KDL::JntArray& left_leg_initial_position,
                            const KDL::JntArray& right_leg_initial_position);
    
    void setDirectionVector(double x, double y, double z);
    const std::vector<std::string>& getLastUsedChain();
    
    // robot area for the footstep planner
    double feasible_area_;
    
    std::list<foot_with_joints> single_check(KDL::Frame left_foot, KDL::Frame right_foot, bool only_ik, bool move, bool left, int dyn_filter_type);
    void setCurrentStanceFoot(bool left);
};

}
#endif //FOOTSTEP_PLANNER_H
