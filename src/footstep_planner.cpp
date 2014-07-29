#include "footstep_planner.h"
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <stdlib.h>     /* srand, rand */
#include <cmath>

using namespace planner;

#define DISTANCE_THRESHOLD 0.02*0.02 //We work with squares of distances, so this threshould is the square of 2cm!

footstepPlanner::footstepPlanner():kinematics(kinematicFilter.kinematics), World_CurrentDirection(1,0,0) //TODO:remove kinematics from here
{
    left_joints.resize(kinematics.left_leg.getNrOfJoints());
    right_joints.resize(kinematics.right_leg.getNrOfJoints());
    leg_joints.resize(kinematics.RL_legs.getNrOfJoints());
    SetToZero(left_joints);
    SetToZero(right_joints);
    SetToZero(leg_joints);
    kinematics.fkLsolver->JntToCart(left_joints,World_StanceFoot);   
    comFilter.setZeroWaistHeight(World_StanceFoot.p[2]);
    coordinate_filter* temp_filter = new coordinate_filter(0,-0.5,1);
    filter_by_coordinates.push_back(temp_filter);
    temp_filter = new coordinate_filter(1,-0.5,0.8);
    filter_by_coordinates.push_back(temp_filter);
    temp_filter = new coordinate_filter(2,-0.5,0.5);
    filter_by_coordinates.push_back(temp_filter);
    
    filter_by_tilt = new tilt_filter();
    
}

void footstepPlanner::setCurrentSupportFoot(KDL::Frame foot_position)
{
    this->World_StanceFoot=foot_position;
}

void footstepPlanner::setParams(double feasible_area_)
{
    this->feasible_area_=feasible_area_;
}

void footstepPlanner::setWorldTransform(KDL::Frame transform)
{
    this->World_Camera=transform;
    world_camera_set=true;
    //current_foot_W=World_Camera*current_foot_W;
}

//Camera link
KDL::Frame footstepPlanner::createFramesFromNormal(pcl::PointXYZRGBNormal normal)
{
    return gs_utils.createFramesFromNormal(normal);
}

void footstepPlanner::setDirectionVector(double x, double y, double z)
{
    World_CurrentDirection.data[0] = x;
    World_CurrentDirection.data[1] = y;
    World_CurrentDirection.data[2] = z;
}

//Camera link frame
void footstepPlanner::setCurrentDirection(KDL::Vector direction)
{
    this->Camera_DesiredDirection=direction;
    gs_utils.setCurrentDirection(direction);
}

void footstepPlanner::generate_frames_from_normals(std::list< polygon_with_normals >& affordances, std::list< foot_with_joints >& steps)
{
    int j=-1;

    for(auto item:affordances)
    {
        j++;
        ROS_INFO("Polygon %d number of normals : %lu ",j,item.normals->size());

        for(unsigned int i=0; i<item.normals->size(); i++)
        {
            KDL::Frame plane_frame=createFramesFromNormal((*item.normals)[i]);
            int k=-1;
            for (double angle=-0.8;angle<=0.8;angle=angle+0.2) 
            {
                //double angle=0.0;
                k++;
                KDL::Frame Camera_MovingFoot;
                KDL::Frame rotz;
                rotz.M=KDL::Rotation::RotZ(angle);
                Camera_MovingFoot=plane_frame*rotz;
                KDL::JntArray joints_position;
                foot_with_joints temp;
                temp.index=i*100+j*10000+k;
                temp.World_MovingFoot=World_Camera*Camera_MovingFoot;
                temp.joints=joints_position;
                steps.push_back(std::move(temp));
            }
        }
    }
}

void footstepPlanner::geometric_filtering(std::list< polygon_with_normals >& affordances, bool left)
{
    ROS_INFO("Number of affordances: %lu ",affordances.size());
    
    filter_by_tilt->filter_normals(affordances);   //filter on the tilt of the normal
    
    ROS_INFO("Number of affordances after tilt filter : %lu ",affordances.size());    
    
    KDL::Frame StanceFoot_Camera =  World_StanceFoot.Inverse()*World_Camera;

    filter_by_coordinates.at(0)->set_stance_foot(StanceFoot_Camera);   //filter on x
    filter_by_coordinates.at(0)->filter_borders(affordances,left);
    ROS_INFO("Number of affordances after geometric filter X on borders: %lu ",affordances.size());

    filter_by_coordinates.at(1)->set_stance_foot(StanceFoot_Camera);   //filter on y
    filter_by_coordinates.at(1)->filter_borders(affordances,left);
    ROS_INFO("Number of affordances after geometric filter XY on borders: %lu ",affordances.size());

    filter_by_coordinates.at(2)->set_stance_foot(StanceFoot_Camera);   //filter on z
    filter_by_coordinates.at(2)->filter_borders(affordances,left);
    
    ROS_INFO("Number of affordances after geometric filter XYZ on borders: %lu ",affordances.size());
    
    filter_by_coordinates.at(0)->filter_points(affordances,left);   //filter on x
    filter_by_coordinates.at(1)->filter_points(affordances,left);   //filter on y
    filter_by_coordinates.at(2)->filter_points(affordances,left);   //filter on z
}

void footstepPlanner::kinematic_filtering(std::list<foot_with_joints>& steps, bool left)
{
    kinematicFilter.filter(steps);
}

void footstepPlanner::dynamic_filtering(std::list<foot_with_joints>& steps, bool left)
{
    comFilter.filter(steps);
}


std::list<foot_with_joints > footstepPlanner::getFeasibleCentroids(std::list< polygon_with_normals >&    affordances, bool left)
{
    if (!world_camera_set)
    {
        throw "camera - world transformation was not set";
    }
    
    setCurrentDirection(World_Camera.Inverse()*World_CurrentDirection); //TODO
    
    static tf::TransformBroadcaster br;
    tf::Transform fucking_transform;
    tf::transformKDLToTF(World_StanceFoot,fucking_transform);
    br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "stance_foot"));
    br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "stance_foot"));
    ros::Duration sleep_time(0.5);
    sleep_time.sleep();
    kinematicFilter.setLeftRightFoot(left);
    kinematicFilter.setWorld_StanceFoot(World_StanceFoot);
        
    
    geometric_filtering(affordances,left); //GEOMETRIC FILTER
    
    std::list<foot_with_joints> steps;
    
    generate_frames_from_normals(affordances,steps); //generating kdl frames to place foot

    ROS_INFO("Number of steps after geometric filter: %lu ",steps.size()); 
    
        
    kinematic_filtering(steps,left); //KINEMATIC FILTER
    
    ROS_INFO("Number of steps after kinematic filter: %lu ",steps.size());  
    
    
    dynamic_filtering(steps,left); //DYNAMIC FILTER
    
    ROS_INFO("Number of steps after dynamic filter: %lu ",steps.size());  
    
    
    prepareForROSVisualization(steps);
    return steps;
}


bool footstepPlanner::prepareForROSVisualization(std::list<foot_with_joints>& steps)
{
    for (auto& step:steps)
    {
        KDL::Frame Waist_StanceFoot;
        KDL::JntArray single_leg;
        auto single_leg_size=left_joints.rows();
        single_leg.resize(single_leg_size);
        auto& joints_position=step.joints;
        for (int k=0; k<single_leg_size; k++)
        {
            single_leg(single_leg_size-k-1)=joints_position(k); //Swap joint indexes because ROS and KDL have different chain order (our fault, but we had to cause of left/right chains)
        }

        kinematicFilter.current_fk_solver->JntToCart(single_leg,Waist_StanceFoot);

        auto temp_pos=joints_position; //We HAVE to copy the vector since we are swapping joint indexes
        auto legs_size=joints_position.rows();
        for (int i=0; i<single_leg_size; i++)
        {
            joints_position(i+single_leg_size)=temp_pos(legs_size-i-1);
        }
        step.World_Waist=World_StanceFoot*(Waist_StanceFoot.Inverse());
    }
    return true;
}

foot_with_joints footstepPlanner::selectBestCentroid(std::list< foot_with_joints > centroids, bool left)
{
    std::vector<foot_with_joints*> minimum_steps;
    double min=100000000000000;
    foot_with_joints* result=&(*centroids.begin());
    for (auto& centroid:centroids)
    {
        KDL::Frame StanceFoot_MovingFoot;
        auto distance=stepQualityEvaluator.distance_from_reference_step(centroid,left,StanceFoot_MovingFoot);
        if (distance-min>DISTANCE_THRESHOLD) //If it is too big, keep the old min
            continue;
        if (min-distance>DISTANCE_THRESHOLD) //New minimum
        {
            min=distance;
            //std::cout<<"new min: "<<min<<" "<<StanceFoot_MovingFoot.p.x()<<" "<<StanceFoot_MovingFoot.p.y()<<" "<<std::endl;
            minimum_steps.clear();
            minimum_steps.push_back(&(centroid)); //TODO: any better idea?
            continue;
        }
        if (distance-min<DISTANCE_THRESHOLD && min-distance>-DISTANCE_THRESHOLD) //If near, keep them both
        {
            minimum_steps.push_back(&(centroid));
            //std::cout<<"another min: "<<distance<<" "<<StanceFoot_MovingFoot.p.x()<<" "<<StanceFoot_MovingFoot.p.y()<<" "<<std::endl;
        }
    }
    
    double maximum=0;
    int maximum_index=(*minimum_steps.begin())->index;
    KDL::Vector World_DesiredDirection=World_Camera*Camera_DesiredDirection;

    for (auto centroid:minimum_steps)
    {
        auto scalar=stepQualityEvaluator.angle_from_reference_direction(*centroid,World_DesiredDirection);
        if (scalar>maximum)
        {
            maximum=scalar;
            result=centroid;
        }
    }
    return *result;
}
