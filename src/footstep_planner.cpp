/* Copyright [2014] [Mirko Ferrati, Alessandro Settimi, Corrado Pavan, Carlos J Rosales]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.*/


#include "footstep_planner.h"
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <stdlib.h>     /* srand, rand */
#include <cmath>

using namespace planner;

#define DISTANCE_THRESHOLD 0.02*0.02 //We work with squares of distances, so this threshould is the square of 2cm!
#define ANGLE_THRESHOLD 0.2
#define WAIST_THRESHOLD 0.2	

footstepPlanner::footstepPlanner(std::string robot_name_, ros_publisher* ros_pub_):kinematicFilter(robot_name_),comFilter(robot_name_),stepQualityEvaluator(robot_name_),kinematics(kinematicFilter.kinematics), World_CurrentDirection(1,0,0) //TODO:remove kinematics from here
{
    KDL::Frame Waist_StanceFoot;
    left_joints.resize(kinematics.wl_leg.chain.getNrOfJoints());
    SetToZero(left_joints);
    kinematics.wl_leg.fksolver->JntToCart(left_joints,Waist_StanceFoot);
    SetToZero(World_Waist.p);//TODO: get external World_Waist
    World_StanceFoot=World_Waist*Waist_StanceFoot;
    std::cout<<"starting World_StanceFoot"<<World_StanceFoot<<std::endl;
    comFilter.setZeroWaistHeight(-Waist_StanceFoot.p[2]);
    coordinate_filter* temp_filter = new coordinate_filter(0,0.0,0.6);
    filter_by_coordinates.push_back(temp_filter);
    temp_filter = new coordinate_filter(1,-0.6,0.1);
    filter_by_coordinates.push_back(temp_filter);
    temp_filter = new coordinate_filter(2,-0.5,0.5);
    filter_by_coordinates.push_back(temp_filter);

    filter_by_tilt = new tilt_filter();
    
    ros_pub = ros_pub_;
}

void footstepPlanner::setCurrentStanceFoot(bool left)
{
    if (left)
    {
        World_StanceFoot=World_Waist*LeftFoot_Waist.Inverse();
    }
    else
    {
        World_StanceFoot=World_Waist*RightFoot_Waist.Inverse();
    }
}


void footstepPlanner::setInitialPosition(const KDL::JntArray& left_leg_initial_position,
                        const KDL::JntArray& right_leg_initial_position
)
{
    SetToZero(World_Waist.p);//TODO: get external World_Waist
    this->left_leg_initial_position=left_leg_initial_position;
    this->right_leg_initial_position=right_leg_initial_position;
    kinematics.wl_leg.fksolver->JntToCart(left_leg_initial_position,LeftFoot_Waist);
    std::cout<<"starting LeftFoot_Waist"<<LeftFoot_Waist<<std::endl;
    kinematics.wr_leg.fksolver->JntToCart(right_leg_initial_position,RightFoot_Waist);
    std::cout<<"starting RightFoot_Waist"<<RightFoot_Waist<<std::endl;
    InitialLeftFoot_Waist=LeftFoot_Waist;
    InitialRightFoot_Waist=RightFoot_Waist;
    InitialMeanFoot_Waist.p=(LeftFoot_Waist.p+RightFoot_Waist.p)/2.0;
}

void footstepPlanner::setCurrentSupportFoot(KDL::Frame World_StanceFoot, bool left)
{
    this->World_StanceFoot=World_StanceFoot;
    if (left) this->LeftFoot_Waist=World_Waist.Inverse()*World_StanceFoot;
    else this->RightFoot_Waist=World_Waist.Inverse()*World_StanceFoot;
}

void footstepPlanner::setParams(double feasible_area_)
{
    this->feasible_area_=feasible_area_;
}

void footstepPlanner::setWorldTransform(KDL::Frame transform)
{
    this->World_Camera=transform;
    world_camera_set=true;
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

const std::vector< std::string >& footstepPlanner::getLastUsedChain()
{
        return last_used_joint_names;
}


//Camera link frame
void footstepPlanner::setCurrentDirection(KDL::Vector direction)
{
    this->Camera_DesiredDirection=direction;
    gs_utils.setCurrentDirection(direction);
}

void footstepPlanner::generate_frames_from_normals(const std::list< polygon_with_normals >& affordances, std::list< foot_with_joints >& steps)
{
    int j=-1;

    for(auto const& item:affordances)
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
                temp.index=(long int)&temp;
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
    
    filter_by_tilt->set_world(World_Camera);
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
    
    filter_by_tilt->filter_single_normals(affordances);
    
    filter_to_avoid_foot.set_stance_foot(StanceFoot_Camera);
    filter_to_avoid_foot.filter_points(affordances,left);
}

void footstepPlanner::kinematic_filtering(std::list<foot_with_joints>& steps, bool left)
{
    kinematicFilter.setLeftRightFoot(left);
    kinematicFilter.setWorld_StanceFoot(World_StanceFoot);
    kinematicFilter.filter(steps);
    last_used_joint_names=kinematicFilter.getJointOrder();
    joint_chain=kinematicFilter.getJointChain();
    
}

void footstepPlanner::dynamic_filtering(std::list<foot_with_joints>& steps, bool left)
{
    comFilter.setLeftRightFoot(left);
    comFilter.setWorld_StanceFoot(World_StanceFoot);
    comFilter.filter(steps);
    last_used_joint_names=comFilter.getJointOrder();
    joint_chain=kinematicFilter.getJointChain();
}


std::list<foot_with_joints > footstepPlanner::getFeasibleCentroids(std::list< polygon_with_normals >& affordances, bool left)
{
    if (!world_camera_set)
    {
        throw "camera - world transformation was not set";
    }
    int i=0;
    //for (auto polygon:affordances)
    //    ros_pub->publish_normal_cloud(polygon.normals,i++); 
    
    setCurrentDirection(World_Camera.Inverse()*World_CurrentDirection); //TODO

//     static tf::TransformBroadcaster br;
//     tf::Transform fucking_transform;
//     tf::transformKDLToTF(World_StanceFoot,fucking_transform);
//     br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "stance_foot"));
//     br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "stance_foot"));
//     ros::Duration sleep_time(0.5);
//     sleep_time.sleep();

    geometric_filtering(affordances,left); //GEOMETRIC FILTER

    std::list<foot_with_joints> steps;
    
    generate_frames_from_normals(affordances,steps); //generating kdl frames to place foot
    color_filtered=1;
    if(steps.size()<=1000) ros_pub->publish_filtered_frames(steps,World_Camera,color_filtered);
    ROS_INFO("Number of steps after geometric filter: %lu ",steps.size()); 

    kinematic_filtering(steps,left); //KINEMATIC FILTER
    color_filtered=2;
    if(steps.size()<=1000) ros_pub->publish_filtered_frames(steps,World_Camera,color_filtered);
    ROS_INFO("Number of steps after kinematic filter: %lu ",steps.size());  
    auto time=ros::Time::now();
    dynamic_filtering(steps,left); //DYNAMIC FILTER
    color_filtered=3;
    if(steps.size()<=1000) ros_pub->publish_filtered_frames(steps,World_Camera,color_filtered);
    std::cout<<"time after kinematic filter"<<time<<std::endl;
    ROS_INFO("Number of steps after dynamic filter: %lu ",steps.size());

    return steps;
}

std::list<foot_with_joints> footstepPlanner::single_check(KDL::Frame left_foot, KDL::Frame right_foot, bool only_ik, bool move, bool left)
{
    KDL::Frame tmp_World_StanceFoot = World_StanceFoot;
    
    setCurrentDirection(World_Camera.Inverse()*World_CurrentDirection);
    
    World_StanceFoot = (left)?left_foot:right_foot;
    
    std::list<foot_with_joints> list;
    foot_with_joints fwj;
    fwj.World_StanceFoot = (left)?left_foot:right_foot;
    fwj.World_MovingFoot = (left)?right_foot:left_foot;
    list.insert(list.begin(),fwj);
    
    kinematic_filtering(list,left);
        
    if(!only_ik) dynamic_filtering(list,left);
    
    if(!move) World_StanceFoot=tmp_World_StanceFoot;
    
    return list;
}

foot_with_joints footstepPlanner::selectBestCentroid(std::list< foot_with_joints >const& centroids, bool left, int loss_function_type)
{
    std::vector<double> w;
    w.push_back(1.0); 	// angle 1.0
    w.push_back(0.1);	// waist 0.3
    w.push_back(0.1);	// mobility 1.0
	  
    if(loss_function_type==1)	// mobility
    {	
	stepQualityEvaluator.set_single_chain(&joint_chain);
	double min=100000000000000;
	foot_with_joints result;
	for (auto centroid:centroids)
	{	
	    bool start = true;
	    auto start_waist_distance=stepQualityEvaluator.waist_orientation(centroid,start);
	    auto end_waist_distance=stepQualityEvaluator.waist_orientation(centroid,!start);
	    auto mobility=stepQualityEvaluator.distance_from_joint_center(centroid);
	    double cost = w.at(1)*(start_waist_distance + end_waist_distance)/M_PI + w.at(2)*mobility/0.67;
	    if (cost<min)
	    {
		min=cost;
		result=centroid;
	    }
	}
        return result;
    }
    else if (loss_function_type==2)	// sum of distance, angle and mobility
    {
	foot_with_joints const* result=&(*centroids.begin());
	std::vector<foot_with_joints const*> minimum_steps;
	double min=100000000000000;
	for (auto const& centroid:centroids)
	{
	    KDL::Frame StanceFoot_MovingFoot;
	    auto distance=stepQualityEvaluator.distance_from_reference_step(centroid,left,StanceFoot_MovingFoot);
	    if (distance-min>DISTANCE_THRESHOLD) //If it is too big, keep the old min
		continue;
	    if (min-distance>DISTANCE_THRESHOLD) //New minimum
	    {
		min=distance;
		minimum_steps.clear();
		minimum_steps.push_back(&(centroid)); //TODO: any better idea?
		continue;
	    }
	    if (distance-min<DISTANCE_THRESHOLD && min-distance>-DISTANCE_THRESHOLD) //If near, keep them both
	    {
		minimum_steps.push_back(&(centroid));
	    }
	}
	std::cout<<"Number of steps after distance minimization "<<minimum_steps.size()<<std::endl;
	
	// SINGLE FUNCTION VERSION
	KDL::Vector World_DesiredDirection=World_Camera*Camera_DesiredDirection;
	min=100000000000000;
	for (auto centroid:minimum_steps)
	{
	  auto angle=stepQualityEvaluator.angle_from_reference_direction(*centroid,World_DesiredDirection);
	  
	  bool start = true;
	  auto start_waist_distance=stepQualityEvaluator.waist_orientation(*centroid,start);//,World_DesiredDirection);
	  auto end_waist_distance=stepQualityEvaluator.waist_orientation(*centroid,!start);
	  
	  stepQualityEvaluator.set_single_chain(&joint_chain);
	  auto mobility=stepQualityEvaluator.distance_from_joint_center(*centroid);
	  
	  double cost = -w.at(0)*fabs(angle) + w.at(1)*(start_waist_distance + end_waist_distance)/M_PI + w.at(2)*mobility/0.67;
	  if (cost < min)
	  {
	    min=cost;
	    result=centroid;
	  }
	}
        return *result;
    }
    else if(loss_function_type==3)	// energy_consumption
    {
	double min=100000000000000;
	foot_with_joints result;
	for (auto centroid:centroids)
	{
	    auto scalar=stepQualityEvaluator.energy_consumption(centroid);
	    if (scalar<min)
	    {
		min=scalar;
		result=centroid;
	    }
	}
        return result;
    }
    else
    {
      	foot_with_joints const* result=&(*centroids.begin());
	std::vector<foot_with_joints const*> minimum_steps;
	double min=100000000000000;
	for (auto const& centroid:centroids)
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
	
	double minimum=100000000000000;
	int maximum_index=(*minimum_steps.begin())->index;
	KDL::Vector World_DesiredDirection=World_Camera*Camera_DesiredDirection;
	for (auto centroid:minimum_steps)
	{
	    bool start = true;
	    auto start_waist_distance=stepQualityEvaluator.waist_orientation(*centroid,start);
	    auto end_waist_distance=stepQualityEvaluator.waist_orientation(*centroid,!start);
	    auto angle=stepQualityEvaluator.angle_from_reference_direction(*centroid,World_DesiredDirection);
	    double cost = - w.at(0)*angle/0.67 + w.at(1)*(start_waist_distance + end_waist_distance)/M_PI; 
	    if (cost<minimum)
	    {
		minimum=cost;
		result=centroid;
	    }
	}
        return *result;
    }
}
