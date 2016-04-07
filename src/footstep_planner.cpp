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
#include <param_manager.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <stdlib.h>     /* srand, rand */
#include <cmath>

using namespace planner;

double DISTANCE_THRESHOLD; //0.02*0.02 //We work with squares of distances, so this threshold is the square of 2cm!
double ANGLE_THRESHOLD;// 0.2
double WAIST_THRESHOLD;// 0.2

footstepPlanner::footstepPlanner(std::string robot_name_, std::string robot_urdf_file_,ros_publisher* ros_pub_):kinematicFilter(robot_name_, robot_urdf_file_),comFilter(robot_name_,robot_urdf_file_),lipmFilter(robot_name_,robot_urdf_file_,ros_pub_),stepQualityEvaluator(robot_name_),kinematics(kinematicFilter.kinematics), World_CurrentDirection(1,0,0) //TODO:remove kinematics from here
{
    param_manager::register_param("DISTANCE_THRESHOLD",DISTANCE_THRESHOLD);
    param_manager::update_param("DISTANCE_THRESHOLD",0.02*0.02);
    param_manager::register_param("ANGLE_THRESHOLD",ANGLE_THRESHOLD);
    param_manager::update_param("ANGLE_THRESHOLD",0.2);
    param_manager::register_param("WAIST_THRESHOLD",WAIST_THRESHOLD);
    param_manager::update_param("WAIST_THRESHOLD",0.2);

    param_manager::register_param("kin_min_angle",min_angle);
    param_manager::update_param("kin_min_angle",-0.8);
    param_manager::register_param("kin_max_angle",max_angle);
    param_manager::update_param("kin_max_angle",0.8);
    param_manager::register_param("kin_angle_step",angle_step);
    param_manager::update_param("kin_angle_step",8);
    
    KDL::Frame Waist_StanceFoot;
    left_joints.resize(kinematics.wl_leg.chain.getNrOfJoints());
    SetToZero(left_joints);
    kinematics.wl_leg.fksolver->JntToCart(left_joints,Waist_StanceFoot);
    SetToZero(World_Waist.p);//TODO: get external World_Waist
    World_StanceFoot=World_Waist*Waist_StanceFoot;
    std::cout<<"starting World_StanceFoot"<<World_StanceFoot<<std::endl;
    comFilter.setZeroWaistHeight(-Waist_StanceFoot.p[2]);
    lipmFilter.setZeroWaistHeight(-Waist_StanceFoot.p[2]);
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
    left_joints.resize(kinematics.wl_leg.chain.getNrOfJoints());
    SetToZero(left_joints);
    SetToZero(World_Waist.p);//TODO: get external World_Waist
    KDL::Frame defaultWaist_StanceFoot;
    if (left)
    {
        kinematics.wl_leg.fksolver->JntToCart(left_joints, defaultWaist_StanceFoot);
        World_InitialWaist=World_Waist*defaultWaist_StanceFoot*Waist_LeftFoot.Inverse();
        World_StanceFoot=World_InitialWaist*Waist_LeftFoot;
    }
    else
    {
        kinematics.wr_leg.fksolver->JntToCart(left_joints, defaultWaist_StanceFoot);
        World_InitialWaist=World_Waist*defaultWaist_StanceFoot*Waist_RightFoot.Inverse();
        World_StanceFoot=World_InitialWaist*Waist_RightFoot;
    }
    
    
    InitialWaist_MeanFoot.p=(Waist_LeftFoot.p+Waist_RightFoot.p)/2.0;
    //auto x_axis=(Waist_LeftFoot.p-Waist_RightFoot.p);
    //x_axis.Normalize();
    double roll,pitch,yaw;
    World_InitialWaist.M.GetRPY(roll,pitch,yaw);
    KDL::Vector Waist_GravityFromIMU(0,0,1);//TODO
    Waist_GravityFromIMU.Normalize();
    auto z_axis=Waist_GravityFromIMU;
    KDL::Vector x_axis=KDL::Rotation::Rot2(z_axis,yaw)*KDL::Vector(1,0,0);
    KDL::Vector y_axis=z_axis*x_axis;
    InitialWaist_MeanFoot.M=KDL::Rotation(x_axis,y_axis,z_axis);
    
    ros::Duration sleep_time(2);
    static tf::TransformBroadcaster br;
    tf::Transform current_robot_transform;
    tf::transformKDLToTF(InitialWaist_MeanFoot,current_robot_transform);
    br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "base_link", "MEAN_Foot"));
    sleep_time.sleep();
    
}


void footstepPlanner::setInitialPosition(const KDL::JntArray& left_leg_initial_position,
                        const KDL::JntArray& right_leg_initial_position
)
{
    this->left_leg_initial_position=left_leg_initial_position;
    this->right_leg_initial_position=right_leg_initial_position;
    kinematics.wl_leg.fksolver->JntToCart(left_leg_initial_position,Waist_LeftFoot);
    std::cout<<"starting Waist_LeftFoot"<<Waist_LeftFoot<<std::endl;
    kinematics.wr_leg.fksolver->JntToCart(right_leg_initial_position,Waist_RightFoot);
    std::cout<<"starting Waist_RightFoot"<<Waist_RightFoot<<std::endl;
    InitialWaist_LeftFoot=Waist_LeftFoot;
    InitialWaist_RightFoot=Waist_RightFoot;
    
}

void footstepPlanner::setCurrentSupportFoot(KDL::Frame World_StanceFoot, bool left)
{
    this->World_StanceFoot=World_StanceFoot;
    if (left) this->Waist_LeftFoot=World_Waist.Inverse()*World_StanceFoot;
    else this->Waist_RightFoot=World_Waist.Inverse()*World_StanceFoot;
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
//         ROS_INFO("Polygon %d number of normals : %lu ",j,item.normals->size());

        for(unsigned int i=0; i<item.normals->size(); i++)
        {
            KDL::Frame plane_frame=createFramesFromNormal((*item.normals)[i]);
            int k=-1;
            for (double angle=min_angle;angle<=max_angle;angle=angle+(max_angle-min_angle)/double(angle_step)) 
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
//     ros_pub->publish_geometric_constraints(0.0,0.6,-0.1,0.6,-0.5,0.5,World_StanceFoot);

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
    
    int total_normals = 0;
    for(auto item:affordances)
        total_normals+=item.normals->size();
    ROS_INFO("Number of normals after geometric filter XYZ on borders: %d ",total_normals);
    
    filter_by_tilt->filter_single_normals(affordances);
    total_normals=0;
    for(auto item:affordances)
        total_normals+=item.normals->size();
    ROS_INFO("Number of affordances after tilt filter on normals: %d ",total_normals);
    
    
    filter_to_avoid_foot.set_stance_foot(StanceFoot_Camera);
    filter_to_avoid_foot.filter_points(affordances,left);

    total_normals=0;
    for(auto item:affordances)
        total_normals+=item.normals->size();
    ROS_INFO("Number of affordances after collision filter on points: %d ",total_normals);
    
}

void footstepPlanner::kinematic_filtering(std::list<foot_with_joints>& steps, bool left)
{
    kinematicFilter.setLeftRightFoot(left);
    kinematicFilter.setWorld_StanceFoot(World_StanceFoot);
    kinematicFilter.filter(steps);
    last_used_joint_names=kinematicFilter.getJointOrder();
    joint_chain=kinematicFilter.getJointChain();
    
}

void footstepPlanner::dynamic_filtering(std::list<foot_with_joints>& steps, bool left, int dyn_filter_type)
{
    comFilter.setLeftRightFoot(left);
    comFilter.setWorld_StanceFoot(World_StanceFoot);
    comFilter.filter(steps);
    last_used_joint_names=comFilter.getJointOrder();
    joint_chain=kinematicFilter.getJointChain();
}

void footstepPlanner::dynamic_filtering(std::list<foot_with_com>& steps, bool left, int dyn_filter_type)
{
    lipmFilter.setLeftRightFoot(left);
    lipmFilter.setWorld_StanceFoot(World_StanceFoot);
    lipmFilter.filter(steps);
    last_used_joint_names=lipmFilter.getJointOrder();
    joint_chain=kinematicFilter.getJointChain();
}


std::list<foot_with_joints > footstepPlanner::getFeasibleCentroids(std::list< polygon_with_normals >& affordances, bool left, int dyn_filter_type)
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
    color_filtered=3;
    if(dyn_filter_type==0)
    {
	dynamic_filtering(steps,left,dyn_filter_type); // COM DYNAMIC FILTER
	if(steps.size()<=1000) ros_pub->publish_filtered_frames(steps,World_Camera,color_filtered);
    }
    if(dyn_filter_type==1)
    {
	std::list<foot_with_com> lipm_steps;
	for( auto step:steps )
	{
	    foot_with_com lipm_step;
	    lipm_step.World_StanceFoot = step.World_StanceFoot;
	    lipm_step.World_MovingFoot = step.World_MovingFoot;
	    lipm_steps.push_back(lipm_step);
	}
	lipm_steps.front().World_StartCom.z[0] = steps.front().World_Waist.p.z(); //TODO

	dynamic_filtering(lipm_steps,left,dyn_filter_type); // LIPM DYNAMIC FILTER
        if(lipm_steps.size()<=1000) ros_pub->publish_filtered_frames(lipm_steps,World_Camera,color_filtered);

	steps.clear();
	for( auto lipm_step:lipm_steps )
	{
	    foot_with_joints step;
	    step.World_StanceFoot = lipm_step.World_StanceFoot;
	    step.World_MovingFoot = lipm_step.World_MovingFoot;
	    steps.push_back(step);
	}
    }
//     std::cout<<"time after dynamic filter: "<<time<<std::endl<<std::endl;
    ROS_INFO("Number of steps after dynamic filter: %lu ",steps.size());

    return steps;
}

std::list<foot_with_joints> footstepPlanner::single_check(KDL::Frame left_foot, KDL::Frame right_foot, bool only_ik, bool move, bool left, int dyn_filter_type)
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
        
    if(!only_ik) dynamic_filtering(list,left,dyn_filter_type);
    
    if(!move) World_StanceFoot=tmp_World_StanceFoot;
    
    return list;
}

foot_with_joints footstepPlanner::selectBestCentroid(std::list< foot_with_joints >const& centroids, bool left, int loss_function_type)
{
    std::vector<double> w;
    w.push_back(1.0); 	// angle 1.0
    w.push_back(0.1);	// waist 0.3
    w.push_back(0.1);	// mobility 1.0
    w.push_back(0.5);   // stability
    w.push_back(0.5);   // reference step
	  
    if(loss_function_type==7) //LIPM
    {
	KDL::Vector World_DesiredDirection=World_Camera*Camera_DesiredDirection;
	double min=100000000000000;
	foot_with_joints result;
	for (auto centroid:centroids)
	{
	    auto angle=stepQualityEvaluator.angle_from_reference_direction(centroid,World_DesiredDirection);
	    KDL::Frame StanceFoot_MovingFoot;
	    auto distance=stepQualityEvaluator.distance_from_reference_step(centroid,left,StanceFoot_MovingFoot);
	    stepQualityEvaluator.set_single_chain(&joint_chain);
	    auto stability=stepQualityEvaluator.stability(centroid);
	    
	    double cost = 10000000;
	    cost = -w.at(0)*fabs(angle) + w.at(3)*stability + w.at(4)*distance;
	    if (cost < min)
	    {
		min=cost;
		result=centroid;
	    }
	}
    }
    
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
    else if (loss_function_type==2 || loss_function_type==4)	// sum of distance, angle and mobility / energy
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
// 	std::cout<<"Number of steps after distance minimization "<<minimum_steps.size()<<std::endl;
	
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
	  auto energy = stepQualityEvaluator.energy_consumption(*centroid);
	  double cost = 10000000;
	  if (loss_function_type==2)
	    cost = -w.at(0)*fabs(angle) + w.at(1)*(start_waist_distance + end_waist_distance)/M_PI + w.at(2)*mobility/0.67;
	  else if (loss_function_type==4)
	    cost = -w.at(0)*fabs(angle) + w.at(1)*(start_waist_distance + end_waist_distance)/M_PI + w.at(2)*energy/0.67;
	  if (cost < min)
	  {
	    min=cost;
	    result=centroid;
	  }
	}
	std::cout<<"Final step cost: "<<min<<std::endl;
        return *result;
    }
    else if(loss_function_type==3)      // energy_consumption
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
