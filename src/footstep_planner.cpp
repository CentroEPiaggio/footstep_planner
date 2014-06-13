#include "footstep_planner.h"
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <stdlib.h>     /* srand, rand */
#include <cmath>
using namespace planner;

#define DISTANCE_THRESHOLD 0.02

footstepPlanner::footstepPlanner()
{
    left_joints.resize(kinematics.left_leg.getNrOfJoints());
    right_joints.resize(kinematics.right_leg.getNrOfJoints());
    leg_joints.resize(kinematics.RL_legs.getNrOfJoints());
    SetToZero(left_joints);
    SetToZero(right_joints);
    SetToZero(leg_joints);
    kinematics.fkLsolver->JntToCart(left_joints,World_StanceFoot);   
}

void footstepPlanner::setCurrentSupportFoot(KDL::Frame foot_position)
{
    this->World_StanceFoot=foot_position;
}

void footstepPlanner::setParams(double feasible_area_)
{
    this->feasible_area_=feasible_area_;
}

bool footstepPlanner::step_is_stable(KDL::Frame centroid)
{
    return true;
}

bool footstepPlanner::centroid_is_reachable(KDL::Frame World_MovingFoot, KDL::JntArray& jnt_pos)
{
    KDL::JntArray jnt_pos_in;
    jnt_pos_in.resize(kinematics.q_min.rows());
    SetToZero(jnt_pos_in);
    KDL::JntArray jnt_pos_out;
    jnt_pos_out.resize(kinematics.q_min.rows());
    auto temp=World_StanceFoot.Inverse()*World_MovingFoot;
    int ik_valid = current_ik_solver->CartToJnt(jnt_pos_in, temp, jnt_pos_out);
//     std::cout<<"mobile_foot in stance_foot"<<temp<<std::endl;
    if (ik_valid>=0)
    {
        jnt_pos=jnt_pos_out;
        return true;
    }

    return false;
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

//Camera link frame
void footstepPlanner::setCurrentDirection(KDL::Vector direction)
{
    this->Camera_DesiredDirection=direction;
    gs_utils.setCurrentDirection(direction);
}


std::map< int, foot_with_joints > footstepPlanner::getFeasibleCentroids(std::vector< polygon_with_normals > polygons, bool left)
{
    if (!world_camera_set)
    {
        throw "camera - world transformation was not set";
    }
    auto World_CurrentDirection=KDL::Vector(1,-0.5,0); //TODO receive from the operator?
    setCurrentDirection(World_Camera.Inverse()*World_CurrentDirection); //TODO
    
    static tf::TransformBroadcaster br;
    tf::Transform fucking_transform;
    tf::transformKDLToTF(World_StanceFoot,fucking_transform);
    br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "stance_foot"));
    br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "stance_foot"));
    ros::Duration sleep_time(0.5);
    sleep_time.sleep();
    KDL::JntArray current_joints;
    if (left)
    {
        current_joints=left_joints;
        current_ik_solver=kinematics.ikLRsolver;
        current_fk_solver=kinematics.fkLsolver;
    }
    else
    {
        current_joints=right_joints;
        current_ik_solver=kinematics.ikRLsolver;
        current_fk_solver=kinematics.fkRsolver;

    }

    std::map< int, foot_with_joints> centroids;
    int j=-1;
    for(auto polygon:polygons)
    {
        j++;
        if(!polygon_in_feasibile_area(polygon.border))
        {
             std::cout<<"!! Polygon "<<j<<" outside the feasible area"<<std::endl;
            continue;
        }
        ROS_INFO("NORMAL size: %lu ",polygon.normals->size());
        int normals=0;

//         int i=0;
        for(unsigned int i=0; i<polygon.normals->size(); i++)
        {
            KDL::Frame plane_frame=createFramesFromNormal((*polygon.normals)[i]);
            int k=-1;
            for (double angle=-0.8;angle<=0.8;angle=angle+0.2) 
            {
                //double angle=0.0;
                k++;
                KDL::Frame Camera_MovingFoot;
                KDL::Frame rotz;
                rotz.M=KDL::Rotation::RotZ(angle);
                Camera_MovingFoot=plane_frame*rotz;
//                 Camera_MovingFoot=temp;
                //std::cout<<"centroid in camera link"<<temp<<std::endl;
                KDL::JntArray joints_position;
                
                if(centroid_is_reachable(World_Camera*Camera_MovingFoot,joints_position)) //check if the centroid id inside the reachable area
                {
                    if(step_is_stable(Camera_MovingFoot))//check if the centroid can be used to perform a stable step
                    {
                        KDL::Frame Waist_StanceFoot;
                        KDL::JntArray single_leg;
                        auto size=left_joints.rows();
                        single_leg.resize(size);
                        for (int k=0; k<size; k++)
                        {
                            single_leg(size-k-1)=joints_position(k);
                        }
                        current_fk_solver->JntToCart(single_leg,Waist_StanceFoot);
                        static tf::TransformBroadcaster br;
                        tf::Transform fucking_transform;
                        auto temp_pos=joints_position;
                        size=joints_position.rows();
                        for (int i=0; i<left_joints.rows(); i++)
                            joints_position(i+left_joints.rows())=temp_pos(size-i-1);
                        centroids[i*100+j*10000+k]=std::make_tuple(World_Camera*Camera_MovingFoot,joints_position,World_StanceFoot*(Waist_StanceFoot.Inverse()));
                        normals++;
                        // j=10000;
                    }
//                         else std::cout<<"!! Step not stable"<<std::endl;
                }
//                     else std::cout<<"!! Step not reachable"<<std::endl;
            }
//             else std::cout<<"!! ERR: Error in computing the centroid !!"<<std::endl;
        }
    }
    return centroids;
}

std::pair<int,foot_with_joints> footstepPlanner::selectBestCentroid(std::map< int, foot_with_joints > centroids, bool left)
{
    std::vector<int> minimum_indexes;
    std::pair<int, foot_with_joints> result= *centroids.begin();//TODO
    double min=100000000000000;
    for (auto centroid:centroids)
    {
        KDL::Frame StanceFoot_MovingFoot = ((std::get<0>(centroid.second)).Inverse()*World_StanceFoot).Inverse(); //World_Camera*Camera_MovingFoot ;
        double refy=0;
        double refx=0;
        if (left)
        {
            refy=-0.15;
            refx=0.20;
        }
        else
        {
            refy=0.15;
            refx=0.20;
        }
        auto distance=pow(StanceFoot_MovingFoot.p.x()-refx,2)+pow(StanceFoot_MovingFoot.p.y()-refy,2);
        
        if (distance-min>DISTANCE_THRESHOLD) //If it is too big, keep the old min
            continue;
        if (min-distance>DISTANCE_THRESHOLD) //New minimum
        {
            min=distance;
            std::cout<<"new min: "<<min<<" "<<StanceFoot_MovingFoot.p.x()<<" "<<StanceFoot_MovingFoot.p.y()<<" "<<std::endl;
            minimum_indexes.clear();
            minimum_indexes.push_back(centroid.first);
            continue;
        }
        if (distance-min<DISTANCE_THRESHOLD && min-distance>-DISTANCE_THRESHOLD) //If near, keep them both
        {
            minimum_indexes.push_back(centroid.first);
            std::cout<<"another min: "<<distance<<" "<<StanceFoot_MovingFoot.p.x()<<" "<<StanceFoot_MovingFoot.p.y()<<" "<<std::endl;
        }
    }
    
    double maximum=0;
    int maximum_index=minimum_indexes[0];
    for (auto index:minimum_indexes)
    {
        KDL::Vector World_DesiredDirection=World_Camera*Camera_DesiredDirection;
        KDL::Vector World_FootDirection = World_StanceFoot*KDL::Vector(1,0,0);
        auto filter=World_DesiredDirection+World_FootDirection;
        auto moving_foot=std::get<0>(centroids[index])*KDL::Vector(1,0,0);
        
        auto scalar=dot(filter,moving_foot);
        std::cout<<"indici minimi"<<std::endl;
        std::cout<<" "<<scalar<<" "<<index;
        
        if (scalar>maximum)
        {
            maximum=scalar;
            maximum_index=index;
        }
    }
    
    
    return std::make_pair(maximum_index,centroids[maximum_index]);
    
}



double footstepPlanner::dist_from_robot(pcl::PointXYZ point, double x,double y,double z)
{
    double x_diff = point.x - x;
    double y_diff = point.y - y;
    double z_diff = point.z - z;

    return sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff);
}

bool footstepPlanner::polygon_in_feasibile_area(pcl::PointCloud< pcl::PointXYZ >::Ptr polygon)
{
    KDL::Frame Camera_StanceFoot = World_Camera.Inverse()*World_StanceFoot;
    KDL::Vector World_direction(1,0,0); //TODO
    auto filter=World_Camera.Inverse()*World_direction;
    for (unsigned int i=0; i<polygon->size(); i++)
    {
        auto& point_cloud=polygon->at(i);
        auto& point_foot=Camera_StanceFoot.p;
        auto& point1=filter;
        KDL::Vector point(point_cloud.x-point_foot.x(),point_cloud.y-point_foot.y(),point_cloud.z-point_foot.z());
        if ((point.x()*point1.x()+point.y()*point1.y()+point.z()*point1.z())>0) //If the point is the direction of the foot motion
            if(dist_from_robot(polygon->at(i),Camera_StanceFoot.p.x(),Camera_StanceFoot.p.y(),Camera_StanceFoot.p.z()) < feasible_area_) 
                return true;
    }
    return false;
}


