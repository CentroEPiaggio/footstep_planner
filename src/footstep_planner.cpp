#include "footstep_planner.h"
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <stdlib.h>     /* srand, rand */

using namespace planner;

footstepPlanner::footstepPlanner()
{
    left_joints.resize(kinematics.left_leg.getNrOfJoints());
    right_joints.resize(kinematics.right_leg.getNrOfJoints());
    leg_joints.resize(kinematics.RL_legs.getNrOfJoints());
    SetToZero(left_joints);
    SetToZero(right_joints);
    SetToZero(leg_joints);
    kinematics.fkLsolver->JntToCart(left_joints,current_foot);
    current_foot=fromWorldToCloud*current_foot;
}

void footstepPlanner::setCurrentSupportFoot(KDL::Frame foot_position)
{
    this->current_foot=foot_position;
}

void footstepPlanner::setParams(double feasible_area_)
{
    this->feasible_area_=feasible_area_;
}

bool footstepPlanner::step_is_stable(KDL::Frame centroid)
{
    return true;
}

bool footstepPlanner::centroid_is_reachable(KDL::Frame centroid, KDL::JntArray& jnt_pos)
{
    KDL::JntArray jnt_pos_in;
    jnt_pos_in.resize(kinematics.q_min.rows());
    SetToZero(jnt_pos_in);
    KDL::JntArray jnt_pos_out;
    jnt_pos_out.resize(kinematics.q_min.rows());
    auto temp=current_foot.Inverse()*centroid;
    int ik_valid = current_ik_solver->CartToJnt(jnt_pos_in, temp, jnt_pos_out);
    //std::cout<<"centroide in foot"<<temp<<std::endl;
    if (ik_valid>=0)
    {
        jnt_pos=jnt_pos_out;
        return true;
    }

    return false;
}

void footstepPlanner::setWorldTransform(KDL::Frame transform)
{
    this->fromWorldToCloud=transform;
    current_foot=fromWorldToCloud*current_foot;
}

//Camera link
KDL::Frame footstepPlanner::createFramesFromNormal(pcl::PointXYZRGBNormal normal)
{
    return gs_utils.createFramesFromNormal(normal);
}

//Camera link frame
void footstepPlanner::setCurrentDirection(KDL::Vector direction)
{
    this->desired_direction=direction;
    gs_utils.setCurrentDirection(direction);
}


std::map< int, foot_with_joints > footstepPlanner::getFeasibleCentroids(std::vector< polygon_with_normals > polygons, bool left)
{
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
//             std::cout<<"!! Polygon "<<j<<" outside the feasible area"<<std::endl;
            continue;
        }
        for(unsigned int i=0; i<polygon.normals->size(); i++)
        {
            KDL::Frame plane_frame=createFramesFromNormal((*polygon.normals)[i]);
            int k=-1;
            for (double angle=-0.8;angle<=0.8;angle=angle+0.1) 
            {
                k++;
                KDL::Frame temp;
                KDL::Frame rotz;
                rotz.M=KDL::Rotation::RotZ(angle);
                temp=temp*rotz;

                //std::cout<<"centroid in camera link"<<temp<<std::endl;
                KDL::JntArray position;
                if(centroid_is_reachable(fromWorldToCloud*temp,position)) //check if the centroid id inside the reachable area
                {
                    std::cout<<"ik result:";
                    for (int i=0; i<position.rows(); i++)
                        std::cout<<position(i)<<" ";
                    std::cout<<std::endl;
                    if(step_is_stable(temp))//check if the centroid can be used to perform a stable step
                    {
                        KDL::Frame foot_position;
                        KDL::JntArray single_leg;
                        auto size=left_joints.rows();
                        single_leg.resize(size);
                        for (int k=0; k<size; k++)
                        {
                            single_leg(size-k-1)=position(k);
                        }
                        current_fk_solver->JntToCart(single_leg,foot_position);
//                             std::cout<<"from foot to waist"<<foot_position.Inverse()<<std::endl;
                        static tf::TransformBroadcaster br;
                        tf::Transform fucking_transform;
                        tf::transformKDLToTF(current_foot,fucking_transform);
                        br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "current_foot"));
                        ros::Duration t(0.1);
                        t.sleep();
//                             tf::transformKDLToTF(current_foot*foot_position.Inverse(),fucking_transform);
//                             br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "mobile_waist"));
//                             t.sleep();
//                             tf::transformKDLToTF(foot_position,fucking_transform);
//                             br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "mobile_foot"));
//                             t.sleep();
//                             t=ros::Duration(1);
                        //t.sleep();
                        auto temp_pos=position;
                        size=position.rows();
                        for (int i=0; i<left_joints.rows(); i++)
                            position(i+left_joints.rows())=temp_pos(size-i-1);
                        centroids[i*100+j*10000+k]=std::make_tuple(current_foot*foot_position.Inverse(),position);
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
    return *centroids.begin();//TODO
}



double footstepPlanner::dist_from_robot(pcl::PointXYZ point)
{
    //read robot pose, for now 0 0 0

    double x_diff = point.x - 0.0;
    double y_diff = point.y - 0.0;
    double z_diff = point.z - 0.0;

    return sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff);
}

bool footstepPlanner::polygon_in_feasibile_area(pcl::PointCloud< pcl::PointXYZ >::Ptr polygon)
{
    for (unsigned int i=0; i<polygon->size(); i++)
    {
        if(dist_from_robot(polygon->at(i)) < feasible_area_) return true;
    }

    return false;
}


