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
    std::cout<<"mobile_foot in stance_foot"<<temp<<std::endl;
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
    this->desired_direction=direction;
    gs_utils.setCurrentDirection(direction);
}


std::map< int, foot_with_joints > footstepPlanner::getFeasibleCentroids(std::vector< polygon_with_normals > polygons, bool left)
{
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
        ROS_INFO("NORMAL size: %d ",polygon.normals->size());
        int normals=0;

//         int i=0;
        for(unsigned int i=0; i<polygon.normals->size(); i++)
        {
            //Eigen::Matrix<double,4,1> centroid;
            //pcl::compute3DCentroid(*polygon.border, centroid );
            KDL::Frame temp;
            temp.p[0]=(*polygon.normals)[i].x;
            temp.p[1]=(*polygon.normals)[i].y;
            temp.p[2]=(*polygon.normals)[i].z;
            temp.M=KDL::Rotation::RPY(0,-1.77,1.57);
            KDL::Frame rotz;
            rotz.M=KDL::Rotation::RPY(0,0,0.0/180.0*3.14159265);
            temp=temp*rotz;
            //KDL::Frame plane_frame=createFramesFromNormal((*polygon.normals)[i]);
            int k=-1;
            //for (double angle=-0.8;angle<=0.8;angle=angle+0.4) 
            {
                //double angle=0.0;
                k++;
                KDL::Frame Camera_MovingFoot;
                //KDL::Frame rotz;
                //rotz.M=KDL::Rotation::RotZ(angle);
                //Camera_MovingFoot=plane_frame*rotz;
                Camera_MovingFoot=temp;
                //std::cout<<"centroid in camera link"<<temp<<std::endl;
                KDL::JntArray joints_position;
                
                if(centroid_is_reachable(World_Camera*Camera_MovingFoot,joints_position)) //check if the centroid id inside the reachable area
                {
                    std::cout<<"ik result:";
                    for (int i=0; i<joints_position.rows(); i++)
                        std::cout<<joints_position(i)<<" ";
                    std::cout<<std::endl;
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
//                         tf::transformKDLToTF(World_StanceFoot,fucking_transform);
//                         br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "stance_foot"));
//                         ros::Duration t(0.1);
//                         t.sleep();
//                         tf::transformKDLToTF(World_StanceFoot*(Waist_StanceFoot.Inverse()),fucking_transform);
//                         br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "mobile_waist"));
//                         t.sleep();
//                         tf::transformKDLToTF(World_Camera*Camera_MovingFoot,fucking_transform);
//                         br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "mobile_foot"));
//                         t.sleep();
//                         t=ros::Duration(1);
//                         t.sleep();
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


