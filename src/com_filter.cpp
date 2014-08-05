#include "com_filter.h"
#include <iCub/iDynTree/iDyn2KDL.h>
#include <eigen3/Eigen/Dense>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>
com_filter::com_filter()
{
    stance_jnts_in.resize(kinematics.wl_leg.chain.getNrOfJoints());
}

bool com_filter::filter(std::list<planner::foot_with_joints> &data)
{
    int total=data.size();
    int counter=0;
    int total_num_examined=0;
    for (auto single_step=data.begin();single_step!=data.end();)
    {
        counter++;
        std::cout<<"percentage: "<<counter<<" / "<<total<<" examined:"<<total_num_examined<<std::endl;
        auto StanceFoot_MovingFoot=StanceFoot_World*single_step->World_MovingFoot;
        single_step->World_StanceFoot=World_StanceFoot;
        auto WaistPositions_StanceFoot=generateWaistPositions_StanceFoot(StanceFoot_MovingFoot);
	int num_inserted=0;
        for (auto WaistPosition_StanceFoot:WaistPositions_StanceFoot)
	{
            total_num_examined++;
	    KDL::JntArray jnt_temp(kinematics.num_joints);
	    if (frame_is_stable(StanceFoot_MovingFoot,WaistPosition_StanceFoot,jnt_temp))
	    {
		planner::foot_with_joints temp;
		temp.joints=jnt_temp;
		temp.World_MovingFoot=single_step->World_MovingFoot;
		temp.World_StanceFoot=single_step->World_StanceFoot;
		temp.World_Waist=single_step->World_StanceFoot*WaistPosition_StanceFoot.Inverse();
		data.insert(single_step,temp);
		num_inserted++;
	    }
	}
	//if (num_inserted==0)
	    single_step=data.erase(single_step);
	//else
	//    single_step++;	
    }
    current_chain_names=current_stance_chain_and_solver->joint_names;
    current_chain_names.insert(current_chain_names.end(),current_moving_chain_and_solver->joint_names.begin(),current_moving_chain_and_solver->joint_names.end());
    
    return true;
}


void com_filter::setWorld_StanceFoot(const KDL::Frame& World_StanceFoot)
{
    this->World_StanceFoot=World_StanceFoot;
}

void com_filter::setLeftRightFoot(bool left)
{
    if (left)
    {
        current_stance_chain_and_solver=&kinematics.wl_leg;
        current_moving_chain_and_solver=&kinematics.wr_leg;
    }
    else
    {
        current_stance_chain_and_solver=&kinematics.wr_leg;
        current_moving_chain_and_solver=&kinematics.wl_leg;
    }
    this->left=left;
}

bool com_filter::frame_is_stable(const KDL::Frame& StanceFoot_MovingFoot,const KDL::Frame& DesiredWaist_StanceFoot, KDL::JntArray& jnt_pos)
{
    auto stance_leg_size=kinematics.wl_leg.chain.getNrOfJoints();
    KDL::JntArray stance_jnts(stance_leg_size);
    SetToZero(stance_jnts_in);
    int result=current_stance_chain_and_solver->iksolver->CartToJnt(stance_jnts_in,DesiredWaist_StanceFoot,stance_jnts);
    if (result<0) return false;

    SetToZero(stance_jnts_in);
    KDL::JntArray moving_jnts(stance_leg_size);
    result=current_moving_chain_and_solver->iksolver->CartToJnt(stance_jnts_in,DesiredWaist_StanceFoot*StanceFoot_MovingFoot,moving_jnts);
    if (result<0) return false;

    for (int j=0;j<stance_leg_size;j++)
    {
        jnt_pos(j)=stance_jnts(j);
        jnt_pos(j+stance_leg_size)=moving_jnts(j);
    }
    return true;
}

std::vector< std::string > com_filter::getJointOrder()
{
    return current_chain_names;
}


void com_filter::setZeroWaistHeight ( double hip_height )
{
    this->desired_hip_height=hip_height;
}


std::list< KDL::Frame > com_filter::generateWaistPositions_StanceFoot ( KDL::Frame StanceFoot_MovingFoot )
{
    std::list<KDL::Frame> DesiredWaist_StanceFoot_list;
    //double angle_ref=atan2(StanceFoot_MovingFoot.p[0],-StanceFoot_MovingFoot.p[1]);//+M_PI*left;
    double angle_ref=0;
    for (double angle=-M_PI/6.0;angle<M_PI/6.1;angle=angle+M_PI/3.0)
	for (double height=-0.15;height<-0.049;height=height+0.05)
	{
	    KDL::Frame DesiredWaist_StanceFoot=computeWaistPosition(StanceFoot_MovingFoot,angle+angle_ref,desired_hip_height+height).Inverse();
	    DesiredWaist_StanceFoot_list.push_back(DesiredWaist_StanceFoot);
	}
    return DesiredWaist_StanceFoot_list;
}


KDL::Frame com_filter::computeWaistPosition(const KDL::Frame& StanceFoot_MovingFoot,double rot_angle,double hip_height)
{
    KDL::Frame StanceFoot_WaistPosition;
    KDL::Vector World_GravityFromIMU(0,0,-1);
    KDL::Vector StanceFoot_GravityFromIMU = StanceFoot_World.M*World_GravityFromIMU;
    StanceFoot_GravityFromIMU.Normalize();
    StanceFoot_WaistPosition.M=KDL::Rotation::Rot2(StanceFoot_GravityFromIMU,rot_angle);
    StanceFoot_WaistPosition.p=KDL::Vector(StanceFoot_GravityFromIMU*(-hip_height));
//     if (hip_height<=desired_hip_height-0.11)
//     {
//     tf::Transform current_robot_transform;
//     tf::transformKDLToTF(World_StanceFoot*StanceFoot_WaistPosition,current_robot_transform);
//     static tf::TransformBroadcaster br;
//     br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(),  "world","NEW_WAIST"));
//     ros::Duration sleep_time(0.1);
//     sleep_time.sleep();
//     }
    return StanceFoot_WaistPosition;
}


