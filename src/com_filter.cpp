#include "com_filter.h"
#include <iCub/iDynTree/iDyn2KDL.h>
#include <eigen3/Eigen/Dense>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>
com_filter::com_filter()
{
    left_joints.resize(kinematics.left_leg.getNrOfJoints());
    right_joints.resize(kinematics.right_leg.getNrOfJoints());
    legs_joints.resize(kinematics.RL_legs.getNrOfJoints());
    SetToZero(left_joints);
    SetToZero(right_joints);
    SetToZero(legs_joints);
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
	auto StanceFoot_WaistPositions=generateWaistPositions(StanceFoot_MovingFoot);
	int num_inserted=0;
	for (auto StanceFoot_WaistPosition:StanceFoot_WaistPositions)
	{
            total_num_examined++;
	    KDL::JntArray jnt_temp;
	    jnt_temp.resize(kinematics.num_joints);
	    if (frame_is_stable(StanceFoot_MovingFoot,StanceFoot_WaistPosition,jnt_temp))
	    {
		planner::foot_with_joints temp;
		temp.joints=jnt_temp;
		temp.World_MovingFoot=single_step->World_MovingFoot;
		temp.World_StanceFoot=single_step->World_StanceFoot;
		temp.World_Waist=single_step->World_StanceFoot*StanceFoot_WaistPosition;
		data.insert(single_step,temp);
		num_inserted++;
	    }
	}
	//if (num_inserted==0)
	    single_step=data.erase(single_step);
	//else
	//    single_step++;	
    }
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
        current_stance_ik_solver=kinematics.ikLsolver;
	current_moving_ik_solver=kinematics.ikRsolver;
    }
    else
    {
        current_stance_ik_solver=kinematics.ikRsolver;
	current_moving_ik_solver=kinematics.ikLsolver;
    }
}

bool com_filter::frame_is_stable(const KDL::Frame& StanceFoot_MovingFoot,const KDL::Frame& DesiredWaist_StanceFoot, KDL::JntArray& jnt_pos)
{
    KDL::JntArray stance_jnts_in, stance_jnts;
    stance_jnts_in.resize(kinematics.left_leg.getNrOfJoints());
    SetToZero(stance_jnts_in);
    stance_jnts.resize(kinematics.left_leg.getNrOfJoints());
    int result=current_stance_ik_solver->CartToJnt(stance_jnts_in,DesiredWaist_StanceFoot,stance_jnts);
    if (result<0) return false;

    KDL::JntArray moving_jnts;
    moving_jnts.resize(kinematics.left_leg.getNrOfJoints());
    result=current_moving_ik_solver->CartToJnt(moving_jnts,DesiredWaist_StanceFoot*StanceFoot_MovingFoot,moving_jnts);
    //if (result<0) return false;

    auto stance_leg_size=kinematics.left_leg.getNrOfJoints();
    for (int j=0;j<stance_leg_size;j++)
    {
        jnt_pos(j)=stance_jnts(j);
        jnt_pos(j+stance_leg_size)=moving_jnts(j);
    }

    return true;
}

void com_filter::setZeroWaistHeight ( double hip_height )
{
    this->desired_hip_height=hip_height;
}


std::list< KDL::Frame > com_filter::generateWaistPositions ( KDL::Frame StanceFoot_MovingFoot )
{
    std::list<KDL::Frame> temp;
    double angle_ref=atan2(StanceFoot_MovingFoot.p[0],-StanceFoot_MovingFoot.p[1]);
    for (double angle=-M_PI/6.0;angle<M_PI/5.9;angle=angle+M_PI/3.0)
	for (double height=-15.0;height<-5.0;height=height+5.0)
	{
	    KDL::Frame DesiredWaist_StanceFoot=computeWaistPosition(StanceFoot_MovingFoot,angle+angle_ref,desired_hip_height+height).Inverse();
	    temp.push_back(DesiredWaist_StanceFoot);
	}
    return temp;
}


KDL::Frame com_filter::computeWaistPosition(const KDL::Frame& StanceFoot_MovingFoot,double rot_angle,double hip_height)
{
    KDL::Frame StanceFoot_WaistPosition;
    KDL::Vector World_GravityFromIMU(0,0,-1);
    std::cout<<"World_StanceFoot.inverse"<<World_StanceFoot.Inverse()<<std::endl;
    KDL::Vector StanceFoot_GravityFromIMU = World_StanceFoot.Inverse().M*World_GravityFromIMU;
    StanceFoot_GravityFromIMU.Normalize();
    StanceFoot_WaistPosition.M=KDL::Rotation::Rot2(StanceFoot_GravityFromIMU,rot_angle);
    StanceFoot_WaistPosition.p=KDL::Vector(StanceFoot_GravityFromIMU*(-hip_height));
    tf::Transform current_robot_transform;
    tf::transformKDLToTF(World_StanceFoot*StanceFoot_WaistPosition,current_robot_transform);
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(),  "world","NEW_WAIST"));
    ros::Duration sleep_time(3);
    sleep_time.sleep();
    return StanceFoot_WaistPosition;
}


