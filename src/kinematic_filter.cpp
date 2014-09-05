#include "kinematic_filter.h"
#ifdef KINEMATICS_OUTPUT
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>
#endif
using namespace planner;

kinematic_filter::kinematic_filter(std::string robot_name):robot_name(robot_name),kinematics(robot_name)
{
}

void kinematic_filter::setWorld_StanceFoot(const KDL::Frame& World_StanceFoot)
{
    this->StanceFoot_World=World_StanceFoot.Inverse();
    this->World_StanceFoot=World_StanceFoot;
}

void kinematic_filter::setLeftRightFoot(bool left)
{
    if (left)
    {
        current_ik_solver=kinematics.lwr_legs.iksolver;
        current_chain_names=kinematics.lwr_legs.joint_names;
        current_fk_solver=kinematics.lw_leg.fksolver;
	current_chain=kinematics.lwr_legs;
    }
    else
    {
        current_ik_solver=kinematics.rwl_legs.iksolver;
        current_chain_names=kinematics.rwl_legs.joint_names;
        current_fk_solver=kinematics.rw_leg.fksolver;
	current_chain=kinematics.rwl_legs;
    }
}

std::vector< std::string > kinematic_filter::getJointOrder()
{
    return current_chain.joint_names;
}

safe_ordered_chain kinematic_filter::getJointChain()
{
    return current_chain;
}

bool kinematic_filter::filter(std::list<foot_with_joints> &data)
{
    jnt_pos_in=kinematics.lwr_legs.joints_value;
    
    for (auto single_step=data.begin();single_step!=data.end();)
    {
        auto StanceFoot_MovingFoot=StanceFoot_World*single_step->World_MovingFoot;
        if (!frame_is_reachable(StanceFoot_MovingFoot,single_step->joints))
            single_step=data.erase(single_step);
        else
        {
            single_step->World_StanceFoot=World_StanceFoot;
            KDL::Frame StanceFoot_Waist;
            KDL::JntArray temp(kinematics.wl_leg.chain.getNrOfJoints());
            for (int i=0;i<temp.rows();i++)
                temp(i)=single_step->joints(i);
            current_fk_solver->JntToCart(temp,StanceFoot_Waist);
            single_step->World_Waist=World_StanceFoot*StanceFoot_Waist;
#ifdef KINEMATICS_OUTPUT
            tf::Transform current_robot_transform;
            tf::transformKDLToTF(single_step->World_Waist,current_robot_transform);
            static tf::TransformBroadcaster br;
            br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(),  "world","KNEW_WAIST"));
            tf::Transform current_moving_foot_transform;
            tf::transformKDLToTF(World_StanceFoot*StanceFoot_MovingFoot,current_moving_foot_transform);
            br.sendTransform(tf::StampedTransform(current_moving_foot_transform, ros::Time::now(),  "world","Kmoving_foot"));
            tf::Transform fucking_transform;
            tf::transformKDLToTF(World_StanceFoot,fucking_transform);
            br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "Kstance_foot"));
#endif
            single_step++;
        }
    }
    return true;
}

bool kinematic_filter::frame_is_reachable(const KDL::Frame& StanceFoot_MovingFoot, KDL::JntArray& jnt_pos)
{
    SetToZero(jnt_pos_in);
    KDL::JntArray jnt_pos_out(kinematics.rwl_legs.chain.getNrOfJoints());
    int ik_valid = current_ik_solver->CartToJnt(jnt_pos_in, StanceFoot_MovingFoot, jnt_pos_out);
    if (ik_valid>=0)
    {
        jnt_pos=jnt_pos_out;
        return true;
    }

    return false;
}
