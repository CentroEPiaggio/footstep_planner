#include "kinematic_filter.h"

kinematic_filter::kinematic_filter()
{
    left_joints.resize(kinematics.left_leg.getNrOfJoints());
    right_joints.resize(kinematics.right_leg.getNrOfJoints());
    leg_joints.resize(kinematics.RL_legs.getNrOfJoints());
    SetToZero(left_joints);
    SetToZero(right_joints);
    SetToZero(leg_joints);
}

void kinematic_filter::setWorld_StanceFoot(const KDL::Frame& World_StanceFoot)
{
    this->StanceFoot_World=World_StanceFoot.Inverse();
}

void kinematic_filter::setLeftRightFoot(bool left)
{
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
}

bool kinematic_filter::filter(std::list<std::tuple<int,KDL::Frame,KDL::JntArray>>& data)
{

    for (auto single_step=data.begin();single_step!=data.end();)
    {
        if (!frame_is_reachable(std::get<1>(*single_step),std::get<2>(*single_step)))
            single_step=data.erase(single_step);
        else
            single_step++;
    }
    return true;
}

bool kinematic_filter::frame_is_reachable(const KDL::Frame& World_MovingFoot, KDL::JntArray& jnt_pos)
{
    KDL::JntArray jnt_pos_in;
    jnt_pos_in.resize(kinematics.q_min.rows());
    SetToZero(jnt_pos_in);
    KDL::JntArray jnt_pos_out;
    jnt_pos_out.resize(kinematics.q_min.rows());
    auto temp=StanceFoot_World*World_MovingFoot;
    int ik_valid = current_ik_solver->CartToJnt(jnt_pos_in, temp, jnt_pos_out);
    if (ik_valid>=0)
    {
        jnt_pos=jnt_pos_out;
        return true;
    }

    return false;
}