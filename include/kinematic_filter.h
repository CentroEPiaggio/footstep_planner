#ifndef KINEMATIC_FILTER_H
#define KINEMATIC_FILTER_H

#include <kdl/jntarray.hpp>
#include <list>
#include <map>
#include <tuple>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kinematics_utilities.h>
#include <data_types.h>


class kinematic_filter
{
public:
    kinematic_filter(std::string robot_name);
    bool filter(std::list<planner::foot_with_joints>& data);
    void setWorld_StanceFoot(const KDL::Frame& World_StanceFoot);
    void setLeftRightFoot(bool left);
    std::vector< std::string > getJointOrder();
public:
    kinematics_utilities kinematics;

private:
    std::string robot_name;
    inline bool frame_is_reachable(const KDL::Frame& World_MovingFoot, KDL::JntArray& jnt_pos);
    KDL::ChainIkSolverPos_NR_JL* current_ik_solver;
    KDL::ChainFkSolverPos* current_fk_solver;
    KDL::Frame StanceFoot_World;
    KDL::Frame World_StanceFoot;
    JointsLeftFootWaistRightFoot jnt_pos_in;
    std::vector< std::string > current_chain_names;

};

#endif // KINEMATIC_FILTER_H
