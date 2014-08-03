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
    kinematic_filter();
    bool filter(std::list<planner::foot_with_joints>& data);
    void setWorld_StanceFoot(const KDL::Frame& World_StanceFoot);
    void setLeftRightFoot(bool left);
public:
    kinematics_utilities kinematics;

private:
    inline bool frame_is_reachable(const KDL::Frame& World_MovingFoot, KDL::JntArray& jnt_pos);
    KDL::ChainIkSolverPos_NR_JL* current_ik_solver;
    KDL::Frame StanceFoot_World;
    KDL::Frame World_StanceFoot;
    JointsLeftFootWaistRightFoot jnt_pos_in;

};

#endif // KINEMATIC_FILTER_H
