#ifndef COM_FILTER_H
#define COM_FILTER_H

#include <data_types.h>
#include "kinematics_utilities.h"
#include <drc_shared/idynutils.h>
#include <list>

class com_filter
{
public:
    com_filter();
    bool filter(std::list<planner::foot_with_joints> &data);
    void setWorld_StanceFoot(const KDL::Frame& World_StanceFoot);
    void setLeftRightFoot(bool left);
    void setZeroWaistHeight ( double hip_height );

private:
    bool frame_is_stable(const KDL::Frame& StanceFoot_MovingFoot,const KDL::Frame& DesiredWaist_StanceFoot, KDL::JntArray& jnt_pos);
    KDL::Frame computeWaistPosition( const KDL::Frame& StanceFoot_MovingFoot, double rot_angle, double hip_height );
    std::list<KDL::Frame> generateWaistPositions ( KDL::Frame StanceFoot_MovingFoot );
    KDL::ChainIkSolverPos_NR_JL *current_stance_ik_solver, *current_moving_ik_solver;

    KDL::Frame StanceFoot_World;
    KDL::JntArray current_joints;
    KDL::JntArray left_joints,right_joints,legs_joints;
    kinematics_utilities kinematics;
    KDL::Frame World_StanceFoot;
    double desired_hip_height;
    bool left;
};

#endif // COM_FILTER_H
