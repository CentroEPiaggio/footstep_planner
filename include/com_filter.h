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
    std::vector<std::string> getJointOrder();

private:
    bool frame_is_stable(const KDL::Frame& StanceFoot_MovingFoot,const KDL::Frame& DesiredWaist_StanceFoot, KDL::JntArray& jnt_pos);
    KDL::Frame computeWaistPosition( const KDL::Frame& StanceFoot_MovingFoot, double rot_angle, double hip_height );
    std::list<KDL::Frame> generateWaistPositions_StanceFoot ( KDL::Frame StanceFoot_MovingFoot );
    KDL::JntArray stance_jnts_in;
    KDL::Frame StanceFoot_World;
    chain_and_solvers* current_stance_chain_and_solver;
    chain_and_solvers* current_moving_chain_and_solver;
    kinematics_utilities kinematics;
    KDL::Frame World_StanceFoot;
    double desired_hip_height;
    bool left;
    std::vector< std::string > current_chain_names;
};

#endif // COM_FILTER_H
