#include "com_filter.h"

com_filter::com_filter()
{
}

bool com_filter::filter(std::list<planner::foot_with_joints> &data)
{
    //TODO
    return true;
}


void setWorld_StanceFoot(const KDL::Frame& World_StanceFoot)
{
//TODO
}

void setLeftRightFoot(bool left)
{
//TODO
}

bool frame_is_stable(const KDL::Frame& World_MovingFoot, KDL::JntArray& jnt_pos)
{
    //TODO
    return true;
}
