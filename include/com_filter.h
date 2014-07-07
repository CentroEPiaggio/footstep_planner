#ifndef COM_FILTER_H
#define COM_FILTER_H

#include <data_types.h>
#include <list>
class com_filter
{
public:
    com_filter();
    bool filter(std::list<planner::foot_with_joints> &data);
    void setWorld_StanceFoot(const KDL::Frame& World_StanceFoot);
    void setLeftRightFoot(bool left);

private:
    inline bool frame_is_stable(const KDL::Frame& World_MovingFoot, KDL::JntArray& jnt_pos);
    KDL::Frame StanceFoot_World;
    KDL::JntArray current_joints;
    KDL::JntArray left_joints,right_joints,leg_joints;

};

#endif // COM_FILTER_H
