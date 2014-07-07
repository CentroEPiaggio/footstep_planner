#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace planner
{

typedef struct
{
    int index;
    KDL::JntArray joints;
    KDL::Frame World_StanceFoot;
    KDL::Frame World_MovingFoot;
    KDL::Frame World_Waist;
}foot_with_joints;

}

#endif // DATA_TYPES_H
