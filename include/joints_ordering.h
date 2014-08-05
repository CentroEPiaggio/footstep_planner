#ifndef JOINTSWAISTLEFTFOOT_H
#define JOINTSWAISTLEFTFOOT_H
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

class JointsWaistLeftFoot: public KDL::JntArray
{
};

class JointsWaistRightFoot: public KDL::JntArray
{
};

class JointsLeftFootWaist: public KDL::JntArray
{
};

class JointsRightFootWaist: public KDL::JntArray
{
};

class JointsLeftFootWaistRightFoot: public KDL::JntArray
{
};

class JointsRightFootWaistLeftFoot: public KDL::JntArray
{
};

class JointsRightFootWaistLeftFootWaist: public KDL::JntArray
{
};

class JointsLeftFootWaistRightFootWaist: public KDL::JntArray
{
};

class JointsWaistRightFootWaistLeftFoot: public KDL::JntArray
{
};

class JointsWaistLeftFootWaistRightFoot: public KDL::JntArray
{
};


class chain_and_solvers
{
public:
    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive* fksolver;
    KDL::ChainIkSolverPos_NR_JL* iksolver;
    KDL::ChainIkSolverVel_pinv* ikvelsolver;
    std::vector<std::string> joint_names;
};


template <class chain_order>
class safe_ordered_chain:public chain_and_solvers
{
public:
chain_order joints_value, q_min, q_max;
};
#endif // JOINTSWAISTLEFTFOOT_H
