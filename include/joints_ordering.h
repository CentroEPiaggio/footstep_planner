/* Copyright [2014] [Mirko Ferrati, Alessandro Settimi, Corrado Pavan, Carlos J Rosales]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

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
    KDL::JntArray average_joints;
    int index;
    KDL::JntArray joints_value, q_min, q_max;
};


// template <class chain_order>
class safe_ordered_chain:public chain_and_solvers
{
public:
};
#endif // JOINTSWAISTLEFTFOOT_H
