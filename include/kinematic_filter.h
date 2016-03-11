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
    kinematic_filter(std::string robot_name, std::string robot_urdf_file_);
    bool filter(std::list<planner::foot_with_joints>& data);
    void setWorld_StanceFoot(const KDL::Frame& World_StanceFoot);
    void setLeftRightFoot(bool left);
    std::vector< std::string > getJointOrder();
    chain_and_solvers getJointChain();
public:
    kinematics_utilities kinematics;

private:
    std::string robot_name;
    inline bool frame_is_reachable(const KDL::Frame& World_MovingFoot, KDL::JntArray& jnt_pos);
    KDL::ChainIkSolverPos_NR_JL* current_ik_solver;
    KDL::ChainFkSolverPos* current_fk_solver;
    KDL::Frame StanceFoot_World;
    KDL::Frame World_StanceFoot;
    KDL::JntArray jnt_pos_in;
    std::vector< std::string > current_chain_names;
    chain_and_solvers current_chain;

};

#endif // KINEMATIC_FILTER_H
