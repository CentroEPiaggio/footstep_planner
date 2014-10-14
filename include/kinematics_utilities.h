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

#ifndef KINEMATICS_UTILITES_H
#define KINEMATICS_UTILITES_H

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <eigen3/Eigen/Eigen>
#include <urdf/model.h>
#include <drc_shared/idynutils.h>
#include <joints_ordering.h>

#define MAX_THREADS 4
// typedef safe_ordered_chain<JointsWaistLeftFoot> ChainWaistLeftFoot;
// typedef safe_ordered_chain<JointsWaistRightFoot> ChainWaistRightFoot;
// typedef safe_ordered_chain<JointsLeftFootWaist> ChainLeftFootWaist;
// typedef safe_ordered_chain<JointsRightFootWaist> ChainRightFootWaist;
// typedef safe_ordered_chain<JointsLeftFootWaistRightFoot> ChainLeftFootWaistRightFoot;
// typedef safe_ordered_chain<JointsRightFootWaistLeftFoot> ChainRightFootWaistLeftFoot;

class ChainWaistLeftFoot :  public chain_and_solvers{};
class ChainWaistRightFoot :  public chain_and_solvers{};
class ChainLeftFootWaist : public chain_and_solvers{};
class ChainRightFootWaist : public chain_and_solvers{};
class ChainLeftFootWaistRightFoot : public chain_and_solvers{};
class ChainRightFootWaistLeftFoot : public chain_and_solvers{};


/*class ChainRightFootWaistLeftFootWaist:public safe_ordered_chain
{
public:
     JointsRightFootWaistLeftFootWaist joints_value, q_min, q_max;
};

class ChainLeftFootWaistRightFootWaist:public safe_ordered_chain
{
public:
     JointsLeftFootWaistRightFootWaist joints_value, q_min, q_max;
};

class ChainWaistRightFootWaistLeftFoot:public safe_ordered_chain
{
public:
     JointsWaistRightFootWaistLeftFoot joints_value, q_min, q_max;
};

class ChainWaistLeftFootWaistRightFoot:public safe_ordered_chain
{
public:
     JointsWaistLeftFootWaistRightFoot joints_value, q_min, q_max;
};*/


class kinematics_utilities
{
public:
    kinematics_utilities(std::string robot_name);
//    Eigen::Matrix<double,6,1> left_foot, right_foot;
    
    urdf::Model coman_urdf_model;
    
    iDynUtils coman_model;
    KDL::Tree coman;

    ChainWaistLeftFoot wl_leg;
    ChainWaistRightFoot wr_leg;
    ChainLeftFootWaist lw_leg;
    ChainRightFootWaist rw_leg;
    ChainLeftFootWaistRightFoot lwr_legs;
    ChainRightFootWaistLeftFoot rwl_legs;
    
    
    std::vector<chain_and_solvers> wl_leg_vector;
    std::vector<chain_and_solvers> wr_leg_vector;
    std::vector<chain_and_solvers> lw_leg_vector;
    std::vector<chain_and_solvers> rw_leg_vector;
    std::vector<chain_and_solvers> lwr_legs_vector;
    std::vector<chain_and_solvers> rwl_legs_vector;
    unsigned int num_joints;
    
/*    KDL::Chain left_leg,right_leg,LR_legs,RL_legs;
    KDL::ChainFkSolverPos_recursive* fkLsolver;
    KDL::ChainFkSolverPos_recursive* fkRsolver;
    KDL::ChainFkSolverPos_recursive* fkRLsolver;
    KDL::ChainFkSolverPos_recursive* fkLRsolver;
    
    KDL::ChainIkSolverPos_NR_JL* ikRsolver;
    KDL::ChainIkSolverPos_NR_JL* ikLsolver;    
    KDL::ChainIkSolverVel_pinv* ikRvelsolver;
    KDL::ChainIkSolverVel_pinv* ikLvelsolver;

    
    KDL::ChainIkSolverPos_NR_JL* ikRLsolver;
    KDL::ChainIkSolverPos_NR_JL* ikLRsolver;
    KDL::ChainIkSolverVel_pinv* ikRLvelsolver;
    KDL::ChainIkSolverVel_pinv* ikLRvelsolver;
    
    int num_joints;
    std::vector<std::string> left_leg_names,right_leg_names;
    std::vector<std::string> joint_names_LR,joint_names_RL;
    KDL::JntArray q_minL,q_minR,q_maxL,q_maxR,q_min,q_max;
    */
        
private:
    std::string robot_name;
  //  bool initJointNames(urdf::Model& robot_model, std::string parent, std::string tip, std::vector< std::string >& joint_names);
    void initialize_solvers(chain_and_solvers* container, KDL::JntArray& joints_value, KDL::JntArray& q_max, KDL::JntArray& q_min, int index);
    
};   
    
#endif //KINEMATICS_UTILITES_H
