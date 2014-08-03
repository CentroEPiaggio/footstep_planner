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


typedef safe_ordered_chain<JointsWaistLeftFoot> ChainWaistLeftFoot;
typedef safe_ordered_chain<JointsWaistRightFoot> ChainWaistRightFoot;
typedef safe_ordered_chain<JointsLeftFootWaistRightFoot> ChainLeftFootWaistRightFoot;
typedef safe_ordered_chain<JointsRightFootWaistLeftFoot> ChainRightFootWaistLeftFoot;

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
    kinematics_utilities();
//    Eigen::Matrix<double,6,1> left_foot, right_foot;
    
    urdf::Model coman_urdf_model;
    
    iDynUtils coman_model;
    KDL::Tree coman;

    ChainWaistLeftFoot wl_leg;
    ChainWaistRightFoot wr_leg;
    ChainLeftFootWaistRightFoot lwr_legs;
    ChainRightFootWaistLeftFoot rwl_legs;
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
  //  bool initJointNames(urdf::Model& robot_model, std::string parent, std::string tip, std::vector< std::string >& joint_names);
    void initialize_solvers(chain_and_solvers* container, KDL::JntArray& joints_value, KDL::JntArray& q_max, KDL::JntArray& q_min);
    
};   
    
#endif //KINEMATICS_UTILITES_H
