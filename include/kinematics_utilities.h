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

class kinematics_utilities
{
public:
    kinematics_utilities();
//    Eigen::Matrix<double,6,1> left_foot, right_foot;
    
    KDL::Frame left_foot,right_foot,world;
    urdf::Model coman_urdf_model;
    
    iDynUtils coman_model;
    KDL::Tree coman;
    KDL::Chain left_leg,right_leg,LR_legs,RL_legs;
    KDL::ChainFkSolverPos_recursive* fkLsolver;
    KDL::ChainFkSolverPos_recursive* fkRsolver;
    KDL::ChainFkSolverPos_recursive* fkRLsolver;
    KDL::ChainFkSolverPos_recursive* fkLRsolver;
    
    KDL::ChainIkSolverPos_NR_JL* ikRLsolver;
    KDL::ChainIkSolverPos_NR_JL* ikLRsolver;
    
    KDL::ChainIkSolverVel_pinv* ikRLvelsolver;
    KDL::ChainIkSolverVel_pinv* ikLRvelsolver;
    
    int num_joints;
    std::vector<std::string> left_leg_names,right_leg_names;
    
    
    
    KDL::Frame getForwardKinematics(KDL::Chain& chain,KDL::ChainFkSolverPos_recursive& solver);
    KDL::JntArray getInverseKinematics(KDL::Chain& chain,KDL::Frame& left_foot,KDL::Frame& right_foot,KDL::ChainIkSolverPos_NR_JL& solver);
    bool readJoints(urdf::Model& robot_model, std::string tip, std::string parent, KDL::JntArray& q_min, KDL::JntArray& q_max);
        
    KDL::JntArray q_minL,q_minR,q_maxL,q_maxR,q_min,q_max;
private:
    bool initJointNames(urdf::Model& robot_model, std::string tip, std::string parent, std::vector< std::string >& joint_names);
    
};   
    
#endif //KINEMATICS_UTILITES_H