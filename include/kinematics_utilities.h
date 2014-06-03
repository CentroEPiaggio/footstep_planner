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
    Eigen::Matrix<double,6,1> left_foot, right_foot;
    
    iDynUtils coman_model;
    KDL::Tree coman;
    KDL::Chain left_leg,right_leg,legs;
    KDL::ChainFkSolverPos_recursive* fkLsolver;
    KDL::ChainFkSolverPos_recursive* fkRsolver;
    KDL::ChainFkSolverPos_recursive* fksolver;
    KDL::ChainIkSolverPos_NR_JL* iksolver;
    KDL::ChainIkSolverVel_pinv* ikvelsolver;
    int num_joints;
    
    
    
    KDL::Frame getForwardKinematics(KDL::Chain& chain,KDL::ChainFkSolverPos_recursive& solver);
    KDL::JntArray getInverseKinematics(KDL::Chain& chain,KDL::Frame& left_foot,KDL::Frame& right_foot,KDL::ChainIkSolverPos_NR_JL& solver);
    bool readJoints(urdf::Model &robot_model);
    
    
    KDL::JntArray q_min,q_max;
    
};   
    
#endif //KINEMATICS_UTILITES_H