#include <kinematics_utilities.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf_model/joint.h>
#include <joints_ordering.h>

const std::string coman_model_folder = std::string(getenv("YARP_WORKSPACE")) + "/coman_yarp_apps/coman_urdf/coman.urdf";

#define IGNORE_JOINT_LIMITS 0

//NEVER call this without setting the container chain!!
void kinematics_utilities::initialize_solvers(chain_and_solvers* container, KDL::JntArray& joints_value,KDL::JntArray& q_max, KDL::JntArray& q_min)
{
    for (KDL::Segment& segment: container->chain.segments)
    {
        if (segment.getJoint().getType()==KDL::Joint::None) continue;
        //std::cout<<segment.getJoint().getName()<<std::endl;
        container->joint_names.push_back(segment.getJoint().getName());
    }
    assert(container->joint_names.size()==container->chain.getNrOfJoints());
    joints_value.resize(container->chain.getNrOfJoints());
    SetToZero(joints_value);
    q_max.resize(container->chain.getNrOfJoints());
    q_min.resize(container->chain.getNrOfJoints());
    container->average_joints.resize(container->chain.getNrOfJoints());
    container->fksolver=new KDL::ChainFkSolverPos_recursive(container->chain);
    container->ikvelsolver = new KDL::ChainIkSolverVel_pinv(container->chain);
    int j=0;
    for (auto joint_name:container->joint_names)
    {
        #if IGNORE_JOINT_LIMITS
        q_max(j)=M_PI/3.0;
        q_min(j)=-M_PI/3.0;
        #else
        q_max(j)=coman_urdf_model.joints_[joint_name]->limits->upper;
        q_min(j)=coman_urdf_model.joints_[joint_name]->limits->lower;
        #endif
	container->average_joints(j)=(q_max(j)+q_min(j))/2.0;
        j++;
    }
    container->iksolver= new KDL::ChainIkSolverPos_NR_JL(container->chain,q_min,q_max,*container->fksolver,*container->ikvelsolver);
}


kinematics_utilities::kinematics_utilities():coman_model()
{
    coman= coman_model.coman_iDyn3.getKDLTree();
    coman.getChain("Waist","l_sole",wl_leg.chain);
    coman.getChain("Waist","r_sole",wr_leg.chain);
    coman.getChain("l_sole","Waist",lw_leg.chain);
    coman.getChain("r_sole","Waist",rw_leg.chain);
    num_joints=wl_leg.chain.getNrOfJoints()+wr_leg.chain.getNrOfJoints();
    coman.getChain("l_sole","Waist",lwr_legs.chain);
    lwr_legs.chain.addChain(wr_leg.chain);
    coman.getChain("r_sole","Waist",rwl_legs.chain);
    rwl_legs.chain.addChain(wl_leg.chain);

    if (!coman_urdf_model.initFile(coman_model_folder))
        std::cout<<"Failed to parse urdf robot model"<<std::endl;

    initialize_solvers(&wl_leg,wl_leg.joints_value,wl_leg.q_max,wl_leg.q_min);
    initialize_solvers(&wr_leg,wr_leg.joints_value,wr_leg.q_max,wr_leg.q_min);
    initialize_solvers(&lw_leg,lw_leg.joints_value,lw_leg.q_max,lw_leg.q_min);
    initialize_solvers(&rw_leg,rw_leg.joints_value,rw_leg.q_max,rw_leg.q_min);
    initialize_solvers(&lwr_legs,lwr_legs.joints_value,lwr_legs.q_max,lwr_legs.q_min);
    initialize_solvers(&rwl_legs,rwl_legs.joints_value,rwl_legs.q_max,rwl_legs.q_min);
}