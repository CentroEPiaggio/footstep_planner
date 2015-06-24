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

#include <kinematics_utilities.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf_model/joint.h>
#include <joints_ordering.h>

#define IGNORE_JOINT_LIMITS 0

//NEVER call this without setting the container chain!!
void kinematics_utilities::initialize_solvers(chain_and_solvers* container, KDL::JntArray& joints_value,KDL::JntArray& q_max, KDL::JntArray& q_min, int index)
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
    container->index=index;
    int j=0;
    for (auto joint_name:container->joint_names)
    {
        #if IGNORE_JOINT_LIMITS
        q_max(j)=M_PI/3.0;
        q_min(j)=-M_PI/3.0;
        #else
        q_max(j)=urdf_model.joints_[joint_name]->limits->upper;
        q_min(j)=urdf_model.joints_[joint_name]->limits->lower;
        #endif
	container->average_joints(j)=(q_max(j)+q_min(j))/2.0;
        j++;
    }
    container->iksolver= new KDL::ChainIkSolverPos_NR_JL(container->chain,q_min,q_max,*container->fksolver,*container->ikvelsolver);
}


kinematics_utilities::kinematics_utilities(std::string robot_name_):robot_name(robot_name_),idyn_model(robot_name_)
//,idyn_model(robot_name_)
{
    robot_kdl= idyn_model.iDyn3_model.getKDLTree();
    urdf_model = *idyn_model.urdf_model;
  
    if(robot_name=="coman")
    {
        robot_kdl.getChain("Waist","l_sole",wl_leg.chain);
	robot_kdl.getChain("Waist","r_sole",wr_leg.chain);
	robot_kdl.getChain("l_sole","Waist",lw_leg.chain);
	robot_kdl.getChain("r_sole","Waist",rw_leg.chain);
	num_joints=wl_leg.chain.getNrOfJoints()+wr_leg.chain.getNrOfJoints();
	robot_kdl.getChain("l_sole","Waist",lwr_legs.chain);
	lwr_legs.chain.addChain(wr_leg.chain);
	robot_kdl.getChain("r_sole","Waist",rwl_legs.chain);
	rwl_legs.chain.addChain(wl_leg.chain);
    }
    
    if(robot_name=="atlas_v3" || robot_name=="walkman" || robot_name=="bigman")
    {
	robot_kdl.getChain("pelvis","l_sole",wl_leg.chain);
	robot_kdl.getChain("pelvis","r_sole",wr_leg.chain);
	robot_kdl.getChain("l_sole","pelvis",lw_leg.chain);
	robot_kdl.getChain("r_sole","pelvis",rw_leg.chain);
	num_joints=wl_leg.chain.getNrOfJoints()+wr_leg.chain.getNrOfJoints();
	robot_kdl.getChain("l_sole","pelvis",lwr_legs.chain);
	lwr_legs.chain.addChain(wr_leg.chain);
	robot_kdl.getChain("r_sole","pelvis",rwl_legs.chain);
	rwl_legs.chain.addChain(wl_leg.chain);
    }
    
    initialize_solvers(&wl_leg,wl_leg.joints_value,wl_leg.q_max,wl_leg.q_min,0);
    initialize_solvers(&wr_leg,wr_leg.joints_value,wr_leg.q_max,wr_leg.q_min,0);
    initialize_solvers(&lw_leg,lw_leg.joints_value,lw_leg.q_max,lw_leg.q_min,0);
    initialize_solvers(&rw_leg,rw_leg.joints_value,rw_leg.q_max,rw_leg.q_min,0);
    initialize_solvers(&lwr_legs,lwr_legs.joints_value,lwr_legs.q_max,lwr_legs.q_min,0);
    initialize_solvers(&rwl_legs,rwl_legs.joints_value,rwl_legs.q_max,rwl_legs.q_min,0);
    
    wl_leg_vector.resize(MAX_THREADS);//[i].chain=wl_leg.chain;
    wr_leg_vector.resize(MAX_THREADS);//[i].chain=wr_leg.chain;
    lw_leg_vector.resize(MAX_THREADS);//[i].chain=lw_leg.chain;
    rw_leg_vector.resize(MAX_THREADS);//[i].chain=rw_leg.chain;
    lwr_legs_vector.resize(MAX_THREADS);//[i].chain=lwr_legs.chain;
    rwl_legs_vector.resize(MAX_THREADS);//[i].chain=rwl_legs.chain;
    
    for (int i=0;i<MAX_THREADS;i++)
    {
        
        wl_leg_vector[i].chain=wl_leg.chain;
        wr_leg_vector[i].chain=wr_leg.chain;
        lw_leg_vector[i].chain=lw_leg.chain;
        rw_leg_vector[i].chain=rw_leg.chain;
        lwr_legs_vector[i].chain=lwr_legs.chain;
        rwl_legs_vector[i].chain=rwl_legs.chain;
        
        initialize_solvers(&wl_leg_vector[i],wl_leg_vector[i].joints_value,wl_leg_vector[i].q_max,wl_leg_vector[i].q_min,i);
        initialize_solvers(&wr_leg_vector[i],wr_leg_vector[i].joints_value,wr_leg_vector[i].q_max,wr_leg_vector[i].q_min,i);
        initialize_solvers(&lw_leg_vector[i],lw_leg_vector[i].joints_value,lw_leg_vector[i].q_max,lw_leg_vector[i].q_min,i);
        initialize_solvers(&rw_leg_vector[i],rw_leg_vector[i].joints_value,rw_leg_vector[i].q_max,rw_leg_vector[i].q_min,i);
        initialize_solvers(&lwr_legs_vector[i],lwr_legs_vector[i].joints_value,lwr_legs_vector[i].q_max,lwr_legs_vector[i].q_min,i);
        initialize_solvers(&rwl_legs_vector[i],rwl_legs_vector[i].joints_value,rwl_legs_vector[i].q_max,rwl_legs_vector[i].q_min,i);
        
    }
    
    
}
