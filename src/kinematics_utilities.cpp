#include <kinematics_utilities.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf_model/joint.h>



kinematics_utilities::kinematics_utilities()
{
    coman= coman_model.coman_iDyn3.getKDLTree();
    coman.getChain("l_sole","Waist",left_leg);
    coman.getChain("l_sole","Waist",legs);
    coman.getChain("Waist","r_sole",right_leg);
    fkLsolver = new KDL::ChainFkSolverPos_recursive(left_leg);
    fkRsolver = new KDL::ChainFkSolverPos_recursive(right_leg);
    legs.addChain(right_leg);
    fksolver = new KDL::ChainFkSolverPos_recursive(legs);
    ikvelsolver = new KDL::ChainIkSolverVel_pinv(legs);
    //readJoints(coman_model.coman_model);
    iksolver= new KDL::ChainIkSolverPos_NR_JL(legs,q_min,q_max,*fksolver,*ikvelsolver);
}



KDL::Frame kinematics_utilities::getForwardKinematics(KDL::Chain& chain,KDL::ChainFkSolverPos_recursive& solver)
{
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    
    // Assign some values to the joint positions
    for(unsigned int i=0;i<nj;i++){
        jointpositions(i)=0;
    }
    KDL::Frame cartpos;    
    
    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = solver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }
    return cartpos;
}


KDL::JntArray kinematics_utilities::getInverseKinematics(KDL::Chain& chain,KDL::Frame& left_foot,KDL::Frame& right_foot,KDL::ChainIkSolverPos_NR_JL& solver)
{
    
}




bool kinematics_utilities::readJoints(urdf::Model &robot_model) {
    num_joints = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink("r_sole");
    boost::shared_ptr<const urdf::Joint> joint;
    
    while (link && link->name != "l_sole") {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (!joint) {
            printf("Could not find joint: %s",link->parent_joint->name.c_str());
            return false;
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            printf( "adding joint: [%s]", joint->name.c_str() );
            num_joints++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    
    q_min.resize(num_joints);
    q_max.resize(num_joints);
    //     info.joint_names.resize(num_joints);
    //     info.link_names.resize(num_joints);
    //     info.limits.resize(num_joints);
    
    link = robot_model.getLink("r_sole");
    unsigned int i = 0;
    while (link && i < num_joints) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            printf( "getting bounds for joint: [%s]", joint->name.c_str() );
            
            float lower, upper;
            int hasLimits;
            if ( joint->type != urdf::Joint::CONTINUOUS ) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
                hasLimits = 1;
            } else {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
            }
            int index = num_joints - i -1;
            
            q_min.data[index] = lower;
            q_max.data[index] = upper;
            //             info.joint_names[index] = joint->name;
            //             info.link_names[index] = link->name;
            //             info.limits[index].joint_name = joint->name;
            //             info.limits[index].has_position_limits = hasLimits;
            //             info.limits[index].min_position = lower;
            //             info.limits[index].max_position = upper;
            i++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    return true;
}
