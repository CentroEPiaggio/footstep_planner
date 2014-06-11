#include <kinematics_utilities.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf_model/joint.h>
const std::string coman_model_folder = std::string(getenv("YARP_WORKSPACE")) + "/coman_yarp_apps/coman_urdf/coman.urdf";

kinematics_utilities::kinematics_utilities():coman_model()
{
    coman= coman_model.coman_iDyn3.getKDLTree();
    coman.getChain("Waist","l_sole",left_leg);
    coman.getChain("Waist","r_sole",right_leg);
    num_joints=left_leg.getNrOfJoints()+right_leg.getNrOfJoints();
    coman.getChain("l_sole","Waist",LR_legs);
    LR_legs.addChain(right_leg);
    coman.getChain("r_sole","Waist",RL_legs);
    RL_legs.addChain(left_leg);
    fkLsolver = new KDL::ChainFkSolverPos_recursive(left_leg);
    fkRsolver = new KDL::ChainFkSolverPos_recursive(right_leg);
    fkLRsolver = new KDL::ChainFkSolverPos_recursive(LR_legs);
    fkRLsolver = new KDL::ChainFkSolverPos_recursive(RL_legs);
    
    ikLRvelsolver = new KDL::ChainIkSolverVel_pinv(LR_legs);
    ikRLvelsolver = new KDL::ChainIkSolverVel_pinv(RL_legs);
    
    //TODO:readJoints(coman_model.coman_model);

    q_min.resize(num_joints);
    q_max.resize(num_joints);
    if (!coman_urdf_model.initFile(coman_model_folder))
        std::cout<<"Failed to parse urdf robot model"<<std::endl;
#ifdef JOINTS_CONSTRAINTS

    readJoints(coman_urdf_model,"l_sole","Waist",q_minL,q_maxL);
    readJoints(coman_urdf_model,"r_sole","Waist",q_minR,q_maxR);
    for (int i=0;i<left_leg.getNrOfJoints();i++)
    {
        q_min(i)=q_minL(i);
        q_max(i)=q_maxL(i);
    }
    for (int i=0;i<right_leg.getNrOfJoints();i++)
    {
        q_min(i+left_leg.getNrOfJoints())=q_minR(i);
        q_max(i+left_leg.getNrOfJoints())=q_maxR(i);
    }
    
    ikLRsolver= new KDL::ChainIkSolverPos_NR_JL(LR_legs,q_min,q_max,*fkLRsolver,*ikLRvelsolver);
    
    for (int i=0;i<right_leg.getNrOfJoints();i++)
    {
        q_min(i)=q_minR(i);
        q_max(i)=q_maxR(i);
    }
    for (int i=0;i<left_leg.getNrOfJoints();i++)
    {
        q_min(i+right_leg.getNrOfJoints())=q_minL(i);
        q_max(i+right_leg.getNrOfJoints())=q_maxL(i);
    }
    ikRLsolver= new KDL::ChainIkSolverPos_NR_JL(RL_legs,q_min,q_max,*fkRLsolver,*ikRLvelsolver);    
#else
    for (int i=0;i<num_joints;i++)
    {
        q_max(i)=M_PI/3.0;
        q_min(i)=-M_PI/3.0;
    }
    ikLRsolver= new KDL::ChainIkSolverPos_NR_JL(LR_legs,q_min,q_max,*fkLRsolver,*ikLRvelsolver);
    ikRLsolver= new KDL::ChainIkSolverPos_NR_JL(RL_legs,q_min,q_max,*fkRLsolver,*ikRLvelsolver);
    #endif    
    if (!initJointNames(coman_urdf_model,"l_sole","Waist",left_leg_names))
        std::cout<<"ERROR in joint name initialization"<<std::endl;
    if (!initJointNames(coman_urdf_model,"r_sole","Waist",right_leg_names))
        std::cout<<"ERROR in joint name initialization"<<std::endl;
        joint_names_LR=left_leg_names;
        joint_names_LR.insert(joint_names_LR.end(),right_leg_names.begin(),right_leg_names.end());
        joint_names_RL=right_leg_names;
        joint_names_RL.insert(joint_names_RL.end(),left_leg_names.begin(),left_leg_names.end());
    
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

bool kinematics_utilities::initJointNames(urdf::Model &robot_model, std::string tip, std::string parent,std::vector<std::string>& joint_names)
{
    int num_joints = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip);    
    boost::shared_ptr<const urdf::Joint> joint;
    std::cout<<"loading joint names"<<std::endl;
    while (link && link->name != parent) {
        std::cout<<link->name<<std::endl;
        joint = robot_model.getJoint(link->parent_joint->name);
        if (!joint) {
            std::cout<<"Could not find joint: "<<link->parent_joint->name<<std::endl;
            return false; 
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            std::cout<<"adding joint: "<<joint->name<<std::endl;
            joint_names.push_back(joint->name);
            num_joints++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    return true;
}


bool kinematics_utilities::readJoints(urdf::Model &robot_model, std::string tip, std::string parent,KDL::JntArray& q_min,KDL::JntArray& q_max) {
    int num_joints = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip);    
    boost::shared_ptr<const urdf::Joint> joint;
    
    while (link && link->name != parent) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (!joint) {
            std::cout<<"Could not find joint: "<<link->parent_joint->name;
            return false; //TODO we cannot search for l_sole starting from r_sole, we need to double this loop and merge the results into q_min and q_max
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            std::cout<<"adding joint: "<<joint->name<<std::endl;
            num_joints++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    q_min.resize(num_joints);
    q_max.resize(num_joints);
    //     info.joint_names.resize(num_joints);
    //     info.link_names.resize(num_joints);
    //     info.limits.resize(num_joints);
    
    link = robot_model.getLink(tip);
    unsigned int i = 0;
    while (link && i < num_joints) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            std::cout<<"getting bounds for joint: "<< joint->name<<std::endl;
            
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
            int index = num_joints - i -1; //TODO WARNING WHY??!?
            
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
