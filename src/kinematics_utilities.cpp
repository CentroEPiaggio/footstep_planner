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
    
    ikLvelsolver = new KDL::ChainIkSolverVel_pinv(left_leg);
    ikRvelsolver = new KDL::ChainIkSolverVel_pinv(right_leg);
    ikLRvelsolver = new KDL::ChainIkSolverVel_pinv(LR_legs);
    ikRLvelsolver = new KDL::ChainIkSolverVel_pinv(RL_legs);
    

    if (!coman_urdf_model.initFile(coman_model_folder))
        std::cout<<"Failed to parse urdf robot model"<<std::endl;

    if (!initJointNames(coman_urdf_model,"l_sole","Waist",left_leg_names))
        std::cout<<"ERROR in joint name initialization"<<std::endl;
    if (!initJointNames(coman_urdf_model,"r_sole","Waist",right_leg_names))
        std::cout<<"ERROR in joint name initialization"<<std::endl;

    joint_names_LR=left_leg_names;
    joint_names_LR.insert(joint_names_LR.end(),right_leg_names.begin(),right_leg_names.end());
    joint_names_RL=right_leg_names;
    joint_names_RL.insert(joint_names_RL.end(),left_leg_names.begin(),left_leg_names.end());


    q_min.resize(num_joints);
    q_max.resize(num_joints);

    int j=0;
    for (auto joint_name:joint_names_LR)
    {
        q_max(j)=coman_urdf_model.joints_[joint_name]->limits->upper;
        q_min(j)=coman_urdf_model.joints_[joint_name]->limits->lower;
        j++;
    }
    ikLRsolver= new KDL::ChainIkSolverPos_NR_JL(LR_legs,q_min,q_max,*fkLRsolver,*ikLRvelsolver);
    
    q_min.resize(left_leg.getNrOfJoints());
    q_max.resize(left_leg.getNrOfJoints());

    j=0;
    for (auto joint_name:left_leg_names)
    {
        q_max(j)=coman_urdf_model.joints_[joint_name]->limits->upper;
        q_min(j)=coman_urdf_model.joints_[joint_name]->limits->lower;
        j++;
    }
    ikLsolver= new KDL::ChainIkSolverPos_NR_JL(left_leg,q_min,q_max,*fkLsolver,*ikLvelsolver);

    q_min.resize(num_joints);
    q_max.resize(num_joints);
    
    j=0;
    for (auto joint_name:joint_names_RL)
    {
        q_max(j)=coman_urdf_model.joints_[joint_name]->limits->upper;
        q_min(j)=coman_urdf_model.joints_[joint_name]->limits->lower;
        j++;
    }

    ikRLsolver= new KDL::ChainIkSolverPos_NR_JL(RL_legs,q_min,q_max,*fkRLsolver,*ikRLvelsolver);
    
    
    q_min.resize(right_leg.getNrOfJoints());
    q_max.resize(right_leg.getNrOfJoints());

    j=0;
    for (auto joint_name:right_leg_names)
    {
        q_max(j)=coman_urdf_model.joints_[joint_name]->limits->upper;
        q_min(j)=coman_urdf_model.joints_[joint_name]->limits->lower;
        j++;
    }
    ikRsolver= new KDL::ChainIkSolverPos_NR_JL(right_leg,q_min,q_max,*fkRsolver,*ikRvelsolver);

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
