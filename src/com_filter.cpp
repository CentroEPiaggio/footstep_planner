#include "com_filter.h"
#include <iCub/iDynTree/iDyn2KDL.h>
#include <eigen3/Eigen/Dense>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>
com_filter::com_filter()
{
    stance_jnts_in.resize(kinematics.wl_leg.chain.getNrOfJoints());
    SetToZero(stance_jnts_in);
}

bool com_filter::filter(std::list<planner::foot_with_joints> &data)
{
    int total=data.size();
    int counter=0;
    int total_num_examined=0;
    int total_num_inserted=0;
    int total_num_failed=0;
    //HACK temp_list
    std::list<planner::foot_with_joints> temp_list;
    for (auto single_step=data.begin();single_step!=data.end();)
    {
        counter++;
        auto StanceFoot_MovingFoot=StanceFoot_World*single_step->World_MovingFoot;
        single_step->World_StanceFoot=World_StanceFoot;
        auto WaistPositions_StanceFoot=generateWaistPositions_StanceFoot(StanceFoot_MovingFoot,StanceFoot_World);
	int num_inserted=0;
        for (auto WaistPosition_StanceFoot:WaistPositions_StanceFoot)
	{
            total_num_examined++;
	    KDL::JntArray jnt_temp(kinematics.num_joints);
	    if (frame_is_stable(StanceFoot_MovingFoot,WaistPosition_StanceFoot,jnt_temp))
	    {
		planner::foot_with_joints temp;
		temp.joints=jnt_temp;
		temp.World_MovingFoot=single_step->World_MovingFoot;
		temp.World_StanceFoot=single_step->World_StanceFoot;
		temp.World_Waist=single_step->World_StanceFoot*WaistPosition_StanceFoot.Inverse();
		data.insert(single_step,temp);
                //HACK temp_list
		temp_list.push_back(temp);
                num_inserted++;
                total_num_inserted++;
                {
                    tf::Transform current_robot_transform;
                    tf::transformKDLToTF(World_StanceFoot*WaistPosition_StanceFoot.Inverse(),current_robot_transform);
                    static tf::TransformBroadcaster br;
                    br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(),  "world","C_Waist"));
                    tf::Transform current_moving_foot_transform;
                    tf::transformKDLToTF(World_StanceFoot*StanceFoot_MovingFoot,current_moving_foot_transform);
                    br.sendTransform(tf::StampedTransform(current_moving_foot_transform, ros::Time::now(),  "world","C_moving_foot"));
                    tf::Transform fucking_transform;
                    tf::transformKDLToTF(World_StanceFoot,fucking_transform);
                    br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "C_stance_foot"));
                    //ros::Duration sleep_time(0.02);
                    //sleep_time.sleep();
                }
	    }
	    else
                total_num_failed++;
	}
	single_step=data.erase(single_step);
        std::cout<<counter<<" / "<<total<<" exam:"<<total_num_examined<<" ins: "<<total_num_inserted<<" fail: "<<total_num_failed<<"\r";std::cout.flush();//std::endl;
    }
    std::cout<<std::endl;
    //HACK temp_list
    temp_list.clear();
    std::cout<<"Checking for the second foot configurations: "<<data.size()<<std::endl;
    //TODO:check for the second foot, i.e. moving foot becomes stable and stance becomes moving
    this->setLeftRightFoot(!left);
    total=total_num_inserted;
    counter=0;
    total_num_examined=0;
    total_num_inserted=0;
    total_num_failed=0;
    for (auto single_step=data.begin();single_step!=data.end();)
    {
        counter++;
        auto MovingFoot_StanceFoot=(StanceFoot_World*single_step->World_MovingFoot).Inverse();
        single_step->World_StanceFoot=World_StanceFoot;
        auto WaistPositions_MovingFoot=generateWaistPositions_StanceFoot(MovingFoot_StanceFoot,single_step->World_MovingFoot.Inverse(),2);
        int num_inserted=0;
        for (auto WaistPosition_MovingFoot:WaistPositions_MovingFoot)
        {
            total_num_examined++;
            KDL::JntArray jnt_temp(kinematics.num_joints);
            if (frame_is_stable(MovingFoot_StanceFoot,WaistPosition_MovingFoot,jnt_temp))
            {
                planner::foot_with_joints temp;
                temp.start_joints=single_step->joints;
                //We are going to swap stance and moving foot joints in order to give back to the planner the same order for start and end joints
                KDL::JntArray swap_jnt=jnt_temp;
                auto leg_size=swap_jnt.rows()/2;
                for (int i=0;i<leg_size;i++)
                {
                    jnt_temp(i)=swap_jnt(i+leg_size);
                    jnt_temp(i+leg_size)=swap_jnt(i);
                }
                temp.end_joints=jnt_temp;
                temp.joints=single_step->joints;
                temp.World_Waist=single_step->World_Waist;
                temp.World_MovingFoot=single_step->World_MovingFoot;
                temp.World_StanceFoot=single_step->World_StanceFoot;
                temp.World_StartWaist=single_step->World_StartWaist;
                temp.World_EndWaist=single_step->World_MovingFoot*WaistPosition_MovingFoot.Inverse();
                data.insert(single_step,temp);
                //HACK temp_list
                temp_list.push_back(temp);
                num_inserted++;
                total_num_inserted++;
            }
            else
                total_num_failed++;
        }
        single_step=data.erase(single_step);
        std::cout<<counter<<" / "<<total<<" exam:"<<total_num_examined<<" ins: "<<total_num_inserted<<" fail: "<<total_num_failed<<"\r";std::cout.flush();//std::endl;
    }
    std::cout<<std::endl;
    this->setLeftRightFoot(!left);
    current_chain_names=current_stance_chain_and_solver->joint_names;
    current_chain_names.insert(current_chain_names.end(),current_moving_chain_and_solver->joint_names.begin(),current_moving_chain_and_solver->joint_names.end());
    //HACK: temp_list
    data.swap(temp_list);
    return true;
}


void com_filter::setWorld_StanceFoot(const KDL::Frame& World_StanceFoot)
{
    this->World_StanceFoot=World_StanceFoot;
    this->StanceFoot_World=World_StanceFoot.Inverse();
}

void com_filter::setLeftRightFoot(bool left)
{
    if (left)
    {
        current_stance_chain_and_solver=&kinematics.wl_leg;
        current_moving_chain_and_solver=&kinematics.wr_leg;
    }
    else
    {
        current_stance_chain_and_solver=&kinematics.wr_leg;
        current_moving_chain_and_solver=&kinematics.wl_leg;
    }
    this->left=left;
}

bool com_filter::frame_is_stable(const KDL::Frame& StanceFoot_MovingFoot,const KDL::Frame& DesiredWaist_StanceFoot, KDL::JntArray& jnt_pos)
{
    auto stance_leg_size=kinematics.wl_leg.chain.getNrOfJoints();
    KDL::JntArray stance_jnts(stance_leg_size);
    int result=current_stance_chain_and_solver->iksolver->CartToJnt(current_moving_chain_and_solver->average_joints,DesiredWaist_StanceFoot,stance_jnts);
    if (result<0) return false;
    
    KDL::JntArray moving_jnts(stance_leg_size);
    result=current_moving_chain_and_solver->iksolver->CartToJnt(current_moving_chain_and_solver->average_joints,DesiredWaist_StanceFoot*StanceFoot_MovingFoot,moving_jnts);
    if (result<0) return false;

    for (int j=0;j<stance_leg_size;j++)
    {
        jnt_pos(j)=stance_jnts(j);
        jnt_pos(j+stance_leg_size)=moving_jnts(j);
    }
    return true;
}

std::vector< std::string > com_filter::getJointOrder()
{
    return current_chain_names;
}


void com_filter::setZeroWaistHeight ( double hip_height )
{
    this->desired_hip_height=hip_height;
}


std::list< KDL::Frame > com_filter::generateWaistPositions_StanceFoot ( const KDL::Frame& StanceFoot_MovingFoot, const KDL::Frame& StanceFoot_World, int level_of_details)
{
    std::list<KDL::Frame> DesiredWaist_StanceFoot_list;
    //TODO this is still a problem
    //double angle_ref=atan2(StanceFoot_MovingFoot.p[0],-StanceFoot_MovingFoot.p[1])+M_PI*left;
    double angle_ref=0;
    //double angle=0;
    for (double angle=-M_PI/6.0;angle<M_PI/6.1;angle=angle+M_PI/30.0)
    {//double height=-0.01;
        for (double height=-0.05-0.01*level_of_details;height<-0.0;height=height+0.02/(1+level_of_details/2.0))
	{
	    KDL::Frame DesiredWaist_StanceFoot=computeStanceFoot_WaistPosition(StanceFoot_World,angle+angle_ref,desired_hip_height+height).Inverse();
	    DesiredWaist_StanceFoot_list.push_back(DesiredWaist_StanceFoot);
        }
    }
    return DesiredWaist_StanceFoot_list;
}


KDL::Frame com_filter::computeStanceFoot_WaistPosition(const KDL::Frame& StanceFoot_World,double rot_angle,double hip_height)
{
    KDL::Frame StanceFoot_WaistPosition;
    KDL::Vector World_GravityFromIMU(0,0,-1);
    KDL::Vector StanceFoot_GravityFromIMU = StanceFoot_World.M*World_GravityFromIMU;
    StanceFoot_GravityFromIMU.Normalize();
    StanceFoot_WaistPosition.M=KDL::Rotation::Rot2(StanceFoot_GravityFromIMU,rot_angle);
    StanceFoot_WaistPosition.p=KDL::Vector(StanceFoot_GravityFromIMU*(-hip_height));
    return StanceFoot_WaistPosition;
}


