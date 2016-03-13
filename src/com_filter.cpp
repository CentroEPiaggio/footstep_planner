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

#include "com_filter.h"
#include <param_manager.h>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>

double MAX_TESTED_POINTS_1;
double MAX_TESTED_POINTS_2;
double LEVEL_OF_DETAILS;
int MAX_THREADS;
int ANGLE_STEP;

bool com_filter::thread_com_filter(std::list<planner::foot_with_joints> &data, int num_threads)
{
    if (num_threads>MAX_THREADS)
        num_threads=MAX_THREADS;
    std::vector<std::list<planner::foot_with_joints>> temp_list;
    temp_list.resize(num_threads);
    std::vector<std::list<planner::foot_with_joints>> result_list;
    result_list.resize(num_threads);
    std::list<planner::foot_with_joints> result;
    int data_initial_size=data.size();
    int partition=data.size()/num_threads;
    int total=0;
    for (int i=0;i<num_threads-1;i++)
    {
        auto it=data.begin();
        for (int j=0;j<partition;j++)
        {
            total++;
            ++it;
        }
        temp_list[i].splice(temp_list[i].begin(),data,data.begin(),it);
    }
    auto it=data.begin();
    int final_size=data_initial_size-total;
    for (int j=0;j<final_size;j++)
    {
        total++;
        ++it;
    }
    temp_list[num_threads-1].splice(temp_list[num_threads-1].begin(),data,data.begin(),it);
    assert(total==data_initial_size);
    std::vector<std::thread> pool;
    for (int i=0;i<num_threads;i++)
    {
        pool.emplace_back(std::thread(&com_filter::internal_filter_first,this,std::ref(temp_list[i]),StanceFoot_World,World_StanceFoot,
                                      std::ref(result_list[i]),
                                      &current_stance_chain_and_solver->at(i),
                                      &current_moving_chain_and_solver->at(i),desired_hip_height));
    }    
    for (int i=0;i<num_threads;i++)
    {
        pool[i].join();
    }
    
    for (int i=0;i<num_threads;i++)
    {
        result.splice(result.end(),temp_list[i]);
    }
    
    data.swap(result);
    
    for (int i=0;i<num_threads-1;i++)
    {
        temp_list[i].clear();
    }
    pool.clear();
    
    data_initial_size=data.size();
    partition=data.size()/num_threads;
    total=0;
    for (int i=0;i<num_threads-1;i++)
    {
        auto it=data.begin();
        for (int j=0;j<partition;j++)
        {
            total++;
            ++it;
        }
        temp_list[i].splice(temp_list[i].begin(),data,data.begin(),it);
    }
    it=data.begin();
    final_size=data_initial_size-total;
    for (int j=0;j<final_size;j++)
    {
        total++;
        ++it;
    }
    temp_list[num_threads-1].splice(temp_list[num_threads-1].begin(),data,data.begin(),it);
    
    assert(total==data_initial_size);
    for (int i=0;i<num_threads;i++)
    {
        pool.emplace_back(std::thread(&com_filter::internal_filter_second,this,std::ref(temp_list[i]),StanceFoot_World,World_StanceFoot,
                                      std::ref(result_list[i]),
                                      &current_stance_chain_and_solver->at(i),
                                      &current_moving_chain_and_solver->at(i),desired_hip_height));
    }
    for (int i=0;i<num_threads;i++)
    {
        pool[i].join();
        result.splice(result.end(),temp_list[i]);
    }
    current_chain_names=current_stance_chain_and_solver->at(0).joint_names;
    current_chain_names.insert(current_chain_names.end(),current_moving_chain_and_solver->at(0).joint_names.begin(),
                               current_moving_chain_and_solver->at(0).joint_names.end());

    //HACK: temp_list
    data.swap(result);
    return true;
    
}


com_filter::com_filter(std::string robot_name_, std::string robot_urdf_file_):kinematics(robot_name_,robot_urdf_file_)
{
    stance_jnts_in.resize(kinematics.wl_leg.chain.getNrOfJoints());
    SetToZero(stance_jnts_in);
    param_manager::register_param("com_max_tested_points_1",MAX_TESTED_POINTS_1);
    param_manager::update_param("com_max_tested_points_1",2000.0);
    param_manager::register_param("com_max_tested_points_2",MAX_TESTED_POINTS_2);
    param_manager::update_param("com_max_tested_points_2",4000.0);
    param_manager::register_param("LEVEL_OF_DETAILS",LEVEL_OF_DETAILS);
    param_manager::update_param("LEVEL_OF_DETAILS",0);
    param_manager::register_param("MAX_THREADS",MAX_THREADS);
    param_manager::update_param("MAX_THREADS",4);
    param_manager::register_param("COM_ANGLE_STEP",ANGLE_STEP);
    param_manager::update_param("COM_ANGLE_STEP",5);
    
}


bool com_filter::filter(std::list<planner::foot_with_joints> &data)
{
   /*
    //HACK: temp_list
    std::list<planner::foot_with_joints> temp_list;
    if (!internal_filter(data,StanceFoot_World,World_StanceFoot,temp_list,current_stance_chain_and_solver,current_moving_chain_and_solver,desired_hip_height))
        return false;
    //this->setLeftRightFoot(!left);
    current_chain_names=current_stance_chain_and_solver->joint_names;
    current_chain_names.insert(current_chain_names.end(),current_moving_chain_and_solver->joint_names.begin(),current_moving_chain_and_solver->joint_names.end());
    //HACK: temp_list
    data.swap(temp_list);
    return true;
    */
   return thread_com_filter(data,MAX_THREADS);
}


bool com_filter::internal_filter_first(std::list<planner::foot_with_joints> &data, KDL::Frame StanceFoot_World, KDL::Frame World_StanceFoot,
                        std::list<planner::foot_with_joints>& temp_list, chain_and_solvers* current_stance_chain_and_solver, 
                        chain_and_solvers* current_moving_chain_and_solver, double desired_hip_height )
{
    int total=data.size();
    int counter=0;
    int total_num_examined=0;
    int total_num_inserted=0;
    int total_num_failed=0;
    int mod = (total/MAX_TESTED_POINTS_1);
    for (auto single_step=data.begin();single_step!=data.end();)
    {
        counter++;
	if (mod>0 && counter%mod !=0) 
	{
          single_step=data.erase(single_step);
	  continue;
	}
	auto StanceFoot_MovingFoot=StanceFoot_World*single_step->World_MovingFoot;
        single_step->World_StanceFoot=World_StanceFoot;
        auto WaistPositions_StanceFoot=generateWaistPositions_StanceFoot(StanceFoot_MovingFoot,StanceFoot_World,LEVEL_OF_DETAILS,desired_hip_height);
	int num_inserted=0;
        for (auto WaistPosition_StanceFoot:WaistPositions_StanceFoot)
	{
            total_num_examined++;
	    KDL::JntArray jnt_temp(current_moving_chain_and_solver->chain.getNrOfJoints()+current_stance_chain_and_solver->chain.getNrOfJoints());
	    if (frame_is_stable(StanceFoot_MovingFoot,WaistPosition_StanceFoot,jnt_temp,current_stance_chain_and_solver,current_moving_chain_and_solver))
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
//         std::cout<<
        ROS_DEBUG_STREAM(counter<<" / "<<total<<" exam:"<<total_num_examined<<" ins: "<<total_num_inserted<<" fail: "<<total_num_failed);//<<std::endl;//<<"\r";std::cout.flush();//std::endl;
    }
    ROS_INFO_STREAM(counter<<" / "<<total<<" exam:"<<total_num_examined<<" ins: "<<total_num_inserted<<" fail: "<<total_num_failed);//<<std::endl;//<<"\r";std::cout.flush();//std::endl;
//     std::cout<<std::endl;
}
    
    
bool com_filter::internal_filter_second(std::list<planner::foot_with_joints> &data, KDL::Frame StanceFoot_World, KDL::Frame World_StanceFoot,
                                           std::list<planner::foot_with_joints>& temp_list, chain_and_solvers* current_stance_chain_and_solver, 
                                           chain_and_solvers* current_moving_chain_and_solver, double desired_hip_height )
    {
    //HACK temp_list
    temp_list.clear();
//     std::cout<<"Checking for the second foot configurations: "<<data.size()<<std::endl;
    int total=data.size();
    int counter=0;
    int total_num_examined=0;
    int total_num_inserted=0;
    int total_num_failed=0;

    auto temp=current_stance_chain_and_solver;
    current_stance_chain_and_solver=current_moving_chain_and_solver;
    current_moving_chain_and_solver=temp;
    //this->setLeftRightFoot(!left);

    //total=total_num_inserted;
    counter=0;
    total_num_examined=0;
    total_num_inserted=0;
    total_num_failed=0;
    int mod = (total/MAX_TESTED_POINTS_2);
    for (auto single_step=data.begin();single_step!=data.end();)
    {
        counter++;
	if (mod>0 && counter%mod !=0) 
	{
          single_step=data.erase(single_step);
	  continue;
	}
        auto MovingFoot_StanceFoot=(StanceFoot_World*single_step->World_MovingFoot).Inverse();
        single_step->World_StanceFoot=World_StanceFoot;
        auto WaistPositions_MovingFoot=generateWaistPositions_StanceFoot(MovingFoot_StanceFoot,single_step->World_MovingFoot.Inverse(),LEVEL_OF_DETAILS,desired_hip_height);
        int num_inserted=0;
        for (auto WaistPosition_MovingFoot:WaistPositions_MovingFoot)
        {
            total_num_examined++;
            KDL::JntArray jnt_temp(current_moving_chain_and_solver->chain.getNrOfJoints()+current_stance_chain_and_solver->chain.getNrOfJoints());
            if (frame_is_stable(MovingFoot_StanceFoot,WaistPosition_MovingFoot,jnt_temp,current_stance_chain_and_solver,current_moving_chain_and_solver))
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
        ROS_DEBUG_STREAM(counter<<" / "<<total<<" exam:"<<total_num_examined<<" ins: "<<total_num_inserted<<" fail: "<<total_num_failed);//<<std::endl;//<<"\r";std::cout.flush();//std::endl;
    }
    ROS_INFO_STREAM(counter<<" / "<<total<<" exam:"<<total_num_examined<<" ins: "<<total_num_inserted<<" fail: "<<total_num_failed);//<<std::endl;//<<"\r";std::cout.flush();//std::endl;
    
//     std::cout<<std::endl;
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
//         current_stance_chain_and_solver=&kinematics.wl_leg;
//         current_moving_chain_and_solver=&kinematics.wr_leg;
            current_stance_chain_and_solver=&kinematics.wl_leg_vector;
            current_moving_chain_and_solver=&kinematics.wr_leg_vector;
    }
    else
    {
//         current_stance_chain_and_solver=&kinematics.wr_leg;
//         current_moving_chain_and_solver=&kinematics.wl_leg;
        current_stance_chain_and_solver=&kinematics.wr_leg_vector;
        current_moving_chain_and_solver=&kinematics.wl_leg_vector;
    }
    this->left=left;
}


/*bool com_filter::frame_is_stable(const KDL::Frame& StanceFoot_MovingFoot,const KDL::Frame& DesiredWaist_StanceFoot, KDL::JntArray& jnt_pos)
{
    return frame_is_stable(StanceFoot_MovingFoot,DesiredWaist_StanceFoot,jnt_pos,current_stance_chain_and_solver,current_moving_chain_and_solver);
}*/


bool com_filter::frame_is_stable(const KDL::Frame& StanceFoot_MovingFoot,const KDL::Frame& DesiredWaist_StanceFoot, KDL::JntArray& jnt_pos,
                                 chain_and_solvers* current_stance_chain_and_solver, chain_and_solvers* current_moving_chain_and_solver)
{
    auto stance_leg_size=current_stance_chain_and_solver->chain.getNrOfJoints();
    KDL::JntArray stance_jnts(stance_leg_size);
    int result=current_stance_chain_and_solver->iksolver->CartToJnt(current_stance_chain_and_solver->average_joints,DesiredWaist_StanceFoot,stance_jnts);
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


/*std::list< KDL::Frame > com_filter::generateWaistPositions_StanceFoot ( const KDL::Frame& StanceFoot_MovingFoot, 
                                                                        const KDL::Frame& StanceFoot_World, int level_of_details)
{
    return generateWaistPositions_StanceFoot(StanceFoot_MovingFoot,StanceFoot_World,level_of_details,desired_hip_height);
}*/


std::list< KDL::Frame > com_filter::generateWaistPositions_StanceFoot ( const KDL::Frame& StanceFoot_MovingFoot, 
                                                                        const KDL::Frame& StanceFoot_World, int level_of_details,
                                                                        double desired_hip_height
                                                                      )
{
    std::list<KDL::Frame> DesiredWaist_StanceFoot_list;
    //TODO this is still a problem
    //double angle_ref=atan2(StanceFoot_MovingFoot.p[0],-StanceFoot_MovingFoot.p[1])+M_PI*left;
    double angle_ref=0;
    //double angle=0;
//     level_of_details=2;
    for (double angle=-M_PI/6.0;angle<M_PI/6.1;angle=angle+((2.0*M_PI/6.0)/double(ANGLE_STEP)))
    {//double height=-0.01;
        for (double height=-desired_hip_height*0.1-0.01*level_of_details;height<-0.0;height=height+(desired_hip_height*0.05)/(1+level_of_details/2.0))
	{
	    KDL::Frame DesiredWaist_StanceFoot=computeStanceFoot_WaistPosition(StanceFoot_World,angle+angle_ref,desired_hip_height+height*2.0).Inverse();
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

