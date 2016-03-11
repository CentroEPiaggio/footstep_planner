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

#include "ros_server.h"
#include <borderextraction.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <xml_pcl_io.h>
#include <param_manager.h>
using namespace planner;

extern volatile bool quit;

rosServer::rosServer(ros::NodeHandle* nh_,double period,std::string robot_name_, std::string robot_urdf_file_):
// RateThread(period),
period(period),
nh(nh_), priv_nh_("~"),publisher(*nh,nh->resolveName("/camera_link"),robot_name_),
command_interface("footstep_planner"),status_interface("footstep_planner"),footstep_planner(robot_name_,robot_urdf_file_,&publisher)
{
    // init publishers and subscribers
    
    camera_link_name = nh->resolveName("/camera_link");
    srv_filter_cloud_ = nh->advertiseService(nh->resolveName("filter_by_curvature"), &rosServer::filterByCurvature, this);
    srv_border_extraction = nh->advertiseService(nh->resolveName("border_extraction"), &rosServer::extractBorders, this);
    srv_footstep_placer = nh->advertiseService(nh->resolveName("footstep_placer"),&rosServer::planFootsteps, this);
    
    publisher.setRobotJoints(footstep_planner.kinematics.urdf_model.joints_);
    
    double curvature_threshold_,voxel_size_,normal_radius_,cluster_tolerance_;
    int min_cluster_size_;
    priv_nh_.param<double>("voxel_size", voxel_size_, 0.01);
    priv_nh_.param<double>("normal_radius", normal_radius_, 0.1);
    priv_nh_.param<double>("curvature_threshold", curvature_threshold_, 0.05);
    priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 50);
    priv_nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.05);
    curvature_filter.setParams(curvature_threshold_,voxel_size_,normal_radius_,min_cluster_size_,cluster_tolerance_);
    
    double feasible_area_=2.5;
    priv_nh_.param<double>("feasible_area", feasible_area_, 2.5);
    footstep_planner.setParams(feasible_area_);
    
    filename="pointcloud.xml";
    priv_nh_.param<std::string>("filename", filename, "pointcloud.xml");

    save_to_file = false;
    status_interface.start();
    
    this->loss_function_type =4;
    param_manager::register_param("loss_function_type",loss_function_type);
    param_manager::update_param("loss_function_type",4);
    left=true;
}

void rosServer::thr_body()
{
    while (!stopped)
    {
        if (!paused)
            run();
        usleep(period);
    }
}


void rosServer::resume()
{
    paused=false;
}

void rosServer::stop()
{
    stopped=true;
    thr.join();
}

bool rosServer::start()
{
    stopped=false;
    paused=false;
    thr = std::thread(&rosServer::thr_body,this);
    return true;
}

void rosServer::suspend()
{
    paused=true;
}


bool rosServer::threadInit()
{
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
    return true;
}

void rosServer::run()
{
    int seq_num;
    status_interface.setStatus("ready");
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    
    //publisher.publish_last_joints_position();
    publisher.publish_starting_position(initial_joints_value);
    
    if(command_interface.getCommand(msg,seq_num))
    {
#ifdef USE_YARP
        std::string command = msg.command;
        int first_param;
        if (command=="set_stance_foot")
            first_param=msg.starting_foot=="left"?0:msg.starting_foot=="right"?1:2;
        if (command=="plan_num")
            first_param=msg.num_steps;
#else
        std::string command = msg.frame_id.data();
        int first_param=msg.seq;
#endif
        std::cout<<" - YARP: Command ["<<seq_num<<"] received: "<<command<<std::endl;
        if (command=="set_stance_foot")
        {
            if (first_param==0)//starting_foot=="left")
                footstep_planner.setCurrentStanceFoot(true);
            else if (first_param==1)//starting_foot=="right")
                footstep_planner.setCurrentStanceFoot(false);
            else
                std::cout<<"could not set starting foot to "<<first_param<<" options are:0 left/1 right"<<std::endl;;
        }
	if (command=="reset_starting_position")
        { 
           setInitialPosition();
        }
	if(command=="cap_plan")
	{
	    save_to_file = false;
	    
	    status_interface.setStatus("capture");
	    
	    if(filterByCurvature(req,res))
	    {
	        status_interface.setStatus("planning");
		planFootsteps(req,res);
	    }
	}
	if(command=="cap_save")
	{
	    status_interface.setStatus("capture");
	    save_to_file = true;
	    filterByCurvature(req,res);
	}
	if(command=="load_plan")
	{
	        status_interface.setStatus("planning one step");
		planFootsteps(req,res);
	}
	if (command=="plan_all")
        {
            status_interface.setStatus("planning until possible");
            bool ok=true;
            while (ok)
                ok=planFootsteps(req,res);
        }
        if (command=="plan_num")
        {
            std::string temp;
            temp="planning for "+std::to_string(first_param)+" steps";
            status_interface.setStatus(temp);
            int i=0;
            bool ok=true;
            while (i<first_param && ok)
            {
                ok=planFootsteps(req,res);
                i++;
            }
        }
	if (command=="draw_path")
        {
            sendPathToRviz();
        }
//         if (command=="send_plan_to_walking")
//         {
//             fs_walking_msg temp;
//             if (!create_steps_vector(temp))
//             {
//                 std::cout<<"path was not planned"<<std::endl;
//                 return;
//             }
//             temp.command="steps";
//             temp.current_left_foot=footstep_planner.InitialWaist_LeftFoot;
//             temp.current_right_foot=footstep_planner.InitialWaist_RightFoot;
//             temp.starting_foot=left?"left":"right";//BUG 
// //             walking_command_interface.sendCommand(temp,seq_num_out++);  //TODO: fix the usage of this, walking has changed
//         }
	if(command=="direction")
	{
            std::cout<<"direction is currently not supported without yarp"<<std::endl;
//             footstep_planner.setDirectionVector(msg.x,msg.y,msg.z);
	}
	if(command=="exit")
        {
            abort();
        }
        if(command=="ik_check")
	{
            std::cout<<"ik_check is currently not supported without yarp"<<std::endl;
            
	    if(single_check(true,false)) status_interface.setStatus("IK check SUCCEDED");
	    else status_interface.setStatus("IK check FAILED");
	}
	if(command=="ik_com_check")
	{
            std::cout<<"ik_com_check is currently not supported without yarp"<<std::endl;
            
	    if(single_check(false,false)) status_interface.setStatus("IK_COM check SUCCEDED");
	    else status_interface.setStatus("IK_COM check FAILED");
	}
	if(command=="custom_step")
	{
            std::cout<<"custom_step is currently not supported without yarp"<<std::endl;
            
	    if(single_check(false,true)) status_interface.setStatus("IK_COM check SUCCEDED - Planning Custom Step");
	    else status_interface.setStatus("IK_COM check FAILED");
	}
    }
}

void rosServer::setInitialPosition()
{
//     yarp::sig::Vector temp;
//     KDL::JntArray kdl_left_leg,kdl_right_leg;
//     
//     left_leg.sensePosition(temp);
//     kdl_left_leg.resize(temp.size());
//     for (int i=0;i<temp.size();i++)
//     {
//         kdl_left_leg(i)=temp[i];
//     }
//     int j=0;
//     for (auto joint:footstep_planner.kinematics.idyn_model.left_leg.joint_names)
//     {
//         //std::cout<<joint<<":"<<temp[j]<<std::endl;
//         initial_joints_value[joint]=temp[j++];
//     }
//     right_leg.sensePosition(temp);
//     kdl_right_leg.resize(temp.size());
//     for (int i=0;i<temp.size();i++)
//         kdl_right_leg(i)=temp[i];
//     j=0;
//     for (auto joint:footstep_planner.kinematics.idyn_model.right_leg.joint_names)
//         initial_joints_value[joint]=temp[j++];
//     left_arm.sensePosition(temp);
//     j=0;
//     for (auto joint:footstep_planner.kinematics.idyn_model.left_arm.joint_names)
//         initial_joints_value[joint]=temp[j++];
//     right_arm.sensePosition(temp);
//     j=0;
//     for (auto joint:footstep_planner.kinematics.idyn_model.right_arm.joint_names)
//         initial_joints_value[joint]=temp[j++];
//     torso.sensePosition(temp);
//     j=0;
//     for (auto joint:footstep_planner.kinematics.idyn_model.torso.joint_names)
//         initial_joints_value[joint]=temp[j++];
//      footstep_planner.setInitialPosition(kdl_left_leg,kdl_right_leg);
// 
//     for (auto joint:initial_joints_value)
//     std::cout<<joint.first<<":"<<joint.second<<std::endl;
    
}

bool rosServer::single_check(bool ik_only, bool move)
{
    std::list<foot_with_joints> World_centroids;// = footstep_planner.single_check(msg.left_foot,msg.right_foot,ik_only, move,left);
    if(World_centroids.size())
    {
	if(move)
	{
	    auto final_centroid=footstep_planner.selectBestCentroid(World_centroids,left);  
	
	    publisher.publish_foot_position(final_centroid.World_MovingFoot,final_centroid.index,left);
	
	    footstep_planner.setCurrentSupportFoot(final_centroid.World_MovingFoot,left);

	    path.push_back(std::make_pair(final_centroid,footstep_planner.getLastUsedChain()));
	    left=!left;
	}
	return true;
    }
    return false;
}

bool rosServer::extractBorders(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    polygons=border_extraction.extractBorders(clusters);
    publisher.publish_plane_borders(polygons); 
//     int i=0;
//     for (auto polygon:polygons)
//         publisher.publish_normal_cloud(polygon.normals,i++);
    if(save_to_file)
    {
	std::cout<<"saving your extracted borders and planes on an xml file so that you will not need to call filter_by_curvature anylonger"<<std::endl;
	xml_pcl_io file_manager;
	file_manager.write_to_file(filename,polygons);
    }

    return true;
}

bool rosServer::singleFoot(bool left)
{
    std::list<polygon_with_normals> poly;
    for (auto polygon:polygons)
    {
      polygon_with_normals temp;
      temp.average_normal=polygon.average_normal;
      temp.border=polygon.border->makeShared();
      temp.normals=polygon.normals->makeShared();
      poly.push_back(temp);
    }
    auto World_centroids=footstep_planner.getFeasibleCentroids(poly,left);
    publisher.publish_plane_borders(polygons);
    ros::Duration sleep_time(0.2);
    sleep_time.sleep();
    if (World_centroids.size()==0)
    {
        std::cout<<"no valid plans found"<<std::endl;
        return false;
    }
#ifdef SINGLE_FOOT_OUTPUT
    int k=0;
    for (auto centroid:World_centroids)
    {
        if (k!=6)
        {
            k++;
        }
        else
        {
            k=0;
            publisher.publish_robot_joints(centroid.joints,footstep_planner.getLastUsedChain());
            tf::transformKDLToTF(centroid.World_Waist,current_robot_transform);
            static tf::TransformBroadcaster br;
            br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "world", "base_link"));
            ros::Duration sleep_time(0.2);
            sleep_time.sleep();
        }
    }
#endif
    auto final_centroid=footstep_planner.selectBestCentroid(World_centroids,left,loss_function_type);  
    publisher.publish_foot_position(final_centroid.World_MovingFoot,final_centroid.index,left);

    footstep_planner.setCurrentSupportFoot(final_centroid.World_MovingFoot,left); //Finally we make the step

    path.push_back(std::make_pair(final_centroid,footstep_planner.getLastUsedChain()));
    return true;
}

bool rosServer::sendPathToRviz()
{
    ros::Duration sleep_time(2);
    static tf::TransformBroadcaster br;
    publisher.publish_starting_position(initial_joints_value);
    tf::transformKDLToTF(footstep_planner.World_InitialWaist,current_robot_transform);
    br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "world", "base_link"));
    sleep_time.sleep();
    for (auto centroid:path)
    {
        /*
         *        tf::Transform current_robot_transform;
         *        tf::transformKDLToTF(centroid.first.World_Waist,current_robot_transform);
         *        br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(),  "world","FNEW_WAIST"));
         *        tf::Transform current_moving_foot_transform;
         *        tf::transformKDLToTF(centroid.first.World_MovingFoot,current_moving_foot_transform);
         *        br.sendTransform(tf::StampedTransform(current_moving_foot_transform, ros::Time::now(),  "world","Fmoving_foot"));
         *        tf::Transform fucking_transform;
         *        tf::transformKDLToTF(centroid.first.World_StanceFoot,fucking_transform);
         *        br.sendTransform(tf::StampedTransform(fucking_transform, ros::Time::now(), "world", "Fstance_foot"));
         *        sleep_time.sleep();
         */
        
        publisher.publish_robot_joints(centroid.first.joints,centroid.second);
        tf::transformKDLToTF(centroid.first.World_Waist,current_robot_transform);
        br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "world", "base_link"));
        sleep_time.sleep();
        if (centroid.first.end_joints.rows()==centroid.first.joints.rows()) //both initial and final configuration are set
        {
            publisher.publish_robot_joints(centroid.first.end_joints,centroid.second);
            tf::transformKDLToTF(centroid.first.World_EndWaist,current_robot_transform);
            br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "world", "base_link"));
            sleep_time.sleep();
        }
    }
}

// bool rosServer::create_steps_vector(fs_walking_msg &temp)
// {
//     if (path.size()==0)
//         return false;
//     for (auto centroid:path)
//     {
//         step_with_name temp_step;
// 	temp_step.pose = footstep_planner.InitialWaist_MeanFoot.Inverse()*footstep_planner.World_InitialWaist.Inverse() *centroid.first.World_MovingFoot;
//         temp.steps.push_back(temp_step);
//     }
//     return true;
// }


bool rosServer::planFootsteps(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

    if(polygons.size()==0) {
        std::cout<<"No polygons to process, trying to read them from xml file"<<std::endl;
        xml_pcl_io file_manager;
        bool read=file_manager.read_from_file(filename,polygons);
        if (!read || polygons.size()==0)
        {
           std::cout<<"problems while reading from file, you should call the [/filter_by_curvature] services first"<<std::endl;
           return false;
        }
    }
    std::cout<<std::endl<<"> Number of polygons: "<<polygons.size()<<std::endl;
    
//    bool left=true;
//    bool right=false;
    bool result=singleFoot(left);
    if (!result) return false;
//     singleFoot(right);
//     singleFoot(left);

    left=!left;
//     sendPathToRviz();
    ROS_INFO("planning completed");
    return true;
}


bool rosServer::filterByCurvature(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    
    // wait for a point cloud
    std::string topic = nh->resolveName("/camera/depth_registered/points");
    ROS_INFO("waiting for a point_cloud2 on topic %s", topic.c_str());
    sensor_msgs::PointCloud2::ConstPtr input = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, *nh, ros::Duration(3.0));
    if (!input)
    {
        ROS_ERROR("no point_cloud2 has been received");
        return false;
    }
    
    // convert from sensor_msgs to a tractable PCL object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *input_cloud_ptr);
    clusters=curvature_filter.filterByCurvature(&publisher,input_cloud_ptr);
    //publisher.publish_plane_clusters(clusters);
    return extractBorders(request,response);
}


void rosServer::init()
{
    tf::StampedTransform transform;
    std::string err_msg;
    
    if(listener.waitForTransform("/world",camera_link_name,ros::Time(0), ros::Duration(2.0), ros::Duration(0.01), &err_msg))
    {
	try{
	    listener.lookupTransform("/world", camera_link_name,
				    ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
	    ROS_ERROR("%s",ex.what());
	}
    }
    else
    {
	ROS_ERROR("Error in tf: %s",err_msg.c_str());
    }
    
    KDL::Frame World_Camera;
    tf::transformTFToKDL(transform,World_Camera);
    footstep_planner.setWorldTransform(World_Camera);
}
