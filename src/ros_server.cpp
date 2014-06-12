#include "ros_server.h"
#include <borderextraction.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
using namespace planner;

extern volatile bool quit;

rosServer::rosServer(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"),publisher(nh,nh.resolveName("/camera_link"))
{
    // init publishers and subscribers
    
    camera_link_name = nh.resolveName("/camera_link");
    srv_filter_cloud_ = nh_.advertiseService(nh_.resolveName("filter_by_curvature"), &rosServer::filterByCurvature, this);
    srv_border_extraction = nh_.advertiseService(nh_.resolveName("border_extraction"), &rosServer::extractBorders, this);
    srv_footstep_placer = nh_.advertiseService(nh_.resolveName("footstep_placer"),&rosServer::planFootsteps, this);
    
    double curvature_threshold_,voxel_size_,normal_radius_,cluster_tolerance_;
    int min_cluster_size_;
    priv_nh_.param<double>("voxel_size", voxel_size_, 0.02);
    priv_nh_.param<double>("normal_radius", normal_radius_, 0.05);
    priv_nh_.param<double>("curvature_threshold", curvature_threshold_, 0.01);
    priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 100);
    priv_nh_.param<double>("cluster_tolerance", cluster_tolerance_, 100);
    curvature_filter.setParams(curvature_threshold_,voxel_size_,normal_radius_,min_cluster_size_,cluster_tolerance_);
    
    double feasible_area_;
    priv_nh_.param<double>("feasible_area", feasible_area_, 2.5);
    footstep_planner.setParams(feasible_area_);
    
 
    current_direction=KDL::Vector(1,0,0); //TODO receive from the operator?
    footstep_planner.setCurrentDirection(current_direction); //TODO
}

bool rosServer::extractBorders(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    polygons=border_extraction.extractBorders(clusters);
    publisher.publish_plane_borders(polygons); 
    int i=0;
    for (auto polygon:polygons)
        publisher.publish_normal_cloud(polygon.normals,i++);
    return true;
}

bool rosServer::planFootsteps(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if(polygons.size()==0) {
        std::cout<<"No polygons to process, you should call the [/filter_by_curvature] and [/border_extraction] services first"<<std::endl; 
        return false;        
    }
    std::cout<<std::endl<<"> Number of polygons: "<<polygons.size()<<std::endl;
    
    bool left=true;
    auto World_centroids=footstep_planner.getFeasibleCentroids(polygons,left);
    for (auto centroid:World_centroids)
    {
        publisher.publish_foot_position(std::get<0>(centroid.second),centroid.first,footstep_planner.getWorldTransform());
        if (left)
            publisher.publish_robot_joints(std::get<1>(centroid.second),footstep_planner.kinematics.joint_names_LR);
        else
            publisher.publish_robot_joints(std::get<1>(centroid.second),footstep_planner.kinematics.joint_names_RL);
            //std::cout<<"world to base link:"<<std::get<2>(centroid.second)<<std::endl;
        tf::transformKDLToTF(std::get<2>(centroid.second),current_robot_transform);
        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "world", "base_link"));
        
        ros::Duration sleep_time(1);
        sleep_time.sleep();
    }
    auto final_centroid=footstep_planner.selectBestCentroid(World_centroids,left);    
    path.push_back(final_centroid);
    
    left=false;
    footstep_planner.setCurrentSupportFoot(std::get<0>(final_centroid.second));
    World_centroids=footstep_planner.getFeasibleCentroids(polygons,left);
    for (auto centroid:World_centroids)
    {
        publisher.publish_foot_position(std::get<0>(centroid.second),centroid.first,footstep_planner.getWorldTransform());
        if (left)
            publisher.publish_robot_joints(std::get<1>(centroid.second),footstep_planner.kinematics.joint_names_LR);
        else
            publisher.publish_robot_joints(std::get<1>(centroid.second),footstep_planner.kinematics.joint_names_RL);
        //std::cout<<"world to base link:"<<std::get<2>(centroid.second)<<std::endl;
        tf::transformKDLToTF(std::get<2>(centroid.second),current_robot_transform);
        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "world", "base_link"));
        ros::Duration sleep_time(1);
        sleep_time.sleep();
    }
    final_centroid=footstep_planner.selectBestCentroid(World_centroids,left);  
    path.push_back(final_centroid);
//     left=true;
//     footstep_planner.setCurrentSupportFoot(std::get<0>(final_centroid.second));
//     centroids=footstep_planner.getFeasibleCentroids(polygons,left);
//     final_centroid=footstep_planner.selectBestCentroid(centroids,left);    
//     path.push_back(final_centroid);
    
//     for (auto centroid:path)
//     {
//         publisher.publish_foot_position(std::get<0>(centroid.second),centroid.first,footstep_planner.getWorldTransform());
//         if (left)
//             publisher.publish_robot_joints(std::get<1>(centroid.second),footstep_planner.kinematics.joint_names_LR);
//         else
//             publisher.publish_robot_joints(std::get<1>(centroid.second),footstep_planner.kinematics.joint_names_RL);
//         //         std::cout<<"world to base link:"<<std::get<2>(centroid.second)<<std::endl;
//              tf::transformKDLToTF(std::get<2>(centroid.second),current_robot_transform);
//              static tf::TransformBroadcaster br;
//              br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "world", "base_link"));
//         
//     }
    return true;
}


bool rosServer::filterByCurvature(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    
    // wait for a point cloud
    std::string topic = nh_.resolveName("/camera/depth_registered/points");
    ROS_INFO("waiting for a point_cloud2 on topic %s", topic.c_str());
    sensor_msgs::PointCloud2::ConstPtr input = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(3.0));
    if (!input)
    {
        ROS_ERROR("no point_cloud2 has been received");
        return false;
    }
    
    // convert from sensor_msgs to a tractable PCL object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *input_cloud_ptr);
    
    clusters=curvature_filter.filterByCurvature(input_cloud_ptr);
    publisher.publish_plane_clusters(clusters);
    return extractBorders(request,response);
}


void rosServer::run()
{
    tf::StampedTransform transform;
    try{
        listener.lookupTransform("/world", camera_link_name,
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    KDL::Frame World_Camera;
    tf::transformTFToKDL(transform,World_Camera);
    footstep_planner.setWorldTransform(World_Camera);
//     static tf::TransformBroadcaster br;
//     static tf::TransformListener lr;
//     br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "world", "base_link"));
//     std::cout<<"transform sent"<<std::endl;
}