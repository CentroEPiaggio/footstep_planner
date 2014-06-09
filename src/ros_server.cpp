#include "ros_server.h"
#include <kdl_conversions/kdl_msg.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
using namespace planner;

extern volatile bool quit;

rosServer::rosServer(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
{
    // init publishers and subscribers
    
    camera_link_name = nh.resolveName("/camera_link");

    pub_cluster_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("cluster_cloud"), 3000);
    pub_border_poly_marker = nh_.advertise<visualization_msgs::Marker>("/border_poly_marker",1,true);
    pub_footstep = nh_.advertise<visualization_msgs::Marker>("/footstep_marker",1,true);
    pub_ik_joints = nh_.advertise<sensor_msgs::JointState>("/footstep/joint_states",1,true);
    
    //sub_input_cloud_ = nh_.subscribe(nh_.resolveName("input_cloud"), 100, &CurvatureFilter::filterByCurvature, this);
    
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
    
    //TODO Mirko set foot initial position
       
}

bool rosServer::extractBorders(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    geometry_msgs::Point point;
    
    visualization_msgs::Marker marker;
    
    marker.header.frame_id=camera_link_name;
    marker.ns="borders";
    marker.type=visualization_msgs::Marker::SPHERE_LIST;
    
    marker.pose.position.x=0;
    marker.pose.position.y=0;
    marker.pose.position.z=0;
    marker.pose.orientation.w=1;
    marker.pose.orientation.x=0;
    marker.pose.orientation.y=0;
    marker.pose.orientation.z=0;
    
    marker.scale.x=0.01;
    marker.scale.y=0.01;
    marker.scale.z=0.01;
    marker.color.a=1;
    
    visualization_msgs::Marker marker2;
    
    marker2.header.frame_id=camera_link_name;
    marker2.ns="border_poly";
    marker2.type=visualization_msgs::Marker::SPHERE_LIST;
    //marker2.type=visualization_msgs::Marker::LINE_STRIP;
    
    marker2.pose.position.x=0;
    marker2.pose.position.y=0;
    marker2.pose.position.z=0;
    marker2.pose.orientation.w=1;
    marker2.pose.orientation.x=0;
    marker2.pose.orientation.y=0;
    marker2.pose.orientation.z=0;
    
    marker2.scale.x=0.02;
    marker2.scale.y=0.02;
    marker2.scale.z=0.02;
    marker2.color.a=1;
    
    std_msgs::ColorRGBA color;
    
    polygons=border_extraction.extractBorders(clusters);
    
    uint8_t r, g, b;
    int32_t rgb; 
    int i=0;
    for (auto border_polygon:polygons)
    {
        r = 255*((double)rand()/(double)RAND_MAX);
        g = 255*((double)rand()/(double)RAND_MAX);
        b = 255*((double)rand()/(double)RAND_MAX);
//         rgb = (r << 16) | (g << 8) | b; 
	
	color.a=marker2.color.a;
	color.r=r;
	color.g=g;
	color.b=b;
                
        for(int po = 0; po < border_polygon->points.size(); po++) 
        { 
            point.x = border_polygon->at(po).x;
            point.y = border_polygon->at(po).y;
            point.z = border_polygon->at(po).z;
            
            marker2.colors.push_back(color);
            marker2.points.push_back(point);
        }
        
        marker2.id=i;
        i++;
        pub_border_poly_marker.publish(marker2);
    }
    return true;
}

bool rosServer::planFootsteps(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if(polygons.size()==0) {
        std::cout<<"No polygons to process, you should call the [/filter_by_curvature] and [/border_extraction] services first"<<std::endl; 
        return false;
        
    }
    std::cout<<std::endl<<"> Number of polygons: "<<polygons.size()<<std::endl;
    tf::StampedTransform transform;
    try{
        listener.lookupTransform(camera_link_name, "/world",
        ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    KDL::Frame temp;
    tf::transformTFToKDL(transform,temp);
    //std::cout<<"from cloud to world:"<<temp<<std::endl;
    footstep_planner.setWorldTransform(temp);
    
    visualization_msgs::Marker marker;
    marker.header.frame_id=camera_link_name;
    marker.ns = "feet";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x=0.1;
    marker.scale.y=0.05;
    marker.scale.z=0.02;
    
    marker.color.a=1;
    marker.color.b=0;
    
    bool left=true;
    auto centroids=footstep_planner.getFeasibleCentroids(polygons,left);
   

    for (auto centroid:centroids)
    {
//         if(left){ marker.color.r=1; marker.color.g=0; left=false;}
//         else { marker.color.r=0; marker.color.g=1; left=true;}
        
        marker.pose.position.x=std::get<0>(centroid.second)[0];
        marker.pose.position.y=std::get<0>(centroid.second)[1];
        marker.pose.position.z=std::get<0>(centroid.second)[2];
        
        marker.pose.orientation.w=0.5; //TODO: orientation (x,y) as the plane where is placed, (z) as we want
        marker.pose.orientation.x=0.5;
        marker.pose.orientation.y=0.5;
        marker.pose.orientation.z=-0.5;
        
        marker.id = centroid.first; //to have a unique id
        marker.lifetime=ros::Duration(60);
        
        footsteps.push_back(marker.pose);
        
        pub_footstep.publish(marker);
        
        sensor_msgs::JointState temp;
        if (left)
        {
            temp.name=footstep_planner.kinematics.left_leg_names;
            temp.name.insert(temp.name.end(),footstep_planner.kinematics.right_leg_names.begin(),footstep_planner.kinematics.right_leg_names.end());
            for (auto nome:temp.name)
                std::cout<<nome<<" ";
        }
        else
        {
            temp.name=footstep_planner.kinematics.right_leg_names;
            temp.name.insert(temp.name.end(),footstep_planner.kinematics.left_leg_names.begin(),footstep_planner.kinematics.left_leg_names.end());
        }
        for (int i=0;i<std::get<1>(centroid.second).rows();i++)
        {
            temp.position.push_back(std::get<1>(centroid.second)(i));
            std::cout<<temp.position[i]<<" ";
        }
        std::cout<<std::endl;
        
        pub_ik_joints.publish(temp);
        pub_ik_joints.publish(temp);
        ros::Duration sleep_time(0.5);
        sleep_time.sleep();
        pub_ik_joints.publish(temp);
        pub_ik_joints.publish(temp);
        sleep_time.sleep();
//         std::cout<<"world to base link:"<<std::get<2>(centroid.second)<<std::endl;
        tf::transformKDLToTF(std::get<2>(centroid.second),current_robot_transform);
        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "world", "base_link"));
        ros::Duration two_sec(2);
        two_sec.sleep();  
    }
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
    //ROS_INFO("Cloud size is %li", input_cloud_ptr->points.size());
    
    
    clusters=curvature_filter.filterByCurvature(input_cloud_ptr);
    int j=0;
    for (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster:clusters)
    {
        sensor_msgs::PointCloud2 cluster_cloud_msg;
        pcl::toROSMsg(*cluster, cluster_cloud_msg);
        cluster_cloud_msg.header.seq = j++;
        cluster_cloud_msg.header.frame_id=input->header.frame_id;
        pub_cluster_cloud_.publish(cluster_cloud_msg);
        
        ROS_INFO( "Cluster %i: %lu points" , j, cluster->points.size() );
        
        ros::Duration half_sec(0.5);
//         half_sec.sleep();
    }
    
//     return true;
return extractBorders(request,response);
}


void rosServer::run()
{
//     static tf::TransformBroadcaster br;
//     static tf::TransformListener lr;
//     br.sendTransform(tf::StampedTransform(current_robot_transform, ros::Time::now(), "world", "base_link"));
//     std::cout<<"transform sent"<<std::endl;
}