#include "../include/ros_publisher.h"
#include <borderextraction.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>

using namespace planner;

ros_publisher::ros_publisher(ros::NodeHandle handle,std::string camera_link_name)
{
    this->node=handle;
    this->camera_link_name=camera_link_name;
    pub_cluster_cloud_ = node.advertise<sensor_msgs::PointCloud2>(node.resolveName("cluster_cloud"), 3000);
    pub_border_poly_marker = node.advertise<visualization_msgs::Marker>("/border_poly_marker",1,true);
    pub_footstep = node.advertise<visualization_msgs::Marker>("/footstep_marker",1,true);
    pub_ik_joints = node.advertise<sensor_msgs::JointState>("/footstep/joint_states",1,true);
    
    borders_marker.header.frame_id=camera_link_name;
    borders_marker.ns="border_poly";
    borders_marker.type=visualization_msgs::Marker::SPHERE_LIST;
    //marker2.type=visualization_msgs::Marker::LINE_STRIP;
    
    borders_marker.pose.position.x=0;
    borders_marker.pose.position.y=0;
    borders_marker.pose.position.z=0;
    borders_marker.pose.orientation.w=1;
    borders_marker.pose.orientation.x=0;
    borders_marker.pose.orientation.y=0;
    borders_marker.pose.orientation.z=0;
    
    borders_marker.scale.x=0.02;
    borders_marker.scale.y=0.02;
    borders_marker.scale.z=0.02;
    borders_marker.color.a=1;
    
    
    foot_marker.header.frame_id=camera_link_name;
    foot_marker.ns = "feet";
    foot_marker.type = visualization_msgs::Marker::CUBE;
    foot_marker.scale.x=0.1;
    foot_marker.scale.y=0.05;
    foot_marker.scale.z=0.02;
    
    foot_marker.color.a=1;
    foot_marker.color.b=0;
    foot_marker.lifetime=ros::Duration(60);
    
}

void ros_publisher::publish_plane_clusters(std::vector< boost::shared_ptr< pcl::PointCloud< pcl::PointXYZRGBNormal > > > clusters)
{
    int j=0;
    for (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster:clusters)
    {
        sensor_msgs::PointCloud2 cluster_cloud_msg;
        pcl::toROSMsg(*cluster, cluster_cloud_msg);
        cluster_cloud_msg.header.seq = j++;
        cluster_cloud_msg.header.frame_id=camera_link_name;//input->header.frame_id;
        pub_cluster_cloud_.publish(cluster_cloud_msg);
        
        ROS_INFO( "Cluster %i: %lu points" , j, cluster->points.size() );
        
        ros::Duration half_sec(0.5);
        //         half_sec.sleep();
    }
}

void ros_publisher::publish_plane_borders(std::vector<polygon_with_normals> borders)
{
    geometry_msgs::Point point;    
    std_msgs::ColorRGBA color;
    uint8_t r, g, b;
    int32_t rgb; 
    int i=0;
    for (polygon_with_normals& border_polygon:borders)
    {
        r = 255*((double)rand()/(double)RAND_MAX);
        g = 255*((double)rand()/(double)RAND_MAX);
        b = 255*((double)rand()/(double)RAND_MAX);
        //         rgb = (r << 16) | (g << 8) | b; 
        
        color.a=borders_marker.color.a;
        color.r=r;
        color.g=g;
        color.b=b;
        
        for(int po = 0; po < border_polygon.border->points.size(); po++) 
        { 
            point.x = border_polygon.border->at(po).x;
            point.y = border_polygon.border->at(po).y;
            point.z = border_polygon.border->at(po).z;
            
            borders_marker.colors.push_back(color);
            borders_marker.points.push_back(point);
        }
        
        borders_marker.id=i;
        i++;
        pub_border_poly_marker.publish(borders_marker);
    }
    
}

void ros_publisher::publish_foot_position(KDL::Frame fromWorldTofoot,int centroid_id, KDL::Frame fromCameraToworld)
{
    
    foot_marker.pose.position.x=fromWorldTofoot.p.x();
    foot_marker.pose.position.y=fromWorldTofoot.p.y();
    foot_marker.pose.position.z=fromWorldTofoot.p.z();
    
    fromWorldTofoot.M.GetQuaternion(foot_marker.pose.orientation.x,foot_marker.pose.orientation.y,foot_marker.pose.orientation.z,foot_marker.pose.orientation.w);
    
    foot_marker.id = centroid_id; //to have a unique id
            
    pub_footstep.publish(foot_marker);
    
}

void ros_publisher::publish_robot_joints(KDL::JntArray joints, std::vector<std::string> joint_names)
{
    sensor_msgs::JointState temp;
    for (int i=0;i<joints.rows();i++)
    {
        temp.position.push_back(joints(i));
    }
    temp.name=joint_names;
    
    pub_ik_joints.publish(temp);
    pub_ik_joints.publish(temp);
    ros::Duration sleep_time(0.5);
    sleep_time.sleep();
    pub_ik_joints.publish(temp);
    pub_ik_joints.publish(temp);
    sleep_time.sleep();
    ros::Duration two_sec(2);
    two_sec.sleep();  
}