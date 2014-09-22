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

#include "../include/ros_publisher.h"
#include <borderextraction.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <urdf_model/joint.h>
#include <Eigen/src/Eigenvalues/RealQZ.h>
using namespace planner;

ros_publisher::ros_publisher(ros::NodeHandle handle,std::string camera_link_name,std::string robot_name_):robot_name(robot_name_)
{
    this->node=handle;
    this->camera_link_name=camera_link_name;
    pub_cluster_cloud_ = node.advertise<sensor_msgs::PointCloud2>(node.resolveName("cluster_cloud"), 3000);
    pub_border_poly_marker = node.advertise<visualization_msgs::Marker>("/border_poly_marker",1,true);
    pub_footstep = node.advertise<visualization_msgs::Marker>("/footstep_marker",1,true);
    pub_ik_joints = node.advertise<sensor_msgs::JointState>("/joint_states",1,true);
    pub_normal_cloud_ = node.advertise<visualization_msgs::MarkerArray>(node.resolveName("normal_cloud"), 1);
    pub_filtered_frames = node.advertise<visualization_msgs::MarkerArray>(node.resolveName("filtered_frames"), 1);
    pub_average_normal_cloud_ = node.advertise<visualization_msgs::MarkerArray>(node.resolveName("average_normal"), 1);
    
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
    
    
    foot_marker.header.frame_id="/world";
    foot_marker.ns = "feet";
    foot_marker.type = visualization_msgs::Marker::CUBE;
    foot_marker.scale.x=0.1;
    foot_marker.scale.y=0.05;
    foot_marker.scale.z=0.02;
    
    foot_marker.color.a=1;
    foot_marker.color.r=255;
    foot_marker.lifetime=ros::Duration(600);
    
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

void ros_publisher::publish_plane_borders(const std::list<polygon_with_normals>& borders)
{
    geometry_msgs::Point point;    
    std_msgs::ColorRGBA color;
    uint8_t r, g, b;
    int32_t rgb; 
    int i=0;
    for (polygon_with_normals const& border_polygon:borders)
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

void ros_publisher::publish_foot_position(KDL::Frame World_MovingFoot,int centroid_id, bool left=true)
{
    
    foot_marker.pose.position.x=World_MovingFoot.p.x();
    foot_marker.pose.position.y=World_MovingFoot.p.y();
    foot_marker.pose.position.z=World_MovingFoot.p.z();
    
    World_MovingFoot.M.GetQuaternion(foot_marker.pose.orientation.x,foot_marker.pose.orientation.y,foot_marker.pose.orientation.z,foot_marker.pose.orientation.w);
    foot_marker.color.r=255*(!left);
    foot_marker.color.g=255*(left);
    foot_marker.id = centroid_id; //to have a unique id
    foot_marker.lifetime = ros::Duration(0);
            
    pub_footstep.publish(foot_marker);
    
}

void ros_publisher::publish_normal_cloud(pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr normals, int i)
{
    visualization_msgs::MarkerArray msg;
    
    visualization_msgs::Marker marker;
    marker.header.stamp=ros::Time::now();
    marker.header.frame_id=camera_link_name;
    marker.ns="normals"+std::to_string(i);
    marker.scale.x=0.003;
    marker.scale.y=0.005;
    marker.scale.z=0.003;
    marker.lifetime=ros::Duration(600);
    marker.color.r=1;
    marker.color.a=1;
    marker.type=visualization_msgs::Marker::ARROW;
    geometry_msgs::Point point1,point2;
    
    for (int i=0;i<normals->size();i++)
    {
        point1.x=(*normals)[i].x;
        point1.y=(*normals)[i].y;
        point1.z=(*normals)[i].z;
        point2.x=(*normals)[i].normal_x/20.0+point1.x;
        point2.y=(*normals)[i].normal_y/20.0+point1.y;
        point2.z=(*normals)[i].normal_z/20.0+point1.z;
        
        marker.points.clear();
        marker.id=i;
        marker.points.push_back(point1);
        marker.points.push_back(point2);
        
        msg.markers.push_back(marker);
    }
    pub_normal_cloud_.publish(msg);
}

void ros_publisher::publish_normal_cloud(pcl::PointCloud< pcl::Normal >::Ptr normals, pcl::PointCloud< pcl::PointXYZRGB >::Ptr points, int i)
{
    visualization_msgs::MarkerArray msg;
    
    visualization_msgs::Marker marker;
    marker.header.stamp=ros::Time::now();
    marker.header.frame_id=camera_link_name;
    marker.ns="normals"+std::to_string(i);
    marker.scale.x=0.003;
    marker.scale.y=0.005;
    marker.scale.z=0.003;
    marker.lifetime=ros::Duration(600);
    marker.color.r=1;
    marker.color.a=1;
    marker.type=visualization_msgs::Marker::ARROW;
    geometry_msgs::Point point1,point2;
    
    for (int i=0;i<normals->size();i++)
    {
        point1.x=(*points)[i].x;
        point1.y=(*points)[i].y;
        point1.z=(*points)[i].z;
        point2.x=(*normals)[i].normal_x/20.0+point1.x;
        point2.y=(*normals)[i].normal_y/20.0+point1.y;
        point2.z=(*normals)[i].normal_z/20.0+point1.z;
        
        marker.points.clear();
        marker.id=i;
        marker.points.push_back(point1);
        marker.points.push_back(point2);
        
        msg.markers.push_back(marker);
    }
    pub_normal_cloud_.publish(msg);
}


void ros_publisher::publish_average_normal(pcl::PointXYZRGBNormal normal, int i)
{
    visualization_msgs::MarkerArray msg;
    
    visualization_msgs::Marker marker;
    marker.header.stamp=ros::Time::now();
    marker.header.frame_id=camera_link_name;
    marker.ns="normals"+std::to_string(i);
    marker.scale.x=0.009;
    marker.scale.y=0.015;
    marker.scale.z=0.009;
    marker.lifetime=ros::Duration(600);
    marker.color.g=1;
    marker.color.a=1;
    marker.type=visualization_msgs::Marker::ARROW;
    geometry_msgs::Point point1,point2;
    
    point1.x=normal.x;
    point1.y=normal.y;
    point1.z=normal.z;
    point2.x=normal.normal_x/20.0+point1.x;
    point2.y=normal.normal_y/20.0+point1.y;
    point2.z=normal.normal_z/20.0+point1.z;
    
    marker.points.clear();
    marker.id=i;
    marker.points.push_back(point1);
    marker.points.push_back(point2);
    
    msg.markers.push_back(marker);

    pub_average_normal_cloud_.publish(msg);
}


void ros_publisher::publish_last_joints_position()
{
    last_joint_states.header.stamp=ros::Time::now();
    pub_ik_joints.publish(last_joint_states);
}

void ros_publisher::setRobotJoints(std::map< std::string, boost::shared_ptr< urdf::Joint > > joints_)
{
    last_joint_states.position.resize(joints_.size());
    last_joint_states.name.resize(joints_.size());
    int j=0;
    for (auto joint:joints_)
    {
        last_joint_states.name[j]=joint.first;
        last_joint_states.position[j]=0;
        joints_name_to_index[joint.first]=j;
        //std::cout<<joint.first<<joints_name_to_index[joint.first]<<"=="<<j<<std::endl;
        j++;
    }
}

void ros_publisher::publish_starting_position()
{
    last_joint_states.header.stamp=ros::Time::now();
    for (int i=0;i<last_joint_states.name.size();i++)
    {
        last_joint_states.position[i]=0;
    }
    pub_ik_joints.publish(last_joint_states);
}

void ros_publisher::publish_robot_joints(KDL::JntArray const& joints, std::vector<std::string> joint_names)
{
    last_joint_states.header.stamp=ros::Time::now();
    int i=0;
    for (auto joint:joint_names)
    {
        //std::cout<<joints_name_to_index[joint]<<joint<<joints(i)<<" ";
        last_joint_states.position[joints_name_to_index[joint]]=joints(i);
        i++;
    }
    //std::cout<<std::endl;
        
    //HACK for visualization
    if(robot_name=="coman")
    {
	last_joint_states.position[joints_name_to_index["RShSag"]]=0.6;
	last_joint_states.position[joints_name_to_index["RShLat"]]=-0.4;
	last_joint_states.position[joints_name_to_index["RElbj"]]=-1.7;
	last_joint_states.position[joints_name_to_index["LShSag"]]=0.6;
	last_joint_states.position[joints_name_to_index["LShLat"]]=0.4;
	last_joint_states.position[joints_name_to_index["LElbj"]]=-1.7;
    }
    
    if(robot_name=="atlas")
    {
	last_joint_states.position[joints_name_to_index["r_arm_shx"]]=1.3;
	last_joint_states.position[joints_name_to_index["r_arm_ely"]]=2;
	last_joint_states.position[joints_name_to_index["r_arm_elx"]]=-1.5;
	last_joint_states.position[joints_name_to_index["r_arm_shy"]]=0.5;
	last_joint_states.position[joints_name_to_index["l_arm_shx"]]=-1.3;
	last_joint_states.position[joints_name_to_index["l_arm_ely"]]=2;
	last_joint_states.position[joints_name_to_index["l_arm_elx"]]=1.5;
	last_joint_states.position[joints_name_to_index["l_arm_shy"]]=0.5;
    }
    
    pub_ik_joints.publish(last_joint_states);
}

void ros_publisher::publish_filtered_frames(std::list< foot_with_joints > steps, KDL::Frame World_Camera, int color)
{
    visualization_msgs::MarkerArray msg;
    
    visualization_msgs::Marker marker;
    marker.header.stamp=ros::Time::now();
    marker.header.frame_id=camera_link_name;
    marker.scale.x=0.02;
    marker.scale.y=0.02;
    marker.scale.z=0.02;
    marker.lifetime=ros::Duration(600);
    marker.color.a=1;
    marker.type=visualization_msgs::Marker::SPHERE_LIST;
    if (color==1) { marker.color.r=1; marker.color.g=0.5; marker.color.b=0.1;}
    if (color==2) { marker.color.r=0.6; marker.color.g=0.6; }
    if (color==3) marker.color.g=1;
    
    for (auto step:steps)
    {	
	KDL::Frame temp = World_Camera.Inverse()*step.World_MovingFoot;
	geometry_msgs::Point point;
	point.x=temp.p.x();
	point.y=temp.p.y();
	point.z=temp.p.z();
	marker.ns="samples"+std::to_string((long long unsigned int)&step);
	marker.points.push_back(point);
        marker.id=(long long unsigned int)&step;
	
        msg.markers.push_back(marker);
    }
    pub_filtered_frames.publish(msg);
}

