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
    pub_geometric_contraints = node.advertise<visualization_msgs::Marker>("/geometric_contraints", 1,true);
    pub_filtered_frames = node.advertise<visualization_msgs::MarkerArray>(node.resolveName("filtered_frames"), 1);
    pub_average_normal_cloud_ = node.advertise<visualization_msgs::MarkerArray>(node.resolveName("average_normal"), 1);
    pub_com = node.advertise<visualization_msgs::Marker>("/com", 1,true);
    pub_ch = node.advertise<visualization_msgs::Marker>("/convex_hull", 1,true);
    
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
    foot_marker.scale.x=0.20;
    foot_marker.scale.y=0.10;
    foot_marker.scale.z=0.02;
    
    foot_marker.color.a=1;
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
    World_MovingFoot = World_MovingFoot*KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(0.035,0,0)); //just for visualization, actually this must be considered always when the foot geometry is used
    
    foot_marker.pose.position.x=World_MovingFoot.p.x();
    foot_marker.pose.position.y=World_MovingFoot.p.y();
    foot_marker.pose.position.z=World_MovingFoot.p.z();
    
    World_MovingFoot.M.GetQuaternion(foot_marker.pose.orientation.x,foot_marker.pose.orientation.y,foot_marker.pose.orientation.z,foot_marker.pose.orientation.w);
    foot_marker.color.r=255*(!left);
    foot_marker.color.g=255*(left);
    foot_marker.id = foot_marker.id+1; //to have a unique id
    foot_marker.lifetime = ros::Duration(0);
            
    pub_footstep.publish(foot_marker);
    
}

void ros_publisher::publish_geometric_constraints(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max, KDL::Frame World_StanceFoot)
{
    visualization_msgs::Marker marker;
    marker.type=visualization_msgs::Marker::CUBE;
    marker.header.frame_id="world";
    marker.header.seq=1;
    marker.id=0;
    marker.pose.position.x = (x_max+x_min)/2.0 + World_StanceFoot.p.x();
    marker.pose.position.y = (y_max+y_min)/2.0 + World_StanceFoot.p.y();
    marker.pose.position.z = (z_max+z_min)/2.0 + World_StanceFoot.p.z();
    World_StanceFoot.M.GetQuaternion(marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,marker.pose.orientation.w);
    marker.scale.x=x_max-x_min;
    marker.scale.y=y_max-y_min;
    marker.scale.z=z_max-z_min;
    marker.lifetime=ros::Duration(0);
    marker.color.b=0.5;
    marker.color.a=0.3;

    pub_geometric_contraints.publish(marker);
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


void ros_publisher::publish_average_normal(std::list< polygon_with_normals >& affordances)
{
    int i;
    visualization_msgs::MarkerArray msg;
    
    visualization_msgs::Marker marker;
    marker.header.stamp=ros::Time::now();
    marker.header.frame_id=camera_link_name;
    marker.ns="average_normals";
    marker.scale.x=0.009;
    marker.scale.y=0.015;
    marker.scale.z=0.009;
    marker.lifetime=ros::Duration(600);
    marker.color.g=1;
    marker.color.a=1;
    marker.type=visualization_msgs::Marker::ARROW;
    geometry_msgs::Point point1,point2;
    
    for(auto aff:affordances)
    {
	point1.x=aff.average_normal.x;
	point1.y=aff.average_normal.y;
	point1.z=aff.average_normal.z;
	point2.x=aff.average_normal.normal_x/20.0+point1.x;
	point2.y=aff.average_normal.normal_y/20.0+point1.y;
	point2.z=aff.average_normal.normal_z/20.0+point1.z;
	
	marker.points.clear();
	marker.id=i++;
	marker.points.push_back(point1);
	marker.points.push_back(point2);
	
	msg.markers.push_back(marker);
    }

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

void ros_publisher::publish_starting_position(std::map<std::string,double> initial_pos)
{
    last_joint_states.header.stamp=ros::Time::now();
    int i=0;
    for (auto joint:last_joint_states.name)
    {
        last_joint_states.position[i]=initial_pos[joint];
        i++;
    }
//     for (int i=0;i<last_joint_states.name.size();i++)
//     {
//         last_joint_states.position[i]=0;
//     }
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
    
    if(robot_name=="atlas_v3")
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
    
    if(robot_name=="walkman" || robot_name=="bigman")
    {
	last_joint_states.position[joints_name_to_index["RShSag"]]=1.2;
	last_joint_states.position[joints_name_to_index["RShYaw"]]=0.5;
	last_joint_states.position[joints_name_to_index["RShLat"]]=-0.2;
	last_joint_states.position[joints_name_to_index["RElbj"]]=-2.45;
	last_joint_states.position[joints_name_to_index["LShSag"]]=1.2;
	last_joint_states.position[joints_name_to_index["LShYaw"]]=-0.5;
	last_joint_states.position[joints_name_to_index["LShLat"]]=0.2;
	last_joint_states.position[joints_name_to_index["LElbj"]]=-2.45;
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

void ros_publisher::publish_com(KDL::Vector com)
{    
    com_marker.header.stamp=ros::Time::now();
    com_marker.header.frame_id="world";
    com_marker.scale.x=0.1;
    com_marker.scale.y=0.1;
    com_marker.scale.z=0.1;
    com_marker.lifetime=ros::Duration(0);
    com_marker.color.a=1;
    com_marker.type=visualization_msgs::Marker::SPHERE;
    com_marker.color.b=1;
    com_marker.pose.position.x = com.x();
    com_marker.pose.position.y = com.y();
    com_marker.pose.position.z = com.z();
    com_marker.pose.orientation.w=1;
    com_marker.id = com_marker.id+1;

    pub_com.publish(com_marker);
}

void ros_publisher::publish_ch(std::vector<KDL::Vector> points)
{
    convex_hull_marker.type = visualization_msgs::Marker::LINE_STRIP;
    convex_hull_marker.action = visualization_msgs::Marker::ADD;
    convex_hull_marker.color.a = 1.0;
    convex_hull_marker.color.r = 0.0;
    convex_hull_marker.color.g = 0.0;
    convex_hull_marker.color.b = 1.0;
    convex_hull_marker.scale.x = 0.02;
    convex_hull_marker.lifetime=ros::Duration(0);
    convex_hull_marker.header.frame_id = "world";
    convex_hull_marker.header.stamp = ros::Time::now();
    convex_hull_marker.points.clear();

    geometry_msgs::Point p;
    for(auto point:points)
    {
	p.x = point.x();
	p.y = point.y();
	p.z = point.z();
	convex_hull_marker.points.push_back(p);
    }

    p.x = points.begin()->x();
    p.y = points.begin()->y();
    p.z = points.begin()->z();

    convex_hull_marker.points.push_back(p);
    pub_ch.publish(convex_hull_marker);
}