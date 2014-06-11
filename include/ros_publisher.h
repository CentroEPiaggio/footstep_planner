#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <kdl/jntarray.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "borderextraction.h"

namespace planner
{

class ros_publisher
{
public:
    ros_publisher(ros::NodeHandle handle, std::string camera_link_name);
    void publish_plane_clusters(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clusters);
    void publish_plane_borders(std::vector< polygon_with_normals> borders);
    void publish_foot_position(KDL::Frame fromWorldTofoot, int centroid_id, KDL::Frame fromCameraToworld);
    void publish_robot_joints(KDL::JntArray joints, std::vector<std::string> joint_names);
private:
    ros::NodeHandle node;
    ros::Publisher pub_cluster_cloud_;
    ros::Publisher pub_border_poly_marker;
    ros::Publisher pub_ik_joints;
    ros::Publisher pub_footstep;
    
    std::string camera_link_name;
    visualization_msgs::Marker borders_marker;
    visualization_msgs::Marker foot_marker;
    
};

}
#endif // ROS_PUBLISHER_H
