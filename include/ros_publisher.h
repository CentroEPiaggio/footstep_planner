#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <kdl/jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <urdf_model/joint.h>
#include "borderextraction.h"
#include "data_types.h"

namespace planner
{

class ros_publisher
{
public:
    ros_publisher(ros::NodeHandle handle, std::string camera_link_name);
    void publish_plane_clusters(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clusters);
    void publish_plane_borders(const std::list< polygon_with_normals>& borders);
    void publish_foot_position(KDL::Frame World_MovingFoot, int centroid_id, bool left);
    void publish_robot_joints(const KDL::JntArray& joints, std::vector< std::string > joint_names);
    void publish_normal_cloud(pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr normals,int i);
        void publish_normal_cloud(pcl::PointCloud< pcl::Normal >::Ptr normals,pcl::PointCloud< pcl::PointXYZRGB >::Ptr points,int i);

    void publish_last_joints_position();
    void setRobotJoints(std::map< std::string, boost::shared_ptr< urdf::Joint > > joints_);
    void publish_starting_position();
    void publish_filtered_frames(std::list<foot_with_joints> steps, KDL::Frame World_Camera, int color);
    
private:
    ros::NodeHandle node;
    ros::Publisher pub_cluster_cloud_;
    ros::Publisher pub_border_poly_marker;
    ros::Publisher pub_ik_joints;
    ros::Publisher pub_footstep;
    ros::Publisher pub_normal_cloud_;
    ros::Publisher pub_filtered_frames;

    std::string camera_link_name;
    visualization_msgs::Marker borders_marker;
    visualization_msgs::Marker foot_marker;
    sensor_msgs::JointState last_joint_states;
    std::map< std::string, int > joints_name_to_index;

};

}
#endif // ROS_PUBLISHER_H
